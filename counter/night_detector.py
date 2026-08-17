import os
import threading
import time
import uuid
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class Track:
    center: tuple
    last_seen: float
    history: list = field(default_factory=list)
    counted: bool = False


class NightLightDetector:
    """Track bright, moving light clusters when the road is dark.

    Frigate supplies the restream. This deliberately counts a light cluster as
    one vehicle and is disabled in daylight, where normal Frigate car events
    remain authoritative.
    """

    def __init__(self, on_count):
        self.stream = os.getenv(
            "NIGHT_STREAM", "rtsp://frigate:8554/road"
        )
        self.enabled = os.getenv("NIGHT_LIGHTS_ENABLED", "true").lower() == "true"
        self.dark_mean = float(os.getenv("NIGHT_DARK_MEAN", "72"))
        self.day_mean = float(os.getenv("NIGHT_DAY_MEAN", "90"))
        self.switch_frames = int(os.getenv("NIGHT_SWITCH_FRAMES", "24"))
        self.bright_threshold = int(os.getenv("NIGHT_BRIGHT_THRESHOLD", "215"))
        self.dim_threshold = int(os.getenv("NIGHT_DIM_THRESHOLD", "105"))
        self.red_saturation = int(os.getenv("NIGHT_RED_SATURATION", "70"))
        self.red_value = int(os.getenv("NIGHT_RED_VALUE", "75"))
        self.line_x = float(os.getenv("NIGHT_LINE_X", "0.29"))
        self.roi = self._polygon(os.getenv(
            "NIGHT_ROI",
            "0.00,0.00;0.54,0.00;0.45,0.10;0.39,0.14;0.33,0.31;0.27,0.38;0.00,0.63",
        ))
        self.on_count = on_count
        self.active = False
        self.status = {"enabled": self.enabled, "active": False, "tracks": 0}
        self.tracks = {}
        self.next_track_id = 1
        self.previous_gray = None
        self.last_count = {"left": 0.0, "right": 0.0}
        self.dark_frames = 0
        self.day_frames = 0
        self.lock = threading.Lock()

    @staticmethod
    def _polygon(value):
        return [tuple(map(float, pair.split(","))) for pair in value.split(";")]

    def snapshot(self):
        with self.lock:
            return dict(self.status)

    def start(self):
        if self.enabled:
            threading.Thread(target=self.run, daemon=True, name="night-lights").start()

    def run(self):
        while True:
            capture = cv2.VideoCapture(self.stream, cv2.CAP_FFMPEG)
            if not capture.isOpened():
                self._set_status(error="stream unavailable")
                time.sleep(5)
                continue

            while capture.isOpened():
                ok, frame = capture.read()
                if not ok:
                    break
                try:
                    self.process(frame)
                except Exception as exc:
                    self._set_status(error=str(exc))
                time.sleep(0.08)
            capture.release()
            self.active = False
            time.sleep(2)

    def _set_status(self, **values):
        with self.lock:
            self.status.update(values)

    def process(self, frame):
        height, width = frame.shape[:2]
        polygon = np.array(
            [[int(x * width), int(y * height)] for x, y in self.roi],
            dtype=np.int32,
        )
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(mask, [polygon], 255)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)
        pixels = gray[mask > 0]
        mean = float(np.mean(pixels)) if pixels.size else 255.0
        if mean <= self.dark_mean:
            self.dark_frames += 1
            self.day_frames = 0
        elif mean >= self.day_mean:
            self.day_frames += 1
            self.dark_frames = 0
        else:
            # Twilight band: retain the current mode instead of oscillating.
            self.dark_frames = 0
            self.day_frames = 0

        if not self.active and self.dark_frames >= self.switch_frames:
            self.active = True
            self.tracks.clear()
            print(f"MODE NIGHT brightness={mean:.1f}", flush=True)
        elif self.active and self.day_frames >= self.switch_frames:
            self.active = False
            self.tracks.clear()
            print(f"MODE DAY brightness={mean:.1f}", flush=True)

        if not self.active:
            self.tracks.clear()
            self.previous_gray = gray
            self._set_status(
                active=False, mode="day", brightness=round(mean, 1), tracks=0
            )
            return

        _, white_lights = cv2.threshold(
            gray, self.bright_threshold, 255, cv2.THRESH_BINARY
        )
        _, dim_lights = cv2.threshold(
            gray, self.dim_threshold, 255, cv2.THRESH_BINARY
        )

        # Red taillights are much darker in grayscale than white headlights.
        # Detect both ends of a vehicle so neither travel direction disappears.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_low = cv2.inRange(
            hsv,
            np.array([0, self.red_saturation, self.red_value]),
            np.array([14, 255, 255]),
        )
        red_high = cv2.inRange(
            hsv,
            np.array([166, self.red_saturation, self.red_value]),
            np.array([179, 255, 255]),
        )
        red_lights = cv2.bitwise_or(red_low, red_high)
        lights = cv2.bitwise_or(dim_lights, red_lights)
        lights = cv2.bitwise_and(lights, mask)
        if self.previous_gray is None:
            self.previous_gray = gray
            return

        # A bright region must also be changing. This removes fixed lamps,
        # reflections and the large stationary glare visible in this camera.
        difference = cv2.absdiff(gray, self.previous_gray)
        _, moving = cv2.threshold(difference, 14, 255, cv2.THRESH_BINARY)
        moving = cv2.dilate(moving, np.ones((11, 11), np.uint8), iterations=2)
        lights = cv2.bitwise_and(lights, moving)
        self.previous_gray = gray
        # Join the two lamps of one car into a single cluster.
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 9))
        lights = cv2.morphologyEx(lights, cv2.MORPH_CLOSE, kernel, iterations=2)
        lights = cv2.dilate(lights, kernel, iterations=1)

        centers = []
        for contour in cv2.findContours(lights, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h
            if 55 <= area <= width * height * 0.035:
                centers.append(((x + w / 2) / width, (y + h / 2) / height))

        now = time.monotonic()
        unmatched = set(self.tracks)
        for center in centers:
            candidates = [
                (np.hypot(center[0] - track.center[0], center[1] - track.center[1]), track_id)
                for track_id, track in self.tracks.items()
                if track_id in unmatched
            ]
            distance, track_id = min(candidates, default=(999, None))
            if distance > 0.10:
                track_id = self.next_track_id
                self.next_track_id += 1
                self.tracks[track_id] = Track(center=center, last_seen=now)
            else:
                unmatched.discard(track_id)

            track = self.tracks[track_id]
            track.center = center
            track.last_seen = now
            track.history.append(center[0])
            track.history = track.history[-20:]
            self._maybe_count(track_id, track)

        self.tracks = {
            track_id: track for track_id, track in self.tracks.items()
            if now - track.last_seen < 1.2
        }
        self._set_status(
            active=True, brightness=round(mean, 1), candidates=len(centers),
            mode="night", tracks=len(self.tracks),
            white_pixels=int(cv2.countNonZero(white_lights)),
            dim_pixels=int(cv2.countNonZero(dim_lights)),
            red_pixels=int(cv2.countNonZero(red_lights)), error=None,
        )

    def _maybe_count(self, track_id, track):
        if track.counted or len(track.history) < 7:
            return
        start = float(np.median(track.history[:3]))
        end = float(np.median(track.history[-3:]))
        displacement = end - start
        crossed = min(start, end) <= self.line_x <= max(start, end)
        if not crossed or abs(displacement) < 0.045:
            return

        direction = "right" if displacement > 0 else "left"
        steps = np.diff(track.history)
        agreement = float(np.mean(steps > 0)) if direction == "right" else float(np.mean(steps < 0))
        if agreement < 0.65:
            return

        now = time.monotonic()
        # Prevent a temporarily split pair of headlights from becoming two cars.
        if now - self.last_count[direction] < 1.8:
            track.counted = True
            return

        event_id = f"night-{int(time.time())}-{uuid.uuid4().hex[:8]}"
        if self.on_count(event_id, direction):
            self.last_count[direction] = now
            print(
                f"NIGHT COUNTED {'→' if direction == 'right' else '←'} "
                f"{event_id} track={track_id} x={start:.3f}->{end:.3f}",
                flush=True,
            )
        track.counted = True
