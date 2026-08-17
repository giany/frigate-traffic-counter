# Frigate Traffic Counter

A self-hosted directional traffic counter built with [Frigate](https://frigate.video/), MQTT, OpenCV, Flask, and SQLite.

It counts vehicles travelling left or right, stores historical totals, and presents hourly, daily, weekly, and monthly reports in a lightweight web dashboard. Day and night detections are recorded separately so experimental light-based night counting does not get mixed with Frigate's daytime object detections.

## Features

- Counts cars crossing a configurable Frigate zone
- Determines travel direction from Frigate trajectory data
- Continues evaluating short or initially ambiguous tracks
- Automatic day/night switching with brightness hysteresis
- Separate daytime and nighttime statistics
- Experimental night tracking for headlights and taillights
- Bucharest-aware timestamps by default
- Hour, day, week, month, and all-time reports
- JSON API for integrations
- Persistent SQLite storage
- Docker Compose deployment

## How it works

During the day, Frigate detects and tracks `car` objects. The counter subscribes to `frigate/events` over MQTT and counts an event after it enters the configured counting zone and its direction can be determined.

At night, an OpenCV worker reads Frigate's go2rtc restream. It detects moving bright or dim light clusters inside a configurable road region, tracks them across a virtual line, and records those results with the separate `night` source.

```text
Camera → go2rtc/Frigate ┬→ Frigate car events → MQTT ─┐
                       └→ Night light tracker ────────┼→ SQLite → Dashboard/API
                                                     ┘
```

## Requirements

- Docker with Docker Compose
- An RTSP-capable camera
- A machine capable of running Frigate object detection

The included Compose file uses Frigate's ARM64 image. On an x86-64 host, change the Frigate image in `docker-compose.yml` to the appropriate image for your platform.

## Setup

1. Clone the repository:

   ```bash
   git clone https://github.com/YOUR_USERNAME/frigate-traffic-counter.git
   cd frigate-traffic-counter
   ```

2. Create your private Frigate configuration:

   ```bash
   cp config/config.example.yml config/config.yml
   ```

3. Edit `config/config.yml`:

   - Replace the example RTSP URL with your camera stream.
   - Draw `road_zone` and `clean_zone` for your camera in Frigate's zone editor.
   - Keep the camera name and counting-zone name aligned with `CAMERA` and `COUNT_ZONE` in `docker-compose.yml`.

4. Start the services:

   ```bash
   docker compose up -d --build
   ```

5. Open the interfaces:

   - Counter dashboard: <http://localhost:8081>
   - Frigate: <https://localhost:8971>
   - go2rtc: <http://localhost:1984>

6. Follow the counter logs:

   ```bash
   docker compose logs -f car-counter
   ```

## Configuration

The most important counter settings are under `car-counter.environment` in `docker-compose.yml`.

| Variable | Default | Purpose |
| --- | ---: | --- |
| `CAMERA` | `road` | Frigate camera name |
| `COUNT_ZONE` | `clean_zone` | Zone a vehicle must enter before it can be counted |
| `TIMEZONE` | `Europe/Bucharest` | Timezone used for stored counts and reports |
| `NIGHT_LIGHTS_ENABLED` | `true` | Enables experimental night-light tracking |
| `NIGHT_STREAM` | `rtsp://frigate:8554/road` | Frigate/go2rtc stream used by the night worker |
| `NIGHT_DARK_MEAN` | `72` | Brightness below which sustained frames select night mode |
| `NIGHT_DAY_MEAN` | `90` | Brightness above which sustained frames select day mode |
| `NIGHT_SWITCH_FRAMES` | `24` | Frames required before changing day/night mode |
| `NIGHT_BRIGHT_THRESHOLD` | `215` | White-headlight diagnostic threshold |
| `NIGHT_DIM_THRESHOLD` | `105` | Monochrome/IR light threshold |
| `NIGHT_RED_SATURATION` | `70` | Minimum saturation for red taillights |
| `NIGHT_RED_VALUE` | `75` | Minimum brightness for red taillights |
| `NIGHT_LINE_X` | `0.29` | Normalized horizontal counting-line position |
| `NIGHT_ROI` | camera-specific polygon | Semicolon-separated normalized road coordinates |
| `DB_PATH` | `/data/counter.db` | SQLite database path inside the container |

Trajectory thresholds such as `MIN_DIRECTION_DISPLACEMENT`, `MIN_DIRECTION_SLOPE`, and the left/right entry limits can also be supplied as environment variables. Their defaults are defined near the top of `counter/app.py`.

## API

`GET /api/counts` returns current totals and histories:

```bash
curl http://localhost:8081/api/counts
```

Notable fields include:

- `mode`: current `day` or `night` mode
- `sources_today`: separate Frigate/day and light/night counts
- `sources_all_time`: all-time totals by source
- `hourly`: today's hourly counts
- `history`: recent hour, day, week, and month buckets
- `night_detector`: live brightness, candidates, tracks, and light-mask diagnostics

## Data and privacy

Runtime data is intentionally excluded from Git:

- `config/config.yml` and Frigate secrets
- Recordings, clips, and thumbnails
- Frigate, counter, and Mosquitto databases
- Environment files and private keys

Never commit your real RTSP URL or camera credentials. Use `config/config.example.yml` as the public template.

## Night-counting limitations

Night counting is experimental. Camera auto-exposure, infrared mode, reflections, vegetation, fixed lamps, occlusion, and the visibility difference between headlights and taillights can all affect accuracy. Day and night results are therefore stored separately.

For production-grade nighttime accuracy, use a custom object-detection model trained on nighttime images from the installed camera. The light tracker is a practical fallback, not a replacement for a properly trained model.

## Troubleshooting

Check service health:

```bash
docker compose ps
docker compose logs --tail=200 frigate
docker compose logs --tail=200 car-counter
```

Healthy Frigate statistics should show camera and process FPS close to the configured detection FPS, with little or no skipped FPS. If the counter receives no daytime events, first confirm that Frigate itself creates `car` events and that those objects enter `clean_zone`.

For night mode, inspect the `night_detector` object from `/api/counts`. A detector that continuously reports tracks without traffic usually indicates glare or moving vegetation inside `NIGHT_ROI`.
