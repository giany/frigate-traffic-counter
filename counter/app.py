import json
import os
import sqlite3
import threading
from datetime import datetime, timedelta
from statistics import median
from zoneinfo import ZoneInfo

from flask import Flask, jsonify
import paho.mqtt.client as mqtt
from night_detector import NightLightDetector


MQTT_HOST = os.getenv("MQTT_HOST", "mosquitto")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))

CAMERA = os.getenv("CAMERA", "road")
COUNT_ZONE = os.getenv("COUNT_ZONE", "clean_zone")

DIRECTION_POINTS = int(os.getenv("DIRECTION_POINTS", "30"))
MIN_DIRECTION_POINTS = int(os.getenv("MIN_DIRECTION_POINTS", "4"))
MIN_DIRECTION_DISPLACEMENT = float(
    os.getenv("MIN_DIRECTION_DISPLACEMENT", "0.06")
)
MIN_DIRECTION_SLOPE = float(os.getenv("MIN_DIRECTION_SLOPE", "0.015"))
MIN_RIGHT_AGREEMENT = float(os.getenv("MIN_RIGHT_AGREEMENT", "0.50"))
MIN_LEFT_AGREEMENT = float(os.getenv("MIN_LEFT_AGREEMENT", "0.50"))
MIN_STEP_MOVEMENT = float(os.getenv("MIN_STEP_MOVEMENT", "0.01"))
RIGHT_ENTRY_MAX_X = float(os.getenv("RIGHT_ENTRY_MAX_X", "0.32"))
LEFT_ENTRY_MIN_X = float(os.getenv("LEFT_ENTRY_MIN_X", "0.34"))
MIN_SHORT_TRACK_DISPLACEMENT = float(
    os.getenv("MIN_SHORT_TRACK_DISPLACEMENT", "0.025")
)
IMMEDIATE_RIGHT_MAX_X = float(os.getenv("IMMEDIATE_RIGHT_MAX_X", "0.25"))
IMMEDIATE_LEFT_MIN_X = float(os.getenv("IMMEDIATE_LEFT_MIN_X", "0.38"))

DB_PATH = os.getenv("DB_PATH", "/data/counter.db")
LOCAL_TIMEZONE = ZoneInfo(os.getenv("TIMEZONE", "Europe/Bucharest"))


app = Flask(__name__)
lock = threading.Lock()

# IDs counted during the current process lifetime.
# SQLite also protects against duplicates across restarts.
counted = set()
night_detector = None


def db():
    conn = sqlite3.connect(DB_PATH)

    conn.execute("""
        CREATE TABLE IF NOT EXISTS counts (
            event_id TEXT PRIMARY KEY,
            timestamp TEXT NOT NULL,
            direction TEXT NOT NULL
        )
    """)

    columns = {
        row[1] for row in conn.execute("PRAGMA table_info(counts)").fetchall()
    }
    if "source" not in columns:
        conn.execute(
            "ALTER TABLE counts ADD COLUMN source TEXT NOT NULL DEFAULT 'day'"
        )
        conn.execute(
            "UPDATE counts SET source = 'night' WHERE event_id LIKE 'night-%'"
        )

    conn.commit()
    return conn


def save_count(event_id, direction, source="day"):
    conn = db()

    try:
        conn.execute(
            """
            INSERT INTO counts (
                event_id,
                timestamp,
                direction
                , source
            )
            VALUES (?, ?, ?, ?)
            """,
            (
                event_id,
                datetime.now(LOCAL_TIMEZONE).isoformat(),
                direction,
                source,
            ),
        )

        conn.commit()

        print(
            f"COUNT {source} {direction}: {event_id}",
            flush=True,
        )

        return True

    except sqlite3.IntegrityError:
        # Already counted in SQLite, probably before a restart.
        print(
            f"ALREADY COUNTED: {event_id}",
            flush=True,
        )

        return False

    finally:
        conn.close()


def determine_direction(path_data):
    """
    Determine travel direction from Frigate path_data.

    Expected format:

        [
            [[x, y], timestamp],
            [[x, y], timestamp],
            ...
        ]

    x increases when moving toward the right side of the image.
    x decreases when moving toward the left side.

    The decision combines three robust signals:

    * median position near the beginning versus near the end
    * Theil-Sen slope (median of all pairwise slopes)
    * proportion of meaningful steps agreeing with that direction

    All three must agree, so one bad tracking point cannot cause a
    count by itself.

    Returns (direction, diagnostics), where direction is None until
    the trajectory has enough consistent horizontal movement.
    """

    if len(path_data) < MIN_DIRECTION_POINTS:
        return None, {
            "displacement": 0.0,
            "slope": 0.0,
            "agreement": 0.0,
        }

    points = path_data[-DIRECTION_POINTS:]

    try:
        samples = [
            (float(point[1]), float(point[0][0]))
            for point in points
        ]
    except (IndexError, TypeError, ValueError):
        return None, {
            "displacement": 0.0,
            "slope": 0.0,
            "agreement": 0.0,
        }

    if len(samples) < MIN_DIRECTION_POINTS:
        return None, {
            "displacement": 0.0,
            "slope": 0.0,
            "agreement": 0.0,
        }

    xs = [x for _, x in samples]

    endpoint_size = max(2, min(4, len(xs) // 3))
    start_x = median(xs[:endpoint_size])
    end_x = median(xs[-endpoint_size:])
    displacement = end_x - start_x

    # Calculate movement per observation rather than per wall-clock second.
    # Frigate can keep an event alive (or repeat an unchanged path) for a
    # long time, which previously diluted an otherwise clear trajectory.
    pairwise_slopes = []
    for start_index, (_, start_position) in enumerate(samples[:-1]):
        for end_index in range(start_index + 1, len(samples)):
            end_position = samples[end_index][1]
            pairwise_slopes.append(
                (end_position - start_position)
                / (end_index - start_index)
            )

    slope = median(pairwise_slopes) if pairwise_slopes else 0.0

    meaningful_steps = [
        current - previous
        for previous, current in zip(xs, xs[1:])
        if abs(current - previous) >= MIN_STEP_MOVEMENT
    ]

    wanted_sign = 1 if displacement > 0 else -1
    agreeing_steps = sum(
        1
        for step in meaningful_steps
        if step * wanted_sign > 0
    )
    agreement = (
        agreeing_steps / len(meaningful_steps)
        if meaningful_steps
        else 0.0
    )

    diagnostics = {
        "displacement": displacement,
        "slope": slope,
        "agreement": agreement,
    }

    if abs(displacement) < MIN_DIRECTION_DISPLACEMENT:
        return None, diagnostics

    required_agreement = (
        MIN_RIGHT_AGREEMENT
        if displacement > 0
        else MIN_LEFT_AGREEMENT
    )

    if agreement < required_agreement:
        return None, diagnostics

    if displacement > 0 and slope > MIN_DIRECTION_SLOPE:
        return "right", diagnostics

    if displacement < 0 and slope < -MIN_DIRECTION_SLOPE:
        return "left", diagnostics

    return None, diagnostics


def get_recent_x_values(path_data):
    try:
        return [
            round(float(point[0][0]), 3)
            for point in path_data[-DIRECTION_POINTS:]
        ]
    except Exception:
        return []


def determine_direction_from_entry(path_data):
    """Fallback for short/occluded tracks, used only when an event ends."""
    if len(path_data) < 2:
        return None, 0.0

    try:
        xs = [float(point[0][0]) for point in path_data]
        entry_x = median(xs[:3]) if len(xs) >= 3 else xs[0]
    except (IndexError, TypeError, ValueError):
        return None, 0.0

    if entry_x <= RIGHT_ENTRY_MAX_X:
        return "right", entry_x

    if entry_x >= LEFT_ENTRY_MIN_X:
        return "left", entry_x

    # Tracks born in the small middle gap can still be classified when
    # their limited samples show unambiguous horizontal movement.
    short_displacement = xs[-1] - xs[0]

    if short_displacement >= MIN_SHORT_TRACK_DISPLACEMENT:
        return "right", entry_x

    if short_displacement <= -MIN_SHORT_TRACK_DISPLACEMENT:
        return "left", entry_x

    return None, entry_x


def determine_immediate_direction(path_data):
    """Count immediately only when the entry side is unambiguous."""
    if len(path_data) < 2:
        return None, 0.0

    try:
        xs = [float(point[0][0]) for point in path_data[:3]]
        entry_x = median(xs) if len(xs) >= 3 else xs[0]
    except (IndexError, TypeError, ValueError):
        return None, 0.0

    if entry_x <= IMMEDIATE_RIGHT_MAX_X:
        return "right", entry_x

    if entry_x >= IMMEDIATE_LEFT_MIN_X:
        return "left", entry_x

    return None, entry_x


def on_connect(
    client,
    userdata,
    flags,
    reason_code,
    properties,
):
    print(
        f"MQTT connected: {reason_code}",
        flush=True,
    )

    client.subscribe("frigate/events")


def on_message(
    client,
    userdata,
    message,
):
    try:
        payload = json.loads(
            message.payload.decode()
        )

    except Exception as e:
        print(
            f"Invalid MQTT JSON: {e}",
            flush=True,
        )
        return

    after = payload.get("after") or {}

    # At night the light-cluster tracker is authoritative. Ignoring Frigate
    # object events here prevents the same vehicle being counted twice.
    if night_detector is not None and night_detector.snapshot().get("active"):
        return

    if after.get("camera") != CAMERA:
        return

    if after.get("label") != "car":
        return

    event_id = after.get("id")

    if not event_id:
        return

    entered_zones = (
        after.get("entered_zones")
        or []
    )

    path_data = (
        after.get("path_data")
        or []
    )

    # Frigate's entered_zones is cumulative for this tracked object.
    #
    # Once clean_zone appears here, we know the car passed through
    # our counting zone. We can keep waiting for later MQTT updates
    # until enough path data exists to determine direction.
    has_entered_count_zone = (
        COUNT_ZONE in entered_zones
    )

    if not has_entered_count_zone:
        return

    with lock:

        if event_id in counted:
            return

        direction, entry_x = determine_immediate_direction(path_data)
        method = "entry-immediate"

        if direction is None:
            direction, diagnostics = determine_direction(path_data)
            method = "trajectory"
        else:
            diagnostics = {
                "displacement": 0.0,
                "slope": 0.0,
                "agreement": 0.0,
            }

        recent_x = get_recent_x_values(
            path_data
        )

        if direction is None and payload.get("type") == "end":
            direction, entry_x = determine_direction_from_entry(path_data)
            method = "entry-fallback"

        if direction is None:

            if payload.get("type") == "end":
                print(
                    f"MISSED AT END "
                    f"{event_id} "
                    f"points={len(path_data)} "
                    f"entry_x={entry_x:.3f} "
                    f"dx={diagnostics['displacement']:.3f} "
                    f"slope={diagnostics['slope']:.4f} "
                    f"agreement={diagnostics['agreement']:.0%} "
                    f"x={recent_x}",
                    flush=True,
                )
                return

            print(
                f"WAITING FOR DIRECTION "
                f"{event_id} "
                f"dx={diagnostics['displacement']:.3f} "
                f"slope={diagnostics['slope']:.4f} "
                f"agreement={diagnostics['agreement']:.0%} "
                f"x={recent_x}",
                flush=True,
            )

            return

        inserted = save_count(
            event_id,
            direction,
        )

        # Even if SQLite says this event already existed,
        # mark it locally so we don't keep processing it.
        counted.add(event_id)

        arrow = (
            "→"
            if direction == "right"
            else "←"
        )

        if inserted:
            print(
                f"COUNTED {arrow} "
                f"{event_id} "
                f"method={method} "
                f"entry_x={entry_x:.3f} "
                f"dx={diagnostics['displacement']:.3f} "
                f"slope={diagnostics['slope']:.4f} "
                f"agreement={diagnostics['agreement']:.0%} "
                f"x={recent_x}",
                flush=True,
            )


def aggregate_counts(conn, period, limit):
    expressions = {
        "hour": "substr(timestamp, 1, 13)",
        "day": "substr(timestamp, 1, 10)",
        "week": "strftime('%Y-W%W', substr(timestamp, 1, 19))",
        "month": "substr(timestamp, 1, 7)",
    }
    expression = expressions[period]

    return conn.execute(
        f"""
        SELECT
            {expression} AS bucket,
            SUM(CASE WHEN direction = 'right' THEN 1 ELSE 0 END),
            SUM(CASE WHEN direction = 'left' THEN 1 ELSE 0 END),
            COUNT(*)
        FROM counts
        GROUP BY bucket
        ORDER BY bucket DESC
        LIMIT ?
        """,
        (limit,),
    ).fetchall()


def serialize_buckets(rows):
    return [
        {
            "period": period,
            "right": right,
            "left": left,
            "total": total,
        }
        for period, right, left, total in rows
    ]


def source_summary(conn, date_prefix=None):
    where = "WHERE timestamp LIKE ?" if date_prefix else ""
    params = (f"{date_prefix}%",) if date_prefix else ()
    rows = conn.execute(
        f"""
        SELECT source, direction, COUNT(*)
        FROM counts
        {where}
        GROUP BY source, direction
        """,
        params,
    ).fetchall()
    result = {
        "day": {"left": 0, "right": 0, "total": 0},
        "night": {"left": 0, "right": 0, "total": 0},
    }
    for source, direction, count in rows:
        bucket = result.setdefault(source, {"left": 0, "right": 0, "total": 0})
        bucket[direction] = count
        bucket["total"] += count
    return result


def render_bucket_table(rows):
    body = "".join(
        f"""
        <tr>
            <td>{period}</td>
            <td>{right}</td>
            <td>{left}</td>
            <td><strong>{total}</strong></td>
        </tr>
        """
        for period, right, left, total in rows
    ) or """
        <tr><td colspan="4">No data yet</td></tr>
    """

    return f"""
        <table>
            <thead>
                <tr>
                    <th>Period</th>
                    <th>→</th>
                    <th>←</th>
                    <th>Total</th>
                </tr>
            </thead>
            <tbody>{body}</tbody>
        </table>
    """


@app.route("/api/counts")
def api_counts():
    conn = db()

    now = datetime.now(LOCAL_TIMEZONE)
    today = now.date().isoformat()
    current_hour = now.strftime("%Y-%m-%dT%H")
    current_month = now.strftime("%Y-%m")
    week_start = (now.date() - timedelta(days=now.weekday())).isoformat()

    rows = conn.execute(
        """
        SELECT direction, COUNT(*)
        FROM counts
        WHERE timestamp LIKE ?
        GROUP BY direction
        """,
        (f"{today}%",),
    ).fetchall()

    this_hour = conn.execute(
        "SELECT COUNT(*) FROM counts WHERE timestamp LIKE ?",
        (f"{current_hour}%",),
    ).fetchone()[0]

    all_time = conn.execute(
        "SELECT COUNT(*) FROM counts"
    ).fetchone()[0]

    this_week = conn.execute(
        "SELECT COUNT(*) FROM counts WHERE substr(timestamp, 1, 10) >= ?",
        (week_start,),
    ).fetchone()[0]

    this_month = conn.execute(
        "SELECT COUNT(*) FROM counts WHERE timestamp LIKE ?",
        (f"{current_month}%",),
    ).fetchone()[0]

    hourly_rows = conn.execute(
        """
        SELECT
            substr(timestamp, 12, 2) AS hour,
            SUM(CASE WHEN direction = 'right' THEN 1 ELSE 0 END),
            SUM(CASE WHEN direction = 'left' THEN 1 ELSE 0 END),
            COUNT(*)
        FROM counts
        WHERE timestamp LIKE ?
        GROUP BY hour
        ORDER BY hour DESC
        """,
        (f"{today}%",),
    ).fetchall()

    history = {
        "hours": serialize_buckets(aggregate_counts(conn, "hour", 24)),
        "days": serialize_buckets(aggregate_counts(conn, "day", 31)),
        "weeks": serialize_buckets(aggregate_counts(conn, "week", 12)),
        "months": serialize_buckets(aggregate_counts(conn, "month", 12)),
    }

    sources_today = source_summary(conn, today)
    sources_all_time = source_summary(conn)

    conn.close()

    result = {
        "left": 0,
        "right": 0,
        "total": 0,
        "today": 0,
        "this_hour": this_hour,
        "this_week": this_week,
        "this_month": this_month,
        "all_time": all_time,
        "hourly": [
            {
                "hour": f"{hour}:00",
                "right": right,
                "left": left,
                "total": total,
            }
            for hour, right, left, total in hourly_rows
        ],
        "history": history,
        "night_detector": night_detector.snapshot() if night_detector else {},
        "mode": "night" if night_detector and night_detector.snapshot().get("active") else "day",
        "sources_today": sources_today,
        "sources_all_time": sources_all_time,
    }

    for direction, count in rows:
        result[direction] = count
        result["total"] += count

    result["today"] = result["total"]

    return jsonify(result)


@app.route("/")
def dashboard():
    conn = db()

    now = datetime.now(LOCAL_TIMEZONE)
    today = now.date().isoformat()
    current_hour = now.strftime("%Y-%m-%dT%H")
    current_month = now.strftime("%Y-%m")
    week_start = (now.date() - timedelta(days=now.weekday())).isoformat()

    rows = conn.execute(
        """
        SELECT direction, COUNT(*)
        FROM counts
        WHERE timestamp LIKE ?
        GROUP BY direction
        """,
        (f"{today}%",),
    ).fetchall()

    this_hour = conn.execute(
        "SELECT COUNT(*) FROM counts WHERE timestamp LIKE ?",
        (f"{current_hour}%",),
    ).fetchone()[0]

    all_time = conn.execute(
        "SELECT COUNT(*) FROM counts"
    ).fetchone()[0]

    this_week = conn.execute(
        "SELECT COUNT(*) FROM counts WHERE substr(timestamp, 1, 10) >= ?",
        (week_start,),
    ).fetchone()[0]

    this_month = conn.execute(
        "SELECT COUNT(*) FROM counts WHERE timestamp LIKE ?",
        (f"{current_month}%",),
    ).fetchone()[0]

    hourly_rows = conn.execute(
        """
        SELECT
            substr(timestamp, 12, 2) AS hour,
            SUM(CASE WHEN direction = 'right' THEN 1 ELSE 0 END),
            SUM(CASE WHEN direction = 'left' THEN 1 ELSE 0 END),
            SUM(CASE WHEN source = 'day' THEN 1 ELSE 0 END),
            SUM(CASE WHEN source = 'night' THEN 1 ELSE 0 END),
            COUNT(*)
        FROM counts
        WHERE timestamp LIKE ?
        GROUP BY hour
        ORDER BY hour DESC
        """,
        (f"{today}%",),
    ).fetchall()

    hour_history = aggregate_counts(conn, "hour", 24)
    day_history = aggregate_counts(conn, "day", 31)
    week_history = aggregate_counts(conn, "week", 12)
    month_history = aggregate_counts(conn, "month", 12)

    recent = conn.execute(
        """
        SELECT timestamp, direction, source
        FROM counts
        ORDER BY timestamp DESC
        LIMIT 20
        """
    ).fetchall()

    sources_today = source_summary(conn, today)
    mode = "NIGHT — light tracking" if night_detector and night_detector.snapshot().get("active") else "DAY — Frigate car tracking"

    conn.close()

    counts = {
        "left": 0,
        "right": 0,
    }

    for direction, count in rows:
        counts[direction] = count

    today_total = (
        counts["left"]
        + counts["right"]
    )

    recent_html = "".join(
        f"""
        <li>
            {timestamp[11:19]}
            {"→" if direction == "right" else "←"}
            <small>{source}</small>
        </li>
        """
        for timestamp, direction, source in recent
    )

    hourly_html = "".join(
        f"""
        <tr>
            <td>{hour}:00–{hour}:59</td>
            <td>{right}</td>
            <td>{left}</td>
            <td>{day}</td>
            <td>{night}</td>
            <td><strong>{total}</strong></td>
        </tr>
        """
        for hour, right, left, day, night, total in hourly_rows
    ) or """
        <tr>
            <td colspan="6">No cars counted today</td>
        </tr>
    """

    hour_history_html = render_bucket_table(hour_history)
    day_history_html = render_bucket_table(day_history)
    week_history_html = render_bucket_table(week_history)
    month_history_html = render_bucket_table(month_history)

    return f"""
    <!doctype html>

    <html>

    <head>

        <meta
            http-equiv="refresh"
            content="5"
        >

        <title>
            Vehicle Counter
        </title>

        <style>

            body {{
                font-family: Arial, sans-serif;
                margin: 50px;
                max-width: 700px;
            }}

            h1 {{
                font-size: 48px;
                margin-bottom: 50px;
            }}

            .label {{
                font-size: 20px;
            }}

            .total {{
                font-size: 64px;
                font-weight: bold;
                margin-top: 10px;
            }}

            .summaries {{
                display: grid;
                grid-template-columns: repeat(3, minmax(0, 1fr));
                gap: 24px;
            }}

            .summary {{
                border: 1px solid #ddd;
                border-radius: 12px;
                padding: 24px;
            }}

            .directions {{
                display: flex;
                gap: 80px;
                margin-top: 40px;
                margin-bottom: 60px;
            }}

            .direction {{
                font-size: 36px;
            }}

            h2 {{
                font-size: 32px;
            }}

            table {{
                width: 100%;
                border-collapse: collapse;
                margin-bottom: 48px;
                font-size: 20px;
            }}

            th, td {{
                border-bottom: 1px solid #ddd;
                padding: 12px;
                text-align: right;
            }}

            th:first-child, td:first-child {{
                text-align: left;
            }}

            li {{
                font-size: 22px;
                margin: 8px;
            }}

            @media (max-width: 650px) {{
                body {{
                    margin: 24px;
                }}

                .summaries {{
                    grid-template-columns: 1fr;
                }}
            }}

        </style>

    </head>

    <body>

        <h1>
            Vehicle Counter
        </h1>

        <p><strong>Current mode:</strong> {mode}</p>

        <div class="summaries">
            <div class="summary">
                <div class="label">Cars total</div>
                <div class="total">{all_time}</div>
            </div>

            <div class="summary">
                <div class="label">This hour</div>
                <div class="total">{this_hour}</div>
            </div>

            <div class="summary">
                <div class="label">Today</div>
                <div class="total">{today_total}</div>
            </div>

            <div class="summary">
                <div class="label">This week</div>
                <div class="total">{this_week}</div>
            </div>

            <div class="summary">
                <div class="label">This month</div>
                <div class="total">{this_month}</div>
            </div>
        </div>

        <div class="summaries">
            <div class="summary">
                <div class="label">Day today (Frigate)</div>
                <div class="total">{sources_today['day']['total']}</div>
                <div>→ {sources_today['day']['right']} &nbsp; ← {sources_today['day']['left']}</div>
            </div>
            <div class="summary">
                <div class="label">Night today (lights)</div>
                <div class="total">{sources_today['night']['total']}</div>
                <div>→ {sources_today['night']['right']} &nbsp; ← {sources_today['night']['left']}</div>
            </div>
        </div>

        <div class="directions">

            <div class="direction">
                → {counts["right"]}
            </div>

            <div class="direction">
                ← {counts["left"]}
            </div>

        </div>

        <h2>Today by hour</h2>

        <table>
            <thead>
                <tr>
                    <th>Hour</th>
                    <th>→</th>
                    <th>←</th>
                    <th>Day</th>
                    <th>Night</th>
                    <th>Total</th>
                </tr>
            </thead>
            <tbody>
                {hourly_html}
            </tbody>
        </table>

        <h2>Recent hours</h2>
        {hour_history_html}

        <h2>Recent days</h2>
        {day_history_html}

        <h2>Recent weeks</h2>
        {week_history_html}

        <h2>Recent months</h2>
        {month_history_html}

        <h2>
            Recent
        </h2>

        <ul>
            {recent_html}
        </ul>

    </body>

    </html>
    """


if __name__ == "__main__":

    db()

    night_detector = NightLightDetector(
        lambda event_id, direction: save_count(event_id, direction, "night")
    )
    night_detector.start()

    mqtt_client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2
    )

    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    mqtt_client.connect(
        MQTT_HOST,
        MQTT_PORT,
        60,
    )

    mqtt_client.loop_start()

    app.run(
        host="0.0.0.0",
        port=8080,
    )
