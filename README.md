# Twist-auto

ESP-IDF firmware for Spyder, a 12-servo quadruped. The ESP32-P4 reads an RPLIDAR and an IMU, builds a live occupancy grid, plays Dynamixel gait clips, and serves that state to the phone over the robot’s own Wi-Fi.

The Android companion is [Spyder](https://github.com/Karthikeya07-tech/Spyder-Auto-). It joins this access point and draws the same map.

Autonomous travel from a point A to a point B is the goal. It is not in this firmware yet. What runs today is sensing, a first pose tracker, live mapping, and queued walk and turn clips.

## Hardware

| Item | Connection |
|---|---|
| Compute | ESP32-P4. Wi-Fi is the on-board ESP32-C6 (ESP-Hosted). |
| LiDAR | RPLIDAR over USB-UART (CP210x or CH34x) on the USB-OTG high-speed port. |
| Servos | 12× Robotis AX, IDs 1–12, Protocol 1 at 222222 baud, USB2AX or U2D2. |
| Servo USB | GPIO 27 D+, GPIO 26 D−. Keep this off the LiDAR port. |
| IMU | I2C GPIO 7 SDA, GPIO 8 SCL. The sensor’s X axis faces the back of the robot. |

Flash and the serial console use UART0. USB Serial/JTAG is left off so it does not fight the USB host pins.

## Build and flash

ESP-IDF 5.1 or newer, target `esp32p4`.

```text
idf.py set-target esp32p4
idf.py build
idf.py -p COMx flash monitor
```

The console prompt is `dxl>`. Useful commands: `help`, `wifi`, `live`, `calibrate`, `imu`, `lscan`, `map`, `scan`, `action`, `move`.

`tools/gen_embed.py` rebuilds the gait binaries from the Gazebo export. The bins already in `main/embed/` are what the firmware plays.

## On the robot Wi-Fi

| | |
|---|---|
| SSID | `Twist-auto` |
| Password | `twistmap1` |
| Base URL | `http://192.168.4.1` |

`GET /` is a small live-map page. The phone does not use that page. `GET /api` is the route catalog.

Frame: x is forward, y is left, yaw is counterclockwise. Yaw 0 is the heading set by calibrate.

## What the firmware does

On boot it opens both USB hosts, starts the gait player, the LiDAR adapter, the IMU, and a 200×200 occupancy grid (5 cm cells, origin −5 m, −5 m), then starts Wi-Fi. The LiDAR motor stays off until something asks for a scan. Neutral plays once the servo adapter is up.

Pose is a start on the tracker the path plan describes:

- Yaw is the calibrated IMU heading.
- x and y stay put unless a walk clip is the action actually running, the scan match score is at least 12, and a neighbouring 5 cm cell beats staying put by at least 4 hits. The pose then steps one cell.
- A turn changes yaw and does not add translation.
- Scan rays are drawn from that pose.

One gait cycle is 54 frames, 1.62 s at 30 ms per frame. `walk_forward` and `walk_backward` are 19 cycles. `turn_left` and `turn_right` are 6. `neutral` is a 90-frame stand.

`action walk_forward 1`, or `POST /api/action/walk_forward?cycles=1`, plays one cycle. `action stop`, `POST /api/action/stop`, and `POST /api/nav/stop` end the current clip and then play neutral.

Metres per walk cycle, degrees per turn cycle, and the body size are not in the firmware. Measure them on the robot before a follower uses them.

## HTTP

| Call | Role |
|---|---|
| `GET /api/status` | LiDAR, IMU, streaming, yaw, map size. |
| `GET /api/live.json`, `GET /ws/live` | Pose, 360 ranges in millimetres, map version. The socket expects the client to send `g`. |
| `GET /api/pose` | `x`, `y`, `yaw_deg`, `localized`, `calibrated`, match score. |
| `GET /api/map.bin` | 40,000 int8 log-odds cells. Above 0 is occupied, below 0 is free, 0 is unknown. |
| `POST /api/scan/start`, `POST /api/scan/stop` | LiDAR motor. `/api/stream/start` and `/api/stream/stop` are the same. |
| `POST /api/imu/calibrate`, `POST /api/imu/zero` | Front heading. Calibrate also clears the live map. |
| `POST /api/map/reset` | Clear occupancy. |
| `GET /api/actions`, `GET /api/action` | Clip catalog, and busy / current / queued. |
| `POST /api/action/<name>` | Queue `neutral`, `walk_forward`, `walk_backward`, `turn_left`, or `turn_right`. Optional `?cycles=N`. |
| `POST /api/action/stop`, `POST /api/nav/stop` | Cancel and play neutral. |

## Not built yet

Saving the room, a second grid for live obstacles, choosing A and B, A* planning, and a path follower. `POST /api/nav/stop` is the only navigation route, and it only stops the gait.

The phased plan is in [path planning and goal selection.md](path%20planning%20and%20goal%20selection.md). The progress note shared with the app is [docs/Spyder_Current_Progress.docx](docs/Spyder_Current_Progress.docx). Regenerate that note with `python tools/write_progress_doc.py` (needs `python-docx`).
