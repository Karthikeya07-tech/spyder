"""Generate Spyder current-progress Word document."""
from pathlib import Path

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_LINE_SPACING
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

NAVY = RGBColor(0x1F, 0x3A, 0x5F)
RED = RGBColor(0xC0, 0x39, 0x2B)
DARK = RGBColor(0x22, 0x22, 0x22)
MUTED = RGBColor(0x55, 0x55, 0x55)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
HEADER_BG = "1F3A5F"
ROW_ALT = "F4F6F8"


def set_run(run, *, size=11, bold=False, color=DARK, name="Calibri"):
    run.font.name = name
    run.font.size = Pt(size)
    run.bold = bold
    run.font.color.rgb = color
    r = run._element
    rPr = r.get_or_add_rPr()
    rFonts = rPr.get_or_add_rFonts()
    rFonts.set(qn("w:eastAsia"), name)


def add_para(doc, text, *, size=11, bold=False, color=DARK, space_after=8, space_before=0, align=None):
    p = doc.add_paragraph()
    p.paragraph_format.space_after = Pt(space_after)
    p.paragraph_format.space_before = Pt(space_before)
    p.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
    if align:
        p.alignment = align
    run = p.add_run(text)
    set_run(run, size=size, bold=bold, color=color)
    return p


def shade_cell(cell, fill):
    tc = cell._tc
    tcPr = tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:fill"), fill)
    shd.set(qn("w:val"), "clear")
    tcPr.append(shd)


def set_cell_border(cell):
    tc = cell._tc
    tcPr = tc.get_or_add_tcPr()
    tcBorders = OxmlElement("w:tcBorders")
    for edge in ("top", "left", "bottom", "right"):
        el = OxmlElement(f"w:{edge}")
        el.set(qn("w:val"), "single")
        el.set(qn("w:sz"), "4")
        el.set(qn("w:space"), "0")
        el.set(qn("w:color"), "C5CDD6")
        tcBorders.append(el)
    tcPr.append(tcBorders)


def cell_text(cell, text, *, bold=False, color=DARK, size=10, center=False):
    cell.text = ""
    p = cell.paragraphs[0]
    p.paragraph_format.space_before = Pt(3)
    p.paragraph_format.space_after = Pt(3)
    if center:
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = p.add_run(text)
    set_run(run, size=size, bold=bold, color=color)


def add_table(doc, headers, rows):
    table = doc.add_table(rows=1 + len(rows), cols=len(headers))
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = True
    for i, h in enumerate(headers):
        cell = table.rows[0].cells[i]
        shade_cell(cell, HEADER_BG)
        set_cell_border(cell)
        cell_text(cell, h, bold=True, color=WHITE, size=10, center=True)
    for r, row in enumerate(rows):
        fill = ROW_ALT if r % 2 else "FFFFFF"
        for c, val in enumerate(row):
            cell = table.rows[r + 1].cells[c]
            shade_cell(cell, fill)
            set_cell_border(cell)
            cell_text(cell, val, size=10)
    doc.add_paragraph().paragraph_format.space_after = Pt(6)
    return table


def heading(doc, text, level=1):
    p = doc.add_heading(text, level=level)
    for run in p.runs:
        set_run(run, size=16 if level == 1 else 13, bold=True, color=NAVY, name="Calibri")
    p.paragraph_format.space_before = Pt(16 if level == 1 else 12)
    p.paragraph_format.space_after = Pt(8)


def bullet(doc, text, bold_prefix=None):
    p = doc.add_paragraph(style="List Bullet")
    p.paragraph_format.space_after = Pt(4)
    p.paragraph_format.left_indent = Cm(1.0)
    if bold_prefix:
        r1 = p.add_run(bold_prefix)
        set_run(r1, size=11, bold=True, color=DARK)
        r2 = p.add_run(text)
        set_run(r2, size=11, color=DARK)
    else:
        r = p.add_run(text)
        set_run(r, size=11, color=DARK)


def build():
    doc = Document()
    section = doc.sections[0]
    section.page_width = Cm(21.0)
    section.page_height = Cm(29.7)
    section.left_margin = Cm(2.2)
    section.right_margin = Cm(2.2)
    section.top_margin = Cm(2.0)
    section.bottom_margin = Cm(2.0)

    footer = section.footer.paragraphs[0]
    footer.alignment = WD_ALIGN_PARAGRAPH.CENTER
    fr = footer.add_run("Spyder  ·  Firmware and application  ·  25 November 2026")
    set_run(fr, size=8, color=MUTED)

    add_para(doc, "SPYDER", size=12, bold=True, color=RED, space_after=0)
    add_para(
        doc,
        "Firmware and application — current progress",
        size=22,
        bold=True,
        color=NAVY,
        space_after=4,
    )
    add_para(
        doc,
        "One system: ESP32-P4 firmware (Twist-auto) and the Android app (Spyder).",
        size=12,
        color=MUTED,
        space_after=4,
    )
    add_para(
        doc,
        "25 November 2026. Updates the 24 November note with the pose tracker and the phone motion controls that are now in the code.",
        size=10,
        color=MUTED,
        space_after=14,
    )

    heading(doc, "1. Goal", 1)
    add_para(
        doc,
        "The goal is for Spyder, a 12-servo quadruped, to move autonomously from point A to point B in a real room.",
    )
    add_para(
        doc,
        "To do that the robot must know the space around it and know which way it is facing. "
        "LiDAR on USB supplies the room and the objects. The IMU supplies orientation. "
        "The phone application is the live view of that same world, and it is now also how walk and turn clips are started. "
        "Autonomous A to B is still the goal. It is not running yet.",
    )
    add_para(doc, "In one line: see the room, know the nose, walk from A to B.")

    heading(doc, "2. Where we are today", 1)
    add_para(
        doc,
        "Firmware and application already run as one loop. The robot opens USB for the motors and the LiDAR, starts the IMU, "
        "keeps a live occupancy grid in RAM, and opens Wi-Fi Twist-auto. The phone joins that network, starts the scan, "
        "calibrates the front, draws the map, and can queue stand, walk, and turn. "
        "A first pose tracker is in the firmware: yaw comes from the IMU, and x and y step one map cell only while a walk clip is actually running and the scan matches the grid.",
    )
    add_table(
        doc,
        ["Piece", "Today"],
        [
            [
                "Bring-up",
                "On boot the firmware opens USB for motors and LiDAR, starts the gait player, starts the IMU, starts the live map, opens Wi-Fi, plays neutral once, then scans the servos.",
            ],
            [
                "LiDAR",
                "RPLIDAR on USB-OTG. Motor stays off until Start scan. Ranges are packed into 360 bins and sent to the phone.",
            ],
            [
                "IMU",
                "On GPIO 7/8. Calibrate front sets the current heading as yaw 0 and clears gyro bias. That heading is the blue nose on the phone.",
            ],
            [
                "Live map",
                "One occupancy grid in RAM, 200 × 200 cells at 5 cm, origin (−5 m, −5 m). Rays are drawn from the current pose. The grid is not saved to flash.",
            ],
            [
                "Pose",
                "Started. x and y stay put unless the robot is walking, the match score is at least 12, and a neighbouring 5 cm cell is at least 4 hits better. Turns change yaw and do not add translation.",
            ],
            [
                "Application",
                "Spyder joins Twist-auto, streams the live map, shows x, y, and yaw, and sends Forward, Back, Left, Right, and Neutral.",
            ],
            [
                "Moving logic",
                "Five embedded clips on 12 AX servos. The phone, the web page, and the serial console can queue them. Stop ends the clip and plays neutral.",
            ],
            [
                "A → B",
                "Not running. There is no saved room, no chosen goal, no path, and no follower. The red A on the phone is the world origin, not a goal.",
            ],
        ],
    )

    heading(doc, "3. What this update adds", 1)
    add_para(
        doc,
        "The 24 November note stopped at sensing, a map drawn from the origin, and gaits started on the robot. "
        "The code since then starts the first two pieces of the path-planning plan.",
    )
    add_table(
        doc,
        ["Area", "Now in the code"],
        [
            [
                "Gait player",
                "One cycle is 54 frames, 1.62 s. Walk clips are 19 cycles. Turn clips are 6. A request can play N cycles. Stop cancels the clip and queues neutral.",
            ],
            [
                "Pose",
                "GET /api/pose and the pose fields in /api/live.json report x, y, yaw, localized, and match score. Scan rays use that pose.",
            ],
            [
                "Phone motion",
                "Forward, Back, Left, Right, and Neutral call POST /api/action/<name>. One clip plays. Two more can wait. A full queue is rejected.",
            ],
            [
                "Phone map",
                "The arrow sits on the live pose. A red A marks (0, 0). Free space is white, walls are near-black, the live sweep is orange. Pinch, pan, and Recenter are on the canvas.",
            ],
        ],
    )
    add_para(
        doc,
        "Still measured on the bench, not in the code: metres per walk cycle, degrees per turn cycle, and the body rectangle. "
        "The follower must not use those numbers until they are written down. A starting inflation guess of 0.25 m stays a guess.",
    )

    heading(doc, "4. The system (firmware + application)", 1)
    add_para(
        doc,
        "There is one product, two programs. Twist-auto on the ESP32-P4 is the robot. "
        "Spyder on Android is the operator view. They talk over the robot’s own Wi-Fi. The phone has no other server.",
    )
    add_table(
        doc,
        ["", "Firmware (Twist-auto)", "Application (Spyder)"],
        [
            [
                "Role",
                "Sense, map, hold the pose, play gaits.",
                "Connect, show the room, start scan and motion.",
            ],
            [
                "Where",
                "ESP-IDF project Twist-auto, target ESP32-P4. GitHub: Karthikeya07-tech/spyder.",
                "Kotlin, Jetpack Compose, package com.example.spyder, Android 10 and up. GitHub: Karthikeya07-tech/Spyder-Auto-.",
            ],
            [
                "Link",
                "SoftAP SSID Twist-auto, password twistmap1, http://192.168.4.1, port 80.",
                "Joins that AP and pins HTTP and WebSocket to 192.168.4.1.",
            ],
            [
                "LiDAR",
                "USB host to the LiDAR adapter. SCAN only after the app asks.",
                "Start scan / Stop. Live ranges as orange dots, drawn from the pose.",
            ],
            [
                "IMU",
                "Read yaw. Calibrate and zero-yaw endpoints. Calibrate also resets the live map.",
                "Calibrate front / Zero arrow. Blue nose follows yaw.",
            ],
            [
                "Map",
                "Integrates each scan into one occupancy grid. Serves /api/live.json, /api/map.bin, /api/pose, and /ws/live.",
                "WebSocket, with a poll fallback. Refetches map.bin when map_version changes.",
            ],
            [
                "Motion",
                "USB2AX on GPIO 26/27, Dynamixel Protocol 1, 12 servos, queued playback, stop to neutral.",
                "Sends the five action names. Does not send goals, plans, or step-by-step servo commands.",
            ],
        ],
    )

    heading(doc, "5. Firmware that is running", 1)
    add_para(
        doc,
        "Boot order is motors, gait player, LiDAR, IMU, live map, then Wi-Fi. "
        "The LiDAR motor does not spin until the application asks. Neutral plays once when the servo adapter is up.",
    )
    heading(doc, "Hardware on the robot", 2)
    add_table(
        doc,
        ["Item", "Connection"],
        [
            ["Compute", "ESP32-P4. On-board ESP32-C6 for Wi-Fi (ESP-Hosted)."],
            ["LiDAR", "RPLIDAR over USB-UART (CP210x or CH34x) on the P4 USB-OTG high-speed port."],
            ["Servos", "12× Robotis AX, IDs 1–12, Protocol 1 at 222222 baud, USB2AX or U2D2."],
            ["Servo USB", "GPIO 27 D+, GPIO 26 D−. Separate from the LiDAR port."],
            ["IMU", "I2C GPIO 7 SDA, GPIO 8 SCL. Mount yaw 180° (sensor X faces the back)."],
        ],
    )
    heading(doc, "Gaits", 2)
    add_para(
        doc,
        "Twelve AX servos get sync-written goal positions. Clips are embedded in flash and played at 30 ms per frame. "
        "A play request returns immediately as queued. One action runs at a time.",
    )
    add_table(
        doc,
        ["Action", "Frames", "What it is"],
        [
            ["neutral", "90 (2.7 s)", "Stand. Not a gait cycle. Played after stop."],
            ["walk_forward", "1026 (19 × 1.62 s)", "Forward walking clip."],
            ["walk_backward", "1026 (19 × 1.62 s)", "Reverse walking clip."],
            ["turn_left", "324 (6 × 1.62 s)", "In-place left turn."],
            ["turn_right", "324 (6 × 1.62 s)", "In-place right turn."],
        ],
    )
    add_para(
        doc,
        "Serial examples: action walk_forward 1 plays one cycle. action stop, POST /api/action/stop, and POST /api/nav/stop end the current clip and then play neutral. "
        "The phone buttons play the whole clip. The cycle count is there for the tape measure and for the later follower.",
    )
    heading(doc, "Pose, as implemented", 2)
    add_para(
        doc,
        "This is the start of Phase 1, in a smaller form than the full scan matcher in the plan. "
        "Heading is the calibrated IMU yaw. Translation does not come from the IMU, and it does not yet come from a predicted step length.",
    )
    bullet(doc, "x and y stay where they are when the robot is still. A turn changes yaw and does not slide the arrow.")
    bullet(doc, "While walk_forward or walk_backward is the clip actually running, the matcher looks at the eight neighbouring 5 cm cells.")
    bullet(doc, "It steps one cell only when the current match score is at least 12 and the neighbour beats staying put by at least 4 hits.")
    bullet(doc, "localized is true only when the match score is at least 12. Rays are integrated from (x, y, yaw).")
    add_para(
        doc,
        "Not in this build: a gait prediction between scans, a search window of about ±0.3 m and ±10°, a second static grid, or a window that recenters on the pose. "
        "The older 20 m map baked into flash is still a separate reference. Navigation uses the live 10 m grid.",
    )
    add_para(
        doc,
        "Bench checks still to run after this firmware is flashed: stand still after calibrate and confirm the arrow stays at the origin; "
        "walk one cycle and confirm the arrow leaves the origin only during that clip; turn one cycle and confirm x and y stay put.",
    )
    heading(doc, "HTTP the phone already uses", 2)
    add_table(
        doc,
        ["Call", "Role"],
        [
            ["GET /api", "Catalog of the routes that exist."],
            ["GET /api/status", "Health: LiDAR, IMU, streaming, yaw, map size."],
            ["GET /api/live.json and GET /ws/live", "Pose, 360 ranges, map version. The socket expects the client to send g."],
            ["GET /api/pose", "x, y, yaw_deg, localized, calibrated, match score."],
            ["GET /api/map.bin", "40,000 int8 log-odds cells. Above 0 occupied, below 0 free, 0 unknown."],
            ["POST /api/scan/start and /stop", "LiDAR motor on or off. /api/stream/start and /stop are the same calls."],
            ["POST /api/imu/calibrate and /zero", "Front heading. Calibrate also resets the live map."],
            ["POST /api/map/reset", "Clear the occupancy grid."],
            ["POST /api/action/<name>", "Queue neutral, walk_forward, walk_backward, turn_left, or turn_right. Optional ?cycles=N."],
            ["GET /api/action", "busy, current, queued."],
            ["POST /api/action/stop and POST /api/nav/stop", "Cancel and play neutral. This is the only navigation route that exists."],
        ],
    )
    add_para(
        doc,
        "A page at GET / draws the same live map in a browser on the robot Wi-Fi. The phone does not use that page. It calls the API itself. "
        "Frame on the robot: x is forward, y is left, yaw is counterclockwise, and yaw 0 is the heading set by calibrate.",
    )

    heading(doc, "6. Application that is running", 1)
    add_para(
        doc,
        "Spyder is one screen. It does not keep a second map. "
        "It joins the robot, draws the occupancy and the live sweep, and sends the motion commands the firmware already plays.",
    )
    heading(doc, "Connection", 2)
    add_para(
        doc,
        "The app looks for SSID Twist-auto, uses password twistmap1, and keeps HTTP and WebSocket on 192.168.4.1 on that network. "
        "Live data is WebSocket /ws/live. The app sends g about five times a second. "
        "If the socket does not open, it polls GET /api/live.json. When map_version changes it loads /api/map.bin. "
        "Action status is polled on GET /api/action. There is no Bluetooth and no USB path in the app.",
    )
    heading(doc, "Controls", 2)
    add_table(
        doc,
        ["On the phone", "On the robot", "What you get"],
        [
            ["Join Twist-auto", "SoftAP", "Link to the robot. The screen stays on while the link is live."],
            ["Start scan / Stop", "POST /api/scan/start and /stop", "LiDAR motor on or off. Stream start and stop are the fallback."],
            ["Calibrate front", "POST /api/imu/calibrate", "Hold still. Yaw 0 becomes the nose. The live map is cleared."],
            ["Zero arrow", "POST /api/imu/zero", "Heading only, without a new bias."],
            ["Reset map", "POST /api/map/reset", "Clear occupancy RAM."],
            ["Forward, Back, Left, Right, Neutral", "POST /api/action/<name>", "Queue a clip. The line under the buttons shows what is playing and what is waiting."],
        ],
    )
    add_para(
        doc,
        "One move plays and two can wait. When that queue is full the buttons stay disabled and the app says so. "
        "The buttons send the whole clip. They do not yet send cycles=1, and there is no Stop-motion button on the phone. "
        "Scan Stop stops the LiDAR, not the gait.",
    )
    heading(doc, "Map on screen", 2)
    add_para(
        doc,
        "The canvas is a plot of the robot frame: +X to the right, +Y up. "
        "The blue arrow is the live pose from the firmware, not a marker glued to the centre. "
        "Red A is (0, 0), the start of the grid, and it is not a goal the operator picked. "
        "Orange dots are the current sweep, placed from that same pose. Pinch zoom, pan, and Recenter are on the canvas. "
        "The telemetry line shows x, y, hit count, and map version.",
    )
    add_table(
        doc,
        ["On screen", "Colour", "Meaning"],
        [
            ["Unknown", "Grey", "Not yet observed."],
            ["Free space", "White", "Open floor."],
            ["Wall / object", "Near-black", "Occupied."],
            ["Live scan", "Orange", "Current LiDAR hits."],
            ["Nose", "Blue", "Heading after calibrate, drawn at the live pose."],
            ["A", "Red", "World origin (0, 0). Not a stored goal."],
        ],
    )

    heading(doc, "7. A to B: started, and not started", 1)
    add_para(
        doc,
        "The plan in “path planning and goal selection” is unchanged. This table is how far the code has gone along it.",
    )
    add_table(
        doc,
        ["Phase", "Status"],
        [
            [
                "0 — Measure, and be able to stop",
                "Started. Cycle length, cancel, and N-cycle play are in the firmware. Metres, degrees, and body size are not measured yet.",
            ],
            [
                "1 — A pose that moves",
                "Started, in the small form above. The full correlative match and the gait prediction are not in.",
            ],
            [
                "2 — Save the room",
                "Not started. One live grid. No static layer, no save, no load.",
            ],
            [
                "3 — Pick A and B",
                "Not started. The phone has no Set A, no tap for B, and no rejection of a point inside a wall.",
            ],
            [
                "4 — Plan only",
                "Not started. No A* and no /api/nav/plan.",
            ],
            [
                "5 — Follow the path",
                "Not started. Stop-to-neutral exists, because Phase 0 needed it. There is no follower state machine.",
            ],
            [
                "6 and 7 — Obstacles and recovery",
                "Not started. A dropped Wi-Fi link does not yet force neutral. A weak pose does not yet abort a mission, because no mission runs.",
            ],
        ],
    )
    add_para(
        doc,
        "Endpoints from the plan that are still absent: /api/map/save, /api/map/load, /api/map/static.bin, "
        "/api/nav/goal, /api/nav/plan, /api/nav/start, and GET /api/nav. POST /api/nav/stop is present and means the same thing as action stop.",
    )

    heading(doc, "8. A session today", 1)
    bullet(doc, "Robot boots. Servo USB, gait player, LiDAR adapter, IMU, live map, Wi-Fi. Neutral plays once the adapter is up.")
    bullet(doc, "Phone opens Spyder and joins Twist-auto.")
    bullet(doc, "Start scan. LiDAR ranges and the occupancy grid arrive on the phone.")
    bullet(doc, "Hold still. Calibrate front. The blue arrow is yaw 0, and the map starts clean.")
    bullet(doc, "Forward, Back, Left, or Right queues a clip. The arrow may step while a walk is playing and the scan matches. A turn should change heading without sliding x and y.")
    bullet(doc, "The room is visible, the nose is known, and the body can be driven. Nothing yet chooses B or walks there alone.")

    heading(doc, "9. Next", 1)
    bullet(doc, "On the robot, time one walk cycle and one turn cycle. Record metres, degrees, and the body rectangle.")
    bullet(doc, "Flash this pose build and run the three bench checks: still, one walk cycle, one turn cycle.")
    bullet(doc, "After those hold, split a saved room from the live obstacles, then add Set A, Set B, and Plan with the servos still idle.")
    add_para(
        doc,
        "Repos: firmware https://github.com/Karthikeya07-tech/spyder and application https://github.com/Karthikeya07-tech/Spyder-Auto-.",
        space_before=6,
    )

    out_dir = Path(__file__).resolve().parents[1] / "docs"
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / "Spyder_Current_Progress.docx"
    doc.save(out)

    spyder = Path(r"C:\Users\siren\AndroidStudioProjects\Spyder")
    if spyder.is_dir():
        doc.save(spyder / "Spyder_Current_Progress.docx")

    print(out)
    return out


if __name__ == "__main__":
    build()
