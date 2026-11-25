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
    fr = footer.add_run("Spyder  ·  Firmware and application  ·  24 November 2026")
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
        "24 November 2026",
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
        "The phone application is the live view of that same world. "
        "The body already has walking and turning gaits so it can move while that map is built.",
    )
    add_para(doc, "In one line: see the room, know the nose, walk from A to B.")

    heading(doc, "2. Where we are today", 1)
    add_para(
        doc,
        "Firmware and application already run as one loop. The robot starts its motors path, IMU, and LiDAR USB. "
        "The phone joins the robot Wi-Fi, starts the scan, calibrates the front, and draws the live map. "
        "Autonomous A to B is the goal of that loop; it is not running yet. "
        "What is running is sensing, orientation, live mapping to the phone, and playable walk/turn gaits.",
    )
    add_table(
        doc,
        ["Piece", "Today"],
        [
            [
                "Bring-up",
                "On boot the firmware opens USB for motors and LiDAR, starts the IMU, starts the live map in RAM, and opens Wi-Fi Twist-auto.",
            ],
            [
                "LiDAR",
                "RPLIDAR A1 on USB-OTG. Motor stays off until the app sends Start scan. Then ranges are packed into 360 bins and sent to the phone.",
            ],
            [
                "IMU",
                "On GPIO 7/8. Calibrate front sets the current heading as yaw 0. That heading is the teal nose on the map.",
            ],
            [
                "Live map",
                "Occupancy grid in RAM (200 × 200, 5 cm). Walls and free space grow from LiDAR rays at the current IMU yaw. Not saved to flash.",
            ],
            [
                "Application",
                "Spyder joins Twist-auto, streams live.json / WebSocket, and paints the room, objects, scan hits, and nose in real time.",
            ],
            [
                "Moving logic",
                "Embedded gaits on 12 AX servos: stand, walk forward, walk backward, turn left, turn right. Played from the robot today.",
            ],
            [
                "A → B",
                "This is the project goal. The robot does not yet pick a goal on the map or walk there by itself.",
            ],
        ],
    )

    heading(doc, "3. The system (firmware + application)", 1)
    add_para(
        doc,
        "There is one product, two programs. Twist-auto on the ESP32-P4 is the robot. "
        "Spyder on Android is the operator view. They talk over the robot’s own Wi-Fi; the phone has no other server.",
    )
    add_table(
        doc,
        ["", "Firmware (Twist-auto)", "Application (Spyder)"],
        [
            [
                "Role",
                "Sense, map, move the body.",
                "Connect, start scan, show the live room.",
            ],
            [
                "Where",
                r"D:\Pony\Twist-auto  —  ESP32-P4, ESP-IDF.",
                r"C:\Users\siren\AndroidStudioProjects\Spyder  —  Android.",
            ],
            [
                "Link",
                "SoftAP SSID Twist-auto, password twistmap1, http://192.168.4.1",
                "Joins that AP and pins HTTP/WebSocket to 192.168.4.1.",
            ],
            [
                "LiDAR",
                "USB host to the A1 CP2102. SCAN only after the app asks.",
                "Start scan / Stop. Live ranges as orange dots.",
            ],
            [
                "IMU",
                "Read yaw. Calibrate and zero-yaw endpoints.",
                "Calibrate front / Zero arrow. Teal nose follows yaw.",
            ],
            [
                "Map",
                "Integrates each scan into occupancy. Serves /api/live.json, /api/map.bin, /ws/live.",
                "Polls or sockets ~5 Hz. Fetches map.bin when the version changes. Draws walls and free space.",
            ],
            [
                "Motion",
                "USB2AX on GPIO 26/27, Dynamixel Protocol 1, 12 servos, gait playback.",
                "Does not send walk commands today. Mapping and calibration only.",
            ],
        ],
    )

    heading(doc, "4. Firmware today", 1)
    add_para(
        doc,
        "Boot order is: USB and motors, IMU orientation, then the live map link. "
        "The LiDAR motor does not spin until the application asks.",
    )
    heading(doc, "Hardware on the robot", 2)
    add_table(
        doc,
        ["Item", "Connection"],
        [
            ["Compute", "ESP32-P4. On-board ESP32-C6 for Wi-Fi (hosted SDIO)."],
            ["LiDAR", "RPLIDAR A1, CP2102 USB-UART, P4 USB-OTG HS."],
            ["Servos", "12× Robotis AX, Protocol 1 at 222222 baud, USB2AX (VID 16D0:06A7)."],
            ["Servo USB", "GPIO 27 D+, GPIO 26 D−, GND, 5 V. Separate from the LiDAR port."],
            ["IMU", "I2C GPIO 7 SDA, GPIO 8 SCL. Mount yaw 180° (sensor X faces the back)."],
        ],
    )
    heading(doc, "What firmware does after power-on", 2)
    bullet(doc, "USB host: LiDAR on OTG HS, USB2AX on GPIO 26/27.")
    bullet(doc, "Open the LiDAR adapter if it is plugged in. Do not SCAN yet.")
    bullet(doc, "Start IMU. Yaw 0 is front after calibrate.")
    bullet(doc, "Start a 200 × 200 occupancy grid in RAM (10 m × 10 m, 5 cm cells).")
    bullet(doc, "Start Wi-Fi Twist-auto and HTTP on port 80.")
    bullet(doc, "On POST /api/scan/start: motor on, SCAN, publish 360 ranges and occupancy to the app.")
    add_para(
        doc,
        "LiDAR: one revolution becomes 360 bins, 1° each. Bin 0 is robot front (180° mount offset). "
        "The USB adapter enumerating is not the same as the A1 answering. "
        "A live feed exists once the A1 returns GET_INFO and SCAN; the app then receives new ranges.",
    )
    heading(doc, "Moving logic on the robot", 2)
    add_para(
        doc,
        "Walking is in the firmware. Twelve AX servos (IDs 1–12) get sync-written goal positions. "
        "Gaits are embedded in flash and played at 30 ms per frame.",
    )
    add_table(
        doc,
        ["Action", "What it is"],
        [
            ["neutral", "Stand / home pose."],
            ["walk_forward", "Forward walking cycle."],
            ["walk_backward", "Reverse walking cycle."],
            ["turn_left", "In-place left turn."],
            ["turn_right", "In-place right turn."],
        ],
    )
    add_para(
        doc,
        "These are the motions the robot uses to move through the room while LiDAR and IMU keep the map updated. "
        "They are started on the robot (serial: action walk_forward, and the other names) when USB2AX is connected.",
    )

    heading(doc, "5. Application today", 1)
    add_para(
        doc,
        "Spyder is the same system on the phone. It does not host a second map. "
        "It connects to the robot and draws the live occupancy and the live LiDAR in the colours chosen in the app.",
    )
    heading(doc, "Connection and controls", 2)
    add_para(
        doc,
        "The app looks for SSID Twist-auto, uses password twistmap1, and keeps HTTP and WebSocket on 192.168.4.1 "
        "on that network. Live data is WebSocket /ws/live (the app sends “g” about five times a second). "
        "If the socket is not available it polls GET /api/live.json at the same rate. When map_version changes it loads /api/map.bin.",
    )
    add_table(
        doc,
        ["On the phone", "On the robot", "What you get"],
        [
            ["Join Twist-auto", "SoftAP", "Link to the robot."],
            ["Start scan", "POST /api/scan/start", "LiDAR motor on; live frames."],
            ["Stop", "POST /api/scan/stop", "LiDAR motor off."],
            ["Calibrate front", "POST /api/imu/calibrate", "Still-bias; yaw 0 = nose."],
            ["Zero arrow", "POST /api/imu/zero", "Heading only."],
            ["Reset map", "POST /api/map/reset", "Clear occupancy RAM."],
        ],
    )
    heading(doc, "Live map on screen", 2)
    add_para(
        doc,
        "The canvas is the present room. Occupied cells are walls and objects. "
        "Free cells are floor the LiDAR has seen. The current sweep is drawn as dots. "
        "The robot is at the origin on the map today; the arrow turns with IMU yaw. "
        "Frame: +X forward (up on screen), +Y left.",
    )
    add_table(
        doc,
        ["On screen", "Colour", "Meaning"],
        [
            ["Unknown", "Dark charcoal (#1A1E24)", "Not yet observed."],
            ["Free space", "Cool grey-blue", "Open floor."],
            ["Wall / object", "Near-white (#F4F7FA)", "Occupied — room and obstacles."],
            ["Live scan", "Orange (#FF7A18)", "Current LiDAR hits."],
            ["Nose", "Teal (#2AD2C9)", "Forward after calibrate."],
            ["Grid", "Muted grey, 1 m", "Scale."],
        ],
    )
    add_para(
        doc,
        "The in-app legend is the same: Nose (teal), Scan (orange), Wall (white). "
        "As the robot turns, white structure fills in and orange dots follow the live sweep.",
    )

    heading(doc, "6. How firmware and the app meet", 1)
    add_para(doc, "A session today looks like this:")
    bullet(doc, "Robot boots. Motors path, IMU, LiDAR adapter, Wi-Fi.")
    bullet(doc, "Phone opens Spyder and joins Twist-auto.")
    bullet(doc, "Start scan. LiDAR data is pushed to the app.")
    bullet(doc, "Calibrate front. IMU heading becomes the teal arrow.")
    bullet(doc, "The map shows the room and real-time objects in the app colours.")
    bullet(doc, "On the robot, walk and turn gaits can run while that map stays live.")
    add_para(
        doc,
        "That is the current product: one firmware and one application, sensing and showing the room, with moving logic on the bot. "
        "The goal those pieces serve is still autonomous travel from point A to point B.",
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
