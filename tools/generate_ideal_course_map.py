#!/usr/bin/env python3
"""Generate an idealized RB3302 project course map and landmark metadata.

The course geometry is taken from the 4-by-6, approximately 0.6 m grid in
L08b_Project-2. The optional obstacle is intentionally excluded because its
location changes between runs and it should be detected by the live costmap.
"""

from __future__ import annotations

import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


CELL_SIZE_M = 0.60
COLUMNS = 6
ROWS = 4
RESOLUTION_M = 0.02
WALL_THICKNESS_M = 0.04

COURSE_WIDTH_M = COLUMNS * CELL_SIZE_M
COURSE_HEIGHT_M = ROWS * CELL_SIZE_M

# Coordinate convention: origin near the south-west outer wall, +x east,
# +y north. Each tuple is (x1, y1, x2, y2), in metres.
WALL_SEGMENTS_M = [
    # Outer boundary.
    (0.0, 0.0, COURSE_WIDTH_M, 0.0),
    (COURSE_WIDTH_M, 0.0, COURSE_WIDTH_M, COURSE_HEIGHT_M),
    (COURSE_WIDTH_M, COURSE_HEIGHT_M, 0.0, COURSE_HEIGHT_M),
    (0.0, COURSE_HEIGHT_M, 0.0, 0.0),
    # Internal wall pattern shown in the project specification.
    (0.6, 1.2, 0.6, 1.8),
    (0.6, 1.8, 1.2, 1.8),
    (0.6, 1.2, 1.2, 1.2),
    (1.2, 0.6, 1.2, 1.2),
    (0.6, 0.6, 1.2, 0.6),
    (1.8, 1.2, 1.8, 1.8),
    (1.8, 1.2, 2.4, 1.2),
    (2.4, 1.2, 2.4, 1.8),
    (2.4, 1.8, 3.0, 1.8),
    (3.0, 1.2, 3.0, 1.8),
    (1.8, 0.6, 3.0, 0.6),
]


def landmark_centres() -> dict[str, tuple[float, float]]:
    centres: dict[str, tuple[float, float]] = {}
    label = ord("A")
    for row_from_north in range(ROWS):
        y = COURSE_HEIGHT_M - (row_from_north + 0.5) * CELL_SIZE_M
        for column in range(COLUMNS):
            x = (column + 0.5) * CELL_SIZE_M
            centres[chr(label)] = (x, y)
            label += 1
    return centres


def draw_occupancy_map() -> tuple[Image.Image, tuple[float, float]]:
    # Include half a wall outside every course boundary so the nominal grid
    # remains exactly 3.6 m by 2.4 m between its reference lines.
    margin_m = WALL_THICKNESS_M / 2.0
    origin = (-margin_m, -margin_m)
    width_px = math.ceil((COURSE_WIDTH_M + 2.0 * margin_m) / RESOLUTION_M)
    height_px = math.ceil((COURSE_HEIGHT_M + 2.0 * margin_m) / RESOLUTION_M)
    wall_px = max(1, round(WALL_THICKNESS_M / RESOLUTION_M))

    image = Image.new("L", (width_px, height_px), 254)
    draw = ImageDraw.Draw(image)

    def pixel(x: float, y: float) -> tuple[int, int]:
        px = round((x - origin[0]) / RESOLUTION_M)
        py_from_south = round((y - origin[1]) / RESOLUTION_M)
        py = height_px - 1 - py_from_south
        return px, py

    for x1, y1, x2, y2 in WALL_SEGMENTS_M:
        draw.line([pixel(x1, y1), pixel(x2, y2)], fill=0, width=wall_px)

    return image, origin


def load_font(size: int, *, bold: bool = False) -> ImageFont.ImageFont:
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(name, size)
    except OSError:
        return ImageFont.load_default()


def draw_reference_preview() -> Image.Image:
    scale = 220.0
    pad_left, pad_top, pad_right, pad_bottom = 92, 72, 92, 96
    width = round(COURSE_WIDTH_M * scale) + pad_left + pad_right
    height = round(COURSE_HEIGHT_M * scale) + pad_top + pad_bottom
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)

    def pixel(x: float, y: float) -> tuple[int, int]:
        return (
            round(pad_left + x * scale),
            round(pad_top + (COURSE_HEIGHT_M - y) * scale),
        )

    grid_colour = (215, 220, 226)
    for column in range(COLUMNS + 1):
        x = column * CELL_SIZE_M
        draw.line([pixel(x, 0.0), pixel(x, COURSE_HEIGHT_M)], fill=grid_colour, width=2)
    for row in range(ROWS + 1):
        y = row * CELL_SIZE_M
        draw.line([pixel(0.0, y), pixel(COURSE_WIDTH_M, y)], fill=grid_colour, width=2)

    wall_colour = (32, 36, 42)
    wall_px = max(3, round(WALL_THICKNESS_M * scale))
    for x1, y1, x2, y2 in WALL_SEGMENTS_M:
        draw.line([pixel(x1, y1), pixel(x2, y2)], fill=wall_colour, width=wall_px)

    label_font = load_font(28, bold=True)
    for label, (x, y) in landmark_centres().items():
        cx, cy = pixel(x, y)
        radius = 24
        draw.ellipse(
            (cx - radius, cy - radius, cx + radius, cy + radius),
            fill=(232, 235, 239),
        )
        box = draw.textbbox((0, 0), label, font=label_font)
        draw.text(
            (cx - (box[2] - box[0]) / 2, cy - (box[3] - box[1]) / 2 - box[1]),
            label,
            font=label_font,
            fill=wall_colour,
        )

    direction_font = load_font(26, bold=True)
    directions = {
        "N": (pad_left + COURSE_WIDTH_M * scale / 2, 28),
        "S": (pad_left + COURSE_WIDTH_M * scale / 2, height - 52),
        "W": (38, pad_top + COURSE_HEIGHT_M * scale / 2),
        "E": (width - 52, pad_top + COURSE_HEIGHT_M * scale / 2),
    }
    for text, (cx, cy) in directions.items():
        box = draw.textbbox((0, 0), text, font=direction_font)
        draw.text(
            (cx - (box[2] - box[0]) / 2, cy - (box[3] - box[1]) / 2 - box[1]),
            text,
            font=direction_font,
            fill=wall_colour,
        )

    note_font = load_font(18)
    note = "Idealized geometry: 6 x 4 cells, 0.60 m per cell, 0.04 m walls"
    box = draw.textbbox((0, 0), note, font=note_font)
    draw.text(
        ((width - (box[2] - box[0])) / 2, height - 26),
        note,
        font=note_font,
        fill=(80, 86, 94),
    )
    return image


def landmarks_yaml() -> str:
    lines = [
        "# Idealized landmark centres from L08b_Project-2.",
        "# Frame convention: origin at the nominal south-west corner; +x east; +y north.",
        f"cell_size: {CELL_SIZE_M:.2f}",
        f"course_width: {COURSE_WIDTH_M:.2f}",
        f"course_height: {COURSE_HEIGHT_M:.2f}",
        "landmarks:",
    ]
    for label, (x, y) in landmark_centres().items():
        lines.extend(
            [
                f"  {label}:",
                f"    x: {x:.2f}",
                f"    y: {y:.2f}",
                "    yaw: 0.0",
            ]
        )
    return "\n".join(lines) + "\n"


def main() -> None:
    project_root = Path(__file__).resolve().parents[1]
    output_dir = project_root / "student_ws" / "maps"
    output_dir.mkdir(parents=True, exist_ok=True)

    occupancy, origin = draw_occupancy_map()
    occupancy.save(output_dir / "rb3302_course_ideal.pgm", format="PPM")

    map_yaml = (
        "image: rb3302_course_ideal.pgm\n"
        "mode: trinary\n"
        f"resolution: {RESOLUTION_M:.3f}\n"
        f"origin: [{origin[0]:.3f}, {origin[1]:.3f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    (output_dir / "rb3302_course_ideal.yaml").write_text(map_yaml, encoding="ascii")
    (output_dir / "rb3302_course_landmarks.yaml").write_text(
        landmarks_yaml(), encoding="ascii"
    )
    draw_reference_preview().save(output_dir / "rb3302_course_ideal_reference.png")


if __name__ == "__main__":
    main()
