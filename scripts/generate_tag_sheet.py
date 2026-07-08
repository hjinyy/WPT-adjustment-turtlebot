#!/usr/bin/env python3
"""Generate printable AprilTag sheets for WPT TurtleBot experiments.

Requires Pillow:
    python3 -m pip install Pillow
"""

from __future__ import annotations

import argparse
from io import BytesIO
from pathlib import Path
from urllib.request import urlopen

from PIL import Image, ImageDraw, ImageFont

from wpt_adjustment_turtlebot.tag_layout import coil_tag_id

BASE_URL = "https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/tag36_11_{tag_id:05d}.png"


def mm_to_px(mm: float, dpi: int) -> int:
    return round(mm / 25.4 * dpi)


def download_tag(tag_id: int) -> Image.Image:
    with urlopen(BASE_URL.format(tag_id=tag_id), timeout=20) as response:
        data = response.read()
    return Image.open(BytesIO(data)).convert("RGB")


def build_tag_ids(shelves: list[int]) -> list[tuple[str, int]]:
    items = []
    for shelf in shelves:
        for pos in ("north", "south", "west", "east"):
            items.append((f"shelf {shelf} {pos}", coil_tag_id(shelf, pos)))
    return items


def generate_sheet(items: list[tuple[str, int]], output_dir: Path, tag_size_mm: float, dpi: int) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    page_w, page_h = mm_to_px(210, dpi), mm_to_px(297, dpi)
    margin = mm_to_px(12, dpi)
    gap = mm_to_px(8, dpi)
    tag_px = mm_to_px(tag_size_mm, dpi)
    label_h = mm_to_px(8, dpi)
    cell_w = tag_px + gap
    cell_h = tag_px + label_h + gap
    cols = max(1, (page_w - 2 * margin) // cell_w)
    rows = max(1, (page_h - 2 * margin) // cell_h)
    per_page = cols * rows
    font = ImageFont.load_default()

    for page_idx in range((len(items) + per_page - 1) // per_page):
        page = Image.new("RGB", (page_w, page_h), "white")
        draw = ImageDraw.Draw(page)
        chunk = items[page_idx * per_page : (page_idx + 1) * per_page]
        for idx, (label, tag_id) in enumerate(chunk):
            row, col = divmod(idx, cols)
            x = margin + col * cell_w
            y = margin + row * cell_h
            tag = download_tag(tag_id).resize((tag_px, tag_px), Image.Resampling.NEAREST)
            page.paste(tag, (x, y))
            draw.text((x, y + tag_px + 2), f"ID {tag_id} / {label}", fill="black", font=font)
        png_path = output_dir / f"apriltag_sheet_{page_idx + 1}.png"
        pdf_path = output_dir / f"apriltag_sheet_{page_idx + 1}.pdf"
        page.save(png_path)
        page.save(pdf_path, resolution=dpi)
        print(f"saved {png_path}")
        print(f"saved {pdf_path}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--shelves", nargs="+", type=int, default=list(range(1, 5)))
    parser.add_argument("--output-dir", type=Path, default=Path("generated_tags"))
    parser.add_argument("--tag-size-mm", type=float, default=60.0)
    parser.add_argument("--dpi", type=int, default=300)
    args = parser.parse_args()
    generate_sheet(build_tag_ids(args.shelves), args.output_dir, args.tag_size_mm, args.dpi)


if __name__ == "__main__":
    main()
