#!/usr/bin/env python3
"""Generate printable AprilTag sheets for WPT TurtleBot experiments.

Requires Pillow:
    python3 -m pip install Pillow
"""

from __future__ import annotations

import argparse
from io import BytesIO
from pathlib import Path
import sys
from urllib.request import urlopen

from PIL import Image, ImageDraw, ImageFont

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from wpt_adjustment_turtlebot.tag_layout import FOUR_COIL_TAGS, coil_tag_id, head_tag_id

BASE_URL = "https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/tag36_11_{tag_id:05d}.png"
RESAMPLE_NEAREST = getattr(getattr(Image, "Resampling", Image), "NEAREST")


def mm_to_px(mm: float, dpi: int) -> int:
    return round(mm / 25.4 * dpi)


def download_tag(tag_id: int) -> Image.Image:
    with urlopen(BASE_URL.format(tag_id=tag_id), timeout=20) as response:
        data = response.read()
    return Image.open(BytesIO(data)).convert("RGB")


def build_tag_ids(shelves: list[int]) -> list[tuple[str, int]]:
    items = []
    for shelf in shelves:
        items.append((f"shelf {shelf} head", head_tag_id(shelf)))
        for pos in ("north", "south", "west", "east"):
            items.append((f"shelf {shelf} {pos}", coil_tag_id(shelf, pos)))
    return items


def build_four_coil_tag_ids() -> list[tuple[str, int]]:
    items = []
    for coil_name in ("coil_1", "coil_2", "coil_3", "coil_4"):
        for pos in ("north", "east", "south", "west"):
            items.append((f"{coil_name} {pos}", FOUR_COIL_TAGS[coil_name][pos]))
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
            tag = download_tag(tag_id).resize((tag_px, tag_px), RESAMPLE_NEAREST)
            page.paste(tag, (x, y))
            draw.text((x, y + tag_px + 2), f"ID {tag_id} / {label}", fill="black", font=font)
        png_path = output_dir / f"apriltag_sheet_{page_idx + 1}.png"
        pdf_path = output_dir / f"apriltag_sheet_{page_idx + 1}.pdf"
        page.save(png_path)
        page.save(pdf_path, resolution=dpi)
        print(f"saved {png_path}")
        print(f"saved {pdf_path}")


def save_individual_tags(items: list[tuple[str, int]], output_dir: Path) -> None:
    tag_dir = output_dir / "individual_tags"
    tag_dir.mkdir(parents=True, exist_ok=True)
    for label, tag_id in items:
        safe_label = label.replace(" ", "_")
        path = tag_dir / f"{safe_label}_id_{tag_id}.png"
        download_tag(tag_id).save(path)
        print(f"saved {path}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--shelves", nargs="+", type=int, default=list(range(1, 9)))
    parser.add_argument("--four-coil-layout", action="store_true", help="Generate the 16 markers used by the four-coil WPT layout.")
    parser.add_argument("--individual-tags", action="store_true", help="Also save one PNG file per marker.")
    parser.add_argument("--output-dir", type=Path, default=Path("generated_tags"))
    parser.add_argument("--tag-size-mm", type=float, default=60.0)
    parser.add_argument("--dpi", type=int, default=300)
    args = parser.parse_args()
    items = build_four_coil_tag_ids() if args.four_coil_layout else build_tag_ids(args.shelves)
    generate_sheet(items, args.output_dir, args.tag_size_mm, args.dpi)
    if args.individual_tags:
        save_individual_tags(items, args.output_dir)


if __name__ == "__main__":
    main()
