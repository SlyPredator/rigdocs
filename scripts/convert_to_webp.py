#!/usr/bin/env python3
"""
Convert images in the content folder to WebP format and delete originals.
Also updates image references in markdown (.md) and reStructuredText (.rst) files.

Usage:
    python scripts/convert_to_webp.py
    python scripts/convert_to_webp.py --quality 90
    python scripts/convert_to_webp.py --dry-run
"""

import argparse
import os
import sys
from pathlib import Path
from typing import List, Tuple

try:
    from PIL import Image
except ImportError:
    print("Error: Pillow is required. Install it using: pip install pillow")
    sys.exit(1)

SUPPORTED_EXTENSIONS = {".png", ".jpg", ".jpeg", ".bmp", ".tiff", ".tif"}


def convert_image(
    image_path: Path, quality: int = 85, lossless: bool = False
) -> Path:
    """Convert a single image to WebP format and return the new path."""
    webp_path = image_path.with_suffix(".webp")

    with Image.open(image_path) as img:
        # Preserve transparency for PNG / RGBA images
        save_kwargs = {
            "format": "WEBP",
            "quality": quality,
            "method": 6,
        }

        # If image has an alpha channel or is PNG and lossless requested
        if lossless and image_path.suffix.lower() == ".png":
            save_kwargs["lossless"] = True

        # Handle color modes
        if img.mode in ("RGBA", "LA") or (
            img.mode == "P" and "transparency" in img.info
        ):
            img_converted = img.convert("RGBA")
        elif img.mode != "RGB":
            img_converted = img.convert("RGB")
        else:
            img_converted = img

        img_converted.save(webp_path, **save_kwargs)

    return webp_path


def update_references(
    content_dir: Path, replacements: List[Tuple[str, str]], dry_run: bool = False
) -> int:
    """Update file references from old image filenames to new .webp filenames in .md and .rst files."""
    if not replacements:
        return 0

    files_updated = 0
    doc_extensions = {".md", ".rst"}

    for doc_file in content_dir.rglob("*"):
        if doc_file.suffix.lower() not in doc_extensions:
            continue

        try:
            content = doc_file.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue

        modified = False
        new_content = content
        for old_name, new_name in replacements:
            if old_name in new_content:
                new_content = new_content.replace(old_name, new_name)
                modified = True

        if modified:
            if not dry_run:
                doc_file.write_text(new_content, encoding="utf-8")
            print(f"  [Ref Updated] {doc_file.relative_to(content_dir.parent)}")
            files_updated += 1

    return files_updated


def main():
    parser = argparse.ArgumentParser(
        description="Convert images in content/ to WebP format, delete originals, and update markdown references."
    )
    parser.add_argument(
        "--dir",
        "-d",
        type=str,
        default=None,
        help="Target directory (default: content/ relative to repo root)",
    )
    parser.add_argument(
        "--quality",
        "-q",
        type=int,
        default=85,
        help="WebP compression quality from 1 to 100 (default: 85)",
    )
    parser.add_argument(
        "--lossless",
        action="store_true",
        help="Use lossless compression for PNG images",
    )
    parser.add_argument(
        "--no-delete",
        action="store_true",
        help="Keep original non-webp images instead of deleting them",
    )
    parser.add_argument(
        "--no-replace-refs",
        action="store_true",
        help="Do not update image references in .md and .rst documentation files",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Simulate the conversion and show what would be converted without modifying files",
    )

    args = parser.parse_args()

    # Determine repo root (script is in <repo_root>/scripts/)
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parent

    target_dir = Path(args.dir).resolve() if args.dir else repo_root / "content"

    if not target_dir.exists() or not target_dir.is_dir():
        print(f"Error: Directory '{target_dir}' does not exist.")
        sys.exit(1)

    print(f"Scanning for images in: {target_dir}")
    if args.dry_run:
        print("[DRY-RUN MODE] No files will be created or deleted.\n")

    # Collect images to convert
    images_to_convert: List[Path] = []
    for file_path in target_dir.rglob("*"):
        if file_path.is_file() and file_path.suffix.lower() in SUPPORTED_EXTENSIONS:
            images_to_convert.append(file_path)

    if not images_to_convert:
        print("No convertible images found (only .webp / .svg or no images present).")
        return

    print(f"Found {len(images_to_convert)} image(s) to convert:\n")

    total_orig_size = 0
    total_new_size = 0
    replacements: List[Tuple[str, str]] = []

    for img_path in images_to_convert:
        orig_size = img_path.stat().st_size
        total_orig_size += orig_size
        rel_path = img_path.relative_to(repo_root)

        old_filename = img_path.name
        new_filename = img_path.with_suffix(".webp").name

        if args.dry_run:
            print(f"  [Would Convert] {rel_path} ({orig_size / 1024:.1f} KB) -> {new_filename}")
            replacements.append((old_filename, new_filename))
            continue

        try:
            webp_path = convert_image(
                img_path, quality=args.quality, lossless=args.lossless
            )

            # Ensure webp file was generated and is valid
            if webp_path.exists() and webp_path.stat().st_size > 0:
                new_size = webp_path.stat().st_size
                total_new_size += new_size
                savings = (1 - (new_size / orig_size)) * 100 if orig_size > 0 else 0

                print(
                    f"  [Converted] {rel_path} ({orig_size / 1024:.1f} KB -> "
                    f"{new_size / 1024:.1f} KB, {savings:+.1f}%)"
                )

                # Delete original if requested
                if not args.no_delete:
                    # Double check we're not deleting the webp itself if name coincided
                    if img_path.resolve() != webp_path.resolve():
                        os.remove(img_path)
                        print(f"  [Deleted]   {rel_path}")

                replacements.append((old_filename, new_filename))
            else:
                print(f"  [Failed]    {rel_path}: Converted file was empty or missing.")
        except Exception as e:
            print(f"  [Error]     {rel_path}: {e}")

    # Summary of image sizes
    if not args.dry_run and total_orig_size > 0 and total_new_size > 0:
        overall_savings = (1 - (total_new_size / total_orig_size)) * 100
        print(
            f"\nTotal image size: {total_orig_size / 1024:.1f} KB -> "
            f"{total_new_size / 1024:.1f} KB ({overall_savings:+.1f}% space saved)"
        )

    # Update references in docs
    if not args.no_replace_refs and replacements:
        print("\nUpdating documentation references...")
        num_docs = update_references(target_dir, replacements, dry_run=args.dry_run)
        print(f"Updated image references in {num_docs} document(s).")

    print("\nDone!")


if __name__ == "__main__":
    main()
