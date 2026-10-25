#!/usr/bin/env python3
"""Rebuild local arena derivatives from the unchanged official artwork, offline."""

import io
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from PIL import Image, ImageChops, ImageDraw, ImageFilter, ImageFont, ImageOps

ROOT = Path(__file__).resolve().parent
SIZE = (768, 256)


def load(name):
    path = ROOT / name
    if path.suffix == ".svg":
        svg = ET.parse(path).getroot()
        _, _, width, height = map(float, svg.attrib["viewBox"].split())
        scale = min(1600 / width, 480 / height)
        svg.set("width", str(round(width * scale)))
        svg.set("height", str(round(height * scale)))
        rendered = subprocess.run(
            ["magick", "-background", "none", "svg:-", "png:-"],
            input=ET.tostring(svg), check=True, capture_output=True,
        ).stdout
        image = Image.open(io.BytesIO(rendered)).convert("RGBA")
    else:
        image = Image.open(path).convert("RGBA")
    return image.crop(image.getbbox())


def white_mask(image, ink=0):
    # Invert coverage over white, keeping white counters inside colored lockups.
    opaque = Image.new("RGB", image.size, "white")
    opaque.paste(image, mask=image.getchannel("A"))
    red, green, blue = opaque.split()
    mask = ImageOps.invert(ImageChops.darker(ImageChops.darker(red, green), blue))
    mask = mask.point(lambda value: min(255, round(value * 255 / (255 - ink))))
    result = Image.new("RGBA", image.size, "white")
    result.putalpha(mask)
    return result


def save(name, image, background):
    panel = Image.new("RGB", SIZE, background)
    image = ImageOps.contain(image, (640, 184), Image.Resampling.LANCZOS)
    panel.paste(image, ((SIZE[0] - image.width) // 2, (SIZE[1] - image.height) // 2), image)
    write(name, panel, background)


def write(name, panel, background):
    assert panel.size == SIZE and panel.mode == "RGB"
    difference = ImageChops.difference(panel, Image.new("RGB", SIZE, background))
    bounds = difference.getbbox()
    assert bounds and 0 < bounds[0] < bounds[2] < SIZE[0]
    assert 0 < bounds[1] < bounds[3] < SIZE[1], name
    path = ROOT / f"{name}_panel.png"
    panel.save(path, optimize=True)
    assert path.stat().st_size < 100_000, path


def hulks():
    background = "#2FAC66"
    original = load("bighulk.png")
    mascot = Image.new("RGBA", original.size, "white")
    mascot.alpha_composite(original)
    # Remove only the connected exterior white backing; retain eyes, teeth and ball.
    ImageDraw.floodfill(mascot, (0, 0), (255, 255, 255, 0), thresh=32)
    mascot = mascot.crop(mascot.getbbox())
    mascot = ImageOps.contain(mascot, (210, 210), Image.Resampling.LANCZOS)
    alpha = Image.new("L", (220, 220))
    position = ((220 - mascot.width) // 2, (220 - mascot.height) // 2)
    alpha.paste(mascot.getchannel("A"), position)
    outline = Image.new("RGBA", alpha.size, "white")
    outline.putalpha(alpha.filter(ImageFilter.MaxFilter(9)))
    outline.alpha_composite(mascot, position)
    panel = Image.new("RGB", SIZE, background)
    panel.paste(outline, (48, 18), outline)
    font_path = subprocess.check_output(
        ["fc-match", "-f", "%{file}", "Noto Sans"], text=True,
    ).strip()
    title = ImageFont.truetype(font_path, 92)
    title.set_variation_by_name("Bold")
    subtitle = ImageFont.truetype(font_path, 23)
    draw = ImageDraw.Draw(panel)
    draw.text((300, 64), "HULKs", font=title, fill="white")
    draw.text((306, 170), "HAMBURG · ROBOCUP", font=subtitle, fill="white")
    write("hulks", panel, background)


def main():
    for name, original, background in (
        ("draeger", "Draeger_L_Standard_RGB_Dark-Blue.svg", "#002766"),
        ("gea", "GEA_Logo_w_Claim_sRGB_VibrantBlue.svg", "#0303B8"),
        ("tuhh", "tuhh_logo.svg", "#00C1D4"),
    ):
        image = load(original)
        white = Image.new("RGBA", image.size, "white")
        white.putalpha(image.getchannel("A"))
        save(name, white, background)
    for name, original, background, ink in (
        ("cc", "cc_logo_blue.svg", "#0C1A27", 12),
        ("haspa", "haspa.png", "#D9261C", 28),
        ("tk", "tk.png", "#009FE3", 69),
    ):
        save(name, white_mask(load(original), ink), background)
    for name, original in (("kdab", "kdab_logo.svg"), ("gegenstrom", "gegenstrom.png")):
        save(name, load(original), "white")
    hulks()


if __name__ == "__main__":
    main()
