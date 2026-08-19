"""Convert supplied 3:2 UI reference PNGs to LVGL RGB565 C assets."""

from pathlib import Path

from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "src" / "ui" / "assets" / "templates"
SOURCES = {
    "ui_template_dash": Path(r"C:\Users\19707\Downloads\ChatGPT Image Jul 21, 2026, 01_16_17 PM.png"),
    "ui_template_meth": Path(r"C:\Users\19707\Downloads\ChatGPT Image Jul 21, 2026, 01_17_10 PM (1).png"),
    "ui_template_tail": Path(r"C:\Users\19707\Downloads\ChatGPT Image Jul 21, 2026, 01_17_11 PM (6).png"),
    "ui_template_temps": Path(r"C:\Users\19707\Downloads\ChatGPT Image Jul 21, 2026, 01_17_11 PM (2).png"),
    "ui_template_diag": Path(r"C:\Users\19707\Downloads\ChatGPT Image Jul 21, 2026, 01_17_11 PM (3).png"),
    "ui_template_knock": Path(r"C:\Users\19707\Downloads\ChatGPT Image Jul 21, 2026, 01_17_12 PM (4).png"),
}


def rgb565_bytes(image: Image.Image) -> bytes:
    data = bytearray()
    for red, green, blue in image.convert("RGB").getdata():
        value = ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3)
        data.extend((value & 0xFF, value >> 8))
    return bytes(data)


def write_asset(symbol: str, source: Path) -> None:
    image = Image.open(source).convert("RGB").resize((480, 320), Image.Resampling.LANCZOS)
    pixels = rgb565_bytes(image)
    preview = OUT / f"{symbol}.png"
    image.save(preview, optimize=True)

    header = OUT / f"{symbol}.h"
    header.write_text(
        "#pragma once\n\n#include <lvgl.h>\n\n"
        f"LV_IMG_DECLARE({symbol});\n",
        encoding="utf-8",
    )

    (OUT / f"{symbol}.bin").write_bytes(pixels)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    for symbol, source in SOURCES.items():
        if not source.is_file():
            raise FileNotFoundError(source)
        write_asset(symbol, source)
        print(f"generated {symbol} from {source.name}")


if __name__ == "__main__":
    main()
