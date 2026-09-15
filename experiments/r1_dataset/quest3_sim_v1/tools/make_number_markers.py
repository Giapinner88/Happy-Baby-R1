#!/usr/bin/env python3
"""Sinh các tấm đánh số để đặt lên bàn làm mục tiêu chỉ tới.

Kho asset của Isaac không có chữ số nào — `Props/UIElements/` chỉ có `arrow_x`
và `frame_prim`. Vật liệu MDL của Isaac Lab cũng không nhận đường dẫn texture
tuỳ ý (`MdlFileCfg` chỉ có mdl_path, project_uvw, albedo_brightness,
texture_scale). Nên tấm số được dựng thẳng bằng USD: một mặt phẳng vuông nằm
ngang, gắn `UsdPreviewSurface` đọc một PNG chữ số sinh bằng PIL.

    conda run --no-capture-output -n unitree_sim_env python \
        experiments/r1_dataset/quest3_sim_v1/tools/make_number_markers.py --count 4

Kết quả nằm ở `assets/markers/`: mỗi số một PNG và một USD trỏ tới PNG đó bằng
đường dẫn tương đối, nên cả thư mục di chuyển được.

Chữ số hướng lên trên và đọc xuôi khi nhìn từ phía robot: robot nhìn dọc +X, nên
"đỉnh" của chữ số quay về +X.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
DEFAULT_OUT = ROOT / "assets" / "markers"

# Màu nền mỗi tấm, để phân biệt được cả khi camera nhìn quá xiên không đọc ra số.
PALETTE = [
    (235, 235, 235), (250, 214, 137), (168, 216, 185),
    (170, 200, 245), (240, 180, 180), (205, 185, 235),
]


def render_digit_png(number: int, path: Path, pixels: int = 512) -> None:
    from PIL import Image, ImageDraw, ImageFont

    background = PALETTE[(number - 1) % len(PALETTE)]
    image = Image.new("RGB", (pixels, pixels), background)
    draw = ImageDraw.Draw(image)
    text = str(number)
    font = None
    for candidate in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    ):
        if Path(candidate).is_file():
            font = ImageFont.truetype(candidate, int(pixels * 0.72))
            break
    if font is None:
        # Không có font nào thì vẫn phải ra được cái gì đó đọc được, dù nhỏ.
        font = ImageFont.load_default()
    left, top, right, bottom = draw.textbbox((0, 0), text, font=font)
    draw.text(
        ((pixels - (right - left)) / 2 - left, (pixels - (bottom - top)) / 2 - top),
        text,
        fill=(20, 20, 20),
        font=font,
    )
    draw.rectangle([0, 0, pixels - 1, pixels - 1], outline=(60, 60, 60), width=max(2, pixels // 64))
    image.save(path)


def author_marker_usd(usd_path: Path, texture_name: str, size_m: float) -> None:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

    stage = Usd.Stage.CreateNew(str(usd_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    root = UsdGeom.Xform.Define(stage, "/Marker")
    stage.SetDefaultPrim(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(stage, "/Marker/Plate")

    half = size_m / 2.0
    mesh.CreatePointsAttr(
        [Gf.Vec3f(-half, -half, 0.0), Gf.Vec3f(half, -half, 0.0),
         Gf.Vec3f(half, half, 0.0), Gf.Vec3f(-half, half, 0.0)]
    )
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    mesh.CreateExtentAttr([Gf.Vec3f(-half, -half, 0.0), Gf.Vec3f(half, half, 0.0)])

    # +X là hướng robot nhìn tới, nên đỉnh chữ số quay về +X: v tăng theo x.
    uvs = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )
    uvs.Set([Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1), Gf.Vec2f(0, 0)])

    material = UsdShade.Material.Define(stage, "/Marker/Material")
    shader = UsdShade.Shader.Define(stage, "/Marker/Material/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    reader = UsdShade.Shader.Define(stage, "/Marker/Material/UvReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    texture = UsdShade.Shader.Define(stage, "/Marker/Material/Texture")
    texture.CreateIdAttr("UsdUVTexture")
    # Đường dẫn tương đối: cả thư mục assets/markers/ di chuyển được.
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./{texture_name}")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result"
    )
    texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture.ConnectableAPI(), "rgb"
    )
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    stage.GetRootLayer().Save()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--count", type=int, default=4, help="Sinh số 1..count.")
    parser.add_argument("--size-m", type=float, default=0.09, help="Cạnh tấm, mét.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--pixels", type=int, default=512)
    args = parser.parse_args()
    if args.count < 1 or args.size_m <= 0.0:
        raise SystemExit("--count phải >= 1 và --size-m phải dương.")

    # `pxr` chỉ import được sau khi Isaac khởi động, nên phải mở app dù chỉ để
    # ghi vài file USD.
    sys.path.insert(0, str(ROOT))
    from isaaclab.app import AppLauncher

    app = AppLauncher(headless=True).app

    out = args.output_dir.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    for number in range(1, args.count + 1):
        png = out / f"marker_{number:02d}.png"
        usd = out / f"marker_{number:02d}.usd"
        if usd.exists():
            usd.unlink()
        render_digit_png(number, png, args.pixels)
        author_marker_usd(usd, png.name, args.size_m)
        print(f"MARKER {number}: {usd.relative_to(ROOT)} ({args.size_m} m)", flush=True)

    import os
    import threading

    sys.stdout.flush()
    closer = threading.Thread(target=app.close, daemon=True)
    closer.start()
    closer.join(timeout=15.0)
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
