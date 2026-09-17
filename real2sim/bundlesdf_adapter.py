"""Adapter for the official Scalable Real2Sim BundleSDF fork."""
from __future__ import annotations
import argparse
import json
import os
import shutil
import subprocess
from pathlib import Path


DEFAULT_OFFICIAL_ROOT = Path("/data1/home/rangeryx/scalable-real2sim")
PINNED_SCALABLE_REAL2SIM = "a8e4d97cbb0c3ea887a69fa313bcd3a252c5a8a3"
PINNED_BUNDLESDF = "4029bb7504b5aa9af2e9bc7161704b9e82df3d32"


def verify_checkout(root: Path) -> dict:
    bundle = root / "scalable_real2sim/BundleSDF"
    if not (bundle / "run_custom.py").exists():
        raise FileNotFoundError(f"official BundleSDF checkout missing: {bundle}")
    commits = {}
    for name, path in (("scalable_real2sim", root), ("BundleSDF", bundle)):
        commits[name] = subprocess.check_output(
            ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
        ).strip()
    if commits["scalable_real2sim"] != PINNED_SCALABLE_REAL2SIM:
        raise RuntimeError(f"unexpected Scalable Real2Sim commit: {commits}")
    if commits["BundleSDF"] != PINNED_BUNDLESDF:
        raise RuntimeError(f"unexpected BundleSDF commit: {commits}")
    return commits


def prepare_dataset(scan_dir: Path) -> None:
    """Validate official YcbineoatReader layout and preserve gripper occlusion masks."""
    required = ["rgb", "depth", "masks", "gripper_masks", "cam_K.txt"]
    missing = [name for name in required if not (scan_dir / name).exists()]
    if missing:
        raise FileNotFoundError(f"BundleSDF inputs missing: {missing}")
    rgb = sorted((scan_dir / "rgb").glob("*.png"))
    if len(rgb) < 8:
        raise RuntimeError(f"BundleSDF needs multi-view coverage; only {len(rgb)} frames")
    for source in rgb:
        stem = source.stem
        for folder in ("depth", "masks", "gripper_masks"):
            if not (scan_dir / folder / f"{stem}.png").exists():
                raise FileNotFoundError(f"unsynchronized frame {folder}/{stem}.png")
    # The official reader recognizes masks_hand as occlusion. Keep a separate
    # gripper_masks directory as the auditable source and mirror it for the reader.
    occ = scan_dir / "masks_hand"
    occ.mkdir(exist_ok=True)
    for mask in (scan_dir / "gripper_masks").glob("*.png"):
        shutil.copy2(mask, occ / mask.name)


def run(scan_dir: Path, output: Path, official_root=DEFAULT_OFFICIAL_ROOT, optimized=False) -> dict:
    scan_dir, output, official_root = map(Path, (scan_dir, output, official_root))
    commits = verify_checkout(official_root)
    prepare_dataset(scan_dir)
    bundle = official_root / "scalable_real2sim/BundleSDF"
    python = Path(os.environ.get("BUNDLESDF_PYTHON", bundle / ".venv/bin/python"))
    if not python.exists():
        raise RuntimeError(
            f"BundleSDF environment missing ({python}); run the official setup.bash first"
        )
    output.mkdir(parents=True, exist_ok=True)
    native_lib = bundle / "native_env/lib"
    cuda_lib = Path("/usr/local/cuda-12.5/lib64")
    old_ld = os.environ.get("LD_LIBRARY_PATH", "")
    env = {
        **os.environ,
        "PYTHONUNBUFFERED": "1",
        "PYOPENGL_PLATFORM": "egl",
        "LD_PRELOAD": str(native_lib / "libjpeg.so.8"),
        "LD_LIBRARY_PATH": ":".join(
            str(p) for p in (native_lib, cuda_lib) if p.exists()
        ) + ((":" + old_ld) if old_ld else ""),
    }
    # The pinned official entrypoint asserts that interpolation is disabled.
    # Object masks already exclude every gripper pixel; masks_hand is retained
    # separately for audit and for readers that consume explicit occlusion masks.
    if optimized:
        entry=Path(__file__).with_name('bundlesdf_optimized_entry.py')
        command=[str(python),str(entry),'--bundle-root',str(bundle),'--video-dir',str(scan_dir.resolve()),'--out-folder',str(output.resolve())]
        if len(list((output/'ob_in_cam').glob('*.txt'))) == len(list((scan_dir/'rgb').glob('*.png'))):
            command.append('--global-only')
    else:
        command = [str(python), "run_custom.py", "--video_dir", str(scan_dir.resolve()),
                   "--out_folder", str(output.resolve()), "--use_gui", "0",
                   "--interpolate_missing_vertices", "0"]
    with (output.parent / (output.name + "_run.log")).open("w") as log:
        subprocess.run(command, cwd=bundle, env=env, stdout=log,
                       stderr=subprocess.STDOUT, check=True)
    material_files = sorted(output.glob("*.mtl"))
    texture_files = sorted(output.glob("*.png"))
    products = {
        "tracking": output / "ob_in_cam",
        "mesh": output / "textured_mesh.obj",
        "textured_mesh": output / "textured_mesh.obj",
        "material": material_files[0] if material_files else output / "material_0.mtl",
        "texture": texture_files[0] if texture_files else output / "material_0.png",
    }
    missing = [str(path) for path in products.values() if not path.exists()]
    if missing:
        raise RuntimeError(f"BundleSDF completed without required products: {missing}")
    result = {"backend": "official_scalable_real2sim_bundlesdf", "commits": commits,"optimized_scan_v2":bool(optimized),
              "command": command, "products": {k: str(v) for k, v in products.items()}}
    (output / "reconstruction.json").write_text(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("scan_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--official-root", type=Path, default=DEFAULT_OFFICIAL_ROOT)
    parser.add_argument("--optimized", action="store_true")
    args = parser.parse_args()
    print(json.dumps(run(args.scan_dir, args.output, args.official_root,args.optimized), indent=2))
