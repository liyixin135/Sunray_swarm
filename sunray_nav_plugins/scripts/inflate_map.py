#!/usr/bin/env python3
import math
import yaml
import cv2
import numpy as np
import argparse
from pathlib import Path

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in_yaml", required=True, help="input map yaml")
    ap.add_argument("--inflate_m", type=float, default=0.5, help="inflate distance in meters")
    ap.add_argument("--out_dir", required=True, help="output directory")
    ap.add_argument("--occ_thresh", type=int, default=65, help="0..255: pixels darker than this are treated as occupied")
    args = ap.parse_args()

    in_yaml = Path(args.in_yaml).resolve()
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    with open(in_yaml, "r") as f:
        meta = yaml.safe_load(f)

    res = float(meta["resolution"])
    r_px = int(math.ceil(args.inflate_m / res))

    img_rel = Path(meta["image"])
    img_path = (in_yaml.parent / img_rel).resolve()

    img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError(f"Failed to read image: {img_path}")

    # occupied mask (PGM: 0=occupied black, 255=free white)
    occ = (img < args.occ_thresh).astype(np.uint8) * 255

    k = 2 * r_px + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    occ_dil = cv2.dilate(occ, kernel, iterations=1)

    out_img = img.copy()
    out_img[occ_dil > 0] = 0  # mark inflated as occupied

    out_img_name = f"{img_path.stem}_inflated_{args.inflate_m:.2f}m.pgm"
    out_img_path = out_dir / out_img_name
    cv2.imwrite(str(out_img_path), out_img)

    out_yaml_name = f"{in_yaml.stem}_inflated_{args.inflate_m:.2f}m.yaml"
    out_yaml_path = out_dir / out_yaml_name

    out_meta = dict(meta)
    out_meta["image"] = out_img_name  # keep relative path inside out_dir
    with open(out_yaml_path, "w") as f:
        yaml.safe_dump(out_meta, f, sort_keys=False)

    print("OK")
    print(f"resolution={res} m/px")
    print(f"inflate={args.inflate_m} m -> radius={r_px} px")
    print(f"input_yaml : {in_yaml}")
    print(f"input_img  : {img_path}")
    print(f"output_yaml: {out_yaml_path}")
    print(f"output_img : {out_img_path}")

if __name__ == "__main__":
    main()