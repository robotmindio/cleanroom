#!/usr/bin/env python3
"""Print-exact checkerboard PDF for camera_calibration.

  ./scripts/checkerboard.py            -> 8x6 inner corners, 25mm, A4
  ./scripts/checkerboard.py 9 6 20     -> cols rows square_mm

Print at 100% / "actual size" (no fit-to-page), then measure a square with a
ruler and pass --size to ros2 run camera_calibration cameracalibrator with the
measured value in metres, not the nominal one.
"""
import sys

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

cols, rows, mm = (int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (8, 6, 25.0)
sq = mm / 25.4  # inches
w, h = (cols + 1) * sq, (rows + 1) * sq
page = (11.69, 8.27) if w > h else (8.27, 11.69)  # A4, auto-oriented
if w > page[0] - 0.4 or h > page[1] - 0.4:
    sys.exit(f"{w:.1f}x{h:.1f}in exceeds A4 printable area - reduce square size")

fig = plt.figure(figsize=page)
ax = fig.add_axes([(page[0] - w) / 2 / page[0], (page[1] - h) / 2 / page[1], w / page[0], h / page[1]])
ax.set_xlim(0, cols + 1), ax.set_ylim(0, rows + 1), ax.axis("off")
for x in range(cols + 1):
    for y in range(rows + 1):
        if (x + y) % 2 == 0:
            ax.add_patch(Rectangle((x, y), 1, 1, color="black"))

out = f"checkerboard-{cols}x{rows}-{mm:g}mm.pdf"
fig.savefig(out)
print(f"{out}  --size {cols}x{rows} --square {mm / 1000:g}")
