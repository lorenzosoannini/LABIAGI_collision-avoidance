#!/usr/bin/env python3
"""Render the C++ simulator's recorded trajectory. No synthetic controller outputs."""
import argparse
import csv
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--trajectory', type=Path, default=Path('assets/trajectory.csv'))
parser.add_argument('--output', type=Path, default=Path('assets/avoidance.gif'))
args = parser.parse_args()
with args.trajectory.open() as handle:
    rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(handle)]
if not rows:
    raise ValueError('Trajectory is empty')


def font(size, bold=False):
    paths = [f'/System/Library/Fonts/Supplemental/Arial{" Bold" if bold else ""}.ttf',
             f'/usr/share/fonts/truetype/dejavu/DejaVuSans{"-Bold" if bold else ""}.ttf']
    for path in paths:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


navy, muted, teal, amber = '#172D40', '#617382', '#007F82', '#D68C29'
W, H = 1000, 620
scale = 102


def xy(x, y):
    return (75 + x*scale, 367 - y*scale)


def circle(draw, x, y, radius, **kwargs):
    px, py = xy(x, y)
    r = radius * scale
    draw.ellipse((px-r, py-r, px+r, py+r), **kwargs)


frames = []
for i, row in enumerate(rows):
    im = Image.new('RGB', (W, H), '#F4F6F5')
    d = ImageDraw.Draw(im)
    d.text((35, 24), 'Steer around. Keep moving.', fill=navy, font=font(31, True))
    d.text((36, 68), 'Laser-based collision avoidance  /  shared C++ controller', fill=muted, font=font(17))
    d.rounded_rectangle((30, 112, 785, 535), 16, fill='white')
    d.rounded_rectangle((805, 112, 970, 535), 16, fill='#E6EDED')
    for x in range(8):
        px, _ = xy(x, 0)
        if px < 770:
            d.line((px, 130, px, 505), fill='#EDF0F0')
            d.text((px-6, 510), str(x), fill=muted, font=font(12))
    for y in [-1, 0, 1, 2]:
        _, py = xy(0, y)
        d.line((45, py, 770, py), fill='#EDF0F0')
    # Direct line is a geometric reference, not an alternative simulated robot.
    for x in range(75, int(xy(6.5, 0)[0]), 15):
        d.line((x, xy(0, 0)[1], x+7, xy(0, 0)[1]), fill='#B9C4CC', width=2)
    circle(d, 2.5, 0, .75, outline='#E9CFA8', width=2)
    circle(d, 2.5, 0, .45, fill='#F0C480', outline=amber, width=2)
    d.text((xy(2.5, 0)[0]-32, xy(2.5, 0)[1]-8), 'obstacle', fill='#7A4E13', font=font(15))
    circle(d, 6.5, 0, .12, outline=teal, width=3)
    d.text((xy(6.5, 0)[0]-15, xy(6.5, 0)[1]+22), 'goal', fill=teal, font=font(14))
    path = [xy(0, 0)] + [xy(r['x'], r['y']) for r in rows[:i+1]]
    d.line(path, fill=teal, width=4)
    circle(d, row['x'], row['y'], .20, fill=navy)
    px, py = xy(row['x'], row['y'])
    d.line((px, py, px+25*math.cos(row['yaw']), py-25*math.sin(row['yaw'])), fill='#00C8BC', width=4)
    d.text((49, 131), 'TOP VIEW  /  metres', fill=muted, font=font(12, True))
    d.line((50, 161, 78, 161), fill=teal, width=3)
    d.text((87, 151), 'executed trajectory', fill=navy, font=font(14))
    d.text((49, 178), 'Dashed line: direct start-to-goal reference', fill=muted, font=font(12))
    d.text((49, 198), 'Light ring: obstacle + robot radius + margin', fill=muted, font=font(12))
    d.text((823, 135), 'CONTROL', fill=muted, font=font(12, True))
    status = 'AVOIDING' if abs(row['w'] - row['desired_w']) > .05 else 'FOLLOWING'
    if i == len(rows)-1:
        status = 'GOAL REACHED'
    d.text((823, 166), status, fill=teal, font=font(15, True))
    for y, label, value in [(221, 'Forward speed', f"{row['v']:.2f} m/s"),
                            (309, 'Turn rate', f"{row['w']:+.2f} rad/s"),
                            (397, 'Surface clearance', f"{row['clearance']:.2f} m")]:
        d.text((823, y), label, fill=muted, font=font(13))
        d.text((823, y+25), value, fill=navy, font=font(21, True))
    d.text((823, 487), f"t = {row['t']:.1f} s", fill=muted, font=font(15))
    d.text((36, 552), 'DETERMINISTIC 2D SIMULATION  |  360-degree laser  |  circular robot', fill=muted, font=font(14, True))
    d.text((36, 578), 'Recorded C++ outputs. External goal guidance supplies intent; the filter only sees velocity + laser points.', fill=muted, font=font(13))
    frames.append(im)
args.output.parent.mkdir(parents=True, exist_ok=True)
frames[0].save(args.output, save_all=True, append_images=frames[1:],
               duration=[100]*(len(frames)-1)+[1600], loop=0, optimize=True)
frames[len(frames)//3].save(args.output.with_suffix('.png'))
print(f'Saved {args.output}: {len(frames)} recorded frames')
