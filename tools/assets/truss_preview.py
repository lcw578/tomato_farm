#!/usr/bin/env python3
"""果串模型预览渲染器：读 truss_generator 产的 DAE，三视图+着色近似贴图效果。"""
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def parse_dae(path):
    s = Path(path).read_text()
    geoms = {}
    for m in re.finditer(r'<geometry id="([^"]+)-mesh".*?<triangles[^>]*>.*?<p>([^<]+)</p>', s, re.S):
        name = m.group(1)
        block = m.group(0)
        pa = re.search(r'<float_array id="[^"]*positions-array" count="\d+">([^<]+)</float_array>', block)
        verts = np.array([float(x) for x in pa.group(1).split()]).reshape(-1, 3)
        idx = np.array([int(x) for x in m.group(2).split()]).reshape(-1, 3, 3)[:, :, 0]
        geoms[name] = (verts, idx)
    return geoms


def color_for(name):
    if 'fruit' in name:
        return '#c62820', 1.0
    if 'sepal' in name:
        return '#3e7c2f', 1.0
    if 'trunk' in name or 'truss' in name:
        return '#4a6b35', 1.0
    return '#3f6b2a', 0.55  # 叶半透明


def render(path, out_png):
    geoms = parse_dae(path)
    fig, axes = plt.subplots(1, 3, figsize=(16, 7), facecolor='#22303f')
    views = [('正视图 (y)', 0, 2), ('侧视图 (x)', 1, 2), ('俯视图', 0, 1)]
    for ax, (title, i, j) in zip(axes, views):
        ax.set_facecolor('#22303f')
        for name, (verts, idx) in geoms.items():
            c, alpha = color_for(name)
            from matplotlib.collections import PolyCollection
            tris = verts[idx]
            ax.add_collection(PolyCollection(tris[:, :, [i, j]], facecolors=c,
                                             edgecolors='none', alpha=alpha))
        ax.set_title(title, color='w')
        ax.autoscale()
        ax.set_aspect('equal')
        ax.tick_params(colors='w')
        for spine in ax.spines.values():
            spine.set_color('#556')
    plt.suptitle(Path(path).stem, color='w')
    plt.tight_layout()
    plt.savefig(out_png, dpi=110, facecolor='#22303f')
    print('saved', out_png)


if __name__ == '__main__':
    render(sys.argv[1], sys.argv[2])
