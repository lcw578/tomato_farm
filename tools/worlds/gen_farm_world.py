#!/usr/bin/env python3
"""温室世界随机化重排：franka 包的 22mx14m 温室世界。

把 75 个槽位上"同一基础模型、yaw 全 0"的果串番茄植株，替换为
asset_extract/out/variants/v0~v11 共 12 个随机化变体（每棵仍是
4 条番茄串的作者资产植株）伪随机均衡指派 + 每实例 yaw 抖动，
并同步重生成世界坐标果实真值 markers（与画面严格对齐）。

变换约定（已用 aoc 世界真值反向验证）：
    world = R(yaw) @ local + (px, py, pz)

用法：
    python3 gen_farm_world.py            # 使用默认种子 42
    python3 gen_farm_world.py --seed 7   # 换一种布局

输出（原地覆盖，旧版本在 git 历史中）：
    franka/worlds/tomato_farm_22mx14m_gazebo_classic.world
    franka/worlds/tomato_farm_markers.json
    franka/worlds/tomato_farm_manifest.json   # 槽位→变体+位姿，可复现
"""
import argparse
import json
import math
import random
import re
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent.parent
WORLD = BASE / 'franka/worlds/tomato_farm_22mx14m_gazebo_classic.world'
MARKERS_OUT = BASE / 'franka/worlds/tomato_farm_markers.json'
MANIFEST_OUT = BASE / 'franka/worlds/tomato_farm_manifest.json'
VARIANTS_DIR = BASE / 'asset_extract/out/variants'
NUM_VARIANTS = 12
YAW_JITTER_DEG = 45.0   # ±45°，与 aoc 世界同分布；株距 0.7m，不会撞邻株

PLANT_BLOCK = re.compile(
    r'<include>\s*'
    r'<uri>model://tomato_vine_repo(?:_v\d+)?</uri>\s*'
    r'<name>vine_plant_(\d+)</name>\s*'
    r'<pose>([^<]+)</pose>\s*'
    r'</include>')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=42)
    args = ap.parse_args()
    rng = random.Random(args.seed)

    text = WORLD.read_text()
    slots = {}
    for m in PLANT_BLOCK.finditer(text):
        idx = int(m.group(1))
        pose = [float(v) for v in m.group(2).split()]
        assert len(pose) == 6, f'vine_plant_{idx} pose 格式异常: {m.group(2)}'
        slots[idx] = pose
    assert sorted(slots) == list(range(75)), f'槽位数量异常: {len(slots)}'

    # 均衡指派：75 = 12×6 + 3，每个变体至少 6 棵，再洗牌
    assign = [k for k in range(NUM_VARIANTS) for _ in range(6)]
    assign += rng.sample(range(NUM_VARIANTS), 75 - len(assign))
    rng.shuffle(assign)

    local_markers = {
        k: json.loads((VARIANTS_DIR / f'v{k}' / 'markers.json').read_text())
        for k in range(NUM_VARIANTS)}
    for k, ms in local_markers.items():
        assert len(ms) == 48, f'v{k} markers 数量异常: {len(ms)}'

    def repl(m):
        idx = int(m.group(1))
        x, y, z = slots[idx][:3]
        v = assign[idx]
        # yaw 量化到 2 位小数：world pose / manifest / markers 三处共用同一值，严格一致
        yaw = round(rng.uniform(-YAW_JITTER_DEG, YAW_JITTER_DEG), 2)
        slots[idx] = [x, y, z, yaw]
        return (f'<include>\n'
                f'            <uri>model://tomato_vine_repo_v{v}</uri>\n'
                f'            <name>vine_plant_{idx}</name>\n'
                f'            <pose>{x} {y} {z} 0 0 {yaw:.2f}</pose>\n'
                f'        </include>')

    new_text, n = PLANT_BLOCK.subn(repl, text)
    assert n == 75, f'替换数量异常: {n}'

    # 世界坐标真值：R(yaw) @ local + (x, y, z)
    markers = []
    for idx in range(75):
        x, y, z, yaw = slots[idx]
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        for lm in local_markers[assign[idx]]:
            lx, ly, lz = lm['translation']
            markers.append({
                'marker_type': 'FRUIT',
                'plant_id': f'vine_plant_{idx}',
                'variant': assign[idx],
                'truss_id': lm['truss_id'],
                'ripeness': lm['ripeness'],
                'translation': [round(c * lx - s * ly + x, 4),
                                round(s * lx + c * ly + y, 4),
                                round(lz + z, 4)]})

    manifest = {
        'seed': args.seed,
        'yaw_jitter_deg': YAW_JITTER_DEG,
        'slots': [{'plant_id': f'vine_plant_{idx}',
                   'variant': assign[idx],
                   'pose': [slots[idx][0], slots[idx][1], slots[idx][2],
                            round(slots[idx][3], 2)]}
                  for idx in range(75)]}

    WORLD.write_text(new_text)
    MARKERS_OUT.write_text(json.dumps(markers, indent=1))
    MANIFEST_OUT.write_text(json.dumps(manifest, indent=1))

    balance = {v: assign.count(v) for v in range(NUM_VARIANTS)}
    print(f'✅ 世界重排完成 (seed={args.seed})')
    print(f'   变体分布: {balance}')
    print(f'   markers: {len(markers)} 条 (75 棵 × 48 颗)')
    print(f'   输出: {WORLD.name}, {MARKERS_OUT.name}, {MANIFEST_OUT.name}')


if __name__ == '__main__':
    main()
