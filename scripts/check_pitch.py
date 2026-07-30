#!/usr/bin/env python3
"""实车 signals 复盘：cmd 与云台实测的稳态差，定位打高/打低。

用法（仓库根目录）：
    python3 scripts/check_pitch.py records/<时间戳>_signals.jsonl

回答的问题：打偏是"cmd 算错了"还是"cmd 对了但云台没跟上"。
这两者的处理方式完全不同——前者调 pitch_offset，后者要调下位机 PID，
调错了只是用一个错误去补偿另一个错误。
"""
import json
import math
import statistics as st
import sys


def pct(v, p):
    if not v:
        return float("nan")
    return sorted(v)[min(len(v) - 1, int(len(v) * p))]


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    rows = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
    # 低频档 key 在多数行缺席，先 ffill 还原（见 data_recording_guide.md）
    last, filled = {}, []
    for r in rows:
        last.update(r)
        filled.append(dict(last))

    ctrl = [r for r in filled if r.get("cmd_control")]
    print(f"总帧 {len(rows)}   有控制 {len(ctrl)}")
    if not ctrl:
        print("没有 cmd_control=1 的帧：自瞄没接管，检查 enemy_color / 模式档位")
        return 0

    # ---- pitch：cmd vs 云台实测 ----
    have = [r for r in ctrl if "gimbal_pitch" in r and "cmd_pitch" in r]
    if have:
        d = [r["gimbal_pitch"] - r["cmd_pitch"] for r in have]
        print()
        print("【pitch】gimbal - cmd（云台是否跟得上 cmd）")
        print(f"  mean={st.mean(d):+.2f}°  p50={st.median(d):+.2f}°  "
              f"p95={pct([abs(x) for x in d], .95):.2f}°")
        dist = [math.hypot(r["x"], r["y"]) for r in have if "x" in r]
        if dist:
            d0 = st.median(dist)
            off = d0 * math.tan(math.radians(abs(st.mean(d)))) * 100
            print(f"  {d0:.1f}m 处折算竖直偏差 {off:.0f}cm（小板半高 6.25cm）")
        if abs(st.mean(d)) > 1.0:
            print("  → 云台稳态跟不上 cmd：这是下位机侧问题，调 PID，不要用 pitch_offset 硬补")
        else:
            print("  → 云台跟得上：若仍打偏，问题在 cmd 侧（弹道/offset/标定）")
    else:
        print("\n【pitch】缺 gimbal_pitch 字段——这份是回放数据，不是实车数据")

    # ---- cmd_pitch 与目标几何角的关系 ----
    geo = [r for r in ctrl if all(k in r for k in ("z", "x", "y", "cmd_pitch"))]
    if geo:
        diff = []
        for r in geo:
            d0 = math.hypot(r["x"], r["y"])
            # cmd 世界系向上为负，故几何角取负号比较
            diff.append(r["cmd_pitch"] - (-math.degrees(math.atan2(r["z"], d0))))
        print()
        print("【cmd_pitch】相对目标几何角的偏移（正常只应含重力补偿，约 -1~-3°）")
        print(f"  mean={st.mean(diff):+.2f}°  p50={st.median(diff):+.2f}°")
        if st.mean(diff) > 2:
            print(f"  → 偏移 {st.mean(diff):+.1f}° 明显超出重力补偿范围，"
                  f"检查 pitch_offset 是否把正确的弹道解算抵消掉了")

    # ---- yaw ----
    hy = [r for r in ctrl if "gimbal_yaw" in r and "cmd_yaw" in r]
    if hy:
        d = [r["gimbal_yaw"] - r["cmd_yaw"] for r in hy]
        print()
        print("【yaw】gimbal - cmd")
        print(f"  mean={st.mean(d):+.2f}°  p95={pct([abs(x) for x in d], .95):.2f}°")

    # ---- 目标运动状态：决定这份录像能验证哪些分支 ----
    tr = [r for r in filled if "w" in r]
    if tr:
        w = [abs(r["w"]) for r in tr]
        print()
        print("【目标运动】")
        print(f"  |w| p50={st.median(w):.2f}  p95={pct(w, .95):.2f}  max={max(w):.2f} rad/s")
        print(f"  r   p50={st.median([r['r'] for r in tr]):.3f} (四轮车真值≈0.2)")
        nis = [r["nis"] for r in tr if "nis" in r]
        if nis:
            m = st.mean(nis)
            print(f"  nis mean={m:.2f}（dof=4 理论≈4.0）", end="")
            print(f" → R 约偏大 {4/m:.0f} 倍" if m > 0 and m < 2 else "")
        if max(w) < 7:
            print("  ⚠ 全程 |w|<7，没进小陀螺分支：这份录像验证不了 coming/leaving 窗口")

    # ---- 开火 ----
    sh = [r for r in ctrl if r.get("cmd_shoot") or r.get("shoot")]
    print()
    print(f"【开火】{len(sh)}/{len(ctrl)} 帧 = {len(sh)/len(ctrl)*100:.0f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())
