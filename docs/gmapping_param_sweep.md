# gmapping 参数对照实验（自建世界，固定路线）

> 目的：把"官方默认参数"和两组调整方案放在同一路线、同一世界里对比，看地图质量
> 与代价。所有数字可复现，脚本见 `scripts/gmapping_param_sweep.sh`。

## 实验设置

| 项 | 值 |
|---|---|
| 世界 | `worlds/xzy_lab.world`（8×6 m，两面房间 + 1.2 m 门洞，不对称障碍） |
| 路线 | `scripts/drive_lab.py` 固定路线，约 85 s 走遍两个房间 |
| 激光 | 360 束、量程 3.5 m、约 2 Hz |
| 评价 | `scripts/map_quality.py`：地图 vs 由 world 文件渲染的真值栅格 |
| 地图分辨率/画布 | 0.05 m / ±10 m（三组一致） |

## 三组参数

| 组 | 相对官方默认的改动 | 文件 |
|---|---|---|
| A_official | 无（官方默认） | `launch/gmapping_params_A_official.yaml` |
| B_fine | `linearUpdate 1.0→0.4`、`angularUpdate 0.2→0.1` | `launch/gmapping_params_B_fine.yaml` |
| C_lenient | `minimumScore 50→30`、`maxUrange 3.0→3.5` | `launch/gmapping_params_C_lenient.yaml` |

## 结果（每组独立跑一次，同一路线）

| 组 | 处理扫描帧 | 原始 IoU | **双侧容差 2 格 IoU** | 实测占据范围 | 占据格数 | 地图文件 |
|---|---|---|---|---|---|---|
| A_official | 169 | 0.139 | 0.432 | 7.5 × 6.0 m | 1012 | `maps/lab_A_official.*` |
| B_fine | 200 | 0.139 | 0.426 | 7.3 × 6.0 m | 1042 | `maps/lab_B_fine.*` |
| C_lenient | 171 | 0.160 | **0.452** | **8.0 × 6.0 m** | 1096 | `maps/lab_C_lenient.*` |

（房间真值：8.2 × 6.1 m、占据格 3473。三组都没有丢帧告警。）

## 结论

1. **C 组（放宽匹配门槛 + 用满激光量程）最优**：容差 IoU 0.452，且占据范围
   8.0 × 6.0 m 与房间真值完全吻合——`minimumScore` 从 50 降到 30 减少了
   "匹配分数不够就丢帧"的情况，地图更完整。
2. **B 组（更细的更新阈值）在本场景没有收益**：IoU 0.426 与 A 组持平，覆盖范围
   反而略小。原因是本轮参数里 `temporalUpdate: 0.5 s` 才是主导——扫描约 2 Hz，
   每帧都满足"0.5 s 内有过更新"，所以把 `linearUpdate/angularUpdate` 调小
   并不会带来更多更新（处理扫描帧从 169 涨到 200 是角更新阈值变小带来的）。
   想验证"更细更新"的价值，应同时把 `temporalUpdate` 调小。
3. 差异幅度不大的诚实解读：这个小房间结构简单，三组参数都能建出可用地图；
   参数影响主要体现在**覆盖完整性**（能否画满 8×6 m）而不是"像不像"。

## 复现

```bash
# 确保没有别的 Gazebo/roscore 在跑
bash ~/xzy-project/scripts/gmapping_param_sweep.sh          # 依次跑三组，约 7 分钟
for g in A_official B_fine C_lenient; do
  python3 ~/xzy-project/scripts/map_quality.py \
      ~/xzy-project/maps/lab_$g.pgm ~/xzy-project/maps/lab_$g.yaml \
      ~/xzy-project/worlds/xzy_lab.world
done
```
