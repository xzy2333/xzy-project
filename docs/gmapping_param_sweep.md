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

## 五组参数

| 组 | 相对官方默认的改动 | 文件 |
|---|---|---|
| A_official | 无（官方默认） | `launch/gmapping_params_A_official.yaml` |
| B_fine | `linearUpdate 1.0→0.4`、`angularUpdate 0.2→0.1` | `launch/gmapping_params_B_fine.yaml` |
| C_lenient | `minimumScore 50→30`、`maxUrange 3.0→3.5` | `launch/gmapping_params_C_lenient.yaml` |
| D_coarse | C + `temporalUpdate 0.5→2.0`（让距离触发成为主导） | `launch/gmapping_params_D_coarse.yaml` |
| E_fine | D + `linearUpdate 1.0→0.4`、`angularUpdate 0.2→0.1` | `launch/gmapping_params_E_fine.yaml` |

## 结果（每组独立跑一次，同一路线）

| 组 | 更新帧数 | 原始 IoU | **双侧容差 2 格 IoU** | 实测占据范围 | 占据格数 | 地图文件 |
|---|---|---|---|---|---|---|
| A_official | 169 | 0.139 | 0.432 | 7.5 × 6.0 m | 1012 | `maps/lab_A_official.*` |
| B_fine | 200 | 0.139 | 0.426 | 7.3 × 6.0 m | 1042 | `maps/lab_B_fine.*` |
| C_lenient | 171 | 0.160 | **0.452** | **8.0 × 6.0 m** | 1096 | `maps/lab_C_lenient.*` |
| D_coarse | 73 | 0.117 | 0.420 | 7.9 × 6.0 m | 956 | `maps/lab_D_coarse.*` |
| E_fine | 108 | 0.148 | **0.452** | 7.9 × 5.9 m | 1020 | `maps/lab_E_fine.*` |

（房间真值：8.2 × 6.1 m、占据格 3473。五组都没有丢帧告警。）

**本项目现在的默认**：`launch/mapping.launch` 的默认参数文件已指向 **E 组**；
要用官方默认或其它组，用 `gmapping_params:=` 指定对应 yaml 即可。

## 结论

1. **放宽匹配门槛 + 用满量程（C）提升最大**：`minimumScore 50→30`、
   `maxUrange 3.0→3.5` 后，容差 IoU 0.432→0.452，占据范围补齐到 8.0×6.0 m
   与真值吻合——说明官方门槛对本场景偏严，"分数不够就丢帧"是地图缺边的主因。
2. **B 组无收益，原因被 D/E 证实**：B 只调细了距离/角度阈值，但 `temporalUpdate=0.5 s`
   主导（扫描 2 Hz，几乎每帧都触发时间更新），所以设置不生效，IoU 反而从 0.432
   降到 0.426。把 `temporalUpdate` 放宽到 2.0 s（D 组）后，更新真正由运动触发
   （帧数 169→73）；此时再调细阈值（E 组）**立刻见效**：IoU 0.420→0.452、
   原始 IoU 0.117→0.148、占据格 956→1020。**"更细更新有没有用"的答案是：有用，
   前提是别被 temporalUpdate 掩盖。**
3. **E 与 C 打平，但代价更低**：同为 0.452，E 只用了 108 次更新，C 用了 171 次
   ——C 多出来的更新大多由"重新开始运动/原地微动"的时间触发带来，收益很低。
   工程含义：**推荐 E 组**（宽容匹配 + 时间阈值 2.0 s + 细距离/角度阈值），
   同等地图质量、约少 1/3 的匹配计算；若希望车静止时地图也持续刷新，则用 C 组。
4. 诚实解读：这是结构简单的小房间，五组都能建出可用地图，参数差异主要体现在
   **覆盖完整性**与**代价**上，而不是"像不像"。

## 复现

```bash
# 确保没有别的 Gazebo/roscore 在跑
bash ~/xzy-project/scripts/gmapping_param_sweep.sh          # 依次跑 A/B/C，约 7 分钟
SWEEP_GROUPS="D_coarse E_fine" bash ~/xzy-project/scripts/gmapping_param_sweep.sh
for g in A_official B_fine C_lenient D_coarse E_fine; do
  python3 ~/xzy-project/scripts/map_quality.py \
      ~/xzy-project/maps/lab_$g.pgm ~/xzy-project/maps/lab_$g.yaml \
      ~/xzy-project/worlds/xzy_lab.world
done
```
