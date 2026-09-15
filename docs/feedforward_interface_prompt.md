# 外环轨迹规划器 — PX4 前馈接口说明（已实现）

> 本文档描述 outer_loop（外环）与 PX4（内环，v1.17 二次开发）之间
> 动力学前馈接口的**已实现**方案。控制律：`u = u_ff + u_fb`，
> PX4 原有 TECS + 姿态/角速度反馈保持不变。

## 传输通道（已实现）

外环程序运行在机载计算机上，经 **MAVLink TUNNEL 消息**（msg id 385，需 MAVLink2）
向飞控发送前馈设定值：

| 项 | 值 |
|---|---|
| MAVLink 消息 | `TUNNEL` (385) |
| `payload_type` | `0xFFF1`（> 32767，本地实验段） |
| `payload` | 12 字节：3 × float32 小端 `[n_n_ff, n_t_ff, p_ff]` |
| `target_system/component` | 飞控 sysid/compid（MavSDK `get_target_sysid/compid()`） |
| 发送频率 | 跟随外环控制频率（50 Hz），不得低于 10 Hz |

PX4 侧数据流：

```
TUNNEL → mavlink_receiver → mavlink_tunnel (uORB)
       → fw_ff_control 解码 → vehicle_ff_setpoint (uORB, 规范输入话题)
       → fw_feedforward (q_ff / p_ff / E_dot_ff)
       → fw_att_control 叠加 roll/pitch 角速率设定点
       → fw_lat_lon_control 将 E_dot_ff 注入 TECS 总能量率需求（油门前馈）
```

- outer_loop 侧实现：`src/feedforward.h`（计算与载荷约定）、
  `Px4Interface::sendFeedforwardSetpoint()`（`src/px4_interface.cpp`）、
  TRACKING 每周期发送（`src/state_machine.cpp`），开关 `[feedforward] enable`。
- PX4 侧实现：`src/modules/fw_ff_control/FwFfControl.cpp` 的 `mavlink_tunnel_poll()`，
  payload_type 常量 `kFfTunnelPayloadType = 0xFFF1`（两侧保持一致）。

## 物理量语义（两侧已对齐）

| 量 | 定义 | 单位 |
|---|---|---|
| `n_n_ff` | 气流坐标系法向过载 `L/(m·g)`，**定直平飞 = 1.0** | - |
| `n_t_ff` | 气流坐标系切向过载 `(T−D)/(m·g) = V̇/g + sin(γ)` | - |
| `p_ff` | **机体系**滚转角速度（姿态参考 `omega_d.x`） | rad/s |

外环由航迹参考 v/a（NED）按比力推导（`computeFeedforward()`）：

```
f      = a − g_vec                 # 比力，g_vec = (0,0,g)（NED 重力向下）
n_t_ff = (f·v̂)/g
n_n_ff = sqrt(|f|² − (f·v̂)²)/g     # 协调飞行下即升力过载
p_ff   = omega_d.x                 # 姿态参考失效时为 0
```

PX4 侧转换（`src/modules/fw_ff_control/FwFfControl.hpp`）：

```
q_ff       = FW_NN_FF_GAIN · g/V_TAS · (n_n_ff − 1)   → 姿态环 pitch 速率设定（低速淡出 + 限幅）
p_ff_out   = FW_P_FF_GAIN · p_ff                      → 姿态环 roll 速率设定
E_dot_ff   = FW_NT_FF_GAIN · V_TAS · g · n_t_ff       → TECS 总能量率需求（直接驱动油门前馈）
```

切向过载不再积分成空速偏置，而是直接映射为比总能量率
`E_dot = V·(T−D)/m = V·g·n_t`，注入 TECS 内部已有的“能量率 → 油门”前馈映射，
因此油门可在误差积累之前提前响应。`q_ff` 在真空速低于 `FW_AIRSPD_MIN` 时线性淡出，
并限幅到 `FW_P_RMAX_NEG/POS`；前馈叠加后仍受速率限幅保护。

## 运行约束

1. **连续发送**：PX4 侧 0.5 s 未收到前馈即将三项清零（前馈退出，不影响 TECS 反馈控制）。
2. **只发有限值**；速度低于 `min_speed`（外环）时外环停发，PX4 自动退出前馈。
3. **无机动时应发送基准值** `n_n_ff≈1, n_t_ff≈sin(γ), p_ff≈0`，而非停发或全 0
   （全 0 的 n_n_ff 会被解读为 0g 过载，产生错误的大幅低头前馈）。
4. PX4 侧纯前馈：不校验物理合理性、不做时间外推；外环应自行饱和与限速。

## PX4 侧参数（默认 0 = 前馈关闭，飞前必须设置）

| 参数 | 含义 | 范围 |
|---|---|---|
| `FW_NN_FF_GAIN` | 法向过载→俯仰角速率前馈增益 K_n | 0–5 |
| `FW_NT_FF_GAIN` | 切向过载→总能量率前馈增益（E_dot_ff = K · V_TAS · g · n_t_ff，经 TECS 油门映射） | 0–5 |
| `FW_P_FF_GAIN` | 滚转角速率前馈增益 | 0–2 |

实机增益需小增益起步、逐步增加（人工流程）。

## 调试与验证

- 无规划器台架自测（PX4 shell）：`fw_ff_control inject <n_n> <n_t> <p_ff> [duration_s]`
- 观测：`listener vehicle_ff_setpoint` / `listener fw_feedforward` /
  `listener vehicle_rates_setpoint` / `listener tecs_status`
- 日志默认记录：`vehicle_ff_setpoint`、`fw_feedforward`（Flight Review 可查）
- 备选输入通道（保留）：uORB/DDS 直接发布 `vehicle_ff_setpoint`
  （DDS topic `/fmu/in/vehicle_ff_setpoint` 已注册），与 TUNNEL 路径并存

## 链路细节（实测确认）

- SITL 机载链路端口：PX4 在 **UDP 14580 监听**、向 14540 发送（`mavlink status`
  中 instance "Onboard ... UDP (14580, remote port: 14540)"）。MavSDK
  `add_any_connection("udp://0.0.0.0:14540")` 监听 14540 并向 PX4 源地址回发，
  因此 TUNNEL 随 PX4 机载链路收发即可，无需额外配置。
- `fw_ff_control` 以 100 Hz 固定周期运行（`ScheduleOnInterval(10_ms)`），
  TUNNEL → `vehicle_ff_setpoint` 转发延迟 ≤ 10 ms；周期调度同时保证
  0.5 s 输入超时后前馈可靠清零。

## 验证记录

- 2026-09-10 台架测试（SIH 固定翼 SITL + 手写 MAVLink2 TUNNEL 帧，20 Hz）：
  `listener mavlink_tunnel` 收到 payload_type 0xFFF1 / 12 字节；
  `vehicle_ff_setpoint` 显示 n_n=1.5 / n_t=0.2 / p=0.3（与注入一致）；
  `fw_feedforward` 显示 `valid=true`、p_ff 与 ste_rate_ff 与公式自洽。链路全通。
  （更早版本的 airspeed_ff 积分路径已按“总能量率前馈”方案替换。）
- 注意：`fw_ff_control` 曾因回调调度挂在 `vehicle_ff_setpoint` 上形成死锁
  （TUNNEL 转发在 Run() 内，而 Run() 等 `vehicle_ff_setpoint` 更新才触发），
  已改为周期调度修复。
