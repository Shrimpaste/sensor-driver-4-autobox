# 智能跟随行李箱 · 跟随代码1（纯跟随，无避障）

**跟随代码1 = 算法4 简化版：仅保留 UWB 跟随 + IMU 航向闭环 + 底盘闭环，移除全部避障传感器和算法。**

以 `算法4` 的经验证传感器驱动和闭环控制为基础，去除激光雷达、超声波传感器和 VFH 避障算法，仅保留纯跟随功能。代码简洁，专注于跟随逻辑。

---

## 系统总体架构

```
                          ┌─────────────────────────┐
                          │      ESP32-S3            │
                          │                          │
   UWB (BU0x) ───────────►│ UART1 (RX=18, TX=37)     │
                          │   ↓ uwb_task (优先级6)    │
                          │                          │
   IMU ──────────────────►│ I2C0 (SDA=39, SCL=38)    │
                          │   ↓ (control_task 内读取) │
                          │                          │
                          │   ┌─────── 共享快照 ──────┐│
                          │   │ g_shared (互斥锁保护) ││
                          │   │ · 目标距离/方位 + 时间戳││
                          │   └───────────────────────┘│
                          │          ↓                  │
                          │   control_task (50Hz, 优先7)│
                          │   ┌───────────────────────┐│
                          │   │ 1. 快照 UWB 数据       ││
                          │   │ 2. 跟随状态机          ││
                          │   │ 3. IMU 航向闭环修正    ││
                          │   │ 4. chassis_set_velocity││
                          │   │ 5. chassis_update()    ││
                          │   └───────────────────────┘│
                          │          ↓                  │
   左 ESC (APO-DL) ◄──────│ GPIO4  RC PWM 50Hz        │
   右 ESC (APO-DL) ◄──────│ GPIO5  RC PWM 50Hz        │
   左编码器 A/B ◄─────────│ GPIO6 / GPIO7  (4x 正交解码 ISR) │
   右编码器 A/B ◄─────────│ GPIO15 / GPIO16 (4x 正交解码 ISR) │
                          └─────────────────────────┘
```

> **注意**：以上引脚为 Kconfig 默认值，可通过 `idf.py menuconfig` 修改。

---

## 与算法4的差异

| 项目 | 算法4 | 跟随代码1 |
|------|-------|-----------|
| 激光雷达 (RPLIDAR C1) | 有 | **移除** |
| 超声波 (A02YYUW x2) | 有 | **移除** |
| VFH-lite 避障算法 | 有 | **移除** |
| UWB 跟随 | 有 | 保留 |
| IMU 航向闭环 | 有 | 保留 |
| 底盘闭环 (ESC + 编码器 PID) | 有 | 保留 |
| 状态机 | 5 态 (IDLE/SEARCH/FOLLOW/AVOID/ESTOP) | **3 态** (IDLE/SEARCH/FOLLOW) |

---

## 软件架构：多任务 RTOS + 共享快照

### 任务调度

所有 3 个任务使用 `xTaskCreate()` 创建（**不绑定核心**），FreeRTOS SMP 调度器自动分配：

| 任务 | 优先级 | 栈 | 职责 |
|------|--------|-----|------|
| `uwb_task` | 6 | 4096 | UART1 接收 UWB 目标定位数据 |
| `control_task` | 7 | 4096 | 50Hz 控制循环（跟随/航向/底盘驱动） |

IMU 在 `control_task` 内同步读取，不占用独立任务。

### 共享快照（g_shared）

UWB 任务将数据写入全局结构体，由互斥锁保护：

```c
typedef struct {
    SemaphoreHandle_t lock;
    float tgt_distance_m;
    float tgt_bearing_rad;
    uint64_t tgt_ts_us;
} shared_t;
```

---

## 跟随状态机

```
          ┌────────────────────────┐
          │                        │
          ▼                        │
     ┌─────────┐   目标丢失     ┌──────────┐
     │  IDLE   │ ◄──────────── │ SEARCH   │
     │  停车    │  搜索超时(6s)  │ 原地旋转  │
     └─────────┘               └──────────┘
          ▲                          │
          │  目标重捕                 │ 目标重捕
          │                          ▼
     ┌──────────────────────────────────┐
     │             FOLLOW                │
     │  直线跟随 (距离P + 方位P + IMU航向) │
     └──────────────────────────────────┘
```

- **FOLLOW**：距离 P 控制线速度 + 方位 P 控制角速度 + IMU 航向闭环 + 转向减速耦合
- **SEARCH**：目标丢失后在最后已知方向旋转搜索，默认 6 秒超时退回 IDLE
- **IDLE**：完全停车

---

## 跟随控制详解

输入：UWB 解析出的目标 `{距离 d, 方位 β}`。

**线速度**（保持车距）：
- `d > 1.0m`：`v = kp_dist × (d − 1.0)`，追上去（`kp_dist = 0.9`）
- `0.75m ≤ d ≤ 1.0m`：`v = 0`，停止带内保持静止
- `d < 0.75m`：`v = 0`，太近，不倒车

**角速度**（对准目标）：`ω = kp_bear × β`（`kp_bear = 1.6`）

**转向-速度耦合**：`v_effective = v × (1 − |β| / 90°)`，转角越大前进越慢。

**IMU 航向闭环**：
```
yaw_ref += ω · dt
ω_cmd = ω + heading_kp × wrap(yaw_ref − yaw_meas)
```

---

## 闭环底盘（chassis）

### 电机驱动：APO-DL ESC + RC PWM

两个 APO-DL 电调各接收一路 50Hz 航模舵机脉冲（LEDC 14 位分辨率）：

| 脉冲宽度 | 含义 |
|---------|------|
| 1000 µs | 全速后退 |
| 1500 µs | 中位 / 停止 |
| 2000 µs | 全速前进 |

### 速度闭环：AB 编码器 + 轮速 PID

```
目标轮速 (m/s) ──► 前馈 ff_us = (500)/0.8 × 目标轮速 ──┐
                                                       ├─► 电调脉冲 = 1500 + 限幅(ff + pid)
实测轮速 (m/s) ──► 误差 ──► PID(kp,ki,kd) ──► 修正 us ─┘
   ▲
   │
AB 编码器 4x 正交解码（GPIO 边沿中断 + 16 状态查找表）→ 计数差 / ticks_per_meter / dt
```

- **前馈**：保证编码器掉线也能走（优雅降级）
- **PID**：`kp=200, ki=300, kd=5`（us 每 m/s），积分器同限幅抗饱和，微分取在测量值上
- **停止处理**：目标速度为 0 时强制中位 + 清零积分器
- **斜率限幅**：1500 us/s，保护减速齿轮和电池

### 差速驱动运动学

```
v_left  = (v − ω × track/2) / vmax
v_right = (v + ω × track/2) / vmax
```

### 安全机制

| 机制 | 参数 | 说明 |
|------|------|------|
| 上电解锁延迟 | 2000ms | 上电后保持中位 2 秒等待电调自检 |
| 失控保护 | 0.3s | 控制任务卡死 → 自动拉回中位停车 |

---

## 构建与烧录

```bash
# 在项目根目录
idf.py set-target esp32s3
idf.py menuconfig   # 配置引脚、PID、跟随参数等
idf.py build
idf.py flash monitor
```

Kconfig 菜单位于 `Follow-only suitcase` 菜单下，可配置：
- ESC 引脚/脉宽
- AB 编码器引脚/每米脉冲数
- 速度 PID 参数（KP/KI/KD/限幅）
- 轮距/最大轮速
- 跟随距离/停止带/最大速度/Kp
- 搜索角速度/超时时间
- UWB 目标新鲜度阈值
- IMU 航向闭环增益
- UWB/I2C 引脚
- 控制循环频率

---

## 关键参数速查

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `follow_distance_mm` | 1000mm | 期望跟车距离 |
| `stop_band_mm` | 250mm | 停止带 |
| `max_linear_mmps` | 700 mm/s | 最大前进速度 |
| `max_angular_mradps` | 1600 mrad/s | 最大转向角速度 |
| `kp_dist` | 900 (milli) | 距离 P 增益 |
| `kp_bear` | 1600 (milli) | 方位 P 增益 |
| `search_timeout_s` | 6s | 搜索超时 |
| `target_fresh_ms` | 700ms | UWB 数据新鲜度 |
| `heading_kp_milli` | 1500 | 航向闭环增益 |
| `ticks_per_meter` | 2000 | 编码器每米 4x 脉冲数（**必须标定**） |
| `max_speed_mmps` | 800 mm/s | 满速轮速 |
| `kp / ki / kd` | 200 / 300 / 5 | 速度 PID（us 每 m/s） |
| `pid_out_limit_us` | 400 µs | PID 单轮最大修正 |
| `esc_min/mid/max_us` | 1000/1500/2000 | ESC 脉冲行程 |
| `control_hz` | 50 | 控制循环频率 |

---

## 引脚对照表

| 外设 | 接口 | 引脚（Kconfig 默认） | 备注 |
|------|------|------|------|
| 左 ESC | LEDC PWM | GPIO4 | 50Hz RC 脉冲，1000–2000µs |
| 右 ESC | LEDC PWM | GPIO5 | 50Hz RC 脉冲 |
| 左编码器 A | GPIO 中断 | GPIO6 | 4x 正交解码 |
| 左编码器 B | GPIO 中断 | GPIO7 | 4x 正交解码 |
| 右编码器 A | GPIO 中断 | GPIO15 | 4x 正交解码 |
| 右编码器 B | GPIO 中断 | GPIO16 | 4x 正交解码 |
| BU UWB | HW UART1 | RX=18, TX=37 | 115200 baud |
| IMU | I2C0 | SDA=39, SCL=38 | 地址 0x23 |

---

## 目录结构

```
├── CMakeLists.txt                    # 根项目：follow_only
├── README.md                         # 本文件
├── components/
│   ├── control/
│   │   ├── chassis/                  # 闭环底盘（ESC PWM + 编码器 PID + 前馈）
│   │   │   ├── chassis.c/h
│   │   │   └── CMakeLists.txt
│   └── sensors/
│       ├── bu_uwb/                   # BU03/BU04 UWB 超宽带（TWR JSON 解析）
│       └── imu_i2c/                  # 九轴 IMU（I2C 寄存器读取 + 四元数/欧拉角）
└── examples/
    └── follow_only/                  # ★ 主程序：纯跟随
        ├── CMakeLists.txt
        ├── sdkconfig.defaults
        └── main/
            ├── CMakeLists.txt
            ├── Kconfig.projbuild
            └── main.c
```

---

## 上电调试顺序

1. **垫高轮子空跑**：先确认电调解锁（上电后等待 2 秒 ESC 自检）
2. **标定编码器方向与每米脉冲**：手推 1m，看日志实测速度符号和增量
3. **标定满速轮速**：给最大前进，量实际轮速
4. **整定速度 PID**：先只留前馈（KI=KD=0）看跟随误差，再加 KI 消静差、KD 抑超调
5. **校航向闭环**：直线跑若往一侧偏，调整 `HEADING_KP_MILLI`
6. **校 UWB 左右**：目标方位方向反了则改 `FR_UWB_LEFT_SIGN` 符号

---

## 注意事项

1. **编码器 `TICKS_PER_METER` 必须标定**：推机器人恰好 1 米，记录日志中编码器计数差值
2. **GPIO39 作 SDA** 仅 ESP32-S3 支持（经典 ESP32 的 GPIO34–39 仅为输入）
3. **无后向感知**：算法刻意不倒车，过近只停车
4. **无避障**：本版本不包含任何避障功能，前方有障碍物时会直接撞上
5. **传感器降级**：编码器掉线退化为前馈开环，IMU 掉线跳过航向闭环
6. **失效保护**：控制任务卡死会在 0.3 秒内自动停车
