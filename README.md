# 智能跟随行李箱 · 算法4（传感器修复 + 闭环控制 + 无预编译条件）

**算法4 = 传感器修改6（经验证可正常收数） + 算法2/3 闭环控制逻辑 + 全部移除 #if/#ifdef**

以 `传感器修改6` 的经验证传感器驱动为基础，整合 `算法2/3` 的闭环控制逻辑，并**移除全部 `#if` / `#ifdef` / `#ifndef` / `#else` / `#endif` 预编译指令**，代码无条件包含全部传感器驱动和闭环控制逻辑。

---

## 算法版本演变

| 版本 | 传感器基础 | 控制算法 | 预编译条件 | 状态 |
|------|-----------|---------|-----------|------|
| 算法1 | 传感器修改系列 | 开环 H 桥 + 跟随/避障 | 有 `#if/#ifdef` | 历史 |
| 算法2 | 传感器修改系列 | 闭环 APO-DL ESC + AB 编码器 PID + IMU 航向 | 有 `#if/#ifdef` | 参考 |
| 算法3 | 传感器驱动有 Bug | 闭环控制（同算法2） | 全部移除（导致传感器问题） | 有问题 |
| **算法4**（本分支） | **传感器修改6（经验证可收数） + 全部 #if 已清理** | 闭环控制（同算法2/3） | **全部移除** | 当前 |

---

## 系统总体架构

```
                          ┌─────────────────────────┐
                          │      ESP32-S3            │
                          │                          │
   UWB (BU0x) ───────────►│ UART1 (RX=18, TX=37)     │
                          │   ↓ uwb_task (优先级6)    │
   RPLIDAR C1 ───────────►│ UART2 (RX=17, TX=9)      │
                          │   ↓ lidar_task (优先级6)  │
   A02YYUW #1 (左前) ────►│ SW UART (GPIO35)        │
                          │   ↓ ultra_task_L (优先5)  │
   A02YYUW #2 (右前) ────►│ SW UART (GPIO36)        │
                          │   ↓ ultra_task_R (优先5)  │
   IMU ──────────────────►│ I2C0 (SDA=39, SCL=38)    │
                          │   ↓ (control_task 内读取) │
                          │                          │
                          │   ┌─────── 共享快照 ──────┐│
                          │   │ g_shared (互斥锁保护) ││
                          │   │ · 目标距离/方位 + 时间戳││
                          │   │ · 障碍物直方图 + 时间戳 ││
                          │   │ · 左/右超声波距离 + 时间戳││
                          │   └───────────────────────┘│
                          │          ↓                  │
                          │   control_task (50Hz, 优先7)│
                          │   ┌───────────────────────┐│
                          │   │ 1. 快照传感器数据      ││
                          │   │ 2. fa_update() 跟随避障││
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

> **注意**：以上引脚为 Kconfig 默认值，可通过 `idf.py menuconfig` 修改。实际使用的引脚以你在 menuconfig 中的配置为准。

---

## 软件架构：多任务 RTOS + 共享快照

### follow_robot 项目 — 任务调度

所有 5 个任务使用 `xTaskCreate()` 创建（**不绑定核心**），FreeRTOS SMP 调度器自动将任务分配到 Core 0 和 Core 1 上运行：

| 任务 | 优先级 | 栈 | 职责 |
|------|--------|-----|------|
| `uwb_task` | 6 | 4096 | UART1 接收 UWB 目标定位数据 |
| `lidar_task` | 6 | 4096 | UART2 接收激光雷达扫描数据 |
| `ultra_task_L` | 5 | 3072 | SW UART 左超声波测距 |
| `ultra_task_R` | 5 | 3072 | SW UART 右超声波测距 |
| `control_task` | 7 | 4096 | 50Hz 控制循环（跟随/避障/航向/底盘驱动） |

```c
// 实际代码 — 未指定核心，调度器自由分配
xTaskCreate(uwb_task,     "uwb",     4096, NULL,        6, NULL);
xTaskCreate(lidar_task,   "lidar",   4096, &s_lidar,    6, NULL);
xTaskCreate(ultra_task,   "ultra_l", 3072, &s_ua_left,  5, NULL);
xTaskCreate(ultra_task,   "ultra_r", 3072, &s_ua_right, 5, NULL);
xTaskCreate(control_task, "control", 4096, &s_chassis,  7, NULL);
```

`sensor_hub` 项目则显式绑定核心（见下文引脚对照表说明）。

### 共享快照（g_shared）

所有传感器任务将数据写入一个全局结构体，由互斥锁 `g_shared.lock` 保护：

```c
typedef struct {
    SemaphoreHandle_t lock;
    float tgt_distance_m;     // UWB: 目标距离（米）
    float tgt_bearing_rad;    // UWB: 目标方位（弧度，机体坐标系）
    uint64_t tgt_ts_us;       // UWB: 最后更新时间戳
    fa_obstacle_field_t field;// 雷达: 障碍物直方图（36扇区）
    uint64_t field_ts_us;     // 雷达: 最后更新时间戳
    float ul_m;               // 左超声: 净空距离
    uint64_t ul_ts_us;
    float ur_m;               // 右超声: 净空距离
    uint64_t ur_ts_us;
} shared_t;
```

### 数据新鲜度判定

控制任务每次循环读取快照后，用当前时间减去各传感器时间戳，与新鲜度阈值比较：

| 传感器 | 新鲜度阈值 | 超时行为 |
|--------|-----------|----------|
| UWB 目标 | 700ms | 目标判为无效 → SEARCH / IDLE |
| 激光雷达 | 500ms | 障碍场判为无效 → 仅靠超声波避障 |
| 超声波 | 500ms | 对应侧判为无效 → 仅用另一侧 |

### 传感器故障降级

- **无激光雷达** → 仅靠两路超声波做前向急停（ESTOP），不进行 VFH 绕行
- **无 UWB** → 无法定位用户，进入 SEARCH 旋转找回，超时后退回 IDLE 停车
- **编码器掉线** → 退化为纯前馈开环驱动（前馈项直接将目标速度映射为 ESC 脉冲）
- **IMU 掉线** → 跳过航向闭环，直接用算法输出的 ω 指令

---

## 传感器层详解

### 1. UWB 超宽带定位 —— 跟随目标

| 属性 | 值 |
|------|-----|
| 型号 | BU03 / BU04（Ai-Thinker） |
| 接口 | HW UART1（Kconfig 默认：RX=18, TX=37） |
| 波特率 | 115200 |
| 协议 | JSxxxx{"TWR":...} JSON 帧，或 "distance: X.XX" 纯距离行 |
| 运行任务 | `uwb_task`，栈 4096，优先级 6 |
| 输出 | 目标距离 `tgt_distance_m`、方位 `tgt_bearing_rad`，带时间戳 |

UWB 模块持续输出定位数据。`uwb_task` 逐行读取 UART，调用 `bu_uwb_parse_twr_line()` 解析 TWR JSON 帧，
从中提取 `Xcm`、`Ycm` 和 `distance_cm`，换算为机体坐标系下的距离与方位角（`atan2(Xcm, Ycm)` + 符号修正）。
解析结果写入互斥锁保护的 `g_shared`。

### 2. RPLIDAR C1 激光雷达 —— 障碍物感知

| 属性 | 值 |
|------|-----|
| 型号 | Slamtec RPLIDAR C1 |
| 接口 | HW UART2（Kconfig 默认：RX=17, TX=9） |
| 波特率 | 460800 |
| 协议 | 5 字节扫描数据包（角度 Q6、距离 Q2、品质、起始标志） |
| 运行任务 | `lidar_task`，栈 4096，优先级 6 |
| 输出 | 前向 180° 极坐标障碍物直方图 `fa_obstacle_field_t`（36 扇区，5°/扇区） |

每收到一个 5 字节数据点，调用 `rplidar_c1_read_point()` 解析角度、距离和品质。
当检测到 `start_bit`（新一圈起始标志）时，将刚完成的一圈直方图发布到 `g_shared.field`。

### 3. A02YYUW 超声波传感器 —— 近场安全兜底

| 属性 | 值 |
|------|-----|
| 型号 | A02YYUW（30–4500mm） |
| 数量 | 2 个（前左 + 前右） |
| 接口 | 软件 UART（bit-bang），Kconfig 默认 RX=35（左）/ 36（右），TX 不接 |
| 波特率 | 9600 |
| 协议 | 4 字节帧：0xFF 头 + DATA_H + DATA_L + 校验和 |
| 运行任务 | `ultra_task_L` / `ultra_task_R`，栈 3072，优先级 5 |
| 输出 | `ul_m` / `ur_m`（前方角落净空距离，米），带时间戳 |

使用软件 UART 而非硬件 UART —— 因为 ESP32-S3 仅 2 个硬件 UART（UART1 给 UWB，UART2 给雷达）。
软件 UART 通过 GPIO 下降沿中断 + `esp_timer` 在每位中心采样实现，9600 波特率下完全可靠。

### 4. IMU 惯性测量单元 —— 航向闭环

| 属性 | 值 |
|------|-----|
| 型号 | 定制 I2C 九轴 IMU |
| 接口 | I2C0（Kconfig 默认：SDA=39, SCL=38） |
| 地址 | 0x23（7 位） |
| 读取 | 加速度计、陀螺仪、磁力计、四元数、欧拉角 |

IMU 不在独立任务中运行，而是在 `control_task` 中同步读取（每次控制循环调用 `imu_read_yaw()`）。

---

## 跟随避障算法详解（follow_avoid）

纯 C 实现（`components/control/follow_avoid/`），无 ESP/FreeRTOS 依赖，可在 PC 上单独单元测试。

### 坐标系

```
       +x （前进方向）
        ^
        |
  +y <--+      角度 0   = 正前方 (+x)
 （左侧）       角度 > 0 = 左侧（逆时针 CCW）
               角度 < 0 = 右侧（顺时针 CW）

  ω > 0 使机体左转（CCW），与底盘差速驱动一致。
```

后两轮为电机驱动轮（差速），前两轮为万向随动脚轮。**不倒车**——车尾无传感器，盲倒不安全。

### 状态机

```
          ┌──────────────────────────────────────┐
          │                                      │
          ▼                                      │
     ┌─────────┐   目标丢失 > 重捕超时    ┌──────────┐
     │  IDLE   │ ◄────────────────────── │ SEARCH   │
     │  停车    │    搜索超时(6s)无目标    │ 原地旋转  │
     └─────────┘                         └──────────┘
          ▲                                    │
          │  目标有效且畅通                      │ 目标重捕
          │                                    ▼
     ┌─────────┐   前向有障碍    ┌──────────┐
     │ FOLLOW  │ ──────────────► │  AVOID   │
     │ 直线跟随 │                 │ 绕行跟随  │
     └─────────┘                 └──────────┘
          │                            │
          │   净空 ≤ 急停距离            │   净空 ≤ 急停距离
          ▼                            ▼
     ┌──────────────────────────────────────┐
     │               ESTOP                   │
     │  刹停 + 朝较空侧原地旋转脱困            │
     └──────────────────────────────────────┘
```

**优先级**：ESTOP > SEARCH/IDLE > AVOID/FOLLOW。急停最高优先，确保安全。

### 跟随控制（FOLLOW 状态）

输入：UWB 解析出的目标 `{距离 d, 方位 β}`。

**线速度**（保持车距）：
- `d > 1.0m`：`v = kp_dist × (d − 1.0)`，追上去（`kp_dist = 0.9`）
- `0.75m ≤ d ≤ 1.0m`：`v = 0`，停止带内保持静止
- `d < 0.75m`：`v = 0`，太近，不倒车

**角速度**（对准目标）：`ω = kp_bear × β + kd_bear × dβ/dt`（`kp_bear = 1.6`, `kd_bear = 0.25`）

**转向-速度耦合**：`v_effective = v × (1 − |β| / 90°)`，转角越大前进越慢。

### VFH-lite 避障（AVOID 状态）

1. **构建极坐标直方图**：前向 ±90° 划分为 36 扇区（5°/扇区），记录最近障碍距离
2. **阻塞判定 + 膨胀**：距离 < `safe_distance_m`(0.6m) 的扇区判为阻塞，按 `asin(半宽/距离)` 向两侧膨胀
3. **航向选择**：`cost = w_goal × |扇区角 − 目标方位| + w_smooth × |扇区角 − 上次航向|`（`w_goal = 1.0`, `w_smooth = 0.35`）
4. **速度调速器**：用前向锥（±40°）和超声波的最近净空 `clearance` 线性压速
5. **超声波兜底**：`clearance ≤ 0.35m` → ESTOP 急停

### 加速度限幅

- 升速限幅：0.8 m/s²
- 降速限幅：2.0 m/s²（刹车比加速快）
- 角加速度限幅：6.0 rad/s²

---

## 闭环底盘详解（chassis）

### 1. 电机驱动：APO-DL ESC + RC PWM

两个 APO-DL 电调各接收一路 50Hz 航模舵机脉冲（LEDC 14 位分辨率）：

| 脉冲宽度 | 含义 |
|---------|------|
| 1000 µs | 全速后退 |
| 1500 µs | 中位 / 停止 |
| 2000 µs | 全速前进 |

### 2. 速度闭环：AB 编码器 + 轮速 PID

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

### 3. 差速驱动运动学

```
v_left  = (v − ω × track/2) / vmax
v_right = (v + ω × track/2) / vmax
```

饱和时等比例缩放保持转弯几何不变（proportional saturation）。

### 4. IMU 航向闭环

```
yaw_ref += ω · dt                            // 积分期望角速度
ω_cmd = ω + heading_kp × wrap(yaw_ref − yaw_meas)  // IMU 航向误差修正
```

仅在 FOLLOW / AVOID 状态启用；SEARCH / ESTOP 时关闭并重置参考。

### 5. 安全机制

| 机制 | 参数 | 说明 |
|------|------|------|
| 上电解锁延迟 | 2000ms | 上电后保持中位 2 秒等待电调自检 |
| 失控保护 | 0.3s | 控制任务卡死 → 自动拉回中位停车 |
| 里程计 | 编码器积分 | 航位推算 (x, y, yaw)，供调试扩展 |

---

## 构建与烧录

### 主项目：follow_robot（完整跟随机器人）

```bash
# 在项目根目录
idf.py set-target esp32s3
idf.py menuconfig   # 配置引脚、PID、跟随参数等
idf.py build
idf.py flash monitor
```

Kconfig 菜单位于 `Follow-me suitcase 算法2 (closed-loop follow + obstacle avoidance)` 菜单下，可配置：
- ESC 引脚/脉宽/反转
- AB 编码器引脚/反转/每米脉冲数
- 速度 PID 参数（KP/KI/KD/限幅）
- 跟随距离/停止带/最大速度
- 避障阈值（紧急/减速/安全距离）+ 机体半宽
- 传感器引脚（UWB/激光/超声波/IMU）
- IMU 航向闭环开关/增益/偏航取反
- 控制循环频率

### 子项目：sensor_hub（传感器测试）

```bash
cd examples/sensor_hub
idf.py set-target esp32s3
idf.py build flash monitor
```

传感器测试程序运行 7 个 FreeRTOS 任务并发读取全部传感器，**显式将超声波任务绑定 Core 0，其余任务绑定 Core 1**：

```c
// 软串口超声波（时序敏感，独占 Core 0）
xTaskCreatePinnedToCore(task_a02yyuw1, "a02_1", 4096, NULL, 4, NULL, 0);
xTaskCreatePinnedToCore(task_a02yyuw2, "a02_2", 4096, NULL, 4, NULL, 0);

// 雷达和 UWB（高速UART，独占 Core 1）
xTaskCreatePinnedToCore(task_rplidar, "rplidar", 4096, NULL, 3, NULL, 1);
xTaskCreatePinnedToCore(task_bu_uwb,  "bu_uwb",  4096, NULL, 2, NULL, 1);

// I2C/ADC 任务（低速，Core 1 共享）
xTaskCreatePinnedToCore(task_fsr,  "fsr",  4096, NULL, 1, NULL, 1);
xTaskCreatePinnedToCore(task_imu,  "imu",  4096, NULL, 2, NULL, 1);
xTaskCreatePinnedToCore(task_vl53l1x, "vl53", 4096, NULL, 2, NULL, 1);
```

---

## 关键参数速查

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `follow_distance_m` | 1.0m | 期望跟车距离 |
| `stop_band_m` | 0.25m | 停止带（0.75–1.0m 内保持静止） |
| `max_linear_mps` | 0.7 m/s | 最大前进速度 |
| `max_angular_rps` | 1.6 rad/s | 最大转向角速度 |
| `emergency_distance_m` | 0.35m | 急停触发距离 |
| `slow_distance_m` | 1.2m | 开始减速的距离 |
| `safe_distance_m` | 0.6m | 扇区阻塞判定距离 |
| `robot_half_width_m` | 0.22m | 机体半宽（膨胀阻塞扇区用） |
| `track_width_m` | 0.30m | 后轮距 |
| `ticks_per_meter` | 2000 | 编码器每米 4x 脉冲数（**必须标定**） |
| `max_speed_mps` | 0.8 m/s | 满速轮速 |
| `kp / ki / kd` | 200 / 300 / 5 | 速度 PID（us 每 m/s） |
| `pid_out_limit_us` | 400 µs | PID 单轮最大修正 |
| `heading_kp` | 1.5 rad/s per rad | 航向闭环增益 |
| `esc_min/mid/max_us` | 1000/1500/2000 | ESC 脉冲行程 |
| `control_hz` | 50 | 控制循环频率 |

更深层参数（`kp_dist=0.9 / kp_bear=1.6 / kd_bear=0.25`、加速度限幅、VFH 权重 `w_goal=1.0 / w_smooth=0.35`）定义在 `follow_avoid.c` 的 `fa_default_config()` 函数中，可在 `main.c` 的 `build_fa_config()` 中覆盖。

---

## 测试

传感器协议解析测试（纯 C 解析逻辑，无硬件依赖）可在 PC 运行：

```bash
bash tests/protocol/run_tests.sh
```

测试内容：A02YYUW 帧解析、BU UWB 距离行/TWR JSON 解析、FSR 线性标定公式。

---

## 引脚对照表

### follow_robot 项目（Kconfig 默认值，通过 `idf.py menuconfig` 可修改）

| 外设 | 接口 | 引脚（Kconfig 默认） | 备注 |
|------|------|------|------|
| 左 ESC | LEDC PWM | GPIO4 | 50Hz RC 脉冲，1000–2000µs |
| 右 ESC | LEDC PWM | GPIO5 | 50Hz RC 脉冲 |
| 左编码器 A | GPIO 中断 | GPIO6 | 4x 正交解码 |
| 左编码器 B | GPIO 中断 | GPIO7 | 4x 正交解码 |
| 右编码器 A | GPIO 中断 | GPIO15 | 4x 正交解码 |
| 右编码器 B | GPIO 中断 | GPIO16 | 4x 正交解码 |
| BU UWB | HW UART1 | RX=18, TX=37 | 115200 baud |
| RPLIDAR C1 | HW UART2 | RX=17, TX=9 | 460800 baud |
| 超声波左 | SW UART | RX=35 | 9600 baud，仅接 RX |
| 超声波右 | SW UART | RX=36 | 9600 baud，仅接 RX |
| IMU | I2C0 | SDA=39, SCL=38 | 地址 0x23 |

> 以上为 Kconfig `.default` 值。你的实际引脚以 menuconfig 中设置的为准。

### sensor_hub 项目（传感器修改6 验证引脚，硬编码在 main.c）

| 外设 | 接口 | 引脚 | 备注 |
|------|------|------|------|
| A02YYUW #1 | SW UART | RX=GPIO4 | 9600 baud，Core 0 |
| A02YYUW #2 | SW UART | RX=GPIO5 | 9600 baud，Core 0 |
| BU UWB | HW UART1 | RX=GPIO6, TX=GPIO7 | 115200 baud，Core 1 |
| FSR | ADC1 | GPIO8 (CH7) | 模拟输入，Core 1 |
| RPLIDAR C1 | HW UART2 | RX=GPIO17, TX=GPIO18 | 460800 baud，Core 1 |
| IMU | I2C0 | SDA=GPIO39, SCL=GPIO38 | 地址 0x23，Core 1 |
| VL53L1X | I2C0 共享 | SDA=GPIO39, SCL=GPIO38 | 地址 0x52(8位)，Core 1 |

---

## 目录结构

```
├── CMakeLists.txt                    # 根项目：follow_robot
├── README.md                         # 本文件
├── components/
│   ├── control/
│   │   ├── chassis/                  # 闭环底盘（ESC PWM + 编码器 PID + 前馈 + IMU 航向）
│   │   │   ├── chassis.c/h
│   │   │   └── CMakeLists.txt
│   │   └── follow_avoid/            # 跟随 + VFH-lite 避障算法（纯 C，无平台依赖）
│   │       ├── follow_avoid.c/h
│   │       └── CMakeLists.txt
│   └── sensors/
│       ├── a02yyuw/                  # A02YYUW 超声波（HW/SW UART 双模式 + 句柄式多实例）
│       ├── bu_uwb/                   # BU03/BU04 UWB 超宽带（TWR JSON 解析）
│       ├── fsr_adc/                  # FSR 薄膜压力（ADC 线性标定）
│       ├── imu_i2c/                  # 九轴 IMU（I2C 寄存器读取 + 四元数/欧拉角）
│       ├── rplidar_c1/              # RPLIDAR C1 激光雷达（5 字节扫描包状态机解析）
│       └── vl53l1x_tof/             # VL53L1X ToF 激光测距
├── examples/
│   ├── follow_robot/                 # ★ 主程序：完整跟随机器人
│   │   ├── CMakeLists.txt
│   │   ├── sdkconfig.defaults
│   │   └── main/
│   │       ├── CMakeLists.txt
│   │       ├── Kconfig.projbuild    # 全部参数配置菜单
│   │       └── main.c               # 多任务 RTOS + 控制循环（xTaskCreate，不绑定核心）
│   └── sensor_hub/                   # 传感器测试程序（7 路并发读取，显式核心绑定）
│       ├── CMakeLists.txt
│       ├── sdkconfig.defaults
│       └── main/
│           ├── CMakeLists.txt
│           ├── Kconfig.projbuild
│           └── main.c
├── docs/
├── tests/
│   ├── algorithm/                    # 算法 + PID 单元测试
│   └── protocol/                     # 传感器协议解析测试（PC 可运行）
├── outputs/
├── scripts/
├── work/
└── AGENTS.md
```

---

## 上电调试顺序

1. **垫高轮子空跑**：先确认电调解锁（上电后等待 2 秒 ESC 自检）
2. **标定编码器方向与每米脉冲**：手推 1m，看日志实测速度符号和增量
3. **标定满速轮速**：给最大前进，量实际轮速
4. **整定速度 PID**：先只留前馈（KI=KD=0）看跟随误差，再加 KI 消静差、KD 抑超调
5. **校航向闭环**：直线跑若往一侧偏，调整 `HEADING_KP_MILLI`；方向修反则开 `IMU_YAW_INVERT`
6. **校激光雷达零位**：`LIDAR_FORWARD_DEG` 对准机体正前方，`LIDAR_CW` 匹配扫描旋向
7. **校 UWB 左右**：目标方位方向反了则开 `UWB_LEFT_IS_POS_X`
8. 确认 FOLLOW/AVOID/ESTOP 切换与转向方向都正确后再落地测试

---

## 注意事项

1. **引脚区分**：`follow_robot` 引脚全部由 Kconfig 决定；`sensor_hub` 引脚硬编码在 main.c 中。两个项目独立，引脚不互用
2. **任务核心分配**：`follow_robot` 使用 `xTaskCreate`（FreeRTOS SMP 自动调度到双核）；`sensor_hub` 使用 `xTaskCreatePinnedToCore` 显式绑定
3. **RPLIDAR 需 5V/800mA 外部供电**，TX/RX 交叉接线
4. **每个超声波只接 RX**，A02YYUW 自主输出，TX 不接
5. **编码器 `TICKS_PER_METER` 必须标定**：推机器人恰好 1 米，记录日志中编码器计数差值
6. **FSR 公式未标定**，使用前需实测标定
7. **GPIO39 作 SDA** 仅 ESP32-S3 支持（经典 ESP32 的 GPIO34–39 仅为输入）
8. **构建 follow_robot** 在项目根目录运行 `idf.py` 命令；构建 `sensor_hub` 需进入 `examples/sensor_hub/` 目录
9. **无后向感知**：算法刻意不倒车，过近只停车
10. **UWB 坐标约定**：跟随方位由 `Xcm/Ycm` 算得，若左右颠倒则打开 `UWB_LEFT_IS_POS_X`
11. **失效保护**：控制任务卡死会在 0.3 秒内自动停车
