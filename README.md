# 智能跟随行李箱 · follow_only

UWB 跟随 + IMU 航向闭环 + 底盘闭环 + WiFi 调试网页，无避障。

---

## 系统架构

```
ESP32-S3
├── uwb_task (优先级6)     ← UART1 接收 UWB 目标定位
├── control_task (优先级7) ← 50Hz: 跟随算法 + IMU 航向 + 底盘驱动
├── flash_log_task (优先级2) ← 异步写 CSV 到 SPIFFS
└── HTTP server (WiFi SoftAP)
    ├── GET /live          ← 实时遥测 JSON
    ├── POST /cmd          ← 遥控命令 (2D)
    ├── POST /estop        ← 急停
    ├── POST /clear        ← 解锁 ARM
    ├── POST /hb           ← 心跳
    └── GET /log           ← 下载 CSV 日志
```

**传感器：**
- UWB (BU0x) — 目标距离 + 方位，EMA 滤波 + 跳点剔除
- IMU (I2C) — 航向角闭环修正
- 编码器 (GPIO 中断) — 4x 正交解码，轮速 PID

**执行器：**
- APO-DL ESC × 2 — RC PWM 50Hz (GPIO4/5)
- 差速驱动运动学

---

## 安全机制

| 保护 | 行为 |
|------|------|
| E-STOP 按钮 | 锁存停车，需重新 CLEAR/ARM |
| WiFi 断连 | 自动锁存 E-STOP |
| Heartbeat 超时 (1.5s) | 自动锁存 E-STOP |
| 命令超时 (500ms) | 目标速度归零 |
| ARM 后未回零 | 等待零命令后才允许运动 |
| 失控保护 | 控制卡死 0.3s → 中位停车 |

---

## 跟随状态机

```
IDLE ←─ 搜索超时(6s) ─── SEARCH ── 目标丢失>0.5s ──→ FOLLOW
 ▲                                                      │
 └──────────────────── 目标重捕 ◄────────────────────────┘
```

- **FOLLOW**: forward gap P 控制线速度 + bearing P 控制角速度 + IMU 航向闭环 + 转向减速
- **SEARCH**: 向最后已知方向旋转搜索
- **IDLE**: 停车

---

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py menuconfig    # 配置引脚、PID、跟随参数
idf.py build
idf.py flash monitor
```

### 关键参数 (Kconfig)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `follow_distance_mm` | 1000 | 期望跟车距离 |
| `max_linear_mmps` | 700 | 最大前进速度 |
| `kp_dist` / `kp_bear` | 900 / 1600 | 距离/方位 P 增益 (milli) |
| `heading_kp_milli` | 1500 | IMU 航向闭环增益 |
| `ticks_per_meter` | 2000 | 编码器每米脉冲数 (**必须标定**) |
| `kp / ki / kd` | 200 / 300 / 5 | 速度 PID |
| `control_hz` | 50 | 控制循环频率 |

---

## 引脚

| 外设 | GPIO | 说明 |
|------|------|------|
| 左 ESC | 4 | RC PWM 50Hz |
| 右 ESC | 5 | RC PWM 50Hz |
| 左编码器 A/B | 6/7 | 4x 正交解码 |
| 右编码器 A/B | 15/16 | 4x 正交解码 |
| UWB RX/TX | 18/37 | UART1 115200 |
| IMU SDA/SCL | 39/38 | I2C0 |

---

## 目录结构

```
├── CMakeLists.txt                         # 根项目
├── components/
│   ├── control/chassis/                   # 闭环底盘
│   ├── debug/web_debug/                   # WiFi 调试组件
│   └── sensors/{bu_uwb, imu_i2c}/         # 传感器驱动
└── examples/
    ├── follow_only/                       # ★ 跟随固件
    └── remote_box/                        # ★ 遥控固件 (无传感器)
```

---

## 注意事项

1. 编码器 `TICKS_PER_METER` 必须物理标定：推机器人 1m，读日志 tick delta
2. GPIO39 作 SDA 仅 ESP32-S3 支持
3. 无后向感知：算法不倒车，过近只停车
4. 传感器降级：编码器掉线→前馈开环，IMU 掉线→跳过航向闭环
5. 遥控命令通过 HTTP POST /cmd 发送，安全门控在 control_task 内统一仲裁
