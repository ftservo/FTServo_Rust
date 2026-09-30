# ftservo · 飞特（FEETECH）总线舵机与 IMU 的 Rust SDK

原生 Rust 实现的总线 SDK，覆盖官方 Python SDK 中的 **SMS/STS、SCSCL、HLS/兼容 HTS 系列舵机**以及**总线 IMU**，提供普通读写、同步读写、REG_WRITE/ACTION、反馈解析、EPROM 锁、HLS 重置与偏移校准、IMU 四元数/陀螺仪/加速度等功能。

```text
协议编解码 ┃ 同步阻塞 ┃ 无 unsafe ┃ 零第三方运行依赖（关闭 serial 时）
```

## 支持的家族

| 家族 | 字节序 | 覆盖能力 | 限制 |
| --- | --- | --- | --- |
| SMS/STS | 小端 word | 位置/速度/加速度写、反馈、同步读写、轮式模式 | 力矩模式不适用；`write_speed` 要求 torque 字段为 0 |
| SCSCL | **大端** word | 位置写、反馈、PWM 模式与 PWM 输出 | 无模式寄存器、**不支持同步读**；位置为无符号 |
| HLS / 兼容 HTS | 小端 word | 位置写（含力矩字段）、反馈、同步读写、轮式模式、偏移校准、状态复位 | — |
| 总线 IMU | 小端 word | 四元数、陀螺仪、加速度、合并采样、多设备同步采样 | 四元数 XYZ 为 binary16，W 由非负根重建 |

**单位保持寄存器原始值**：位置、速度、电流、电压、温度、量程配置均不做物理量换算；IMU 的 `.signed()` 只解析方向位，不转换为 °/s 或 g。

## 快速开始

在本目录构建（默认开启 `serial`）：

```sh
cargo build --locked
cargo test --locked
```

在应用 `Cargo.toml` 中以本地路径引入（本 crate 尚未发布到 crates.io）：

```toml
[dependencies]
ftservo = { path = "../rust-sdk" }

# 只使用自定义 Transport，不引入 serialport：
# ftservo = { path = "../rust-sdk", default-features = false }
```

最小可用示例：

```rust
use ftservo::{Bus, Imu, Motion, Request, Servo};
use std::time::Duration;

fn main() -> ftservo::Result<()> {
    // 一个物理串口只创建一个 Bus；clone 共享串口与事务锁，不会重复打开
    let bus = Bus::open("COM3", 1_000_000, Duration::from_millis(100))?;
    let servo = Servo::sms_sts(bus.clone());
    let imu = Imu::new(bus.clone());
    let request = Request::with_timeout(Duration::from_secs(1))?;

    // 写目标位置：SDK 不自动使能力矩，设备需已处于正确模式并允许输出
    servo.write_position(&request, Motion::new(1, 2048))?;

    // 批量读取：values 保留本次有效结果，errors/missing 表示故障与未应答
    let batch = servo.sync_read_feedback(&request, &[1, 2])?;
    for (id, feedback) in &batch.values {
        println!("servo {id}: {feedback:?}");
    }
    if !batch.is_ok() {
        println!("errors={:?} missing={:?}", batch.errors, batch.missing);
    }

    // IMU：原始值 + 方向位解码值
    let sample = imu.read_sample(&request, 3)?;
    println!("quaternion: {:?}", sample.quaternion);
    println!("gyro raw: {:?} signed: {:?}", sample.gyro, sample.gyro.signed());

    bus.close()?;
    Ok(())
}
```

串口名：Windows 用 `COM3`，Linux 用 `/dev/ttyUSB0`，macOS 用实际 `/dev/cu.*`。适配器需自行处理半双工收发方向，SDK 不切换 RTS。

## 示例

`examples/` 按设备组织，共 24 个入口，与官方 Python 示例一一对应。**默认只编译不执行**；`write`/`sync_write`/`reg_write`/`wheel` 会发送真实运动命令。

```sh
# 连通性探测：不带 ID 扫描 0..=252，命中后自动读固件版本/型号（地址 0..=4）
cargo run --example ping -- COM3
cargo run --example ping -- COM3 1            # 单 ID
cargo run --example ping -- COM3 0 1 2 3      # 多个 ID

# SMS/STS
cargo run --example sms_sts-read -- COM3 1
cargo run --example sms_sts-write -- COM3 1
cargo run --example sms_sts-reg_write -- COM3 1 2048 300 20
cargo run --example sms_sts-sync_read -- COM3 1 2
cargo run --example sms_sts-sync_write -- COM3 1 2
cargo run --example sms_sts-wheel -- COM3 1 300 20

# SCSCL（大端；wheel 走 PWM，无同步读）
cargo run --example scscl-read -- COM3 1
cargo run --example scscl-write -- COM3 1
cargo run --example scscl-reg_write -- COM3 1 500 1500
cargo run --example scscl-sync_write -- COM3 1 2
cargo run --example scscl-wheel -- COM3 1 1000

# HLS（运动参数含力矩字段；ofscal 为偏移校准，reset 为状态/圈数复位而非出厂复位）
cargo run --example hls-read -- COM3 1
cargo run --example hls-write -- COM3 1
cargo run --example hls-reg_write -- COM3 1 2048 300 20 500
cargo run --example hls-sync_read -- COM3 1 2
cargo run --example hls-sync_write -- COM3 1 2
cargo run --example hls-wheel -- COM3 1 300 20
cargo run --example hls-ofscal -- COM3 1 2048
cargo run --example hls-reset -- COM3 1

# IMU
cargo run --example imu-read_acc -- COM3 2
cargo run --example imu-read_gyro -- COM3 2
cargo run --example imu-read_slfp -- COM3 2
cargo run --example imu-sync_read -- COM3 2 3
```

示例约定：

- `read` / `sync_read` 系列为 **50Hz 连续读取**（每 20ms 一轮），无应答或故障只累计计数、不退出循环；每轮打印数据与实测频率（`rounds=`/`failures=`/`rate=`），按 Ctrl+C 停止。控制台渲染可能成为吞吐瓶颈（`rate=` 会如实反映），需要真实 50Hz 时可将输出重定向到文件。
- `imu-sync_read` 对非最后一个 ID 只显示本轮读取结果，最后一个 ID 输出完整采样；舵机的 `sync_read` 对全部 ID 输出逐字段解释。
- `write` / `sync_write` 将上游的 `while 1` 无限往复改为有限次（3 次）；均不调用 `enable_torque`，与上游脚本一致——设备未使能力矩时不会运动。

## 架构

| 模块 | 职责 |
| --- | --- |
| `src/protocol.rs` | 编解码、状态帧、ID/包长/寄存器范围校验 |
| `src/bus.rs` | `Transport` 抽象、共享 `Bus`、完整事务锁、`Request` 取消/截止、`Batch` 部分结果 |
| `src/device.rs` | 不可变家族字节序、原始寄存器读写、写响应策略、EPROM 锁 |
| `src/servo.rs` | 家族、运动参数、三类舵机控制与反馈解码 |
| `src/imu.rs` / `src/encoding.rs` | 间隔布局、四元数、方向位、binary16 |
| `src/serial.rs` | 可选 `serialport` 适配层（平台逻辑不侵入协议层） |
| `src/error.rs` / `src/registers.rs` | 结构化错误与有依据的寄存器表 |
| `tests/sdk.rs` | 基于模拟传输的无硬件回归测试 |

## 协议与并发约定

1. **一个物理串口只创建一个 `Bus`**；`clone` 共享串口与事务锁，不重新打开端口，不旁路读取端口。
2. SMS/STS、HLS、IMU 为**小端 word**，SCSCL 为**大端 word**；`u32` 遵循“低 word 在前、每个 word 按家族字节序”，SCSCL 因此**不是**常规大端 u32。
3. **方向位不是二补码**：位置（非 SCSCL）、速度、电流一般用 bit15，负载/PWM 用 bit10；不要用 `as i16` 直接解释设备值。
4. 单播 ID 为 `0..=252`，`254` 为广播，`253`/`255` 保留；广播 Ping/Read 非法，同步写无逐设备 ACK。
5. 包最大 250 字节；参数与家族限制在**任何串口 I/O 之前**校验，不自动拆包。
6. 不自动重试可能已生效的写操作，不隐式启动力矩、切换模式、解锁 EPROM、校准或重置。
7. `Request` 的取消是协作式的，不能中断任意阻塞的驱动调用。

## 验证

```sh
cargo fmt --check
cargo test --locked
cargo test --locked --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo clippy --locked --all-targets --no-default-features -- -D warnings
cargo build --locked --examples
cargo doc --locked --no-deps
```

关闭默认 feature 时核心协议仅依赖标准库；`serialport` 的默认 libudev feature 已关闭，按已知设备路径打开串口无需 Linux libudev 开发包。
