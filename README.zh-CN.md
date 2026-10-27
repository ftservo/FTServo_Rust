[English](README.md) | **简体中文**

# ftservo · 飞特（FEETECH）总线舵机与 IMU 的 Rust SDK

原生 Rust 实现的总线 SDK，覆盖官方 Python SDK 中的 **SMS/STS、SCSCL、HLS/兼容 HTS 系列舵机**以及**总线 IMU**，提供普通读写、同步读写、REG_WRITE/ACTION、反馈字段读取、RESET 指令、EPROM 锁、HLS 偏移校准、IMU 四元数/陀螺仪/加速度等功能。

## 支持的家族

| 家族 | 字节序 | 覆盖能力 | 限制 |
| --- | --- | --- | --- |
| SMS/STS | 小端 word | 位置/速度写（含加速度字段）、同步读写、轮式模式 | 力矩模式不适用；`write_speed` 要求 torque 字段为 0 |
| SCSCL | **大端** word | 位置写、PWM 模式与 PWM 输出、同步写 | 无模式寄存器、**无同步读**；位置为无符号 |
| HLS / 兼容 HTS | 小端 word | 位置/速度写（含加速度、力矩字段）、同步读写、轮式模式、偏移校准 | — |
| 总线 IMU | 小端 word | 四元数、陀螺仪、加速度 | 四元数 XYZ 为 binary16（半精度浮点 S1E5F10），W 由非负根重建 |

## 快速开始

在本目录构建（默认开启 `serial`）：

```sh
cargo build --locked
cargo test --locked
```

在应用 `Cargo.toml` 中以本地路径引入（本 crate 尚未发布到 crates.io）：

```toml
[dependencies]
ftservo = { path = "../FTServo_Rust" }

# 只使用自定义 Transport，不引入 serialport：
# ftservo = { path = "../FTServo_Rust", default-features = false }
```

最小可用示例：

```rust
use ftservo::{decode_sign_magnitude, registers, Batch, Bus, Imu, Motion, Request, Servo};
use std::time::Duration;

fn main() -> ftservo::Result<()> {
    // 一个物理串口只创建一个 Bus；clone 共享串口与事务锁，不会重复打开。
    // 第三个参数是**写方向**预算；读超时由协议算出，见 Bus::command。
    let bus = Bus::open("COM3", 1_000_000, Duration::from_millis(100))?;
    let servo = Servo::sms_sts(bus.clone());
    let imu = Imu::new(bus.clone());
    // Request 只承载取消，不含超时；读的时限来自协议。
    let request = Request::new();

    // 写目标位置：SDK 不自动使能力矩，设备需已处于正确模式并允许输出
    servo.write_position(&request, Motion::new(1, 2048))?;

    // 多设备同步读取：事务由 Bus::sync_read_decoded 完成，15 字节反馈块的解析由应用负责
    // ——块内布局是家族约定（字节序、方向位、字段偏移），库不替调用者猜。这里只取位置两字节
    // 作示意；完整解析见 examples/hls/sync_read.rs 或 examples/sms_sts/sync_read.rs，
    // 以及 examples/imu/sync_read.rs（首个 ID 是 IMU，其余是 HLS 舵机，共用地址 56）。
    let mut batch = Batch::new();
    bus.sync_read_decoded(
        &request,
        registers::PRESENT_POSITION,
        15,
        &[1, 2],
        |p| decode_sign_magnitude(u16::from_le_bytes([p[0], p[1]]), 15),
        &mut batch,
    )?;
    for (id, position) in &batch.values {
        println!("servo {id}: position={position}");
    }
    if !batch.is_ok() {
        println!("errors={:?} missing={:?}", batch.errors, batch.missing);
    }

    // IMU：四元数由二分之一精度浮点转换而来，W 由非负根重建；陀螺仪/加速度各自单读
    let quaternion = imu.read_quaternion(&request, 3)?;
    println!("quaternion: {quaternion:?}");
    let gyro = imu.read_gyro(&request, 3)?;
    println!("gyro raw: {gyro:?} signed: {:?}", gyro.signed());

    bus.close()?;
    Ok(())
}
```

## 示例

```sh
# 连通性探测：不带 ID 扫描 0..=253，命中后自动读固件版本/型号（地址 0..=4）
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

# HLS（运动参数含力矩字段；ofscal 为偏移校准；reset 为设备级 RESET 指令，HLS 上表示状态/圈数复位而非出厂复位）
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
cargo run --example imu-sync_read -- COM3 200 10 11   # 首个 ID 是 IMU，其余是同一总线上的 HLS 舵机
```

## 架构

| 模块 | 职责 |
| --- | --- |
| `src/protocol.rs` | 编解码、状态帧、ID/包长/寄存器范围校验、字段编解码（方向位、半精度浮点）、`Device`（家族字节序、原始寄存器读写、写响应策略、EPROM 锁、ping/reset） |
| `src/bus.rs` | `Transport` 抽象、共享 `Bus`、完整事务锁、`Request` 取消、`Batch` 部分结果 |
| `src/servo.rs` | 家族规格表、运动参数与校验、三类舵机控制、单字段反馈读取 |
| `src/imu.rs` | 间隔布局、四元数（binary16 转换、W 重建）、陀螺仪与加速度读取 |
| `src/serial.rs` | 可选 `serialport` 适配层（平台逻辑不侵入协议层） |
| `src/error.rs` / `src/registers.rs` | 结构化错误与有依据的寄存器表 |
| `tests/sdk.rs` | 基于模拟传输的无硬件回归测试 |
