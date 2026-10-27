---
name: FTServo_Python 对齐改造建议
overview: 以官方 FTServo_Python SDK 为基准逐层核对 Rust 库（协议层、三族舵机、IMU），输出分级修改建议：注释纠错、小范围行为对齐、报文级 parity 测试、README 差异表，并明确列出不应照抄的 Python 缺陷。
todos:
  - id: fix-python-contradicted-docs
    content: 更正三处注释：servo.rs 的 SCSCL PRESENT_CURRENT、imu.rs 的 LOCK、protocol.rs 的 MAX_ID，均写明 Python 源依据
    status: completed
  - id: align-offset-and-reg-action
    content: calibrate_offset 改用 self.order.bytes()，新增 Servo::reg_action() 广播 ACTION，并同步三个 reg_write 示例
    status: completed
    dependencies:
      - fix-python-contradicted-docs
  - id: add-parity-tests
    content: 在 tests/sdk.rs 增加三族完整报文 parity 用例，覆盖 write/reg_write/sync_write/action/SCSCL PWM/calibrate_offset
    status: completed
    dependencies:
      - align-offset-and-reg-action
  - id: document-python-diff
    content: 在 README 新增「与官方 Python SDK 的对照」：报文布局表、保留的严格语义、不照抄的缺陷清单
    status: completed
    dependencies:
      - add-parity-tests
  - id: verification-checklist
    content: 整理验证清单（cargo test --locked / cargo check --all-targets）与逐项比对结论，交用户在本机执行确认
    status: completed
    dependencies:
      - document-python-diff
---

## 用户需求

以官方 Python SDK（https://github.com/ftservo/FTServo_Python，main 分支）为基准，逐层比对当前 Rust 库，给出**可落地的修改建议**：哪些必须改、哪些只补文档、哪些不要照抄。

## 产品/库范围

- 协议层：帧格式、校验和、ID/长度校验、字节序、超时与重同步语义
- 设备层：SMS/STS、SCSCL、HLS 三族舵机的寄存器表与写帧布局，IMU 的寄存器表与读取长度
- 交付物：分级建议清单 + 对应代码/注释/测试/文档改动
- 边界：不重写架构、不引入新依赖、公共 API 只做增量（`Error` 已 `#[non_exhaustive]`）、保持“整事务锁 + `sync_read_decoded` 零分配”现状
- 视觉/交互：无（纯库代码）

## 核心结论（已核实）

1. 三族写帧与 Python 完全一致：SMS/STS 7 字节 `[acc,pos,0,0,speed]`@41、HLS 7 字节 `[acc,pos,torque,speed]`@41、SCSCL 6 字节大端 `[pos,time,speed]`@42、SCSCL PWM 走 44 且 sign-bit 10，故**无需改协议帧**。
2. 真正需要改的是三处与 Python 源矛盾的注释、`calibrate_offset` 的字节序一致性、REG_WRITE 的 ACTION 广播语义，以及缺失的报文级 parity 测试与“与 Python 的差异”文档。
3. `MAX_ID=0xFD` 的判断与 Python 运行时检查（`rxPacket` 用 `ID > 0xFD` 判非法、`scservo_def.MAX_ID=0xFC` 为死常量）一致，只需把依据写进注释。

## 技术栈

Rust 2021 / MSRV 1.85，依赖不变（仅可选 `serialport = "=4.10.1"`），`#![forbid(unsafe_code)]` 保持；测试沿用 `tests/sdk.rs` 的 mock `Transport` 模式。

## 实施策略

按“改代码 → 补测试 → 写差异文档”三层推进，**不为对齐而破坏 Rust 已有优势**：

- **A 类：必须改（与 Python 源直接矛盾）**
- `src/servo.rs` scscl `PRESENT_CURRENT` 注释：Python `scscl.py` 明确有 `SCSCL_PRESENT_CURRENT_L/H = 69/70`，删除“Not in the official memory map”说法并改为“官方表有，历史值一致”。
- `src/imu.rs` `LOCK` 注释：Python `imu.py` 明确有 `IMU_LOCK = 55`，同样更正。
- `src/protocol.rs` `MAX_ID` 注释：补上依据——Python `rxPacket` 预筛用 `ID > 0xFD`，`scservo_def.MAX_ID = 0xFC` 在 Python 内未被任何检查引用。
- **B 类：小范围一致性/易用性**
- `Servo::calibrate_offset` 改用 `self.order.bytes(position)`，与 Python `reOfsCal` 的 `scs_lobyte/scs_hibyte` 语义一致（当前仅 HLS 可用，属预防性修正）。
- 新增 `Servo::reg_action(&Request)`：向 `BROADCAST_ID` 发 ACTION，等价 Python 各家族的 `RegAction()`；并把 `examples/{sms_sts,hls,scscl}/reg_write.rs` 由单播 `action(&r, id)` 改为 `reg_action`，多机预置才符合 REG_WRITE 原意。
- **C 类：只文档化，不改行为**
- 单播事务遇外来 ID 帧：Python 丢弃后继续等到超时；Rust 立即 `Error::UnexpectedId`。Rust 每事务清空接收缓冲，外来帧意味着总线错序，严格更可诊断 —— 保留现状，在 README 差异表与 `Error::UnexpectedId` 文档中说明；现场若被“迟到帧”困扰再提供跳过策略。
- Python 的 `readTx/readRx` 分离式读写、15 字节反馈块整体解析：明确列为不支持/有意不做。
- **D 类：明确不照抄的 Python 缺陷**（写入 README，防止后来者“对齐”回去）
`is_using` 布尔锁非线程安全；`syncReadRx` 不校验校验和/不按 ID 匹配；`read1ByteRx` 失败返回 0 无法区分；`writeTxOnly` 与 `writeTxRx` 返回结构不一致；`hls.WritePosEx` 误用 `scs_tohost` 编码负数（靠大整数截断偶然正确）；`ping/readTx` 只校验 `> BROADCAST_ID`；`rxPacket` 找包头残留 `idx` 的越界隐患；`MAX_ID` 常量与自身检查不一致。

## 报文对照（用作测试向量，均已逐字核对）

| 家族 | 方法 | 起始地址 | 报文 |
| --- | --- | --- | --- |
| SMS/STS | WritePosEx / SyncWritePosEx / RegWritePosEx | 41 | 7 字节 `[acc, posL, posH, 0, 0, speedL, speedH]` |
| HLS | 同上 | 41 | 7 字节 `[acc, posL, posH, torqueL, torqueH, speedL, speedH]` |
| SCSCL | WritePos / SyncWritePos / RegWritePos | 42 | 6 字节大端 `[pos, time, speed]` |
| SCSCL | WritePWM | 44 | 2 字节大端，`scs_toscs(v,10)` |
| SCSCL | PWMMode | 9 | 4 字节 `[0,0,0,0]` |
| 全部 | RegAction | — | ACTION(0x05) → 广播 0xFE |


## 执行要点

- 注释改动必须同时给出 Python 侧依据（文件名 + 常量名），避免下一次又被“看起来像错的”注释改回去。
- `reg_action` 属新增 API，`Servo` 已有 `Deref<Target = Device>`，实现只需一行广播 ACTION，不新增字段、不影响 `Clone`。
- 示例改广播 ACTION 后仍只操作已 REG_WRITE 的设备，行为安全；README 命令不变。
- 测试沿用现有 `Mock`：按 `tx[3]+4` 切包、`state.packets` 保留原始字节，可直接断言 `packets[0][5..]` 的地址/数据段。
- README 差异表放在“架构”之后，新增“与官方 Python SDK 的对照”一节，避免污染快速开始。
- 本机 `cargo` 不在 PATH，改动后需用户在 Rust 1.85+ 环境执行 `cargo test --locked`（或 `cargo check --all-targets`）确认；计划中不包含在此环境跑构建。

## 架构与目录

不改分层（`serial` → `bus` ↔ `protocol` → `servo`/`imu`），不改 `Bus` 事务模型与 `Batch` 复用契约，仅动注释、一个字节序参数、一个便捷方法、三个示例、测试与文档。

```
FTServo_Rust/
├── src/
│   ├── protocol.rs   # [MODIFY] MAX_ID 注释补 Python 依据（rxPacket 的 ID > 0xFD；scservo_def.MAX_ID 为死常量）
│   ├── servo.rs      # [MODIFY] scscl PRESENT_CURRENT 注释纠错；calibrate_offset 改 self.order.bytes()；新增 reg_action()
│   └── imu.rs        # [MODIFY] LOCK 注释纠错（Python 有 IMU_LOCK = 55）
├── examples/
│   ├── sms_sts/reg_write.rs  # [MODIFY] 改用 reg_action()（广播 ACTION）
│   ├── hls/reg_write.rs      # [MODIFY] 同上
│   └── scscl/reg_write.rs    # [MODIFY] 同上
├── tests/
│   └── sdk.rs        # [MODIFY] 新增 parity 用例：三族 write/reg_write/sync_write/action 完整报文、SCSCL PWM 大端、calibrate_offset 字节序、reg_action 广播
└── README.md         # [MODIFY] 新增「与官方 Python SDK 的对照」：报文布局表、保留的严格行为、不照抄的缺陷清单
```

## 关键签名（新增）

```rust
impl Servo {
    /// Python 各家族 `RegAction()` 的等价物：ACTION 广播，执行所有已 REG_WRITE 的设备。
    pub fn reg_action(&self, r: &Request) -> Result<()>;
}
```