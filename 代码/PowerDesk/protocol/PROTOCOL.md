# 1.4.0 / MCU 0.3.0 栅极实测与受限 Buck 扩展

当前 schema CRC 为 **CB716592**，协议帧版本仍为 V2。旧 CRC A67D8A3A（MCU 0.2.2）与 EA47CA7E 均为历史版本；新增 GATE_TEST 改变了 schema，MCU 与上位机必须同时更新，否则握手阶段直接拒绝控制。

**GATE_TEST opcode=0x24，CH1，非零 seq。** 请求 `gate_test_request` 为 4 字节：action:u8、index:u8、duration_ms:u16（小端）。

| action | index | duration_ms | 语义 |
|---:|---|---|---|
| 0 | 0 | 0 | 查询 `gate_test_status` |
| 1 | 1..4 | 100..2000 | 驱动该输出所在的**整个半桥**：被选中的输出 20% 窄脉冲，同一桥的另一路 80%（低侧窗口每周期把开关节点拉到地，UCC27301A 内部自举二极管才能给高侧充电）。1=PA8/CHA1，2=PA9/CHA2，3=PA10/CHB1，4=PA11/CHB2；到时自动关闭 |
| 2 | 0..3 | 0 | 记录实测映射并解锁功率级启动；index 即映射码 |
| 3 | 0 | 0 | 清除映射确认，重新上锁 |

判据：窄脉冲（20%）落在哪颗 MOS 栅极，那颗就是 index 对应的器件。

映射码：0=PA8→输入桥高侧、PA10→输出桥高侧（与原理图/网表一致）；1=实测输入桥到达低侧；2=实测输出桥到达低侧；3=两桥都到达低侧。网表与 UCC27301A 数据手册（D 封装：3=HO、8=LO）已确认实物设计就是映射码 0，因此 **只有 0 能解锁功率级**；1/2/3 只是"实物与网表不一致"的诊断结果，固件会保持上锁（HO 以开关节点为参考，接到低侧栅极会过压），此时必须先查线，不要上电。

所有 action 的成功 ACK 返回 6 字节 `gate_test_status`：confirmed:u8、mapping:u8、active_index:u8、running:u8、remaining_ms:u16。

安全联锁：action=1 仅在输出已关闭、无故障、采样健康、**VIN < 5 V 且 VOUT < 1 V** 时执行；否则 NACK NOT_READY 并写日志。该过程只接通 +12 V 驱动供电，不接通 20 V 功率输入。功率级 OUTPUT=1 在映射未确认（action=2）或 CH1 输出电流未完成实测校零时返回 NOT_READY。

# 1.3.0 / MCU 0.2.2 校零扩展

当前 schema CRC 为 **A67D8A3A**，协议帧版本仍为 V2。新增字段改变 schema，必须同步更新 MCU 的生成头文件；下文旧 CRC EA47CA7E 仅是历史版本。

CH1 CAPS 新增 ZERO_CALIBRATION=64；CH2 和演示设备不声明支持。原 NOMINAL_CALIBRATION 保留，因为零点修正不代表增益/VREF 已完整标定。

ZERO opcode=0x23，CH1，非零 seq。请求 `zero_request` 为 1 字节 action：0 查询；1 开始输出电流校零；2 开始电感电流校零；3 恢复输出电流标称零点；4 恢复电感电流标称零点。其他 action 返回 RANGE，CH2 返回 UNSUPPORTED。修改操作需停机、正常模式和板级校验通过；进行中返回 BUSY。命令超时不自动重发。

所有成功 ACK 返回 24 字节 little-endian `zero_status`：state:u8、sensor:u8、applied:u8、reason:u8、samples:u16、reserved:u16=0、iout_uv:u32、il_uv:u32、pp_uv:u32、span_uv:u32。state：0 未采集/恢复标称、1 等待稳定、2 采样、3 已应用、4 失败/取消。sensor：0 未选择、1 IOUT、2 IL。applied bit0 表示 IOUT 有实测零点，bit1 表示 IL 有实测零点。reason：0 无、1 条件不满足、2 采样或均值越界、3 波动/分段变化过大、4 超时、5 停止或故障取消。

iout_uv/il_uv 是当前应用零点；失败时仍返回之前有效值。pp_uv 是本次采样峰峰值，span_uv 是八段各 512 点均值的最大跨度，非测量标准差。未完成时统计字段可为零。起始 ACK state=1 只表示已受理；每 500 ms 查询 action=0，必须等 state=3 才报告成功。结果只存 MCU RAM，复位恢复默认；不修正输入电流零点。

LOG code 13 为校零辅助日志，可能受发送队列丢帧影响；ZERO 状态回读是结果依据。校零期间拒绝 SET_MODE、TRACE_ARM，正在采集快照时拒绝修改零点。STOP/故障会取消校零且不应用未完成结果。

# PowerDesk STM32 串口协议 V2

本文是本次上位机及后续 MCU 的共同契约。原硬件交接文件中的文本命令只是旧建议，此处二进制协议优先。字段布局由同目录 `schema.json` 定义，任何不兼容字段或语义变更都必须提升协议版本、重新生成头文件和测试向量，更新双方软件。

## 1. 物理层和数据类型

PC USB → CH340K → USART3 PC10 TX / PC11 RX；115200 baud、8 数据位、无校验、1 停止位、无流控。DTR/RTS 不用于复位或功率使能。不配置 STM32 USB CDC。

所有多字节字段 **little-endian，无填充、无对齐空洞**。schema 类型：B=u8，H=u16，h=i16，I=u32，i=i32，f=IEEE754 binary32；Ns=N 字节 ASCII，字符串用 0 填充，可恰好占满 N 字节。负数使用二补码。禁止 NaN/Inf；测量无效用 valid 位表达，不使用浮点 NaN 或特殊整数。

## 2. 帧格式

| 字节偏移 | 长度 | 内容 |
|---|---:|---|
| 0 | 2 | 同步字 A5 5A |
| 2 | 1 | version=2 |
| 3 | 1 | kind：1 REQUEST，2 ACK，3 NACK，4 EVENT |
| 4 | 2 | seq |
| 6 | 1 | opcode |
| 7 | 1 | channel：0 系统，1/2 输出通道 |
| 8 | 2 | payload length，0–512 |
| 10 | N | payload |
| 10+N | 2 | CRC16，低字节先发 |

CRC16/CCITT-FALSE：poly=0x1021，init=0xFFFF，refin=false，refout=false，xorout=0；覆盖 **偏移 2 到 9+N**，不包括同步字和 CRC。`123456789` 的 CRC 应为 0x29B1。

完整帧共 N+12 字节。接收端用流式解析器处理任意拆分、粘连、噪声；先检查版本、长度、kind、channel，再 CRC。错误帧丢弃，不执行命令；找下一个同步字。半帧字节间隔超 250 ms 清除未完成帧。连续正常遥测期间的损坏长度最多等待其声明的有界帧长，再校验失败重同步。输入队列必须有容量上限，溢出丢弃并报告诊断计数。

普通请求 seq=1…65535 递增回绕；ACK/NACK 必须原样返回 seq/opcode/channel。EVENT 使用 seq=0。**仅 HEARTBEAT 与 STOP_ALL 允许 REQUEST seq=0，表示无需应答；其他请求 seq=0 拒绝执行。** 正常主机操作使用非零 seq（心跳和会话失效时的尽力停机除外）。

命令默认等待 800 ms，超时不自动重发。NACK payload 固定 `error`（u16 code）。成功 ACK payload 见命令表，不得把字符串日志混入二进制串口。未知 opcode 返回 UNSUPPORTED。CRC 不符不回复，避免噪声引发应答风暴。

## 3. 命令表

布局名对应 schema.json；空表示 payload length=0。

| opcode | 名称 | channel | 请求布局 | ACK 布局 | 语义 |
|---|---|---|---|---|---|
| 0x01 | INFO | 0 | 空 | info | 协议定义校验、固件、设备号、看门狗 |
| 0x02 | CAPS | 1/2 | 空 | caps | 通道能力及已验收范围 |
| 0x03 | STATUS | 1/2 | 空 | telemetry | 同异步遥测结构 |
| 0x04 | HEARTBEAT | 0 | 空 | 空 | 续期独立通信看门狗 |
| 0x05 | STOP_ALL | 0 | 空 | 空 | 请求全部通道安全停止；所有模式/故障状态均允许 |
| 0x10 | SET_TARGET | 1/2 | target | target | 原子设置 Vout 目标 mV 与 Iout 限制 mA；可运行时调整，MCU 限斜率 |
| 0x11 | SET_MODE | 1/2 | mode | mode | 0 正常 CV/CC，1 受限 IL 调试；仅关闭状态切换 |
| 0x12 | OUTPUT | 1/2 | output | output | enabled 0 停止、1 启动；其他值非法；启动不隐含改目标 |
| 0x13 | CLEAR_FAULT | 1/2 | 空 | 空 | 仅输出已关闭且物理故障解除时清锁存；保持输出关闭 |
| 0x20 | GET_PID | 1/2 | loop | pid | loop 0 IL，1 CV，2 Iout CC |
| 0x21 | SET_PID | 1/2 | pid | pid | 仅 OFF 下原子替换完整参数，RAM 生效；ACK 返回实际值 |
| 0x22 | DEBUG_REF | 1/2 | debug_ref | debug_ref | IL 目标 mA 和运行期限 ms；仅 IL 调试且 OFF |
| 0x30 | TRACE_ARM | 1/2 | trace_arm | trace_info | 从下一有效控制采样开始捕获，不启动输出 |
| 0x31 | TRACE_INFO | 1/2 | 空 | trace_info | 查询捕获状态 |
| 0x32 | TRACE_READ | 1/2 | trace_read | trace_chunk + trace_point[count] | 捕获完成后读取指定分片 |
| 0x80 | TELEMETRY | 1/2 | 不接受 REQUEST | EVENT: telemetry | 默认每通道 20 Hz 异步回报 |
| 0x81 | LOG | 0/1/2 | 不接受 REQUEST | EVENT: log | 带 MCU 时间戳的诊断日志，见 V2 增量 |
| 0x23 | ZERO | 1 | zero_request | zero_status | CH1 电流零点查询/校准/恢复 |
| 0x24 | GATE_TEST | 1 | gate_test_request | gate_test_status | 栅极映射实测与确认；见下节 |

ACK 表示命令已被接受或配置完成，不是 RUN 状态保证。启动仍须经过 MCU 安全状态机；上位机必须等待遥测报告实际状态。对不支持功能/越界参数明确 NACK，不可偷偷钳位后返回原请求值。SET_PID ACK 后上位机再 GET_PID 校验 float32 完全一致；MCU 不应自动缩放 Ki/Kd 后改变回报语义。

## 4. 信息和能力

`info.schema_crc`：对 schema.json 解析后的对象使用键排序、无多余空白的 JSON（Python `json.dumps(sort_keys=True,separators=(',',':'))`，UTF-8）计算 CRC32/ISO-HDLC。生成工具已算好 `PD_SCHEMA_CRC32`，MCU 直接上报此常量，不需实现 JSON。它用于发现版本相同但字段定义漂移，不是身份认证。

`watchdog_ms`：本方案推荐 1000 ms；主机仅接受 500–1500 ms。MCU 上电输出必须关闭，重连/握手/心跳不得触发启动。

`caps.features` 位定义：bit0 正常模式与 SET_TARGET，bit1 IL 调试，bit2 PID 读写，bit3 快速快照；bit4 TARGET_STAGING 表示 V/I 仅预置，不能开启功率输出；bit5 NOMINAL_CALIBRATION 表示换算采用未实测的标称校准。其他位保留零。每通道可不同。能力存在不代表硬件 ready。某环路尚未实现可对对应 GET/SET_PID 返回 UNSUPPORTED，频率上报 0。纯测量阶段 features 可为 0；本次 MCU 诊断版本可开放参数预置、PID 存储和快照，另置 TARGET_STAGING 位，ready 始终为 0。

`vmax_mv`、`imax_ma`、`ilmax_ma` 是当前验收构建允许的最大输出电压、输出限流设置和调试电感电流，非传感器量程；未验收为 0。正常固件的范围由硬件验收配置，不能复制演示的 30 V/3 A/1 A。TARGET_STAGING 置位时，这些范围仅表示用户指定的参数预置范围，不是已经验收的实际输出范围。新增 vmin_mv 给出设定下限；负目标本版不支持。

`current_hz`、`voltage_hz`、`cc_hz` 是实际控制计算频率，不是带宽。`max_debug_ms` 是 IL 调试最大运行时长，允许请求 10…max_debug_ms。`trace_max_points` 最大 256，0 表示不可用；`trace_base_hz` 为有效同步采样基础速率。

## 5. 遥测与状态

`telemetry` 字段顺序、大小见生成头文件/布局表。测量电压=mV，电流=mA（有符号），温度=0.01°C，占空比=0.01%（0…10000）。电压和电流的正方向分别为输入向转换器、输出向负载、电感从输入桥向输出桥。Vref/Iref 是已接受命令目标，ILref 为当次内环目标；软启动中的内部斜坡值可与命令目标不同。

valid bit0 Vin，1 Iin，2 Vout，3 Iout，4 IL，5 Tmos，6 TL；其余保留零。采样过期、未校准而不可用、ADC 过量程或传感器诊断失效时清对应 valid 位，输出关闭不必清有效测量位。快照只允许所记录字段均有效，否则 NACK NOT_READY/FAULT；严禁把无效 ADC 当作正常电流。

uptime_ms 为 MCU 单调毫秒计数 u32，可自然回绕；主机趋势使用主机接收时间，快速快照使用 MCU 采样率。enabled 表示 MCU 逻辑上已允许功率输出（0/1），不是栅极实测；ready 表示该通道安全初始化、配置校验及当前启动前提全部满足（0/1），不能由主机命令设置。

状态编码沿用硬件交接文件：0 BOOT、1 SELF_TEST、2 OFF、3 BOOTSTRAP_PRECHARGE、4 SOFT_START、5 RUN_CV、6 RUN_CC、7 STOPPING、8 FAULT_LATCHED，新增 9 RUN_IL_DEBUG。

fault 为**锁存故障位**：bit0 硬件 OC，1 TMCS ALERT/诊断，2 输出过压，3 过温，4 通信超时，5 采样故障，6 输入异常，7 内部保护；其余保留，新增必须更新契约。物理故障解除不自动清位、不自动重新使能输出。

20 Hz/通道遥测帧负载远低于 115200 / 10 = 11520 byte/s 的物理吞吐上限。快照上传限制在低优先级通信任务，先保证停止命令、ACK 和遥测；不向 UART 连续推送 100 kSPS 数据。

## 6. PID 的精确定义

三环均为连续参数、并联形式：`u = Kp * e + I + D`，`e = r - y`，Ts=1/该环实际上报频率。Kd=0 表示 PI。

| loop | r/y 单位 | u 单位 | Kp | Ki | Kd |
|---|---|---|---|---|---|
| 0 IL | A | V，电感平均电压修正量 | V/A | V/(A·s) | V·s/A |
| 1 CV | V | A，电感电流请求 | A/V | A/(V·s) | A·s/V |
| 2 Iout CC | A | A，电感电流上限请求 | A/A | A/(A·s) | A·s/A |

电流内环的输出不是直接 PWM 占空比，后续调制器结合 Vin/Vout、前馈和可用占空比范围形成两桥指令。输出恒流限制与电压环请求按后续控制设计取限制值；IL 不等于 Iout。

离散执行契约：

1. `P = Kp * (r - y)`。
2. Kd=0 时 D=0；Kd>0 要求 tau>0，`D[n] = tau/(tau+Ts)*D[n-1] - Kd/(tau+Ts)*(y[n]-y[n-1])`。D 对测量值求导，避免设定阶跃微分冲击。
3. `I_candidate = I[n-1] + Ki*Ts*e`，`u_candidate = P + I_candidate + D`。
4. 如果 `u_candidate > out_max 且 Ki*Ts*e > 0`，或 `u_candidate < out_min 且 Ki*Ts*e < 0`，保持旧 I；否则接受 I_candidate。
5. `u = clamp(P + I + D, out_min, out_max)`。外层限幅/调制饱和时还需固件跟踪抗饱和；不得取消保护以满足 PID 请求。

Kp/Ki/Kd/tau 必须有限且非负，out_min<out_max；主机仅做绝对值≤1e9的序列化卫生限制，**不是硬件允许的增益范围**。MCU 必须按环路、固件构建和调制限制进一步拒绝危险或不支持值。启动/停机/故障时复位积分器、微分状态，首次导数历史 y 取当前测量值；软启动管理 r。运行中不修改 PID，本版不承诺在线无扰切换。

## 7. 电感电流受限调试

SET_MODE=1 → DEBUG_REF(ilref_ma,duration_ms) → OUTPUT=1。每次 SET_MODE 都使旧调试目标配置失效；未重新设置 DEBUG_REF 时，IL 模式 OUTPUT=1 返回 NOT_READY，不能继承正常模式的 ILref。MCU 在首次接受启动时记录单调截止时间，到时按安全停机流程关闭输出；重复 OUTPUT=1 必须幂等，不能延长截止时间，DEBUG_REF 在运行中拒绝。退出模式或故障后禁止自动恢复。

电流内环测试必须保留输出过压、输入/传感器检查及硬件 OC。未有可靠同步 ADC、电流闭环、自举策略、安全停机及保护时，**features.bit1 和 ready 不得开放**。PC 不是定时关断的唯一执行者。

## 8. 快速快照

TRACE_ARM 请求 points 16…min(trace_max_points,256)，decimation 1…1000 且整除 trace_base_hz。本版 UI 提供常用点数/抽取档位。实际 sample_hz = trace_base_hz/decimation。

state：0 空，1 正在采样，2 完成，3 失败。capture_id 是 u32 单调递增采集号，同一次采集的 ARM/INFO/READ 必须相同。每次从下一有效控制采样点开始，无预触发；无自动阶跃或外部触发。最多一份采集正在运行；忙时 NACK BUSY。停止或故障不能被采集阻塞，采集被中止时返回失败状态，禁止上传拼接的旧新样本。

每点 20 字节：IL mA、ILref mA、Vout mV、Vref mV、输入桥占空比 u16、输出桥占空比 u16；四个量及两个占空比必须来自同一控制时刻的有效快路径快照，较慢 Vout 必须说明为最近有效测量保持值。禁止用当前低速遥测反推历史数据。

TRACE_READ 请求 capture_id/offset/count；offset 是点索引，count 1…24。ACK 前 8 字节为 trace_chunk，之后 count*20 字节。响应最大 488 字节 payload，符合 512 限制。必须回显请求编号、起点和数量；越界 NACK RANGE，旧 ID NACK NOT_FOUND。主机先等完成再顺序拉取全部分片，校验长度/编号后才标记成功，最多等采样完成 4 秒。MCU 保持完成记录直到下一次 ARM 或复位。

256 点需 5120 字节缓冲（不含状态），无需在中断中分配内存。采样 ISR 只向固定 RAM 填入数据；序列化/CRC/发送在 FreeRTOS 通信任务完成。

## 9. 失效和恢复

主机正常每 250 ms 发 HEARTBEAT；MCU 独立监控最近**合法心跳**，其他无效帧/普通命令不续期。已经使能任意输出时，心跳超过 watchdog_ms：硬件/状态机安全关断全部输出，并锁存通信故障。后续心跳只能恢复链路健康，不能清故障或启动。

PC 任一路遥测 500 ms 超时或页面轮询 1500 ms 超时后，发送 seq=0 STOP_ALL 并停止心跳；即使页面轮询恢复也要求重新连接。断开/退出时等待最多 400 ms 的 STOP_ALL ACK，然后关闭串口，未确认则显示未知。不能依赖操作系统、浏览器或最后一个 UART 帧一定正常退出。

对 OUTPUT=0 / STOP_ALL 的 ACK，只能在安全关断请求已实际交给不可被后续低优先级队列覆盖的状态机后返回。功率固件必须确保关闭 HRTIM 输出并拉低独立 EN，包含硬件 Fault 输入取消后仍禁止重启；此语义不是上位机已经替 MCU 实现了保护。

## 10. 错误码和测试向量

| code | 名称 | 含义 |
|---:|---|---|
| 1 | BAD_LENGTH | payload 长度错误 |
| 2 | BAD_CHANNEL | 通道无效 |
| 3 | UNSUPPORTED | 命令或该环路/功能尚未实现 |
| 4 | RANGE | 参数无效、越界或不符合约束 |
| 5 | NOT_OFF | 操作要求停机或模式不符 |
| 6 | NOT_READY | 初始化/保护/测量尚未就绪 |
| 7 | FAULT | 存在未解除的故障 |
| 8 | BUSY | 操作正在进行 |
| 9 | NOT_FOUND | 捕获记录不存在 |
| 10 | PROTOCOL | 协议语义不符 |

完整 INFO、SET_TARGET、STOP_ALL 帧见 `test_vectors.json`。MCU 接入前先验证 CRC 0x29B1、各向量收发字节、头文件 SIZE/OFFSET，再接状态机。C 端不要直接把 UART 数组转成未对齐 struct；明确逐字节读写小端整数，对 float 使用 memcpy 和已校验长度。

## V2 增量（2026-09-09）

MCU 0.2.0 增加 LOG code 12：`SHADOW ref=... y=... P=... I=... D=... U=...`，仅 CH1 IL 模式下约 2 Hz，ref/y 单位 A，其余为算法控制量；不表示实际 PWM。普通 LOG 字段布局与协议版本/hash 不变，旧上位机仍能把它作为日志显示。code 9 在 0.2.0 中改为只读配置校验信息，删除 0.1.1 的 PC0 启动测试；code 10 为 ADC 寄存器、code 11 为 UART 计数。CH2 PID 不调度，current_hz=0。

首个真实 MCU 对接版本。PowerDesk V1.1.0 使用帧 version=2；V1 上位机不兼容，必须重启更新后的本机服务。所有常量已同步生成到 MCU App/powerdesk_protocol.h，测试会逐字节比较。

- CAPS 在 features 后新增 i32 vmin_mv；所有字段偏移以生成头文件为准。
- 新增 EVENT opcode=0x81 LOG，channel=0/1/2，seq=0。payload 固定 log 布局：u32 uptime_ms，u8 level（0调试、1信息/提示、2错误），u16 code，ASCII message[96]（零填充）。同样 CRC，不可与裸 printf 混发。
- 本次固件日志 code：1启动/诊断边界；2坏帧；3拒绝启动；4心跳超时；5UART异常；6RX溢出；7采样计数/时序/丢帧计数；8CH1 ADC 原始值。
- NOMINAL_CALIBRATION 置位时，有效电压/电流可供初步测量，但精度未确认；NTC 未确认温度模型时保持温度 valid=0，只回报原始 ADC 日志。
- 当前诊断固件的 current_hz=100000 表示不连接 PWM 的 PID 影子计算频率，电压/恒流环仅存储参数，频率上报 0。这不是已经闭合了电流环。
- 诊断固件 OUTPUT=1 一律 NACK NOT_READY。CH2 SET_TARGET/SET_MODE/DEBUG_REF 不开放；STOP 和测量始终可用。
