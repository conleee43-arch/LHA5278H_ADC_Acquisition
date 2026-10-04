# FPGA IO 与 LHA5278H 信号引脚对应表

## 1. 工程基本信息

- **目标 FPGA 器件**：Intel / Altera Cyclone IV E - `EP4CE10F17C8`（256-pin FBGA）
- **目标板卡**：E10 开发板
- **ADC 芯片型号**：LHA5278H（8 通道 24-bit 模数转换器）
---

## 2. LHA5278H ADC 接口引脚映射表

| 序号 | FPGA 端口名 | FPGA 物理引脚 | 方向 (FPGA视角) | I/O 电平标准 | 驱动/上下拉属性 | 采样模式功能 | SPI 配置模式功能 | 功能详细说明与极性规定 |
| :---: | :--- | :---: | :---: | :---: | :--- | :--- | :--- | :--- |
| 1 | `adc_sync` | **PIN_T5** | Output | 3.3-V LVCMOS | 4mA | **SYNC** | **SYNC** | **低电平有效 / 低脉冲触发（Pulse Low）**<br>平时保持高电平(1)；触发时产生低脉冲(0)停止转换并复位内部数字滤波器计数器，随后拉高重新启动转换 |
| 2 | `adc_drdy` | **PIN_T6** | Input | 3.3-V LVCMOS | 启用内部弱上拉 (Weak Pull-Up) | **DRDYn** | **DRDYn** | 数据转换完成就绪信号（低有效，下降沿指示本帧转换完成） |
| 3 | `adc_din` | **PIN_R10** | Output | 3.3-V LVCMOS | 4mA | **DIN** | **DIN** | 级联数据输入（单片应用时默认拉低输出 0） |
| 4 | `adc_sclk` | **PIN_N5** | Output | 3.3-V LVCMOS | 4mA | **SCLK** | **SCLK** | 串行时钟输出（门控时钟：采样模式输出 24 周期，SPI 模式移位时钟） |
| 5 | `adc_dout1` | **PIN_R3** | Input | 3.3-V LVCMOS | - | **DOUT1** | - | 通道 1 串行 24-bit 转换数据输入 |
| 6 | `adc_dout2` | **PIN_N15** | Input | 3.3-V LVCMOS | - | **DOUT2** | - | 通道 2 串行 24-bit 转换数据输入 |
| 7 | `adc_dout3` | **PIN_R16** | Inout | 3.3-V LVCMOS | 4mA | **DOUT3** | **SDI** | **双向引脚**：采样模式为 CH3 数据输入；SPI 模式复用为 FPGA 输出向 ADC 写入配置命令与数据（SDI） |
| 8 | `adc_dout4` | **PIN_N12** | Input | 3.3-V LVCMOS | - | **DOUT4** | **SDO** | 采样模式为 CH4 数据输入；SPI 模式复用为 ADC 回读数据输出（输入至 FPGA 的 SDO） |
| 9 | `adc_dout5` | **PIN_T14** | Input | 3.3-V LVCMOS | - | **DOUT5** | - | 通道 5 串行 24-bit 转换数据输入 |
| 10 | `adc_dout6` | **PIN_T13** | Input | 3.3-V LVCMOS | - | **DOUT6** | - | 通道 6 串行 24-bit 转换数据输入 |
| 11 | `adc_dout7` | **PIN_P8** | Input | 3.3-V LVCMOS | - | **DOUT7** | - | 通道 7 串行 24-bit 转换数据输入 |
| 12 | `adc_dout8` | **PIN_M6** | Input | 3.3-V LVCMOS | - | **DOUT8** | - | 通道 8 串行 24-bit 转换数据输入 |
| 13 | `adc_clkdiv_csn`| **PIN_R13** | Output | 3.3-V LVCMOS | 4mA | **CLKDIV** | **CSn** | **复用引脚**：采样模式输出 CLKDIV 电平控制时钟分频；SPI 模式输出片选使能（低电平有效） |
| 14 | `adc_test1` | **PIN_T15** | Output | 3.3-V LVCMOS | 4mA | **TEST1** | **TEST1** | **固定恒定输出 0 (`1'b0`)**<br>严禁置 1，防止与 TEST0 组合进入 `2'b11` 引脚测试模式 |
| 15 | `adc_test0` | **PIN_P16** | Output | 3.3-V LVCMOS | 4mA | **TEST0** | **TEST0** | **工作模式切换控制位**：<br>正常采样模式输出 **0**（`TEST[1:0]=2'b00`）；<br>SPI 配置模式输出 **1**（`TEST[1:0]=2'b01`） |

---

## 3. LHA5278H 工作模式控制表 (TEST[1:0])

根据芯片手册严格定义，LHA5278H 的工作模式由引脚 **`TEST[1:0]`** 组合状态决定：

| TEST1 (PIN_T15) | TEST0 (PIN_P16) | 组合值 `TEST[1:0]` | 对应工作模式 | 模式行为与设计安全要求 |
| :---: | :---: | :---: | :--- | :--- |
| **0** | **0** | `2'b00` | **正常采样模式 (Normal Operation)** | 8 通道并行 24-bit 转换输出，DOUT3/DOUT4 作为数据输出通道 |
| **0** | **1** | `2'b01` | **SPI 寄存器配置模式 (SPI Reg Access)** | DOUT3 复用为 SDI 输入，DOUT4 复用为 SDO 输出，CLKDIV 复用为 CSn 片选 |
| **1** | **0** | `2'b10` | **保留模式 (Do not use)** | **禁止使用** |
| **1** | **1** | `2'b11` | **芯片引脚连续性测试模式 (Pin Test Mode)** | **重大风险**：芯片切断内部正常逻辑，进行引脚对联测试，导致正常采集与 SPI 读写完全失效 |

> [!CAUTION]
> **逻辑控制重大安全规则**：
> 在进入 SPI 配置模式时，`TEST[1:0]` 必须且只能为 `2'b01`。
> FPGA 侧逻辑中 **`adc_test1`（PIN_T15）必须在所有模式下固定输出 `1'b0`**；仅由 **`adc_test0`（PIN_P16）负责模式切换**（采样模式拉低为 0，SPI 配置时拉高为 1）。

---

## 4. SYNC 信号控制与极性定义

- **触发极性**：**低电平有效 / 低脉冲触发（Active-Low Pulse）**
- **稳态电平**：平时保持 **高电平（`1'b1`）**
- **脉冲行为**：
  1. 上电复位延时完成后，FPGA 输出一个宽度约为 10μs 的 **负脉冲（拉低为 `1'b0`）**，触发芯片停止当前转换并清零内部数字滤波器计数器；
  2. 随后恢复为 **高电平（`1'b1`）**，芯片重新同步并启动 ADC 转换；
  3. 当上位机通过命令向 `REG_CTRL.bit1` 写入 1 请求触发同步时，同样产生该 10μs 的负脉冲。

```
adc_sync (PIN_T5) 脉冲时序示意:
       平时高电平                    10μs 低脉冲                 恢复高电平重新开始转换
───────┐                                                     ┌──────────────────────
       │                                                     │
       └─────────────────────────────────────────────────────┘
```

---

## 5. 板载基础系统与外设引脚

| 端口名 | FPGA 物理引脚 | 方向 | 电平标准 | 功能说明 |
| :--- | :---: | :---: | :---: | :--- |
| `sys_clk` | **PIN_E1** | Input | 3.3-V LVCMOS | 板载 50MHz 有源晶振输入 |
| `sys_rst_n` | **PIN_E15** | Input | 3.3-V LVCMOS | 板载复位按键输入（低电平复位，按键触发重新执行 RMW 翻转） |
| `sys_ready` | **PIN_A4** | Output | 3.3-V LVCMOS | 状态指示（RMW 校验成功后拉高输出 3.3V，可接板载 LED） |
| `uart_rxd` | **PIN_B13** | Input | 3.3-V LVCMOS | UART 接收端口（连接板载 CH340 TXD，波特率 115200bps） |
| `uart_txd` | **PIN_A13** | Output | 3.3-V LVCMOS | UART 发送端口（连接板载 CH340 RXD，波特率 115200bps） |

---

## 6. 硬件电气与时序设计注意事项

1. **输出电流与保护**：
   - 全部输出控制信号（`adc_sync`、`adc_din`、`adc_sclk`、`adc_clkdiv_csn`、`adc_test1`、`adc_test0`、`adc_dout3(SDI)`）均在 QSF 中配置了 `CURRENT_STRENGTH_NEW 4MA` 进行限流保护。
2. **DRDY 内部弱上拉**：
   - `adc_drdy`（PIN_T6）已开启 FPGA 内部弱上拉（`WEAK_PULL_UP_RESISTOR ON`），防止芯片未就绪或未供电时悬空引起 FPGA 误触发。
3. **SPI 读写与采样时序**：
   - SPI 模式采用 **SPI Mode 1 (CPOL=0, CPHA=1)**，FPGA 在 SCLK 上升沿更新 SDI、在下降沿采样 SDO；芯片侧下降沿采样 SDI、上升沿更新 SDO；
   - 退出 SPI 模式回到采样模式时，`adc_test0` 拉低至 0，芯片重回正常多通道数据输出。
