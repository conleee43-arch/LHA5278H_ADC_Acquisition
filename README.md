# LHA5278H_ADC_Acquisition

> **LHA5278H 8 通道 24-Bit 高精度 Delta-Sigma ADC FPGA 采集与驱动控制工程**  
> 目标平台：Intel / Altera Cyclone IV E (`EP4CE10F17C8`)

---

## 1. 工程简介

本项目为国产高性能 8 通道 24-bit 模数转换芯片 **LHA5278H**（兼容 ADS1278）在 FPGA 平台上的完整驱动与采集控制系统。系统实现了：
1. **上电智能时序调度**：上电复位等待、芯片软复位、寄存器配置与自校验、工作模式动态切换及数字滤波器同步（SYNC 负脉冲）。
2. **多模式物理引脚复用 MUX**：
   - **SPI 配置模式**（`TEST[1:0] = 2'b01`）：复用引脚转换为 CSn、SDI、SDO 与 5MHz SCLK，实现内部寄存器配置与回读校验。
   - **常规高速采集模式**（`TEST[1:0] = 2'b00`）：复用引脚恢复为 8 通道数据输出（DOUT1~DOUT8）与 CLKDIV，支持 512kSPS 高速采样下 25MHz 突发门控时钟数据抓取。
3. **安全防护机制**：`adc_test1` 硬件死锁为 `1'b0`，杜绝进入芯片引脚测试模式（Pin Test Mode）。

---

## 2. 目录架构

```text
LHA5278H_ADC_Acquisition/
├── rtl/
│   ├── lha5278h_top.v             # 系统控制顶层 (主状态机调度、引脚复用MUX、时钟分频、系统就绪指示)
│   ├── lha5278h_spi_reg_ctrl.v    # SPI 寄存器配置引擎 (支持写/读校验与软复位)
│   └── lha5278h_data_rx.v         # 8 通道 24-bit 高速数据并行捕获接收器 (25MHz 门控突发时钟)
├── tb/
│   ├── tb_lha5278h_top.v          # 顶层系统自动化自检验 Testbench
│   └── tb_lha5278h_spi_reg_ctrl.v # SPI 配置模块独立单元测试 Testbench
├── sim/
│   ├── run_sim.bat                # ModelSim 自动化编译与仿真运行脚本
│   └── sim.do                     # ModelSim 波形与加载配置脚本
├── FPGA IO 与 LHA5278H 信号引脚对应表.md # 详细引脚电气定义与映射表
├── pin_assignment.tcl             # Quartus 管脚约束 TCL 脚本
├── lha5278h_top.qpf               # Quartus 工程工程文件
├── lha5278h_top.qsf               # Quartus 芯片、引脚及综合配置文件
└── lha5278h_top.sdc               # 时序约束 SDC 文件
```

---

## 3. 硬件平台与关键参数

| 参数项 | 配置值 / 说明 |
| :--- | :--- |
| **FPGA 芯片型号** | Intel / Altera Cyclone IV E `EP4CE10F17C8` (FBGA256) |
| **系统输入时钟** | 50.0 MHz (`PIN_E1`) |
| **低电平复位** | `PIN_E15` (低电平有效) |
| **ADC 芯片型号** | LHA5278H (8 通道、24-bit 分辨率、最高 512kSPS) |
| **SPI 配置时钟** | 5.0 MHz (`SPI_SCLK_FREQ = 5_000_000`) |
| **采集突发时钟** | 25.0 MHz (`RX_SCLK_FREQ = 25_000_000`) |
| **复位与模式切换时间** | 软复位等待 10ms，模式切换稳定 100μs，SYNC 脉宽 10μs |

---

## 4. 仿真与验证

进入 `sim/` 目录运行批处理脚本即可一键启动 ModelSim 仿真验证：

```powershell
cd sim
.\run_sim.bat
```
