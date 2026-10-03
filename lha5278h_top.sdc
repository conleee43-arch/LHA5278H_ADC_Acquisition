# ==============================================================================
# SDC (Synopsys Design Constraints) File for LHA5278H FPGA Controller
# 适用工具: TimeQuest Timing Analyzer (Quartus II 13.1)
# ==============================================================================

# 1. 约束系统 50MHz 主时钟 (周期 20.000 ns)
create_clock -name sys_clk -period 20.000 [get_ports {sys_clk}]

# 2. 自动派生时钟不确定度 (Jitter / Margin)
derive_clock_uncertainty

# 3. 对异步复位信号进行伪路径处理 (False Path)
set_false_path -from [get_ports {sys_rst_n}]

# 4. 对准静态模式控制和静态输出引脚设置伪路径
set_false_path -to [get_ports {adc_test1}]
set_false_path -to [get_ports {adc_test0}]
set_false_path -to [get_ports {adc_din}]
set_false_path -to [get_ports {adc_sync}]
set_false_path -to [get_ports {sys_ready}]
