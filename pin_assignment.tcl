# ==============================================================================
# Pin Assignment Tcl Script for LHA5278H on Altera Cyclone IV E (EP4CE10F17C8)
# 工程名称: SPI
# 适用工具: Quartus II 13.1 (64-bit)
# ==============================================================================

# 1. 系统时钟与复位按键
set_location_assignment PIN_E1  -to sys_clk
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to sys_clk

set_location_assignment PIN_E15 -to sys_rst_n
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to sys_rst_n

# 2. LHA5278H 控制与模式引脚
set_location_assignment PIN_T5  -to adc_sync
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to adc_sync

set_location_assignment PIN_R10 -to adc_din
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to adc_din

set_location_assignment PIN_N5  -to adc_sclk
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to adc_sclk

set_location_assignment PIN_R13 -to adc_clkdiv_csn
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to adc_clkdiv_csn

set_location_assignment PIN_T15 -to adc_test1
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to adc_test1

set_location_assignment PIN_P16 -to adc_test0
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to adc_test0

# 3. 数据就绪脉冲 DRDY (开启内部弱上拉)
set_location_assignment PIN_T6  -to adc_drdy
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to adc_drdy
set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to adc_drdy

# 4. 数据线 DOUT1 ~ DOUT8
set_location_assignment PIN_R3  -to adc_dout1
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to adc_dout1

set_location_assignment PIN_N15 -to adc_dout2
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to adc_dout2

set_location_assignment PIN_R16 -to adc_dout3
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to adc_dout3

set_location_assignment PIN_N12 -to adc_dout4
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to adc_dout4

set_location_assignment PIN_T14 -to adc_dout5
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to adc_dout5

set_location_assignment PIN_T13 -to adc_dout6
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to adc_dout6

set_location_assignment PIN_P8  -to adc_dout7
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to adc_dout7

set_location_assignment PIN_M6  -to adc_dout8
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to adc_dout8

# 5. 系统状态与指示引脚
set_location_assignment PIN_A4  -to sys_ready
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to sys_ready

# 6. 未使用引脚保护设置: 三态输入带弱上拉 (防止悬空或误驱动外部电路)
set_global_assignment -name RESERVE_ALL_UNUSED_PINS_WEAK_PULLUP "AS INPUT TRI-STATED WITH WEAK PULL-UP"

puts ">>> All Pin Assignments & Electrical Constraints have been applied successfully! <<<"
