@echo off
rem ==============================================================================
rem ModelSim 一键编译与系统级闭环仿真脚本 (LHA5278H 顶层控制器与数据接收)
rem ==============================================================================

set MODELSIM_BIN=C:\modeltech64_2019.2\win64

echo [1/3] Creating simulation library ...
if not exist "work" (
    "%MODELSIM_BIN%\vlib.exe" work
)

echo [2/3] Compiling RTL and Testbench ...
"%MODELSIM_BIN%\vlog.exe" -work work ../rtl/lha5278h_top.v ../rtl/lha5278h_spi_reg_ctrl.v ../rtl/lha5278h_data_rx.v ../tb/tb_lha5278h_top.v
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Compilation failed!
    pause
    exit /b %ERRORLEVEL%
)

echo [3/3] Running simulation in batch mode ...
"%MODELSIM_BIN%\vsim.exe" -c -lib work tb_lha5278h_top -do "run -all; quit -f"

echo ==============================================================================
echo Simulation finished!
pause
