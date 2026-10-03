//==============================================================================
// Testbench Name: tb_lha5278h_top
// Description:    针对 LHA5278H 顶层控制器进行系统级闭环全流程仿真验证
//                 流程:
//                 1. 芯片上电 -> 软复位 (Reg[0x0A] <= 0x16)
//                 2. 配置 FIR 512kSPS 模式 (Reg[0x0F] <= 0x28)
//                 3. 双帧回读 Reg[0x0F] 验证 (读回 0x28) -> sys_ready (PIN_A4) 拉高
//                 4. 模式切换: TEST[1:0] 从 2'b01 (SPI) 切换到 2'b00 (正常 ADC 转换模式)
//                 5. SYNC 负脉冲同步: 触发 10μs 低脉冲并恢复高电平
//                 6. 正常采样模式验证: 外部 ADC 产生 DRDY 脉冲，FPGA 突发 24 个 SCLK
//                    读取 8 通道 24-bit 转换结果并校验一致性
// Standard:       Verilog-2001
//==============================================================================

`timescale 1ns / 1ps

module tb_lha5278h_top;

    // 系统 50MHz 时钟与复位
    reg        sys_clk;
    reg        sys_rst_n;

    // 物理 ADC 接口
    wire       adc_sync;
    reg        adc_drdy;
    wire       adc_din;
    wire       adc_sclk;
    reg        adc_dout1;
    reg        adc_dout2;
    wire       adc_dout3; // 双向引脚: 采样模式 CH3 / SPI 模式 SDI
    wire       adc_dout4; // 复用引脚: 采样模式 CH4 / SPI 模式 SDO
    reg        adc_dout5;
    reg        adc_dout6;
    reg        adc_dout7;
    reg        adc_dout8;
    wire       adc_clkdiv_csn;
    wire       adc_test1;
    wire       adc_test0;

    wire       sys_ready;

    //==========================================================================
    // 实例化系统顶层 (DUT)
    //==========================================================================
    lha5278h_top #(
        .SYS_CLK_FREQ      (50_000_000),
        .SPI_SCLK_FREQ     (5_000_000),
        .RX_SCLK_FREQ      (25_000_000),
        .RESET_WAIT_MS     (1),    // 仿真加速: 1ms
        .SWITCH_WAIT_US    (10),   // 仿真加速: 10us
        .SYNC_PULSE_US     (10)    // 10us SYNC 负脉冲
    ) u_top (
        .sys_clk       (sys_clk),
        .sys_rst_n     (sys_rst_n),
        .adc_sync      (adc_sync),
        .adc_drdy      (adc_drdy),
        .adc_din       (adc_din),
        .adc_sclk      (adc_sclk),
        .adc_dout1     (adc_dout1),
        .adc_dout2     (adc_dout2),
        .adc_dout3     (adc_dout3),
        .adc_dout4     (adc_dout4),
        .adc_dout5     (adc_dout5),
        .adc_dout6     (adc_dout6),
        .adc_dout7     (adc_dout7),
        .adc_dout8     (adc_dout8),
        .adc_clkdiv_csn(adc_clkdiv_csn),
        .adc_test1     (adc_test1),
        .adc_test0     (adc_test0),
        .sys_ready     (sys_ready)
    );

    // 时钟产生 (50 MHz, 20ns 周期)
    initial begin
        sys_clk = 1'b0;
        forever #10 sys_clk = ~sys_clk;
    end

    //==========================================================================
    // DOUT3 / DOUT4 引脚在 SPI 模式与采样模式下的总线驱动逻辑
    //==========================================================================
    reg tb_dout3_reg;
    reg tb_dout4_reg;

    reg [7:0]  adc_regs [0:127];
    reg [6:0]  slave_addr;
    reg [15:0] slave_rx_shift;
    reg [7:0]  slave_tx_shift;
    reg [4:0]  slave_bit_cnt;
    reg        slave_sdo_reg;
    reg        is_f2;

    wire chip_cs_n = adc_clkdiv_csn;
    wire chip_sdi  = adc_dout3;

    // 当 adc_test0=1 (SPI 模式): DOUT3 为 DUT 输出 (SDI)，TB 释放高阻；
    // 当 adc_test0=0 (采样模式): DOUT3 为 DUT 输入 (CH3)，TB 驱动 tb_dout3_reg。
    assign adc_dout3 = (adc_test0 == 1'b0) ? tb_dout3_reg : 1'bz;

    // 当 adc_test0=1 (SPI 模式): DOUT4 由 SPI 从机模型驱动 SDO；
    // 当 adc_test0=0 (采样模式): DOUT4 由 TB 驱动 CH4 数据。
    assign adc_dout4 = (adc_test0 == 1'b1) ? ((!chip_cs_n && is_f2) ? slave_sdo_reg : 1'b0)
                                           : tb_dout4_reg;

    //==========================================================================
    // SPI 从机行为模型 (SPI Mode 1)
    //==========================================================================
    integer i;
    initial begin
        for (i = 0; i < 128; i = i + 1) begin
            adc_regs[i] = 8'h00;
        end
        adc_regs[7'h00] = 8'h04; // 0x00 默认值

        slave_addr     = 7'd0;
        slave_rx_shift = 16'h0000;
        slave_tx_shift = 8'h00;
        slave_bit_cnt  = 5'd0;
        slave_sdo_reg  = 1'b0;
        is_f2          = 1'b0;

        tb_dout3_reg   = 1'b0;
        tb_dout4_reg   = 1'b0;
    end

    // Mode 1: 片选下降沿初态
    always @(negedge chip_cs_n) begin
        if (adc_test0) begin
            slave_bit_cnt  <= 5'd0;
            slave_rx_shift <= 16'h0000;
            if (is_f2) begin
                slave_tx_shift <= adc_regs[slave_addr];
                slave_sdo_reg  <= 1'b0;
            end else begin
                slave_sdo_reg  <= 1'b0;
            end
        end
    end

    // Mode 1: SCLK 上升沿从机驱动 SDO
    always @(posedge adc_sclk) begin
        if (adc_test0 && !chip_cs_n && is_f2) begin
            if (slave_bit_cnt < 5'd8) begin
                slave_sdo_reg  <= slave_tx_shift[7];
                slave_tx_shift <= {slave_tx_shift[6:0], 1'b0};
            end else begin
                slave_sdo_reg  <= 1'b0;
            end
        end
    end

    // Mode 1: SCLK 下降沿从机采样 SDI
    always @(negedge adc_sclk) begin
        if (adc_test0 && !chip_cs_n) begin
            slave_rx_shift <= {slave_rx_shift[14:0], chip_sdi};
            slave_bit_cnt  <= slave_bit_cnt + 1'b1;
        end
    end

    // 片选上升沿: 完成帧解析
    always @(posedge chip_cs_n) begin
        if (sys_rst_n && adc_test0) begin
            if (is_f2) begin
                $display("[ADC CHIP MODEL] %t: Read Frame 2 Finished for Reg[0x%02X], Data Out = 0x%02X", 
                         $time, slave_addr, adc_regs[slave_addr]);
                is_f2 <= 1'b0;
            end else if (slave_rx_shift[15] == 1'b0) begin
                // 写操作
                adc_regs[slave_rx_shift[14:8]] <= slave_rx_shift[7:0];
                if (slave_rx_shift[14:8] == 7'h0A && slave_rx_shift[7:0] == 8'h16) begin
                    $display("[ADC CHIP MODEL] %t: *** Soft Reset Command Received (Reg[0x0A] <= 0x16) ***", $time);
                    adc_regs[7'h00] <= 8'h04;
                    adc_regs[7'h09] <= 8'h00; // 复位后 0x09 默认值为 0x00
                end else if (slave_rx_shift[14:8] == 7'h09) begin
                    $display("[ADC CHIP MODEL] %t: *** Reg[0x09] Written <= 0x%02X ***", 
                             $time, slave_rx_shift[7:0]);
                end else if (slave_rx_shift[14:8] == 7'h0F) begin
                    $display("[ADC CHIP MODEL] %t: *** ADC_CTL0 Written <= 0x%02X ***", 
                             $time, slave_rx_shift[7:0]);
                end
            end else begin
                // 读操作 Frame 1
                slave_addr <= slave_rx_shift[14:8];
                $display("[ADC CHIP MODEL] %t: Read Frame 1 Latched Target Reg = 0x%02X", 
                         $time, slave_rx_shift[14:8]);
                is_f2 <= 1'b1;
            end
        end
    end

    //==========================================================================
    // 8 通道测试样本数据预设
    //==========================================================================
    localparam [23:0] EXP_CH1 = 24'h123456;
    localparam [23:0] EXP_CH2 = 24'h234567;
    localparam [23:0] EXP_CH3 = 24'h345678;
    localparam [23:0] EXP_CH4 = 24'h456789;
    localparam [23:0] EXP_CH5 = 24'h56789A;
    localparam [23:0] EXP_CH6 = 24'h6789AB;
    localparam [23:0] EXP_CH7 = 24'h789ABC;
    localparam [23:0] EXP_CH8 = 24'h89ABCD;

    //==========================================================================
    // 测试激励主进程
    //==========================================================================
    integer bit_idx;

    initial begin
        sys_rst_n    = 1'b0;
        adc_drdy     = 1'b1;
        adc_dout1    = 1'b0;
        adc_dout2    = 1'b0;
        tb_dout3_reg = 1'b0;
        tb_dout4_reg = 1'b0;
        adc_dout5    = 1'b0;
        adc_dout6    = 1'b0;
        adc_dout7    = 1'b0;
        adc_dout8    = 1'b0;

        $display("============================================================================");
        $display(">>> Starting Testbench: Reset -> Read 0x09 -> Write 0x09=0x20 -> Read 0x09 -> SYNC <<<");
        $display("============================================================================");

        #200;
        sys_rst_n = 1'b1;
        $display("[TB] %t: System Reset Released. TEST[1:0]=%b%b (SPI Access Mode)", 
                 $time, adc_test1, adc_test0);

        // 1. 等待回读 0x0F 验证通过 (sys_ready 置 1)
        @(posedge sys_ready);
        $display("----------------------------------------------------------------------------");
        $display("[TB CHECK 1] %t: sys_ready asserted! 1st Read 0x09=0x%02X, 2nd Read 0x09=0x%02X, Read 0x0F=0x%02X (Exp: 0x28)",
                 $time, u_top.reg_09_val1, u_top.reg_09_val2, u_top.reg_0f_val);
        $display("----------------------------------------------------------------------------");

        // 2. 监测 TEST[1:0] 模式切换
        @(negedge adc_test0);
        $display("[TB CHECK 2] %t: Mode Switched! TEST[1:0] = {%b, %b} (Normal Conversion Mode)", 
                 $time, adc_test1, adc_test0);

        // 3. 监测 SYNC 负脉冲同步
        @(negedge adc_sync);
        $display("[TB CHECK 3] %t: SYNC Low Pulse Started (Filter Reset & Sync Triggered)...", $time);
        @(posedge adc_sync);
        $display("[TB CHECK 3] %t: SYNC Pulse Released High. ADC Conversion started!", $time);

        // 4. 等待数据接收使能 rx_en 拉高
        wait(u_top.rx_en == 1'b1);
        $display("[TB CHECK 4] %t: rx_en asserted! Data Receiver module activated.", $time);

        // 5. 模拟外部 ADC 完成首次模拟采样并产生 DRDYn 低脉冲 (持续 100ns)
        #2000; // 延时 2us 代表模拟采样与 FIR 计算时间
        $display("\n[ADC CHIP MODEL] %t: Conversion complete! Generating DRDY low pulse...", $time);
        adc_drdy = 1'b0;
        #100;
        adc_drdy = 1'b1;

        // 6. 伴随 FPGA 突发的 24 个 SCLK，模拟 8 通道并行输出数据
        //    在 SCLK 下降沿准备好下一位数据，供 FPGA 在 SCLK 上升沿采样
        for (bit_idx = 23; bit_idx >= 0; bit_idx = bit_idx - 1) begin
            // 等待 SCLK 上升沿前准备好数据
            adc_dout1    <= EXP_CH1[bit_idx];
            adc_dout2    <= EXP_CH2[bit_idx];
            tb_dout3_reg <= EXP_CH3[bit_idx];
            tb_dout4_reg <= EXP_CH4[bit_idx];
            adc_dout5    <= EXP_CH5[bit_idx];
            adc_dout6    <= EXP_CH6[bit_idx];
            adc_dout7    <= EXP_CH7[bit_idx];
            adc_dout8    <= EXP_CH8[bit_idx];
            @(posedge adc_sclk);
        end

        // 7. 等待 FPGA 产生 data_valid 脉冲
        @(posedge u_top.data_valid);
        $display("----------------------------------------------------------------------------");
        $display("[TB SUCCESS] %t: data_valid asserted! 8-Channel 24-bit Data Received:", $time);
        $display("  CH1 Data = 0x%06X (Exp: 0x%06X) %s", u_top.ch1_data, EXP_CH1, (u_top.ch1_data === EXP_CH1) ? "[OK]" : "[FAIL]");
        $display("  CH2 Data = 0x%06X (Exp: 0x%06X) %s", u_top.ch2_data, EXP_CH2, (u_top.ch2_data === EXP_CH2) ? "[OK]" : "[FAIL]");
        $display("  CH3 Data = 0x%06X (Exp: 0x%06X) %s", u_top.ch3_data, EXP_CH3, (u_top.ch3_data === EXP_CH3) ? "[OK]" : "[FAIL]");
        $display("  CH4 Data = 0x%06X (Exp: 0x%06X) %s", u_top.ch4_data, EXP_CH4, (u_top.ch4_data === EXP_CH4) ? "[OK]" : "[FAIL]");
        $display("  CH5 Data = 0x%06X (Exp: 0x%06X) %s", u_top.ch5_data, EXP_CH5, (u_top.ch5_data === EXP_CH5) ? "[OK]" : "[FAIL]");
        $display("  CH6 Data = 0x%06X (Exp: 0x%06X) %s", u_top.ch6_data, EXP_CH6, (u_top.ch6_data === EXP_CH6) ? "[OK]" : "[FAIL]");
        $display("  CH7 Data = 0x%06X (Exp: 0x%06X) %s", u_top.ch7_data, EXP_CH7, (u_top.ch7_data === EXP_CH7) ? "[OK]" : "[FAIL]");
        $display("  CH8 Data = 0x%06X (Exp: 0x%06X) %s", u_top.ch8_data, EXP_CH8, (u_top.ch8_data === EXP_CH8) ? "[OK]" : "[FAIL]");
        $display("----------------------------------------------------------------------------");

        if (u_top.ch1_data === EXP_CH1 &&
            u_top.ch2_data === EXP_CH2 &&
            u_top.ch3_data === EXP_CH3 &&
            u_top.ch4_data === EXP_CH4 &&
            u_top.ch5_data === EXP_CH5 &&
            u_top.ch6_data === EXP_CH6 &&
            u_top.ch7_data === EXP_CH7 &&
            u_top.ch8_data === EXP_CH8) begin
            $display(">>> ALL TESTS PASSED! Full System Closed-Loop Verification Successful! <<<");
        end else begin
            $display(">>> TEST FAILED: Data mismatch found in captured channels! <<<");
        end
        $display("============================================================================\n");

        #2000;
        $stop;
    end

endmodule
