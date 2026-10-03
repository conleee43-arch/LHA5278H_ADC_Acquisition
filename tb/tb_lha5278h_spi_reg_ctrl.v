//==============================================================================
// Testbench Name: tb_lha5278h_spi_reg_ctrl
// Description:    ModelSim 仿真验证平台 - 针对 LHA5278H SPI 寄存器控制器
//                 协议模式: SPI Mode 0 (CPOL=0, CPHA=0)
//                 内置 LHA5278H ADC 芯片 SPI Mode 0 从机行为模型
// Author:         FPGA 硬件系统专家
// Date:           2026-10-01
// Standard:       Verilog-2001
//==============================================================================

`timescale 1ns / 1ps

module tb_lha5278h_spi_reg_ctrl;

    // 仿真时钟与时序参数定义
    localparam integer SYS_CLK_PERIOD = 20; // 50 MHz 系统时钟周期 (20ns)
    localparam integer SYS_CLK_FREQ   = 50_000_000;
    localparam integer SPI_SCLK_FREQ  = 5_000_000; // 5 MHz SPI 时钟
    localparam integer CS_GAP_CYCLES  = 4;
    localparam integer CS_LEAD_CYCLES = 2;
    localparam integer CS_LAG_CYCLES  = 2;

    // 待测模块 (DUT) 接口信号
    reg        sys_clk;
    reg        sys_rst_n;

    reg        wr_en;
    reg        rd_en;
    reg  [6:0] addr;
    reg  [7:0] wdata;
    wire [7:0] rdata;
    wire       busy;
    wire       done;

    wire       spi_cs_n;
    wire       spi_sclk;
    wire       spi_sdi;
    wire       spi_sdo; // 由 ADC 从机模型驱动

    // 实例化待测模块 (DUT)
    lha5278h_spi_reg_ctrl #(
        .SYS_CLK_FREQ  (SYS_CLK_FREQ),
        .SPI_SCLK_FREQ (SPI_SCLK_FREQ),
        .CS_GAP_CYCLES (CS_GAP_CYCLES),
        .CS_LEAD_CYCLES(CS_LEAD_CYCLES),
        .CS_LAG_CYCLES (CS_LAG_CYCLES)
    ) u_dut (
        .sys_clk   (sys_clk),
        .sys_rst_n (sys_rst_n),
        .wr_en     (wr_en),
        .rd_en     (rd_en),
        .addr      (addr),
        .wdata     (wdata),
        .rdata     (rdata),
        .busy      (busy),
        .done      (done),
        .spi_cs_n  (spi_cs_n),
        .spi_sclk  (spi_sclk),
        .spi_sdi   (spi_sdi),
        .spi_sdo   (spi_sdo)
    );

    // 系统时钟产生 (50 MHz)
    initial begin
        sys_clk = 1'b0;
        forever #(SYS_CLK_PERIOD / 2) sys_clk = ~sys_clk;
    end

    //==========================================================================
    // LHA5278H ADC 芯片 SPI Mode 0 从机行为仿真模型
    //==========================================================================
    reg [7:0]  adc_reg_file [0:127];
    reg [6:0]  slave_read_addr;
    reg [15:0] slave_rx_shift;
    reg [7:0]  slave_tx_shift;
    reg [3:0]  slave_bit_cnt;
    reg        slave_sdo_out;
    reg        is_frame2_active;

    assign spi_sdo = (!spi_cs_n && is_frame2_active) ? slave_sdo_out : 1'b0;

    integer i;
    initial begin
        for (i = 0; i < 128; i = i + 1) begin
            adc_reg_file[i] = 8'h00;
        end
        slave_read_addr  = 7'd0;
        slave_rx_shift   = 16'h0000;
        slave_tx_shift   = 8'h00;
        slave_bit_cnt    = 4'd0;
        slave_sdo_out    = 1'b0;
        is_frame2_active = 1'b0;
    end

    // Mode 0: CSN 下降沿提前建立首位 (MSB)
    always @(negedge spi_cs_n) begin
        slave_bit_cnt  <= 4'd0;
        slave_rx_shift <= 16'h0000;
        if (is_frame2_active) begin
            slave_tx_shift <= adc_reg_file[slave_read_addr];
            slave_sdo_out  <= adc_reg_file[slave_read_addr][7];
        end else begin
            slave_sdo_out  <= 1'b0;
        end
    end

    // Mode 0: SCLK 上升沿从机采样 SDI (MOSI)
    always @(posedge spi_sclk) begin
        if (!spi_cs_n) begin
            slave_rx_shift <= {slave_rx_shift[14:0], spi_sdi};
            slave_bit_cnt  <= slave_bit_cnt + 1'b1;
        end
    end

    // Mode 0: SCLK 下降沿从机更新 SDO (MISO)
    always @(negedge spi_sclk) begin
        if (!spi_cs_n && is_frame2_active) begin
            if (slave_bit_cnt < 4'd8) begin
                slave_sdo_out  <= slave_tx_shift[6];
                slave_tx_shift <= {slave_tx_shift[6:0], 1'b0};
            end else begin
                slave_sdo_out  <= 1'b0;
            end
        end
    end

    // CSN 上升沿: 帧结束判定
    always @(posedge spi_cs_n) begin
        if (sys_rst_n) begin
            if (is_frame2_active) begin
                $display("[ADC SLAVE MODEL] %t: Read Frame 2 Finished for Reg [0x%02X], Data = 0x%02X",
                         $time, slave_read_addr, adc_reg_file[slave_read_addr]);
                is_frame2_active <= 1'b0;
            end else if (slave_rx_shift[15] == 1'b0) begin
                // 写操作: 写入寄存器
                adc_reg_file[slave_rx_shift[14:8]] <= slave_rx_shift[7:0];
                $display("[ADC SLAVE MODEL] %t: Write Reg [0x%02X] <= 0x%02X",
                         $time, slave_rx_shift[14:8], slave_rx_shift[7:0]);
            end else begin
                // 读操作 Frame 1: 锁存读取地址
                slave_read_addr  <= slave_rx_shift[14:8];
                $display("[ADC SLAVE MODEL] %t: Read Frame 1 Latched Reg Addr = 0x%02X",
                         $time, slave_rx_shift[14:8]);
                is_frame2_active <= 1'b1;
            end
        end
    end

    //==========================================================================
    // 测试激励流程
    //==========================================================================
    reg [7:0] test_read_val;
    integer   error_count;

    initial begin
        sys_rst_n      = 1'b0;
        wr_en          = 1'b0;
        rd_en          = 1'b0;
        addr           = 7'd0;
        wdata          = 8'd0;
        test_read_val  = 8'h00;
        error_count    = 0;

        $display("=================================================================");
        $display(">>> Starting Testbench for LHA5278H SPI Controller (Mode 0) <<<");
        $display("=================================================================");

        #(SYS_CLK_PERIOD * 10);
        @(negedge sys_clk);
        sys_rst_n = 1'b1;
        $display("[TB] %t: System Reset Released.", $time);
        #(SYS_CLK_PERIOD * 10);

        // 测试 1: 写入 0x00 寄存器 0xA5
        $display("\n[TB] Writing 0xA5 to Address 0x00 (Mode 0 Single-Frame Write) ...");
        @(posedge sys_clk);
        wr_en <= 1'b1;
        addr  <= 7'h00;
        wdata <= 8'hA5;
        @(posedge sys_clk);
        wr_en <= 1'b0;

        @(posedge done);
        $display("[TB] %t: Write Operation Completed.", $time);

        // 测试 2: 双帧回读 0x00 寄存器 (Frame 2 发送 16'h0000)
        #(SYS_CLK_PERIOD * 20);
        $display("\n[TB] Reading from Address 0x00 (Mode 0 Two-Frame Read) ...");
        @(posedge sys_clk);
        rd_en <= 1'b1;
        addr  <= 7'h00;
        @(posedge sys_clk);
        rd_en <= 1'b0;

        @(posedge done);
        test_read_val = rdata;
        $display("[TB] %t: Read Operation Completed! Readback = 0x%02X", $time, test_read_val);

        if (test_read_val === 8'hA5) begin
            $display("[TB CHECK PASS] Readback Matched: Expected=0xA5, Actual=0x%02X", test_read_val);
        end else begin
            $display("[TB CHECK FAIL] Readback Mismatch! Expected=0xA5, Actual=0x%02X", test_read_val);
            error_count = error_count + 1;
        end

        #(SYS_CLK_PERIOD * 50);
        if (error_count == 0) begin
            $display("\n>>> ALL TESTS PASSED! SPI Mode 0 Verified 100%% Successfully! <<<");
        end else begin
            $display("\n>>> TEST FAILED with %0d errors! <<<", error_count);
        end
        $stop;
    end

endmodule
