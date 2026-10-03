//==============================================================================
// Module Name: lha5278h_spi_reg_ctrl
// Description: LHA5278H 8通道 24位 Delta-Sigma ADC 芯片通用 SPI 寄存器读写控制模块
//              实现对 LHA5278H 内部寄存器的配置与状态回读
// Standard:    Verilog-2001 (纯同步时序设计，无门控时钟，无锁存器)
// Protocol:    SPI Mode 1 (CPOL = 0, CPHA = 1)
//              - 空闲时钟: SCLK = 0, CSN = 1
//              - 数据更新: SCLK 上升沿更新 MOSI(SDI) 与从机更新 MISO(SDO)
//              - 数据采样: SCLK 下降沿采样 MISO(SDO) 与从机采样 MOSI(SDI)
//              - 建立与保持余量:
//                CSN 拉低后延时 CS_LEAD_CYCLES 个周期再启动 SCLK 翻转;
//                传输结束后延时 CS_LAG_CYCLES 个周期再拉高 CSN。
//              - 帧结构:
//                单帧写操作: 发送 {1'b0, ADDR[6:0], WDATA[7:0]}
//                双帧读操作:
//                  Frame 1: 发送 {1'b1, ADDR[6:0], 8'h00}
//                  CSN间歇: 拉高保持 CS_GAP_CYCLES 个周期
//                  Frame 2: CSN拉低，MOSI 发送 16'h8000，前8个SCLK下降沿采样 RDATA[7:0]
//==============================================================================

`timescale 1ns / 1ps

module lha5278h_spi_reg_ctrl #(
    parameter integer SYS_CLK_FREQ   = 50_000_000, // 系统主时钟频率 (默认 50MHz)
    parameter integer SPI_SCLK_FREQ  = 5_000_000,  // SPI SCLK 目标频率 (默认 5MHz)
    parameter integer CS_GAP_CYCLES  = 4,          // 双帧读取时两帧之间 CSN 保持高电平的 SCLK 周期数 (>=4)
    parameter integer CS_LEAD_CYCLES = 2,          // CSN 拉低后到第一个 SCLK 上升沿的建立延时 SCLK 周期数 (1~2)
    parameter integer CS_LAG_CYCLES  = 2           // 最后一个 SCLK 结束后到 CSN 拉高的保持延时 SCLK 周期数 (1~2)
)(
    //--------------------------------------------------------------------------
    // 系统时钟与复位
    //--------------------------------------------------------------------------
    input  wire        sys_clk,     // 系统主时钟输入 (50MHz)
    input  wire        sys_rst_n,   // 全局异步复位，低电平有效

    //--------------------------------------------------------------------------
    // 用户逻辑控制接口
    //--------------------------------------------------------------------------
    input  wire        wr_en,       // 写寄存器请求使能 (单周期高电平脉冲)
    input  wire        rd_en,       // 读寄存器请求使能 (单周期高电平脉冲)
    input  wire [6:0]  addr,        // 目标寄存器地址 (7位: [6:0])
    input  wire [7:0]  wdata,       // 待写入寄存器的数据 (8位: [7:0])
    output reg  [7:0]  rdata,       // 读回的寄存器数据 (8位: [7:0])
    output reg         busy,        // 模块忙指示信号 (高电平表示正在执行SPI事务)
    output reg         done,        // 操作完成指示脉冲 (高电平单周期脉冲)

    //--------------------------------------------------------------------------
    // 物理 SPI 接口 (连接至 LHA5278H 芯片引脚)
    //--------------------------------------------------------------------------
    output reg         spi_cs_n,    // SPI 片选信号 (复用至 ADC 的 CLKDIV/CSn 引脚，低电平有效)
    output reg         spi_sclk,    // SPI 串行同步时钟 (复用至 ADC 的 SCLK 引脚，CPOL=0)
    output reg         spi_sdi,     // SPI 主机数据输出 (复用至 ADC 的 DOUT3/SDI 引脚, MOSI)
    input  wire        spi_sdo      // SPI 主机数据输入 (连接至 ADC 的 DOUT4/SDO 引脚, MISO)
);

    //==========================================================================
    // 常量计算与时钟分频参数
    //==========================================================================
    localparam integer CLK_DIV_TOTAL = SYS_CLK_FREQ / SPI_SCLK_FREQ; // 默认 50M / 5M = 10
    localparam integer CLK_DIV_HALF  = CLK_DIV_TOTAL / 2;            // 默认 5
    localparam integer CNT_WIDTH     = 16;

    //==========================================================================
    // 状态机状态编码定义
    //==========================================================================
    localparam [3:0]
        ST_IDLE      = 4'd0,  // 空闲状态，监听 wr_en / rd_en
        ST_WR_LEAD   = 4'd1,  // 写操作 CSN 前置建立延时 (1~2 SCLK 周期)
        ST_WR_TX     = 4'd2,  // 写操作 16-bit 移位传输
        ST_WR_LAG    = 4'd3,  // 写操作 CSN 后置保持延时 (1~2 SCLK 周期)
        ST_RD_F1_LEAD= 4'd4,  // 读 Frame 1 前置建立延时
        ST_RD_F1_TX  = 4'd5,  // 读 Frame 1 传输 {1'b1, addr, 8'h00}
        ST_RD_F1_LAG = 4'd6,  // 读 Frame 1 后置保持延时
        ST_RD_GAP    = 4'd7,  // 读操作双帧间隔 (CSN 保持高电平 >=4 SCLK 周期)
        ST_RD_F2_LEAD= 4'd8,  // 读 Frame 2 前置建立延时
        ST_RD_F2_TX  = 4'd9,  // 读 Frame 2 传输 16'h0000 并采样 SDO
        ST_RD_F2_LAG = 4'd10, // 读 Frame 2 后置保持延时
        ST_DONE      = 4'd11; // 完成状态，输出 done 脉冲与更新 rdata

    reg [3:0] cur_state;
    reg [3:0] nxt_state;

    //==========================================================================
    // 内部寄存器与信号声明
    //==========================================================================
    // 异步输入同步打拍 (SDO 信号跨时钟域两级防亚稳态处理)
    reg        sdo_sync_m;
    reg        sdo_sync;

    // 分频计数器与关键节拍
    reg [CNT_WIDTH-1:0] div_cnt;
    wire                tick_sample;// SCLK 高电平中间采样点 (MISO 数据稳定)
    wire                tick_half;  // 半周期翻转点 (div_cnt == CLK_DIV_HALF - 1, SCLK 下降沿)
    wire                tick_full;  // 周期结束点 (div_cnt == CLK_DIV_TOTAL - 1)

    // 传输位计数器与余量延时计数器
    reg [4:0]           bit_cnt;
    reg [7:0]           delay_cnt;

    // 数据移位寄存器
    reg [15:0]          tx_shift;
    reg [7:0]           rx_shift;

    //==========================================================================
    // 1. SDO 异步信号两级打拍同步处理 (防亚稳态)
    //==========================================================================
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            sdo_sync_m <= 1'b0;
            sdo_sync   <= 1'b0;
        end else begin
            sdo_sync_m <= spi_sdo;
            sdo_sync   <= sdo_sync_m;
        end
    end

    //==========================================================================
    // 2. SPI 分频计数器逻辑
    //    当处于忙碌状态(busy=1)时持续以 CLK_DIV_TOTAL 循环计数
    //==========================================================================
    assign tick_sample = (div_cnt == (CLK_DIV_HALF - 1)); // Mode 1: SCLK 下降沿数据采样点 (数据稳定建立半周期后采样)
    assign tick_half   = (div_cnt == (CLK_DIV_HALF - 1)); // 半周期翻转点 (SCLK 下降沿)
    assign tick_full   = (div_cnt == (CLK_DIV_TOTAL - 1)); // 周期结束点

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            div_cnt <= {CNT_WIDTH{1'b0}};
        end else if (!busy) begin
            div_cnt <= {CNT_WIDTH{1'b0}};
        end else if (tick_full) begin
            div_cnt <= {CNT_WIDTH{1'b0}};
        end else begin
            div_cnt <= div_cnt + 1'b1;
        end
    end

    //==========================================================================
    // 3. 状态机跳转时序逻辑 (第 1 段)
    //==========================================================================
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            cur_state <= ST_IDLE;
        end else begin
            cur_state <= nxt_state;
        end
    end

    //==========================================================================
    // 4. 状态机次态组合逻辑 (第 2 段)
    //==========================================================================
    always @(*) begin
        nxt_state = cur_state;
        case (cur_state)
            ST_IDLE: begin
                if (wr_en) begin
                    nxt_state = ST_WR_LEAD;
                end else if (rd_en) begin
                    nxt_state = ST_RD_F1_LEAD;
                end else begin
                    nxt_state = ST_IDLE;
                end
            end

            //------------------------------------------------------------------
            // 写操作流程: 前置建立 -> 16位传输 -> 后置保持
            //------------------------------------------------------------------
            ST_WR_LEAD: begin
                if ((delay_cnt >= (CS_LEAD_CYCLES - 1)) && tick_full) begin
                    nxt_state = ST_WR_TX;
                end
            end

            ST_WR_TX: begin
                if ((bit_cnt == 5'd15) && tick_full) begin
                    nxt_state = ST_WR_LAG;
                end
            end

            ST_WR_LAG: begin
                if ((delay_cnt >= (CS_LAG_CYCLES - 1)) && tick_full) begin
                    nxt_state = ST_DONE;
                end
            end

            //------------------------------------------------------------------
            // 读操作 Frame 1: 前置建立 -> 发送寻址帧 -> 后置保持
            //------------------------------------------------------------------
            ST_RD_F1_LEAD: begin
                if ((delay_cnt >= (CS_LEAD_CYCLES - 1)) && tick_full) begin
                    nxt_state = ST_RD_F1_TX;
                end
            end

            ST_RD_F1_TX: begin
                if ((bit_cnt == 5'd15) && tick_full) begin
                    nxt_state = ST_RD_F1_LAG;
                end
            end

            ST_RD_F1_LAG: begin
                if ((delay_cnt >= (CS_LAG_CYCLES - 1)) && tick_full) begin
                    nxt_state = ST_RD_GAP;
                end
            end

            //------------------------------------------------------------------
            // 双帧间隔: CSN 保持高电平 >= CS_GAP_CYCLES 个时钟周期
            //------------------------------------------------------------------
            ST_RD_GAP: begin
                if ((delay_cnt >= (CS_GAP_CYCLES - 1)) && tick_full) begin
                    nxt_state = ST_RD_F2_LEAD;
                end
            end

            //------------------------------------------------------------------
            // 读操作 Frame 2: 前置建立 -> 发送 16'h0000 并读入数据 -> 后置保持
            //------------------------------------------------------------------
            ST_RD_F2_LEAD: begin
                if ((delay_cnt >= (CS_LEAD_CYCLES - 1)) && tick_full) begin
                    nxt_state = ST_RD_F2_TX;
                end
            end

            ST_RD_F2_TX: begin
                if ((bit_cnt == 5'd15) && tick_full) begin
                    nxt_state = ST_RD_F2_LAG;
                end
            end

            ST_RD_F2_LAG: begin
                if ((delay_cnt >= (CS_LAG_CYCLES - 1)) && tick_full) begin
                    nxt_state = ST_DONE;
                end
            end

            //------------------------------------------------------------------
            // 事务完成
            //------------------------------------------------------------------
            ST_DONE: begin
                nxt_state = ST_IDLE;
            end

            default: begin
                nxt_state = ST_IDLE;
            end
        endcase
    end

    //==========================================================================
    // 5. 数据通路控制与 SPI Mode 1 接口时序输出 (第 3 段)
    //    Mode 1 (CPOL=0, CPHA=1):
    //    - 空闲: SCLK = 0, CSN = 1
    //    - 建立延时(Lead): CSN 拉低, SCLK 保持 0, 提前预置首位 MOSI
    //    - 周期前半 (0 ~ HALF-1): SCLK = 1, 上升沿更新 MOSI / 从机更新 MISO
    //    - 周期后半 (HALF ~ TOTAL-1): SCLK = 0, 下降沿采样 MISO / 从机采样 MOSI
    //    - 保持延时(Lag): SCLK 保持 0, 延时后拉高 CSN
    //==========================================================================
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            bit_cnt   <= 5'd0;
            delay_cnt <= 8'd0;
            tx_shift  <= 16'h0000;
            rx_shift  <= 8'h00;
            rdata     <= 8'h00;
            busy      <= 1'b0;
            done      <= 1'b0;
            spi_cs_n  <= 1'b1; // CSN 默认空闲高电平
            spi_sclk  <= 1'b0; // CPOL = 0, 空闲时钟为低电平
            spi_sdi   <= 1'b0; // MOSI 默认低电平
        end else begin
            case (cur_state)
                ST_IDLE: begin
                    done      <= 1'b0;
                    spi_cs_n  <= 1'b1;
                    spi_sclk  <= 1'b0;
                    spi_sdi   <= 1'b0;
                    bit_cnt   <= 5'd0;
                    delay_cnt <= 8'd0;

                    if (wr_en) begin
                        busy      <= 1'b1;
                        // 装载 16 位写帧数据: {1'b0(写), addr[6:0], wdata[7:0]}
                        tx_shift  <= {1'b0, addr, wdata};
                        spi_cs_n  <= 1'b0; // CSN 拉低开启操作
                        spi_sdi   <= 1'b0; // 提前驱动首位 (最高位为 0)
                    end else if (rd_en) begin
                        busy      <= 1'b1;
                        // 装载 16 位读 Frame 1 数据: {1'b1(读), addr[6:0], 8'h00}
                        tx_shift  <= {1'b1, addr, 8'h00};
                        spi_cs_n  <= 1'b0; // CSN 拉低开启操作
                        spi_sdi   <= 1'b1; // 提前驱动首位 (最高位为 1)
                    end else begin
                        busy <= 1'b0;
                    end
                end

                //--------------------------------------------------------------
                // 写操作: 前置建立延时 (1~2 SCLK 周期)
                // SCLK 保持 0, SDI 保持首位数据, CSN 保持 0
                //--------------------------------------------------------------
                ST_WR_LEAD: begin
                    spi_cs_n <= 1'b0;
                    spi_sclk <= 1'b0;
                    spi_sdi  <= tx_shift[15]; // 首位提前稳定保持
                    bit_cnt  <= 5'd0;
                    if (tick_full) begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                //--------------------------------------------------------------
                //--------------------------------------------------------------
                // 写操作: 16-bit 时钟移位传输 (Mode 1: 上升沿移出, 下降沿采样)
                // 前半周期为高 (SCLK=1), 后半周期为低 (SCLK=0)
                //--------------------------------------------------------------
                ST_WR_TX: begin
                    spi_cs_n  <= 1'b0;
                    delay_cnt <= 8'd0;

                    // 生成 SCLK 时钟波形 (前半为高，后半为低)
                    if (div_cnt < CLK_DIV_HALF) begin
                        spi_sclk <= 1'b1;
                    end else begin
                        spi_sclk <= 1'b0;
                    end

                    // Mode 1: SCLK 上升沿时刻更新移出下一位 SDI
                    if (div_cnt == 0) begin
                        spi_sdi  <= tx_shift[15];
                        tx_shift <= {tx_shift[14:0], 1'b0};
                    end

                    // 在周期结束时刻更新 bit 计数器
                    if (tick_full) begin
                        if (bit_cnt < 5'd15) begin
                            bit_cnt <= bit_cnt + 1'b1;
                        end else begin
                            bit_cnt <= 5'd0;
                        end
                    end
                end

                //--------------------------------------------------------------
                // 写操作: 后置保持延时 (1~2 SCLK 周期)
                // 最后一个脉冲结束后 SCLK 保持 0，延时满足后拉高 CSN
                //--------------------------------------------------------------
                ST_WR_LAG: begin
                    spi_sclk <= 1'b0;
                    spi_sdi  <= 1'b0;
                    bit_cnt  <= 5'd0;
                    if (tick_full) begin
                        delay_cnt <= delay_cnt + 1'b1;
                        if (delay_cnt >= (CS_LAG_CYCLES - 1)) begin
                            spi_cs_n  <= 1'b1; // 保持延时结束，拉高 CSN
                            delay_cnt <= 8'd0;
                        end
                    end
                end

                //--------------------------------------------------------------
                // 读 Frame 1: 前置建立延时
                //--------------------------------------------------------------
                ST_RD_F1_LEAD: begin
                    spi_cs_n <= 1'b0;
                    spi_sclk <= 1'b0;
                    spi_sdi  <= tx_shift[15];
                    bit_cnt  <= 5'd0;
                    if (tick_full) begin
                        delay_cnt <= delay_cnt + 1'b1;
                        if (delay_cnt >= (CS_LEAD_CYCLES - 1)) begin
                            delay_cnt <= 8'd0;
                        end
                    end
                end

                //--------------------------------------------------------------
                // 读 Frame 1: 发送寻址帧 {1'b1, addr, 8'h00} (Mode 1)
                //--------------------------------------------------------------
                ST_RD_F1_TX: begin
                    spi_cs_n  <= 1'b0;
                    delay_cnt <= 8'd0;

                    if (div_cnt < CLK_DIV_HALF) begin
                        spi_sclk <= 1'b1;
                    end else begin
                        spi_sclk <= 1'b0;
                    end

                    // Mode 1: SCLK 上升沿时刻更新移出下一位 SDI
                    if (div_cnt == 0) begin
                        spi_sdi  <= tx_shift[15];
                        tx_shift <= {tx_shift[14:0], 1'b0};
                    end

                    if (tick_full) begin
                        if (bit_cnt < 5'd15) begin
                            bit_cnt <= bit_cnt + 1'b1;
                        end else begin
                            bit_cnt <= 5'd0;
                        end
                    end
                end

                //--------------------------------------------------------------
                // 读 Frame 1: 后置保持延时
                //--------------------------------------------------------------
                ST_RD_F1_LAG: begin
                    spi_sclk <= 1'b0;
                    spi_sdi  <= 1'b0;
                    bit_cnt  <= 5'd0;
                    if (tick_full) begin
                        delay_cnt <= delay_cnt + 1'b1;
                        if (delay_cnt >= (CS_LAG_CYCLES - 1)) begin
                            spi_cs_n  <= 1'b1; // 结束 Frame 1，拉高 CSN
                            delay_cnt <= 8'd0;
                        end
                    end
                end

                //--------------------------------------------------------------
                // 读双帧间隔 (CSN 保持高电平 >= CS_GAP_CYCLES 周期)
                //--------------------------------------------------------------
                ST_RD_GAP: begin
                    spi_cs_n  <= 1'b1;
                    spi_sclk  <= 1'b0;
                    spi_sdi   <= 1'b1; // Frame 2 MOSI 发送 {1'b1, 7'b0, 8'h00} (16'h8000, 0x80 0x00)
                    tx_shift  <= 16'h8000;
                    bit_cnt   <= 5'd0;

                    if (tick_full) begin
                        delay_cnt <= delay_cnt + 1'b1;
                        if (delay_cnt >= (CS_GAP_CYCLES - 1)) begin
                            // 间歇结束，准备进入 Frame 2 前置建立延时
                            spi_cs_n  <= 1'b0;
                            delay_cnt <= 8'd0;
                        end
                    end
                end

                //--------------------------------------------------------------
                // 读 Frame 2: 前置建立延时
                //--------------------------------------------------------------
                ST_RD_F2_LEAD: begin
                    spi_cs_n <= 1'b0;
                    spi_sclk <= 1'b0;
                    spi_sdi  <= tx_shift[15]; // 首位保持 1 (Read-1)
                    bit_cnt  <= 5'd0;
                    if (tick_full) begin
                        delay_cnt <= delay_cnt + 1'b1;
                        if (delay_cnt >= (CS_LEAD_CYCLES - 1)) begin
                            delay_cnt <= 8'd0;
                        end
                    end
                end

                //--------------------------------------------------------------
                // 读 Frame 2: MOSI 发送 16'h8000 (0x80 0x00)，SCLK 下降沿采样 SDO (Mode 1)
                //--------------------------------------------------------------
                ST_RD_F2_TX: begin
                    spi_cs_n  <= 1'b0;
                    delay_cnt <= 8'd0;

                    if (div_cnt < CLK_DIV_HALF) begin
                        spi_sclk <= 1'b1;
                    end else begin
                        spi_sclk <= 1'b0;
                    end

                    // Mode 1: SCLK 上升沿时刻更新移出下一位 SDI
                    if (div_cnt == 0) begin
                        spi_sdi  <= tx_shift[15];
                        tx_shift <= {tx_shift[14:0], 1'b0};
                    end

                    // Mode 1: 在 SCLK 下降沿时刻 (tick_sample)，采样从机 SDO 输出数据
                    // 仅在前 8 个周期 (bit_cnt 0 ~ 7) 锁存，高位在先 (MSB First)
                    if (tick_sample) begin
                        if (bit_cnt < 5'd8) begin
                            rx_shift <= {rx_shift[6:0], sdo_sync};
                        end
                    end

                    // 在周期结束时刻更新 bit 计数器
                    if (tick_full) begin
                        if (bit_cnt < 5'd15) begin
                            bit_cnt <= bit_cnt + 1'b1;
                        end else begin
                            bit_cnt <= 5'd0;
                        end
                    end
                end

                //--------------------------------------------------------------
                // 读 Frame 2: 后置保持延时
                //--------------------------------------------------------------
                ST_RD_F2_LAG: begin
                    spi_sclk <= 1'b0;
                    spi_sdi  <= 1'b0;
                    bit_cnt  <= 5'd0;
                    if (tick_full) begin
                        delay_cnt <= delay_cnt + 1'b1;
                        if (delay_cnt >= (CS_LAG_CYCLES - 1)) begin
                            spi_cs_n  <= 1'b1;     // 结束 Frame 2，拉高 CSN
                            delay_cnt <= 8'd0;
                            rdata     <= rx_shift; // 锁存最终读出数据
                        end
                    end
                end

                //--------------------------------------------------------------
                // 事务完成状态
                //--------------------------------------------------------------
                ST_DONE: begin
                    busy     <= 1'b0;
                    done     <= 1'b1;
                    spi_cs_n <= 1'b1;
                    spi_sclk <= 1'b0;
                    spi_sdi  <= 1'b0;
                end

                default: begin
                    busy     <= 1'b0;
                    done     <= 1'b0;
                    spi_cs_n <= 1'b1;
                    spi_sclk <= 1'b0;
                    spi_sdi  <= 1'b0;
                end
            endcase
        end
    end

endmodule
