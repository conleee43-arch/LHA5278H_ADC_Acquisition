//==============================================================================
// Module Name: lha5278h_data_rx
// Description: LHA5278H 8 通道并行高速数据接收模块
//              响应异步 DRDYn 下降沿，产生 24 个突发 SCLK 脉冲，
//              并行移位锁存 DOUT1~DOUT8 的 24-bit 补码转换结果并输出 valid 脉冲。
// Standard:    Verilog-2001 (纯同步时序设计，无门控时钟，无锁存器)
//==============================================================================

`timescale 1ns / 1ps

module lha5278h_data_rx #(
    parameter integer SYS_CLK_FREQ = 50_000_000, // FPGA 系统主时钟 (如 50MHz)
    parameter integer RX_SCLK_FREQ = 25_000_000  // 采集突发 SCLK 频率 (默认 25MHz，2分频，适配 512kSPS)
)(
    //--------------------------------------------------------------------------
    // 系统时钟与复位
    //--------------------------------------------------------------------------
    input  wire        sys_clk,       // FPGA 内部主时钟 (50MHz)
    input  wire        sys_rst_n,     // 全局异步复位，低有效

    //--------------------------------------------------------------------------
    // 控制与使能接口 (来自顶层 FSM)
    //--------------------------------------------------------------------------
    input  wire        rx_en,         // 数据接收总使能 (高电平开启采集)
    output reg         rx_busy,       // 正在进行一帧数据接收
    output reg         data_valid,    // 8 通道 24-bit 数据更新有效指示 (单周期高脉冲)

    //--------------------------------------------------------------------------
    // 8 通道解调后的并行数据输出 (24位二进制补码)
    //--------------------------------------------------------------------------
    output reg  [23:0] ch1_data,      // 通道 1 数据
    output reg  [23:0] ch2_data,      // 通道 2 数据
    output reg  [23:0] ch3_data,      // 通道 3 数据 (复用自 DOUT3)
    output reg  [23:0] ch4_data,      // 通道 4 数据 (复用自 DOUT4)
    output reg  [23:0] ch5_data,      // 通道 5 数据
    output reg  [23:0] ch6_data,      // 通道 6 数据
    output reg  [23:0] ch7_data,      // 通道 7 数据
    output reg  [23:0] ch8_data,      // 通道 8 数据

    //--------------------------------------------------------------------------
    // 物理 ADC 采集接口 (连接至 LHA5278H 物理管脚)
    //--------------------------------------------------------------------------
    input  wire        adc_drdy,      // DRDYn 数据就绪脉冲 (低有效，下降沿指示转换完成)
    output reg         rx_sclk,       // 驱动 ADC SCLK 引脚的 24 周期突发时钟
    input  wire [7:0]  adc_dout       // 8 路并行数据输入 {DOUT8, DOUT7, ..., DOUT1}
);

    //==========================================================================
    // 时钟分频与关键参数计算
    //==========================================================================
    // 计算每个 SCLK 周期对应的系统时钟数 (50MHz / 12.5MHz = 4)
    localparam integer CLK_DIV_TOTAL = (SYS_CLK_FREQ / RX_SCLK_FREQ < 2) ? 2 : (SYS_CLK_FREQ / RX_SCLK_FREQ);
    localparam integer CLK_DIV_HALF  = CLK_DIV_TOTAL / 2;
    localparam integer CNT_WIDTH     = 16;

    //==========================================================================
    // DRDY 异步信号同步与下降沿检测 (3级寄存器防亚稳态)
    //==========================================================================
    reg drdy_d1;
    reg drdy_d2;
    reg drdy_d3;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            drdy_d1 <= 1'b1;
            drdy_d2 <= 1'b1;
            drdy_d3 <= 1'b1;
        end else begin
            drdy_d1 <= adc_drdy;
            drdy_d2 <= drdy_d1;
            drdy_d3 <= drdy_d2;
        end
    end

    wire drdy_fall = (drdy_d3 == 1'b1) && (drdy_d2 == 1'b0);

    //==========================================================================
    // 状态机定义
    //==========================================================================
    localparam [1:0]
        S_RX_IDLE  = 2'd0,  // 空闲状态，等待 DRDY 下降沿
        S_RX_BURST = 2'd1,  // 突发产生 24 周期 SCLK 并接收 8 路数据
        S_RX_DONE  = 2'd2;  // 数据接收完成，锁存输出并拉高 valid

    reg [1:0]           rx_state;
    reg [CNT_WIDTH-1:0] div_cnt;
    reg [4:0]           bit_cnt; // 0 ~ 23，共 24 位

    // 8 组 24-bit 移位寄存器
    reg [23:0] shift_reg [0:7];

    integer i;

    //==========================================================================
    // 接收状态机与突发时钟生成逻辑
    //==========================================================================
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            rx_state   <= S_RX_IDLE;
            rx_busy    <= 1'b0;
            data_valid <= 1'b0;
            rx_sclk    <= 1'b0;
            div_cnt    <= {CNT_WIDTH{1'b0}};
            bit_cnt    <= 5'd0;
            ch1_data   <= 24'd0;
            ch2_data   <= 24'd0;
            ch3_data   <= 24'd0;
            ch4_data   <= 24'd0;
            ch5_data   <= 24'd0;
            ch6_data   <= 24'd0;
            ch7_data   <= 24'd0;
            ch8_data   <= 24'd0;
            for (i = 0; i < 8; i = i + 1) begin
                shift_reg[i] <= 24'd0;
            end
        end else begin
            data_valid <= 1'b0; // 默认拉低单周期脉冲

            case (rx_state)
                //--------------------------------------------------------------
                // 1. 空闲状态: 等待 DRDY 下降沿
                //--------------------------------------------------------------
                S_RX_IDLE: begin
                    rx_busy <= 1'b0;
                    rx_sclk <= 1'b0;
                    div_cnt <= {CNT_WIDTH{1'b0}};
                    bit_cnt <= 5'd0;
                    if (rx_en && drdy_fall) begin
                        rx_busy  <= 1'b1;
                        rx_state <= S_RX_BURST;
                    end
                end

                //--------------------------------------------------------------
                // 2. 突发 SCLK 生成与 8 通道数据移位采样
                //    每个 SCLK 周期分为前半周期(SCLK=1)与后半周期(SCLK=0)
                //    在 SCLK 上升沿时刻(div_cnt==0)启动时钟并采样当前 DOUT
                //--------------------------------------------------------------
                S_RX_BURST: begin
                    rx_busy <= 1'b1;

                    // 时钟波形生成: 前半周期高电平，后半周期低电平
                    if (div_cnt < CLK_DIV_HALF) begin
                        rx_sclk <= 1'b1;
                    end else begin
                        rx_sclk <= 1'b0;
                    end

                    // 分频计数与采样触发
                    if (div_cnt == 0) begin
                        // SCLK 上升沿点: 采样 8 路 DOUT 数据并移入移位寄存器
                        for (i = 0; i < 8; i = i + 1) begin
                            shift_reg[i] <= {shift_reg[i][22:0], adc_dout[i]};
                        end
                    end

                    if (div_cnt < CLK_DIV_TOTAL - 1) begin
                        div_cnt <= div_cnt + 1'b1;
                    end else begin
                        div_cnt <= {CNT_WIDTH{1'b0}};
                        if (bit_cnt < 5'd23) begin
                            bit_cnt <= bit_cnt + 1'b1;
                        end else begin
                            // 24 位全部接收完毕
                            rx_sclk  <= 1'b0;
                            rx_state <= S_RX_DONE;
                        end
                    end
                end

                //--------------------------------------------------------------
                // 3. 接收完成: 锁存 8 通道并行数据并输出 data_valid
                //--------------------------------------------------------------
                S_RX_DONE: begin
                    rx_sclk    <= 1'b0;
                    rx_busy    <= 1'b0;
                    data_valid <= 1'b1; // 单周期脉冲指示数据有效

                    ch1_data   <= shift_reg[0];
                    ch2_data   <= shift_reg[1];
                    ch3_data   <= shift_reg[2];
                    ch4_data   <= shift_reg[3];
                    ch5_data   <= shift_reg[4];
                    ch6_data   <= shift_reg[5];
                    ch7_data   <= shift_reg[6];
                    ch8_data   <= shift_reg[7];

                    rx_state   <= S_RX_IDLE;
                end

                default: begin
                    rx_state <= S_RX_IDLE;
                end
            endcase
        end
    end

endmodule
