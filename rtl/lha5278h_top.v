//==============================================================================
// Module Name: lha5278h_top
// Description: LHA5278H 8 通道 24-bit ADC FPGA 采集驱动控制系统顶层
//              包含工作模式调度 FSM (上电 -> SPI寄存器配置 -> 模式切换 -> SYNC同步 -> 512kSPS常规采集)
//              管理 FPGA 与 ADC 引脚复用多路选择器 (MUX)
// Standard:    Verilog-2001
//==============================================================================

`timescale 1ns / 1ps

module lha5278h_top #(
    parameter integer SYS_CLK_FREQ       = 50_000_000, // 板载 50MHz 时钟
    parameter integer SPI_SCLK_FREQ      = 5_000_000,  // SPI 配置模式 5MHz 时钟
    parameter integer RX_SCLK_FREQ       = 25_000_000, // 采样模式 25MHz 突发时钟 (512kSPS下24周期耗时960ns，留足充足传输余量)
    parameter integer RESET_WAIT_MS      = 10,         // 软复位后等待时间 (默认 10ms，满足芯片 2^18 CLK 周期，约 8~10ms)
    parameter integer SWITCH_WAIT_US     = 100,        // 模式切换后的稳定等待时间 (默认 100us)
    parameter integer SYNC_PULSE_US      = 10          // SYNC 同步负脉冲宽度 (默认 10us)
)(
    //--------------------------------------------------------------------------
    // 板载基础时钟与控制
    //--------------------------------------------------------------------------
    input  wire        sys_clk,         // 板载 50MHz 主时钟 (PIN_E1)
    input  wire        sys_rst_n,       // 板载复位按键 (PIN_E15)

    //--------------------------------------------------------------------------
    // LHA5278H 物理管脚接口
    //--------------------------------------------------------------------------
    output wire        adc_sync,        // PIN_T5, 低脉冲复位并同步内部滤波器
    input  wire        adc_drdy,        // PIN_T6, 数据转换完成就绪指示 (低脉冲)
    output wire        adc_din,         // PIN_R10, 级联数据输入 (单片模式恒接 0)
    output wire        adc_sclk,        // PIN_N5, SCLK 时钟 (复用: SPI 配置 / 采集突发)
    input  wire        adc_dout1,       // PIN_R3, CH1 数据
    input  wire        adc_dout2,       // PIN_N15, CH2 数据
    inout  wire        adc_dout3,       // PIN_R16, 复用引脚: 采样模式 CH3 / SPI 模式 SDI
    input  wire        adc_dout4,       // PIN_N12, 复用引脚: 采样模式 CH4 / SPI 模式 SDO
    input  wire        adc_dout5,       // PIN_T14, CH5 数据
    input  wire        adc_dout6,       // PIN_T13, CH6 数据
    input  wire        adc_dout7,       // PIN_P8, CH7 数据
    input  wire        adc_dout8,       // PIN_M6, CH8 数据
    output wire        adc_clkdiv_csn,  // PIN_R13, 复用引脚: 采样模式 CLKDIV / SPI 模式 CSn
    output wire        adc_test1,       // PIN_T15, 严格恒定输出 0 (禁止置 1)
    output reg         adc_test0,       // PIN_P16, 模式切换: 0=正常采样, 1=SPI配置

    //--------------------------------------------------------------------------
    // 板载状态指示接口
    //--------------------------------------------------------------------------
    output wire        sys_ready        // 系统就绪/寄存器配置校验成功指示 (1: 读写校验成功)
);

    //==========================================================================
    // 内部数据流接口线网声明 (连接至内部 FIFO/DMA/UART，不作为外部物理引脚引出)
    //==========================================================================
    wire        data_valid;
    wire [23:0] ch1_data;
    wire [23:0] ch2_data;
    wire [23:0] ch3_data;
    wire [23:0] ch4_data;
    wire [23:0] ch5_data;
    wire [23:0] ch6_data;
    wire [23:0] ch7_data;
    wire [23:0] ch8_data;

    //==========================================================================
    // 物理安全引脚强制电平与单片配置
    //==========================================================================
    // TEST1 严格锁定为 0，防止与 TEST0 组合进入 2'b11 危险测试模式
    assign adc_test1 = 1'b0;
    // 单片应用时 DIN 固定拉低
    assign adc_din   = 1'b0;

    // SYNC 控制寄存器
    reg adc_sync_reg;
    assign adc_sync = adc_sync_reg;

    //==========================================================================
    // SPI 寄存器配置驱动
    //==========================================================================
    reg         spi_wr_en;
    reg         spi_rd_en;
    reg  [6:0]  spi_addr;
    reg  [7:0]  spi_wdata;
    wire [7:0]  spi_rdata;
    wire        spi_busy;
    wire        spi_done;

    wire        spi_cs_n;
    wire        spi_sclk;
    wire        spi_sdi;
    wire        spi_sdo;

    lha5278h_spi_reg_ctrl #(
        .SYS_CLK_FREQ (SYS_CLK_FREQ),
        .SPI_SCLK_FREQ(SPI_SCLK_FREQ),
        .CS_GAP_CYCLES(4)
    ) u_spi_ctrl (
        .sys_clk   (sys_clk),
        .sys_rst_n (sys_rst_n),
        .wr_en     (spi_wr_en),
        .rd_en     (spi_rd_en),
        .addr      (spi_addr),
        .wdata     (spi_wdata),
        .rdata     (spi_rdata),
        .busy      (spi_busy),
        .done      (spi_done),
        .spi_cs_n  (spi_cs_n),
        .spi_sclk  (spi_sclk),
        .spi_sdi   (spi_sdi),
        .spi_sdo   (spi_sdo)
    );

    //==========================================================================
    // 8 通道并行数据接收器
    //==========================================================================
    reg         rx_en;
    wire        rx_busy;
    wire        rx_sclk;
    wire [7:0]  dout_bus;

    lha5278h_data_rx #(
        .SYS_CLK_FREQ(SYS_CLK_FREQ),
        .RX_SCLK_FREQ(RX_SCLK_FREQ)
    ) u_data_rx (
        .sys_clk   (sys_clk),
        .sys_rst_n (sys_rst_n),
        .rx_en     (rx_en),
        .rx_busy   (rx_busy),
        .data_valid(data_valid),
        .ch1_data  (ch1_data),
        .ch2_data  (ch2_data),
        .ch3_data  (ch3_data),
        .ch4_data  (ch4_data),
        .ch5_data  (ch5_data),
        .ch6_data  (ch6_data),
        .ch7_data  (ch7_data),
        .ch8_data  (ch8_data),
        .adc_drdy  (adc_drdy),
        .rx_sclk   (rx_sclk),
        .adc_dout  (dout_bus)
    );

    //==========================================================================
    // 引脚复用与三态门控制逻辑 (MUX & Tri-state Control)
    // 根据 adc_test0 (0: 采样模式, 1: SPI 模式) 进行无缝切换
    //==========================================================================
    wire is_spi_mode = adc_test0;

    // SCLK 引脚复用: SPI 模式由 spi_sclk 驱动，采样模式由 rx_sclk 驱动
    assign adc_sclk = is_spi_mode ? spi_sclk : rx_sclk;

    // CLKDIV / CSn 引脚复用: SPI 模式输出片选 spi_cs_n，采样模式输出 1'b0 (控制时钟分频电平)
    assign adc_clkdiv_csn = is_spi_mode ? spi_cs_n : 1'b0;

    // DOUT3 / SDI 双向引脚控制:
    // SPI 模式下作为 FPGA 输出驱动 SDI；采样模式下作为高阻输入接收 CH3 数据
    assign adc_dout3 = is_spi_mode ? spi_sdi : 1'bz;

    // DOUT4 / SDO 引脚连接:
    // SDO 连接至 SPI 控制器
    assign spi_sdo   = adc_dout4;

    // 8 通道并行接收数据总线拼接
    assign dout_bus  = {adc_dout8, adc_dout7, adc_dout6, adc_dout5,
                        adc_dout4, adc_dout3, adc_dout2, adc_dout1};

    //==========================================================================
    // 顶层状态机设计:
    //==========================================================================
    // 执行流程:
    // 1. 上电等待: TEST[1:0]=01 (adc_test0 <= 1)，等待芯片偏置稳定 (约 200us)
    // 2. 软件复位: 写入 0x0A 地址具体值为 0x16 (单帧写入 0x0A 0x16)
    // 3. 等待软复位完成: 芯片内部复位需 2^18 个 CLK 周期 (约 8~10ms，默认 10ms = 500,000 周期)
    // 4. 第 1 次读 0x09: 读取 0x09 寄存器复位后默认值
    // 5. 写 0x09 寄存器: 向 0x09 寄存器写入数据 0x20
    // 6. 再次读 0x09: 回读 0x09 寄存器，验证 0x20 是否成功写入
    // 7. 写 Reg[0x0F] = 0x28: 配置 FIR 滤波器, 目标 512kSPS
    // 8. 双帧回读 Reg[0x0F]: 验证写入 0x28 是否成功 (预期 0x28)
    // 9. TEST 切为 00: 将 TEST[1:0] 切换至 2'b00 (adc_test0 <= 0)，等待 100μs 稳定
    // 10. SYNC 同步: 产生 10μs 的 adc_sync 负脉冲，同步复位内部数字滤波器并启动 512kSPS 数据转换
    // 11. 正常采样: 使能 rx_en=1，以 25MHz 突发时钟持续接收 8 通道 24-bit 转换数据
    //==========================================================================
    localparam [4:0]
        S_IDLE          = 5'd0,  // 上电等待芯片偏置稳定 (TEST=01)
        S_WR_RESET      = 5'd1,  // 发起向 0x0A 写入 0x16 软复位
        S_WAIT_RESET_WR = 5'd2,  // 等待软复位写操作完成
        S_RESET_DELAY   = 5'd3,  // 等待软复位完成 (10ms)
        S_RD_09_1       = 5'd4,  // 发起第 1 次读取 0x09 寄存器
        S_WAIT_RD_09_1  = 5'd5,  // 等待第 1 次读取完成并锁存读回值
        S_WR_09         = 5'd6,  // 发起向 0x09 写入 0x20
        S_WAIT_WR_09    = 5'd7,  // 等待写入 0x20 完成
        S_RD_09_2       = 5'd8,  // 发起第 2 次读取 0x09 寄存器
        S_WAIT_RD_09_2  = 5'd9,  // 等待第 2 次读取完成
        S_WR_CTL0       = 5'd10, // 发起向 0x0F 写入 0x28 (目标 512kSPS)
        S_WAIT_CTL0_WR  = 5'd11, // 等待 0x0F 写入完成
        S_RD_CTL0       = 5'd12, // 发起双帧读取 0x0F 寄存器
        S_WAIT_RD_CTL0  = 5'd13, // 等待 0x0F 回读校验 (预期 0x28)
        S_SWITCH_MODE   = 5'd14, // TEST 切为 00，并等待 100μs
        S_SYNC_PULSE    = 5'd15, // SYNC 负脉冲同步 (10μs 低脉冲)
        S_SAMPLING      = 5'd16; // 正常采样运行状态 (rx_en=1，常规 512kSPS 数据接收)

    localparam integer RESET_WAIT_CYCLES  = (SYS_CLK_FREQ / 1000) * RESET_WAIT_MS;
    localparam integer SWITCH_WAIT_CYCLES = (SYS_CLK_FREQ / 1_000_000) * SWITCH_WAIT_US;
    localparam integer SYNC_PULSE_CYCLES  = (SYS_CLK_FREQ / 1_000_000) * SYNC_PULSE_US;

    reg [4:0]   top_fsm;
    reg [23:0]  timer_cnt;
    reg         test_pass_reg;
    reg [7:0]   reg_09_val1;     // 第 1 次读取 0x09 的初始值
    reg [7:0]   reg_09_val2;     // 第 2 次读取 0x09 的修改值
    reg [7:0]   reg_0f_val;      // 读取 0x0F 的回读验证值 (预期 0x28)

    assign sys_ready = test_pass_reg; // 寄存器 0x0F (512kSPS FIR) 回读校验成功指示 (PIN_A4)

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            top_fsm       <= S_IDLE;
            timer_cnt     <= 24'd0;
            adc_test0     <= 1'b1; // 上电默认 TEST[1:0]=2'b01 (进入 SPI 访问模式)
            adc_sync_reg  <= 1'b1; // SYNC 平时恒为高电平
            spi_wr_en     <= 1'b0;
            spi_rd_en     <= 1'b0;
            spi_addr      <= 7'd0;
            spi_wdata     <= 8'd0;
            rx_en         <= 1'b0;
            test_pass_reg <= 1'b0;
            reg_09_val1   <= 8'h00;
            reg_09_val2   <= 8'h00;
            reg_0f_val    <= 8'h00;
        end else begin
            case (top_fsm)
                //--------------------------------------------------------------
                // 1. 上电等待: TEST=01，等待供电与偏置建立 (约 200us)
                //--------------------------------------------------------------
                S_IDLE: begin
                    adc_test0     <= 1'b1; // 保持 TEST[1:0]=01 进入 SPI 访问模式
                    adc_sync_reg  <= 1'b1;
                    rx_en         <= 1'b0;
                    spi_wr_en     <= 1'b0;
                    spi_rd_en     <= 1'b0;
                    test_pass_reg <= 1'b0;
                    if (timer_cnt < 24'd10_000) begin
                        timer_cnt <= timer_cnt + 1'b1;
                    end else begin
                        timer_cnt <= 24'd0;
                        top_fsm   <= S_WR_RESET;
                    end
                end

                //--------------------------------------------------------------
                // 2. 软件复位: 单帧写入 0x0A 寄存器为 0x16
                //--------------------------------------------------------------
                S_WR_RESET: begin
                    spi_wr_en <= 1'b1;
                    spi_addr  <= 7'h0A; // 复位寄存器地址 0x0A
                    spi_wdata <= 8'h16; // 复位数值 0x16
                    top_fsm   <= S_WAIT_RESET_WR;
                end

                S_WAIT_RESET_WR: begin
                    spi_wr_en <= 1'b0;
                    if (spi_done) begin
                        timer_cnt <= 24'd0;
                        top_fsm   <= S_RESET_DELAY;
                    end
                end

                //--------------------------------------------------------------
                // 3. 等待软复位完成: 芯片内部复位需 2^18 个 CLK 周期 (约 8~10ms)
                //--------------------------------------------------------------
                S_RESET_DELAY: begin
                    if (timer_cnt < RESET_WAIT_CYCLES) begin
                        timer_cnt <= timer_cnt + 1'b1;
                    end else begin
                        timer_cnt <= 24'd0;
                        top_fsm   <= S_RD_09_1; // 复位完成，发起第 1 次读取 0x09 寄存器
                    end
                end

                //--------------------------------------------------------------
                // 4. 发起第 1 次读取 0x09 寄存器
                //--------------------------------------------------------------
                S_RD_09_1: begin
                    spi_rd_en <= 1'b1;
                    spi_addr  <= 7'h09; // 读取地址 0x09
                    top_fsm   <= S_WAIT_RD_09_1;
                end

                S_WAIT_RD_09_1: begin
                    spi_rd_en <= 1'b0;
                    if (spi_done) begin
                        reg_09_val1 <= spi_rdata; // 锁存第 1 次读取结果
                        top_fsm     <= S_WR_09;   // 发起向 0x09 写入 0x20
                    end
                end

                //--------------------------------------------------------------
                // 5. 向 0x09 写入 0x20
                //--------------------------------------------------------------
                S_WR_09: begin
                    spi_wr_en <= 1'b1;
                    spi_addr  <= 7'h09; // 目标地址 0x09
                    spi_wdata <= 8'h20; // 写入数据 0x20
                    top_fsm   <= S_WAIT_WR_09;
                end

                S_WAIT_WR_09: begin
                    spi_wr_en <= 1'b0;
                    if (spi_done) begin
                        top_fsm <= S_RD_09_2; // 写入完成，发起第 2 次读取 0x09
                    end
                end

                //--------------------------------------------------------------
                // 6. 再次读取 0x09 寄存器
                //--------------------------------------------------------------
                S_RD_09_2: begin
                    spi_rd_en <= 1'b1;
                    spi_addr  <= 7'h09; // 再次读取地址 0x09
                    top_fsm   <= S_WAIT_RD_09_2;
                end

                S_WAIT_RD_09_2: begin
                    spi_rd_en <= 1'b0;
                    if (spi_done) begin
                        reg_09_val2 <= spi_rdata; // 锁存第 2 次读取结果
                        top_fsm     <= S_WR_CTL0; // 0x09 读写完毕，进入 0x0F FIR 512kSPS 配置！
                    end
                end

                //--------------------------------------------------------------
                // 7. 写 Reg[0x0F] = 0x28: 配置 0x0F 寄存器, 目标 512kSPS
                //--------------------------------------------------------------
                S_WR_CTL0: begin
                    spi_wr_en <= 1'b1;
                    spi_addr  <= 7'h0F; // ADC_CTL0 寄存器地址 0x0F
                    spi_wdata <= 8'h28; // 写入 0x28
                    top_fsm   <= S_WAIT_CTL0_WR;
                end

                S_WAIT_CTL0_WR: begin
                    spi_wr_en <= 1'b0;
                    if (spi_done) begin
                        top_fsm <= S_RD_CTL0; // 写入完成，发起双帧读取验证
                    end
                end

                //--------------------------------------------------------------
                // 8. 双帧回读 Reg[0x0F] 验证写入的 0x28
                //--------------------------------------------------------------
                S_RD_CTL0: begin
                    spi_rd_en <= 1'b1;
                    spi_addr  <= 7'h0F; // 回读地址 0x0F
                    top_fsm   <= S_WAIT_RD_CTL0;
                end

                S_WAIT_RD_CTL0: begin
                    spi_rd_en <= 1'b0;
                    if (spi_done) begin
                        reg_0f_val <= spi_rdata; // 锁存 0x0F 回读值
                        if (spi_rdata == 8'h28) begin
                            test_pass_reg <= 1'b1; // 回读匹配 0x28，校验通过！
                        end else begin
                            test_pass_reg <= 1'b0;
                        end
                        timer_cnt <= 24'd0;
                        top_fsm   <= S_SWITCH_MODE; // 进入模式切换
                    end
                end

                //--------------------------------------------------------------
                // 9. 模式切换: 将 TEST[1:0] 切换至 2'b00 并等待 100μs 稳定
                //--------------------------------------------------------------
                S_SWITCH_MODE: begin
                    adc_test0 <= 1'b0; // 切为 TEST[1:0] = 2'b00 (正常工作模式)
                    if (timer_cnt < SWITCH_WAIT_CYCLES) begin
                        timer_cnt <= timer_cnt + 1'b1;
                    end else begin
                        timer_cnt    <= 24'd0;
                        adc_sync_reg <= 1'b0; // 启动 SYNC 低脉冲同步
                        top_fsm      <= S_SYNC_PULSE;
                    end
                end

                //--------------------------------------------------------------
                // 10. SYNC 同步负脉冲: 持续 10μs 低脉冲
                //     复位内部数字滤波器计数器，重新同步并启动 ADC 转换
                //--------------------------------------------------------------
                S_SYNC_PULSE: begin
                    if (timer_cnt < SYNC_PULSE_CYCLES) begin
                        timer_cnt <= timer_cnt + 1'b1;
                    end else begin
                        timer_cnt    <= 24'd0;
                        adc_sync_reg <= 1'b1; // 恢复高电平，转换正式开始
                        rx_en        <= 1'b1; // 开启数据接收器
                        top_fsm      <= S_SAMPLING;
                    end
                end

                //--------------------------------------------------------------
                // 11. 正常采样运行状态: 保持在常规模式，以 25MHz 突发时钟持续采集 512kSPS 数据
                //--------------------------------------------------------------
                S_SAMPLING: begin
                    rx_en     <= 1'b1;
                    adc_test0 <= 1'b0; // 持续保持 TEST[1:0] = 2'b00
                end

                default: begin
                    top_fsm <= S_IDLE;
                end
            endcase
        end
    end

endmodule
