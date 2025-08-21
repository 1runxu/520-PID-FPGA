//# 520-PID-FPGA
//实现速度环控制
//PID,反馈时间，successful
module encoder_counter (
    input               clk,
    input               rst_n,          // 低电平复位
    input               encoder_a,
    input               encoder_b,
    input               cnt_clear,
    output reg signed [15:0]   count,      // 计数输出（带符号）
    output reg          direction,      // 1=顺时针，0=逆时针
    output reg          data_ready      // 数据就绪信号，1秒脉冲
);

reg [19:0] timer;
reg [1:0] curr_state, prev_state;  // 当前/上一状态（{A,B}）
reg [1:0] sync_a, sync_b;          // 同步寄存器，用于消除亚稳态

// 同步编码器信号，消除亚稳态
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sync_a <= 2'b00;
        sync_b <= 2'b00;
    end else begin
        sync_a <= {sync_a[0], encoder_a};
        sync_b <= {sync_b[0], encoder_b};
    end
end

// 状态转换逻辑（四倍频）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        count <= 16'd0;
		timer <= 0;
        direction <= 1'b0;
        curr_state <= 2'b00;
        prev_state <= 2'b00;
        data_ready <= 1'b0;
    end else if (cnt_clear) begin  // UART发送完成后清零
        count <= 0;
    end else begin
        // 锁存当前状态（使用同步后的信号）
        prev_state <= curr_state;
        curr_state <= {sync_a[1], sync_b[1]};
        
        data_ready <= 1'b0;  // 默认数据未就绪
        
		if(timer < 20'd249999) begin
            timer <= timer + 1;
            
            // 只有状态变化时才进行计数判断
            if (curr_state != prev_state) begin
                case ({prev_state, curr_state})
                    4'b0001, 4'b0111, 4'b1110, 4'b1000: begin  // 顺时针
                        count <= count + 1'b1;
                        direction <= 1'b1;
                    end
                    4'b0010, 4'b1011, 4'b1101, 4'b0100: begin  // 逆时针
                        count <= count - 1'b1;
                        direction <= 1'b0;
                    end
                    default: begin  // 无效转换（如抖动），保持计数
                        count <= count;
                        direction <= direction;
                    end
                endcase
            end
		end else begin
            timer <= 0;               // 1秒到，重置定时器
            data_ready <= 1'b1;       // 置位数据就绪信号
            // 保持计数值，等待UART发送完成后再清零
        end
	end
end

endmodule

module speed_calc_uart(
    input clk,
    input rst_n,
    input signed [15:0] pulse_cnt,  // 来自计数模块的脉冲数
    input data_ready,        // 数据就绪信号（1秒一次）
    output reg uart_tx,      // 串口发送线
    output reg [15:0] rpm,
    output reg cnt_clear     // 计数清零信号
);

    // 参数定义
    parameter CLK_FREQ = 50000000;       // 系统时钟频率 (Hz)
    parameter BAUD_RATE = 115200;        // 波特率
    parameter DIVISOR = CLK_FREQ/BAUD_RATE;  // 波特率分频器
    parameter BEISHU = 12000;
    
    // 内部信号
    reg [31:0] clk_div;                  // 时钟分频计数器
    reg bit_tick;                        // 波特率时钟使能
    reg [3:0] bit_cnt;                   // 位计数器
    reg [7:0] tx_data;                   // 当前发送字节
    reg [7:0] tx_buffer [0:11];          // 发送缓冲区
    reg [3:0] buffer_idx;                // 缓冲区索引
    reg tx_busy;                         // 发送忙标志
    reg [3:0] state;                     // 状态机状态
    reg data_ready_dly;                  // 数据就绪信号延迟
    reg new_data;                        // 新数据标志
    reg [15:0] pulse_cnt_1;
    // 状态机状态定义
    localparam IDLE     = 4'd0;
    localparam LOAD     = 4'd1;
    localparam START    = 4'd2;
    localparam DATA     = 4'd3;
    localparam STOP     = 4'd4;
    localparam CLEAR    = 4'd5;
    
    // 检测数据就绪信号的上升沿
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_ready_dly <= 1'b0;
            new_data <= 1'b0;
        end else begin
            data_ready_dly <= data_ready;
            new_data <= data_ready & ~data_ready_dly;  // 上升沿检测
        end
    end
    
// 补码转换（时序逻辑版）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pulse_cnt_1 <= 16'd0;  // 复位时初始化为0
    end else begin
        if (pulse_cnt[15] == 1'b1) begin  // 负数：补码转绝对值（取反加1）
            pulse_cnt_1 <= ~pulse_cnt + 1'b1;
        end else begin  // 正数：直接赋值
            pulse_cnt_1 <= pulse_cnt;
        end
    end
end

// 转速计算（时序逻辑版）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rpm <= 16'd0;  // 复位时初始化为0
    end else begin
        if (new_data) begin  // 仅在新数据就绪时计算一次
            rpm <= (pulse_cnt_1 * BEISHU) / 1320;  // 公式不变
        end
        // 若new_data无效，保持上一次计算结果（相比原组合逻辑的"else rpm=0"更合理）
    end
end
  
    // 波特率生成器
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            clk_div <= 0;
            bit_tick <= 0;
        end
        else begin
            if(clk_div >= DIVISOR-1) begin
                clk_div <= 0;
                bit_tick <= 1;
            end
            else begin
                clk_div <= clk_div + 1;
                bit_tick <= 0;
            end
        end
    end
    
    // UART发送状态机 - 每1秒发送一次数据
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state <= IDLE;
            tx_busy <= 0;
            uart_tx <= 1;  // 空闲状态为高电平
            buffer_idx <= 0;
            cnt_clear <= 0;
        end
        else begin
            cnt_clear <= 0;  // 默认不清零计数
            
            case(state)
                IDLE: begin
                    if(new_data) begin  // 检测到新数据就绪
                        state <= LOAD;
                        tx_busy <= 1;
                        
                        // 分解RPM值为十进制数字并转换为ASCII
                        tx_buffer[0] <= (rpm / 10000) % 10 + 48;  // 万位
                        tx_buffer[1] <= (rpm / 1000) % 10 + 48;   // 千位
                        tx_buffer[2] <= (rpm / 100) % 10 + 48;    // 百位
                        tx_buffer[3] <= (rpm / 10) % 10 + 48;     // 十位
                        tx_buffer[4] <= rpm % 10 + 48;             // 个位
                        
                        // 添加单位和换行符
                        tx_buffer[5] <= " ";
                        tx_buffer[6] <= "r";
                        tx_buffer[7] <= "/";
                        tx_buffer[8] <= "m";
                        tx_buffer[9] <= "i";
                        tx_buffer[10] <= "n";
                        tx_buffer[11] <= 10;  // 换行符('\n')
                        
                        buffer_idx <= 0;
                    end
                end
                
                LOAD: begin
                    if(bit_tick) begin
                        tx_data <= tx_buffer[buffer_idx];
                        state <= START;
                        bit_cnt <= 0;
                    end
                end
                
                START: begin
                    if(bit_tick) begin
                        uart_tx <= 0;  // 起始位
                        state <= DATA;
                    end
                end
                
                DATA: begin
                    if(bit_tick) begin
                        uart_tx <= tx_data[bit_cnt];
                        bit_cnt <= bit_cnt + 1;
                        if(bit_cnt >= 7)
                            state <= STOP;
                    end
                end
                
                STOP: begin
                    if(bit_tick) begin
                        uart_tx <= 1;  // 停止位
                        buffer_idx <= buffer_idx + 1;
                        if(buffer_idx >= 11) begin  // 所有字符发送完毕
                            state <= CLEAR;
                        end
                        else
                            state <= LOAD;
                    end
                end
                
                CLEAR: begin
                    cnt_clear <= 1'b1;  // 发送完成，清零计数器
                    state <= IDLE;
                    tx_busy <= 0;
                end
                
                default: state <= IDLE;
            endcase
        end
    end
    
endmodule

module encoder_speed_system (
    input               clk,            // 系统时钟
    input               rst_n,          // 复位信号，低电平有效
    input               encoder_a,      // 编码器A相
    input               encoder_b,      // 编码器B相
    output              uart_tx,         // 串口发送端
	  output		    		     pin1,
	  output		    		     pin2,
	  output				       pwm
);

    // 内部信号声明
    wire signed [15:0]  pulse_count;    // 编码器脉冲计数
    wire                direction;      // 旋转方向指示
    wire [15:0]         rpm;
    wire signed [15:0]  feedback_rpm;
    wire signed [15:0]  pid_output;
    wire [15:0]         speeddata;
    wire                direction_reg;//后半部分的方向
    wire                data_ready;
    wire                cnt_clear;

    // 实例化编码器计数模块
    encoder_counter encoder_counter_inst (
        .clk            (clk),
        .rst_n          (rst_n),
        .encoder_a      (encoder_a),
        .encoder_b      (encoder_b),
        .cnt_clear      (cnt_clear),
        .count          (pulse_count),
      .data_ready     (data_ready),
        .direction      (direction)
    );
    
    // 实例化速度计算与UART发送模块
    speed_calc_uart speed_calc_uart_inst (
        .clk            (clk),
        .rst_n          (rst_n),
        .pulse_cnt      (pulse_count),
        .rpm            (rpm),
 .data_ready     (data_ready),
 .cnt_clear      (cnt_clear),
        .uart_tx        (uart_tx)
    );

    rpm_converter rpm_converter_inst(
    .clk                  (clk)  ,
    .rst_n                (rst_n),
    .rpm                  (rpm),               // 无符号转速（仅表示大小）
    .motor_dir            (direction),                // 电机方向（1=正转，0=反转）
    .feedback_rpm         (feedback_rpm)// 有符号反馈转速（含方向）
    );
/*
parameter Kp_Q8        = 77,    // 0.3 = 77/256（减小比例增益）
          Ki_Q8        = 26,    // 0.1 = 26/256（减小积分增益）
          Kd_Q8        = 5,     // 0.02 = 5/256（减小微分增益）
          INTEGRAL_MIN = -400,  // 收紧积分限幅
          INTEGRAL_MAX = 400;
*/   
    motor_pid_controller #(
    .WIDTH        (16),            // 数据位宽（与输入输出匹配）
    .Kp_Q8        (358),           // 比例系数（Q8格式，1.4=358/256）
    .Ki_Q8        (8),            // 积分系数（0.03=8/256）
    .Kd_Q8        (8),            // 微分系数（0.03=8/256）
    .DEADBAND     (8),             // 死区范围（±4rpm，误差在此范围内不调节）
    .OUTPUT_MIN   (-100),          // 输出下限（如-100对应最大反转速度）
    .OUTPUT_MAX   (100),           // 输出上限（如100对应最大正转速度）
    .INTEGRAL_MIN (-800),          // 积分项下限（防止积分饱和）
    .INTEGRAL_MAX (800),           // 积分项上限
    .DT_Q8        (13)             // 采样时间（Q8格式，根据实际时钟配置）
    ) u_motor_pid (
    .clk           (clk),      // 输入系统时钟
    .reset         (rst_n),      // 输入系统复位
    .feedback_rpm  (feedback_rpm), // 输入反馈转速（带符号）
    .control_output(pid_output)    // 输出PID控制量（带符号）
    );

    motor_speed_controller motor_speed_controller_inst(
    .clk                (clk) ,        // 系统时钟
    .reset_n            (rst_n),    // 复位信号，低电平有效
    .Speed_in           (pid_output),   // 输入速度值，可正可负（32位）
    .SpeedParam         (speeddata), // 输出PWM参数
    .direction_reg      (direction_reg) // 电机方向寄存器
    );

     motor_control motor_control_inst(
    .clock                (clk),          // 系统时钟
    .reset_n              (rst_n),        // 复位信号，低电平有效
    .en                   (1'b1) ,             // 使能信号
    .direction            (direction_reg),      // 方向输入（1:向前，0:向后）
    .speeddata            (speeddata) ,      // 速度参数（决定PWM占空比）
    .DC_Motor_IN1         (pin1),   // 电机控制信号1
    .DC_Motor_IN2         (pin2),   // 电机控制信号2
    .PWM                  (pwm)    // PWM输出信号
    );

endmodule

module motor_pid_controller #(
    parameter WIDTH        = 16,
    parameter Kp_Q8        = 154,    // 比例系数（Q8格式：0.6=154/256）
    parameter Ki_Q8        = 51,     // 积分系数（0.2=51/256）
    parameter Kd_Q8        = 64,     // 微分系数（0.25=64/256）
    parameter DEADBAND     = 4,      // 死区范围（±4rpm）
    parameter OUTPUT_MIN   = -100,   // 输出下限
    parameter OUTPUT_MAX   = 100,    // 输出上限
    parameter INTEGRAL_MIN = -800,   // 积分限幅下限
    parameter INTEGRAL_MAX = 800,    // 积分限幅上限
    // 采样时间DT=5ms，转换为Q8格式（DT_Q8 = DT(秒) * 256）
    parameter DT_Q8        = 1,      // 5ms≈0.005秒，0.005*256≈1.28→取1（Q8格式）
    // 1/DT的Q8格式：INV_DT_Q8 = 256 / DT_Q8（因DT=DT_Q8/256，故1/DT=256/DT_Q8）
    parameter INV_DT_Q8    = 256     // 256/1=256（对应DT=5ms的倒数）
)(
    input  wire                    clk,         // 系统时钟（50MHz）
    input  wire                    reset,       // 低电平复位（0=复位）
    input  wire signed [WIDTH-1:0] feedback_rpm,// 反馈转速（带符号）
    output reg  signed [WIDTH-1:0] control_output// 控制输出（带符号）
);

// --------------------------
// 1. 生成5ms周期的采样脉冲（核心修改）
// --------------------------
reg [17:0] sample_cnt;  // 50MHz时钟下，5ms需要计数：5ms/20ns=250000次
reg        sample_pulse; // 5ms周期的单周期脉冲（每5ms触发一次PID计算）

always @(posedge clk or negedge reset) begin
    if (!reset) begin
        sample_cnt <= 18'd0;
        sample_pulse <= 1'b0;
    end else begin
        if (sample_cnt >= 18'd249_999) begin  // 计数到250000-1时溢出
            sample_cnt <= 18'd0;
            sample_pulse <= 1'b1;  // 产生5ms触发脉冲
        end else begin
            sample_cnt <= sample_cnt + 1'b1;
            sample_pulse <= 1'b0;
        end
    end
end

// --------------------------
// 2. 内部信号定义
// --------------------------
reg signed [WIDTH-1:0] error, prev_error;  // 误差=目标-反馈；上一周期误差
reg signed [WIDTH-1:0] integral;           // 积分项
reg signed [WIDTH-1:0] derivative;         // 微分项
reg signed [2*WIDTH-1:0] p_term, i_term, d_term;  // 中间项（防溢出）
reg signed [WIDTH-1:0] target_rpm;         // 目标转速（可外部输入，此处固定）

// --------------------------
// 3. 误差计算（基于5ms采样脉冲触发）
// --------------------------
/*always @(posedge clk or negedge reset) begin
    if (!reset) begin
        error <= 16'sd0;
        prev_error <= 16'sd0;
        target_rpm <= 16'sd200;  // 固定目标转速（可改为外部输入）
    end else if (sample_pulse) begin  // 每5ms更新一次误差
        prev_error <= error;  // 保存上一周期误差（用于微分计算）
        error <= target_rpm - feedback_rpm;  // 当前误差=目标-反馈
    end
end
*/
always @(posedge clk or negedge reset) begin
    if (!reset) begin
        error <= 16'sd0;
        prev_error <= 16'sd0;
        target_rpm <= 16'sd0;  // 固定目标转速（可改为外部输入）
    end else if (sample_pulse) begin  // 每5ms更新一次误差
        prev_error <= error;  // 保存上一周期误差（用于微分计算）
        error <= target_rpm - feedback_rpm;  // 当前误差=目标-反馈
        target_rpm <= 16'sd160;  // 固定目标转速（可改为外部输入）
    end
end
// --------------------------
// 4. PID控制逻辑（基于5ms采样脉冲触发）
// --------------------------
always @(posedge clk or negedge reset) begin
    if (!reset) begin
        integral <= 16'sd0;
        derivative <= 16'sd0;
        p_term <= 32'sd0;
        i_term <= 32'sd0;
        d_term <= 32'sd0;
        control_output <= 16'sd0;
    end else if (sample_pulse) begin  // 每5ms计算一次PID输出
        // 死区判断：误差在±DEADBAND内时不动作
        if (error > DEADBAND || error < -DEADBAND) begin
            // 比例项：Kp × error（Q8×Q8→右移8位恢复Q8）
            p_term <= (Kp_Q8 * error) >>> 8;

            // 积分项：Ki × 积分累积 × 采样时间DT（Q8格式）
            integral <= integral + error;  // 累积误差
            // 积分限幅（防止饱和）
            if (integral > INTEGRAL_MAX) integral <= INTEGRAL_MAX;
            if (integral < INTEGRAL_MIN) integral <= INTEGRAL_MIN;
            // Ki×integral×DT：Q8×Q16×Q8 → 右移16位恢复Q16
            i_term <= (Ki_Q8 * integral * DT_Q8) >>> (8 + 8);

            // 微分项：Kd × (误差变化率) → 变化率=Δerror/DT
            // Δerror=error-prev_error，乘以INV_DT_Q8（1/DT的Q8格式）→ 右移8位
            derivative <= ((error - prev_error) * INV_DT_Q8) >>> 8;
            d_term <= (Kd_Q8 * derivative) >>> 8;  // Kd×derivative（Q8×Q8→右移8位）

            // 总输出（P+I+D），限幅到输出范围
            control_output <= p_term + i_term + d_term;
            if (control_output > OUTPUT_MAX) begin
                control_output <= OUTPUT_MAX;
            end else if (control_output < OUTPUT_MIN) begin
                control_output <= OUTPUT_MIN;
            end
        end else begin
            // 死区内：积分缓慢衰减，输出0
            integral <= integral >>> 1;  // 积分除以2（缓慢归零）
            control_output <= 16'sd0;
        end
    end
end

endmodule


module motor_speed_controller (
    input               clk,        // 系统时钟
    input               reset_n,    // 复位信号，低电平有效
    input     signed   [15:0]  Speed_in,   // 输入速度值，可正可负（32位）
    output reg  [15:0]  SpeedParam, // 输出PWM参数
    output reg          direction_reg // 电机方向寄存器
);

// 本地参数定义
localparam CYCLE_WIDTH_MINI = 16'd50;   // 最小PWM周期参数
localparam CYCLE_WIDTH_MAX  = 16'd2500; // 最大PWM周期参数

// 速度控制逻辑
always @(posedge clk or negedge reset_n) begin
    if (~reset_n) begin
        SpeedParam <= 16'd0;
        direction_reg <= 1'b0;
    end else begin
        // 计算PWM参数
        if (Speed_in > 16'd0) begin
            SpeedParam <= CYCLE_WIDTH_MINI + (Speed_in * (CYCLE_WIDTH_MAX - CYCLE_WIDTH_MINI) / 32'd100);
        end else if (Speed_in < 16'd0) begin
            SpeedParam <= CYCLE_WIDTH_MINI + ((-Speed_in) * (CYCLE_WIDTH_MAX - CYCLE_WIDTH_MINI) / 32'd100);
        end else begin
            SpeedParam <= 16'd0;
        end

        // 设置电机的转向
        // 当Speed_in为负数时（最高位为1），direction_reg为0；否则为1
        direction_reg <= Speed_in[15] ? 1'b0 : 1'b1;
    end
end

endmodule

module rpm_converter (
    input clk,
    input rst_n,
    input [15:0] rpm,               // 无符号转速（仅表示大小）
    input motor_dir,                // 电机方向（1=正转，0=反转）
    output reg signed [15:0] feedback_rpm  // 有符号反馈转速（含方向）
);

// 转换逻辑：结合方向信号生成有符号feedback_rpm
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        feedback_rpm <= 16'sd0;  // 复位时输出0
    end else begin
        if (motor_dir == 1'b1) begin  // 正转：反馈为正数
            feedback_rpm <= $signed(rpm);  // 无符号转有符号（正数）
        end else begin  // 反转：反馈为负数（补码表示）
            feedback_rpm <= -$signed(rpm); // 无符号转有符号后取反（负数）
        end
    end
end

endmodule
 
module motor_control (
    input           clock,          // 系统时钟
    input           reset_n,        // 复位信号，低电平有效
    input           en,             // 使能信号
    input           direction,      // 方向输入（1:向前，0:向后）
    input   [15:0]  speeddata,      // 速度参数（决定PWM占空比）
    output          DC_Motor_IN1,   // 电机控制信号1
    output          DC_Motor_IN2,   // 电机控制信号2
    output   reg    PWM             // PWM输出信号
);

// 内部寄存器定义32
reg             motor_movement;         // 电机运动，1为开始、0为停止
reg             motor_direction;        // 电机转向，1为向前、0为向后
reg             motor_fast_decay;       // 电机减速，1为快制动、0为慢制动
reg             PWM_OUT;                // PWM输出寄存器
reg     [15:0]  total_dur;              // 总持续时间
reg     [15:0]  high_dur;               // 高位时间（决定占空比）
reg     [15:0]  tick;                   // 计数器
reg             DC_Motor_IN1;           // 电机控制信号1寄存器
reg             DC_Motor_IN2;           // 电机控制信号2寄存器

// 时序逻辑：初始化和参数更新
always @(posedge clock or negedge reset_n) begin
    if (~reset_n) begin
        // PWM参数复位
        high_dur <= 0;
        total_dur <= 0;
        
        // 电机状态复位
        motor_movement <= 1'b0;
        motor_direction <= 1'b1;
        motor_fast_decay <= 1'b1;
    end else if (en) begin
        // 使能时更新参数
        total_dur <= 16'd2500;          // 固定总周期
        high_dur  <= speeddata;         // 从输入获取高电平时间
        // 组合控制信号（快制动、方向、使能）
        {motor_fast_decay, motor_direction, motor_movement} <= {1'b1, direction, 1'b1};
    end
end

// 组合逻辑：方向和制动控制
always @(*) begin
    if (motor_fast_decay) begin  // 快制动模式
        if (motor_movement) begin  // 电机运行中
            if (motor_direction) begin  // 向前
                {DC_Motor_IN2, DC_Motor_IN1, PWM} <= {1'b0, 1'b1, PWM_OUT};//改过
            end else begin  // 向后
                {DC_Motor_IN2, DC_Motor_IN1, PWM} <= {1'b1, 1'b0, PWM_OUT};//改过
            end
        end else begin  // 电机停止（快制动）
            {DC_Motor_IN2, DC_Motor_IN1, PWM} <= {1'b1, 1'b1, 1'b0};  // 短路制动
        end
    end else begin  // 慢制动模式（补充缺失的慢制动逻辑）
        if (motor_movement) begin  // 电机运行中
            if (motor_direction) begin  // 向前
                {DC_Motor_IN2, DC_Motor_IN1, PWM} <= {1'b0, 1'b1, PWM_OUT};
            end else begin  // 向后
                {DC_Motor_IN2, DC_Motor_IN1, PWM} <= {1'b1, 1'b0, PWM_OUT};
            end
        end else begin  // 电机停止（慢制动）
            {DC_Motor_IN2, DC_Motor_IN1, PWM} <= {1'b0, 1'b0, 1'b0};  // 断开制动
        end
    end
end

// PWM计数器逻辑
always @(posedge clock or negedge reset_n) begin
    if (~reset_n) begin
        tick <= 1;  // 计数器初始化为1
    end else if (tick >= total_dur) begin
        tick <= 1;  // 达到周期终点，重置计数器
    end else begin
        tick <= tick + 1;  // 计数器递增
    end
end

// PWM输出生成
always @(posedge clock) begin
    PWM_OUT <= (tick <= high_dur) ? 1'b1 : 1'b0;  // 占空比控制
end

endmodule
