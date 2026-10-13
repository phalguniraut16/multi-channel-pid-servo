`timescale 1ns/1ps

module multi_channel_pid_servo #(
    parameter integer N_CHANNELS = 4,
    parameter integer CLK_HZ = 100_000_000,
    parameter integer PWM_HZ = 50,
    parameter integer PWM_MIN_US = 1000,
    parameter integer PWM_MAX_US = 2000,

    // Signed Q8.8 gains. Tune these for the actual plant and feedback units.
    parameter signed [15:0] KP = 16'sd256,
    parameter signed [15:0] KI = 16'sd8,
    parameter signed [15:0] KD = 16'sd0,

    parameter signed [31:0] OUT_MIN = -32'sd30000,
    parameter signed [31:0] OUT_MAX =  32'sd30000,
    parameter signed [31:0] I_MIN   = -32'sd20000,
    parameter signed [31:0] I_MAX   =  32'sd20000
) (
    input  wire clk,
    input  wire rst,

    // Simple configuration write port
    input  wire cfg_we,
    input  wire [$clog2(N_CHANNELS)-1:0] cfg_channel,
    input  wire [1:0] cfg_reg,          // 0=setpoint, 1=feedback, 2=enable
    input  wire [15:0] cfg_wdata,

    output reg  [N_CHANNELS-1:0] servo_pwm,
    output reg  [N_CHANNELS-1:0] channel_update,
    output wire busy,
    output reg  [$clog2(N_CHANNELS)-1:0] active_channel,
    output reg  saturation_event
);
    localparam integer PWM_PERIOD = (CLK_HZ / PWM_HZ);
    localparam integer PWM_MIN_TICKS = (CLK_HZ * PWM_MIN_US) / 1_000_000;
    localparam integer PWM_MAX_TICKS = (CLK_HZ * PWM_MAX_US) / 1_000_000;
    localparam integer PWM_COUNT_W = (PWM_PERIOD <= 2) ? 1 : $clog2(PWM_PERIOD);
    localparam integer CH_W = (N_CHANNELS <= 2) ? 1 : $clog2(N_CHANNELS);

    localparam [2:0] ST_IDLE  = 3'd0,
                     ST_ERROR = 3'd1,
                     ST_P     = 3'd2,
                     ST_I     = 3'd3,
                     ST_D     = 3'd4,
                     ST_COMMIT= 3'd5;

    reg [2:0] state;
    reg [CH_W-1:0] channel_ptr;

    reg signed [15:0] setpoint [0:N_CHANNELS-1];
    reg signed [15:0] feedback [0:N_CHANNELS-1];
    reg enabled [0:N_CHANNELS-1];

    reg signed [31:0] integral_state [0:N_CHANNELS-1];
    reg signed [15:0] previous_error [0:N_CHANNELS-1];
    reg signed [31:0] p_term_state [0:N_CHANNELS-1];
    reg signed [31:0] i_delta_state [0:N_CHANNELS-1];
    reg signed [31:0] d_term_state [0:N_CHANNELS-1];
    reg signed [15:0] command_state [0:N_CHANNELS-1];
    reg [PWM_COUNT_W-1:0] pulse_ticks [0:N_CHANNELS-1];

    reg [PWM_COUNT_W-1:0] pwm_counter;

    reg signed [15:0] error_q;
    reg signed [15:0] derivative_q;
    reg signed [31:0] candidate_integral_q;
    reg signed [31:0] product_q;

    // Exactly one multiplication operator in the RTL datapath. Its operands
    // are selected by state, so P, I and D share the same arithmetic operator.
    reg signed [15:0] mult_a;
    reg signed [15:0] mult_b;
    wire signed [31:0] shared_product = mult_a * mult_b;

    reg signed [31:0] raw_sum;
    reg signed [31:0] integral_for_sum;
    reg signed [31:0] clamped_sum;
    reg signed [31:0] error_wide;
    reg signed [31:0] derivative_wide;
    reg signed [31:0] pwm_numerator;
    reg [PWM_COUNT_W-1:0] mapped_ticks;

    integer idx;

    assign busy = (state != ST_IDLE);

    // Select multiplier operands combinationally. The result is captured in
    // the active FSM state on a clock edge.
    always @* begin
        mult_a = 16'sd0;
        mult_b = 16'sd0;
        case (state)
            ST_P: begin
                mult_a = error_q;
                mult_b = KP;
            end
            ST_I: begin
                mult_a = error_q;
                mult_b = KI;
            end
            ST_D: begin
                mult_a = derivative_q;
                mult_b = KD;
            end
            default: begin
                mult_a = 16'sd0;
                mult_b = 16'sd0;
            end
        endcase
    end

    // Convert a saturated signed command into a pulse duration. This division
    // is by a parameter-derived constant and occurs only at commit time.
    function [PWM_COUNT_W-1:0] command_to_ticks;
        input signed [31:0] command;
        reg signed [63:0] span;
        reg signed [63:0] scale;
        reg signed [63:0] mapped;
        begin
            span = OUT_MAX - OUT_MIN;
            // Use division by a constant scale rather than a second multiply
            // operator, keeping the PID P/I/D products on the shared multiplier.
            if ((span <= 0) || ((PWM_MAX_TICKS - PWM_MIN_TICKS) <= 0)) begin
                command_to_ticks = PWM_MIN_TICKS;
            end else begin
                scale = span / (PWM_MAX_TICKS - PWM_MIN_TICKS);
                if (scale <= 0)
                    scale = 1;
                mapped = (command - OUT_MIN) / scale;
                if (mapped < 0)
                    mapped = 0;
                if (mapped > (PWM_MAX_TICKS - PWM_MIN_TICKS))
                    mapped = PWM_MAX_TICKS - PWM_MIN_TICKS;
                command_to_ticks = PWM_MIN_TICKS + mapped;
            end
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            state <= ST_IDLE;
            channel_ptr <= 0;
            active_channel <= 0;
            pwm_counter <= 0;
            error_q <= 0;
            derivative_q <= 0;
            candidate_integral_q <= 0;
            product_q <= 0;
            raw_sum <= 0;
            integral_for_sum <= 0;
            clamped_sum <= 0;
            error_wide <= 0;
            derivative_wide <= 0;
            pwm_numerator <= 0;
            mapped_ticks <= PWM_MIN_TICKS;
            servo_pwm <= {N_CHANNELS{1'b0}};
            channel_update <= {N_CHANNELS{1'b0}};
            saturation_event <= 1'b0;
            for (idx = 0; idx < N_CHANNELS; idx = idx + 1) begin
                setpoint[idx] <= 0;
                feedback[idx] <= 0;
                enabled[idx] <= 1'b0;
                integral_state[idx] <= 0;
                previous_error[idx] <= 0;
                p_term_state[idx] <= 0;
                i_delta_state[idx] <= 0;
                d_term_state[idx] <= 0;
                command_state[idx] <= 0;
                pulse_ticks[idx] <= PWM_MIN_TICKS;
            end
        end else begin
            channel_update <= {N_CHANNELS{1'b0}};
            saturation_event <= 1'b0;

            if (cfg_we && (cfg_channel < N_CHANNELS)) begin
                case (cfg_reg)
                    2'd0: setpoint[cfg_channel] <= $signed(cfg_wdata);
                    2'd1: feedback[cfg_channel] <= $signed(cfg_wdata);
                    2'd2: enabled[cfg_channel] <= cfg_wdata[0];
                    default: ;
                endcase
            end

            if (pwm_counter >= PWM_PERIOD-1)
                pwm_counter <= 0;
            else
                pwm_counter <= pwm_counter + 1'b1;

            for (idx = 0; idx < N_CHANNELS; idx = idx + 1)
                servo_pwm[idx] <= enabled[idx] && (pwm_counter < pulse_ticks[idx]);

            case (state)
                ST_IDLE: begin
                    active_channel <= channel_ptr;
                    if (enabled[channel_ptr]) begin
                        state <= ST_ERROR;
                    end else begin
                        if (channel_ptr == N_CHANNELS-1)
                            channel_ptr <= 0;
                        else
                            channel_ptr <= channel_ptr + 1'b1;
                    end
                end

                ST_ERROR: begin
                    error_wide = $signed(setpoint[active_channel]) -
                                 $signed(feedback[active_channel]);
                    derivative_wide = error_wide - $signed(previous_error[active_channel]);

                    if (error_wide > 32767)
                        error_q <= 16'sd32767;
                    else if (error_wide < -32768)
                        error_q <= -16'sd32768;
                    else
                        error_q <= error_wide[15:0];

                    if (derivative_wide > 32767)
                        derivative_q <= 16'sd32767;
                    else if (derivative_wide < -32768)
                        derivative_q <= -16'sd32768;
                    else
                        derivative_q <= derivative_wide[15:0];

                    state <= ST_P;
                end

                ST_P: begin
                    product_q <= shared_product;
                    p_term_state[active_channel] <= $signed(shared_product) >>> 8;
                    state <= ST_I;
                end

                ST_I: begin
                    product_q <= shared_product;
                    i_delta_state[active_channel] <= $signed(shared_product) >>> 8;
                    candidate_integral_q <= integral_state[active_channel] +
                                            ($signed(shared_product) >>> 8);
                    state <= ST_D;
                end

                ST_D: begin
                    product_q <= shared_product;
                    d_term_state[active_channel] <= $signed(shared_product) >>> 8;
                    state <= ST_COMMIT;
                end

                ST_COMMIT: begin
                    // Default: the candidate integral participates in output.
                    integral_for_sum = candidate_integral_q;

                    // Conditional integration: reject an update that pushes
                    // the output farther beyond either saturation limit.
                    raw_sum = p_term_state[active_channel] +
                              candidate_integral_q +
                              d_term_state[active_channel];

                    if ((raw_sum > OUT_MAX && error_q > 0) ||
                        (raw_sum < OUT_MIN && error_q < 0)) begin
                        integral_for_sum = integral_state[active_channel];
                        saturation_event <= 1'b1;
                    end

                    if (integral_for_sum > I_MAX)
                        integral_for_sum = I_MAX;
                    else if (integral_for_sum < I_MIN)
                        integral_for_sum = I_MIN;

                    if (integral_for_sum != integral_state[active_channel])
                        integral_state[active_channel] <= integral_for_sum;

                    clamped_sum = p_term_state[active_channel] +
                                  integral_for_sum +
                                  d_term_state[active_channel];

                    if (clamped_sum > OUT_MAX) begin
                        clamped_sum = OUT_MAX;
                        saturation_event <= 1'b1;
                    end else if (clamped_sum < OUT_MIN) begin
                        clamped_sum = OUT_MIN;
                        saturation_event <= 1'b1;
                    end

                    if (clamped_sum > 32767)
                        command_state[active_channel] <= 16'sd32767;
                    else if (clamped_sum < -32768)
                        command_state[active_channel] <= -16'sd32768;
                    else
                        command_state[active_channel] <= clamped_sum[15:0];

                    pulse_ticks[active_channel] <= command_to_ticks(clamped_sum);
                    previous_error[active_channel] <= error_q;
                    channel_update[active_channel] <= 1'b1;

                    if (active_channel == N_CHANNELS-1)
                        channel_ptr <= 0;
                    else
                        channel_ptr <= active_channel + 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
