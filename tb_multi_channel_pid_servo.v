`timescale 1ns/1ps

module tb_multi_channel_pid_servo;
    localparam integer N = 4;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst = 1'b1;
    reg cfg_we = 1'b0;
    reg [$clog2(N)-1:0] cfg_channel = 0;
    reg [1:0] cfg_reg = 0;
    reg [15:0] cfg_wdata = 0;

    wire [N-1:0] servo_pwm;
    wire [N-1:0] channel_update;
    wire busy;
    wire [$clog2(N)-1:0] active_channel;
    wire saturation_event;

    integer update_count = 0;
    integer cycle_count = 0;
    integer ch;

    multi_channel_pid_servo #(
        .N_CHANNELS(N),
        .CLK_HZ(100_000),
        .PWM_HZ(50),
        .PWM_MIN_US(1000),
        .PWM_MAX_US(2000),
        .KP(16'sd128),
        .KI(16'sd4),
        .KD(16'sd0),
        .OUT_MIN(-32'sd30000),
        .OUT_MAX(32'sd30000),
        .I_MIN(-32'sd15000),
        .I_MAX(32'sd15000)
    ) dut (
        .clk(clk),
        .rst(rst),
        .cfg_we(cfg_we),
        .cfg_channel(cfg_channel),
        .cfg_reg(cfg_reg),
        .cfg_wdata(cfg_wdata),
        .servo_pwm(servo_pwm),
        .channel_update(channel_update),
        .busy(busy),
        .active_channel(active_channel),
        .saturation_event(saturation_event)
    );

    task write_reg;
        input [1:0] reg_id;
        input [$clog2(N)-1:0] channel;
        input [15:0] data;
        begin
            @(negedge clk);
            cfg_we = 1'b1;
            cfg_reg = reg_id;
            cfg_channel = channel;
            cfg_wdata = data;
            @(negedge clk);
            cfg_we = 1'b0;
        end
    endtask

    initial begin
        $dumpfile("multi_channel_pid_servo.vcd");
        $dumpvars(0, tb_multi_channel_pid_servo);

        repeat (5) @(posedge clk);
        rst = 1'b0;

        // Channel 0: positive error. Channel 1: negative error.
        write_reg(0, 0, 16'sd12000);
        write_reg(1, 0, 16'sd2000);
        write_reg(2, 0, 16'd1);

        write_reg(0, 1, -16'sd10000);
        write_reg(1, 1, 16'sd1000);
        write_reg(2, 1, 16'd1);

        // Wait long enough for the round-robin engine to revisit both channels.
        repeat (800) @(posedge clk);

        if (update_count < 4) begin
            $display("FAIL: expected repeated updates, observed %0d", update_count);
            $fatal(1);
        end

        if (servo_pwm === {N{1'bx}}) begin
            $display("FAIL: PWM bus is unknown");
            $fatal(1);
        end

        $display("PASS: observed %0d PID channel updates in %0d cycles",
                 update_count, cycle_count);
        $finish;
    end

    always @(posedge clk) begin
        if (!rst) begin
            cycle_count <= cycle_count + 1;
            for (ch = 0; ch < N; ch = ch + 1) begin
                if (channel_update[ch]) begin
                    update_count <= update_count + 1;
                    $display("update: channel=%0d active=%0d pwm=%b saturation=%b",
                             ch, active_channel, servo_pwm[ch], saturation_event);
                end
            end
        end
    end
endmodule
