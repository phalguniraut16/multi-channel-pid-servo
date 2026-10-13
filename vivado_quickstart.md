# Vivado quick start

1. Create a new RTL project and select the actual Xilinx part or board.
2. Add `rtl/multi_channel_pid_servo.v` under Design Sources.
3. Add `tb/tb_multi_channel_pid_servo.v` under Simulation Sources.
4. Set `tb_multi_channel_pid_servo` as the simulation top.
5. Select **Run Simulation → Run Behavioral Simulation**.
6. Inspect `state`, `active_channel`, `channel_update`, `saturation_event`, `servo_pwm`, and the per-channel arrays.
7. For synthesis, set `multi_channel_pid_servo` as the synthesis top and run **Run Synthesis**.
8. Open **Report Utilization** and **Report Timing Summary**. Confirm DSP use and check the critical path.
9. Before implementation, add real pin constraints for your board and a clock constraint appropriate for the oscillator.

The testbench uses a reduced clock value so simulation completes quickly. Do not use its PWM timing parameters as a real-board configuration.
