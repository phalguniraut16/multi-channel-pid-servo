# Multi-Channel PID Servo Controller

A resource-shared PID controller written in Verilog for Xilinx Vivado. The idea is to run several control channels without duplicating the full PID arithmetic for every channel.

The scheduler visits enabled channels in round-robin order. Each channel keeps its own setpoint, feedback, integral state, previous error and PWM command. P, I and D multiplication operations are scheduled through one shared RTL multiplier operator. The PWM outputs run independently of the PID scheduler, so updating one channel does not stop the other servo pulses.

## Design highlights

- Parameterized number of channels
- Round-robin, multi-cycle PID scheduler
- One shared signed multiplier operator for P, I and D terms
- Per-channel state/register bank
- Q8.8 gain convention
- Conditional-integration anti-windup
- Output saturation and bounded integral state
- Independent 50 Hz-style servo PWM outputs
- Configuration write port and status/debug outputs
- Self-checking Verilog testbench
- Python fixed-point reference helper
- Vivado setup notes and XDC template

## Repository layout

```text
rtl/
  multi_channel_pid_servo.v
tb/
  tb_multi_channel_pid_servo.v
scripts/
  fixed_point_reference.py
  run_iverilog.sh
docs/
  architecture.md
  register_map.md
  vivado_quickstart.md
constraints/
  example_pins.xdc
```

## Quick start

1. Download and extract the ZIP.
2. In Vivado, create an RTL project and select your FPGA part.
3. Add `rtl/multi_channel_pid_servo.v` as a design source.
4. Add `tb/tb_multi_channel_pid_servo.v` as a simulation source.
5. Set `tb_multi_channel_pid_servo` as simulation top and run behavioral simulation.
6. For synthesis, use `multi_channel_pid_servo` as the top module and inspect DSP, LUT, FF and timing reports.

If Icarus Verilog is installed, run:

```bash
bash scripts/run_iverilog.sh
```

## Interface summary

`cfg_we` writes one channel register selected by `cfg_channel` and `cfg_reg`.

| `cfg_reg` | Register | Data |
|---|---|---|
| `0` | Setpoint | signed 16-bit |
| `1` | Feedback | signed 16-bit |
| `2` | Enable | bit 0 |

`channel_update` pulses when a channel's new PID command has been committed. `active_channel` and `busy` are useful for simulation/debugging.

## Fixed-point conventions

- Setpoint, feedback and error: signed 16-bit integers in application-defined units
- Gains `KP`, `KI`, `KD`: signed Q8.8 values
- Product-to-term conversion: arithmetic right shift by 8
- Integral state and PID sum: signed 32-bit
- Final command: saturated to `OUT_MIN..OUT_MAX`
- PWM mapping: integer-scaled mapping from output limits to `PWM_MIN_US..PWM_MAX_US`

Gain values are deliberately parameters rather than hard-coded tuning. Start with conservative gains and tune for the actual motor/servo and feedback sensor.

## Engineering notes

This is a hardware design exercise, not a safety-certified motion controller. The RTL contains one explicit multiplication expression behind a state-controlled operand mux, which is the intended resource-sharing structure. Vivado can still transform the logic during synthesis; check the utilization report to confirm actual DSP inference on your selected part. No hardware measurements are claimed by this repository.

The included testbench checks scheduler activity, bounded PWM command updates and basic configuration behavior. It does not prove closed-loop stability. Test with the servo disconnected first, and verify PWM timing and signed scaling on the target board.

## License

MIT. See `LICENSE`.
