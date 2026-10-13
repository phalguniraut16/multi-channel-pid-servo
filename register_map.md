# Configuration interface

This is a simple synchronous write interface. Assert `cfg_we` for one rising edge.

| `cfg_reg` | Field | `cfg_wdata` |
|---|---|---|
| `2'd0` | Setpoint | signed 16-bit value |
| `2'd1` | Feedback | signed 16-bit measured value |
| `2'd2` | Enable | bit 0; `1` enables the channel |

`cfg_channel` selects the channel. The top-level output `channel_update` is a one-cycle pulse per channel after its output command is committed. `busy` is high while the PID FSM is processing a channel. `active_channel` shows the channel currently being processed. `saturation_event` pulses when output saturation or anti-windup rejection is detected.

There is no readback path in this revision. A natural extension is an AXI-Lite slave with registers for setpoint, feedback, enable, gains, output command and status.
