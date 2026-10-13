# Architecture and implementation notes

## Motivation

A fully parallel PID design can update every channel with a dedicated P/I/D datapath. That is straightforward but scales arithmetic resources with channel count. This design explores the other end of the trade-off: one scheduled multiplier and per-channel state.

## Block diagram

```text
         cfg write port
               |
               v
    +-----------------------+
    | Per-channel registers |
    | setpoint / feedback   |
    | integral / prev error |
    | command / pulse ticks |
    +-----------+-----------+
                |
                v
    +-----------------------+
    | Round-robin scheduler |
    | IDLE -> ERROR -> P    |
    | I -> D -> COMMIT      |
    +-----------+-----------+
                |
                v
    +-----------------------+
    | Operand mux           |
    | one shared multiplier |
    +-----------+-----------+
                |
                v
    +-----------------------+
    | Q8.8 scaling          |
    | anti-windup + clamp   |
    +-----------+-----------+
                |
                v
    +-----------------------+
    | Per-channel PWM state |
    | common frame counter  |
    +-----------------------+
```

## Scheduling

The FSM takes five active states per enabled channel: error/derivative calculation, P product, I product, D product, and commit. A channel's state is retained in its own register bank. `channel_update[ch]` pulses when the result is committed.

The current scheduler is round-robin among enabled channels, with one important behavior: a disabled channel is skipped while the FSM is idle. With `N` continuously enabled channels, expect approximately five processing clocks per channel plus selection overhead. This is a design estimate, not a timing measurement.

## Shared multiplier

There is one explicit signed multiplication expression:

```verilog
wire signed [31:0] shared_product = mult_a * mult_b;
```

The state-controlled operand mux selects `(error, KP)`, `(error, KI)`, or `(derivative, KD)`. The P/I/D products are captured on different cycles. This is the intended RTL resource-sharing pattern. Synthesis may still optimize the design or map arithmetic differently, so use Vivado's synthesized schematic and utilization report to confirm DSP inference.

## Fixed-point scaling

The gains use signed Q8.8. The 16x16 signed product is 32 bits; shifting right by eight returns the scaled PID term. The integral state and PID sum are 32-bit signed values. The final output is clamped to `OUT_MIN..OUT_MAX`, then mapped to the pulse range using an integer scale. This keeps the PID multiplier shared but can introduce a small quantization error.

## Anti-windup

The implementation uses conditional integration. If the candidate sum exceeds a saturation boundary and the current error would drive it farther out, the previous integral value is used for the current output. The integral value is also clamped to `I_MIN..I_MAX`. This is a basic anti-windup strategy; it is not equivalent to every industrial PID implementation.

## PWM generation

A common frame counter is compared against each channel's stored pulse duration. The pulse duration is updated only when that channel's PID calculation commits. This keeps the PWM waveform running while the shared arithmetic engine processes other channels.

## Trade-offs

| Design choice | Benefit | Cost |
|---|---|---|
| One shared multiplier | Lower intended multiplier count | Multi-cycle channel latency |
| Per-channel state | Independent control history | Registers scale with channel count |
| Round-robin scheduling | Predictable service order | Update rate falls as channels increase |
| Conditional integration | Simple anti-windup | Less flexible than back-calculation |
| Common PWM counter | Avoids one counter per channel | Shared frame timing |

## Known limitations / next steps

- The config interface is a simple write port, not AXI-Lite or APB.
- The design does not include a sensor interface or a physical servo driver.
- Closed-loop stability and actual servo behavior depend on plant dynamics and gain tuning.
- Confirm the target FPGA's DSP mapping, Fmax, resource use, and PWM timing in Vivado.
- Add a register readback port, configurable gains, timeout/fault handling, and assertions for production use.
- Run hardware tests with the servo disconnected first; do not connect a real actuator until pulse width and reset behavior are confirmed.
