#!/usr/bin/env python3
"""Tiny integer reference helper for Q8.8 PID term scaling.

This is for checking fixed-point conventions, not a bit-accurate RTL simulator.
"""
def q88_product(error: int, gain_q88: int) -> int:
    product = error * gain_q88
    # Python // rounds negative values down; RTL arithmetic shift is also
    # sign-extending, so this matches the intended >>> 8 behavior.
    return product >> 8

def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))

def pid_terms(error: int, previous_error: int, integral: int,
              kp_q88: int, ki_q88: int, kd_q88: int,
              out_min: int = -30000, out_max: int = 30000):
    derivative = error - previous_error
    p = q88_product(error, kp_q88)
    i_delta = q88_product(error, ki_q88)
    d = q88_product(derivative, kd_q88)
    candidate_i = integral + i_delta
    raw = p + candidate_i + d

    # Conditional integration anti-windup, matching the design intent.
    if (raw > out_max and error > 0) or (raw < out_min and error < 0):
        used_i = integral
    else:
        used_i = candidate_i
    output = clamp(p + used_i + d, out_min, out_max)
    return {"error": error, "derivative": derivative, "p": p,
            "i_delta": i_delta, "d": d, "integral_used": used_i,
            "output": output}

if __name__ == "__main__":
    result = pid_terms(error=1200, previous_error=800, integral=100,
                       kp_q88=256, ki_q88=8, kd_q88=16)
    for key, value in result.items():
        print(f"{key:>14}: {value}")
