#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
iverilog -g2012 -Wall -o /tmp/multi_channel_pid_servo_sim \
  rtl/multi_channel_pid_servo.v tb/tb_multi_channel_pid_servo.v
vvp /tmp/multi_channel_pid_servo_sim
