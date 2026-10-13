# Example only. Replace these placeholders with the correct package pins for
# your exact FPGA board before running implementation.
#
# set_property PACKAGE_PIN <CLK_PIN> [get_ports clk]
# set_property IOSTANDARD LVCMOS33 [get_ports clk]
# create_clock -name sys_clk -period 10.000 [get_ports clk]
#
# Add one LOC and IOSTANDARD constraint per servo_pwm output, plus constraints
# for cfg_* inputs. Do not use this file unchanged on hardware.
