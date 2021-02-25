# \copyright
# MIT License
#
# Copyright (c) 2021 Infineon Technologies AG
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# \endcopyright
#
# \author Infineon Technologies AG

# Type of shape. Valid values are "up", "up-down", "down-up", "down".
string      type

# Number of repetitions of the shape. This number must be a power of 2.
uint32      num_repetitions

# Power mode after end of the shape set. Valid values are "active",
# "idle", "deep sleep", and "deep sleep continue".
string      following_power_mode

# Delay after end of shape set in seconds.
float32     end_delay_s

# Lower frequency of the FMCW chirp in Hz.
uint32      lower_frequency_Hz

# Upper frequency of the FMCW chirp in Hz.
uint32      upper_frequency_Hz

# The value determines the power of the emitted signal. Higher values correspond
# to higher power settings. The values do not directly correspond to a physical quantity.
# Allowed values are [0-31].
uint32      tx_power_level

# Gain of high pass filter in dB. Allowed values are [18, 30].
# The total gain is given as the sum of hp_gain_dB and vga_gain_dB.
uint32      hp_gain_level

# Gain of integrated gain adjustment circuitry in dB. Allowed values are [0,5,10,15,20,25,30].
# The total gain is given as the sum of hp_gain_dB and vga_gain_dB.
float32     vga_gain_level

# Settings for up chirp. Setting is ignored if type is "down". In this case the setting can be omitted.
ifx_bgt60_chirp_obj     up_chirp

# Settings for down chirp. Setting is ignored if type is "up". In this case the setting can be omitted.
ifx_bgt60_chirp_obj     down_chirp

