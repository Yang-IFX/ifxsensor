# /**
# * \copyright
# * MIT License
# *
# * Copyright (c) 2020 Infineon Technologies AG
# *
# * Permission is hereby granted, free of charge, to any person obtaining a copy
# * of this software and associated documentation files (the "Software"), to deal
# * in the Software without restriction, including without limitation the rights
# * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# * copies of the Software, and to permit persons to whom the Software is
# * furnished to do so, subject to the following conditions:
# *
# * The above copyright notice and this permission notice shall be included in all
# * copies or substantial portions of the Software.
# *
# * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# * SOFTWARE
# *
# * \endcopyright
# *
# * \author Infineon Technologies AG
# *
# *
# * @{
# */


# Bitmask where each bit represents one RX antenna of the radar device. If a bit
# is set the according RX antenna is enabled during the chirps and the signal
# received through that antenna is captured. The least significant bit
# corresponds to antenna 1.
uint32      rx_antennas_mask

# Bitmask where each bit represents one TX antenna. Defines like above rx_mask
uint32      tx_antennas_mask

# The value determines the power of the emitted signal. Higher values correspond
# to higher power settings. The values do not directly correspond to a physical
# quantity. Allowed values are [0-31].
uint8       tx_power_level

# Amplification factor in decibel that is applied to the IF signal coming from
# the RF mixer before it is fed into the analog-to-digital converter (ADC).
# Allowed values are [18-31].
uint8       if_gain_dB

# Lower frequency of the FMCW chirp in Hz.
uint64      lower_frequency_Hz

# Upper frequency of the FMCW chirp in Hz.
uint64      upper_frequency_Hz

# Number of chirps in a single frame.
uint32      num_chirps_per_frame

# Number of samples in a single chirp (data frame).
uint32      num_samples_per_chirp

# Time between the start of two consecutive chirps in a frame in seconds. Also
# known as chirp-to-chirp time or pulse repetition time.
float32     chirp_repetition_time_s

# Time between two consecutive frame in seconds and the inverse of the frame
# rate. Also known as frame period or frame time.
float32     frame_repetition_time_s

# Configuration of mimo mode. Possible values are:
#   1. "off": MIMO is disabled.
#   2. "tdm": Time-division multiplexing. This value is currently only allowed
#       with ATR24C with both transmitting antennas enabled.
string      mimo_mode

# Sample rate of the analog-to-digital converter (ADC) in Hz.
uint32      sample_rate_Hz

# /**
#
#
# * @}
#
# */