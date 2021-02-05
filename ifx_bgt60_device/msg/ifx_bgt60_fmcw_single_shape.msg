# ===========================================================================
# Copyright (c) 2021, Infineon Technologies AG All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ===========================================================================
#


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
