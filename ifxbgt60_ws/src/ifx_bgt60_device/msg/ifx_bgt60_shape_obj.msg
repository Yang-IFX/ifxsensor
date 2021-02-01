# ===========================================================================
# Copyright (c) 2020, Infineon Technologies AG All rights reserved.
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
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ===========================================================================
#

# Type of shape. Valid values are "up", "up-down", "down-up", "down".
string      type

# Number of repetitions of the shape. This number must be a power of 2.
uint32_t    num_repetitions

# Power mode after end of the shape set. Valid values are "active",
# "idle", "deep sleep", and "deep sleep continue".
string      following_power_mode

# Delay after end of shape set in seconds.
float       end_delay_s

# Lower frequency of the FMCW chirp in Hz.
uint32_t    lower_frequency_Hz

# Upper frequency of the FMCW chirp in Hz.
uint32_t    upper_frequency_Hz

# The value determines the power of the emitted signal. Higher values correspond
# to higher power settings. The values do not directly correspond to a physical quantity.
# Allowed values are [0-31].
uint32_t    tx_power_level

# Gain of high pass filter in dB. Allowed values are [18, 30].
# The total gain is given as the sum of hp_gain_dB and vga_gain_dB.
uint32_t    hp_gain_level

# Gain of integrated gain adjustment circuitry in dB. Allowed values are [0,5,10,15,20,25,30].
# The total gain is given as the sum of hp_gain_dB and vga_gain_dB.
float       vga_gain_level

# Settings for up chirp. Setting is ignored if type is "down". In this case the setting can be omitted.
ifx_bgt60_chirp_obj     up_chirp

# Settings for down chirp. Setting is ignored if type is "up". In this case the setting can be omitted.
ifx_bgt60_chirp_obj     down_chirp

