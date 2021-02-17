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

# Number of repetitions of the shape set.
uint32      num_repetitions

# Power mode after end of the shape set. Valid values are "active", "idle",
# "deep sleep", and "deep sleep continue"
string      following_power_mode

# Delay after end of shape set in seconds.
float32     end_delay_s

# Sample rate of the analog-to-digital converter (ADC) in Hz.
uint32      sample_rate_Hz

# Array of shapes. The array must have at least one element and must not
# have more than 4 elements.
ifx_bgt60_shape_obj[]   shape_objs

