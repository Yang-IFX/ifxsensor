# \copyright
# MIT License
#
# Copyright (c) 2020 Infineon Technologies AG
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
# SOFTWARE
#
# \endcopyright
#
# \author Infineon Technologies AG

# Message [Header] comes from std_msgs/Header.std_msg:
# http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
# Not used for the moment.
Header header


# Types for which the configuration is applicable.
# Valid entries are "BGT60TR13" and "BGT60ATR24"
string device_type

# Unique ids (uuids) of applicable radar sensors
string device_uuid

# Configuration with full control over the shape configuration.
ifx_bgt60_fmcw_single_shape single_shape
