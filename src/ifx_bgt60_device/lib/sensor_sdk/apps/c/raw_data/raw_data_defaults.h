/**
* \copyright
* MIT License
*
* Copyright (c) 2020 Infineon Technologies AG
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE
*
* \endcopyright
*
* \author Infineon Technologies AG
*
* \file raw_data_defaults.h
*
* \brief   This file defines the default configuration values for raw data app.
*
*
* @{
*/


#ifndef RAW_DATA_DEFAULTS_H
#define RAW_DATA_DEFAULTS_H

// Device configuration

#define IFX_FRAME_RATE_HZ               (1)         /**< This is the frame acquisition rate for raw data.
                                                         Valid range is [0.016 - 100]Hz.*/

#define IFX_TX_POWER_LEVEL              (31)        /**< TX Power level to be used on the radar transceiver.
                                                         Valid range is [1 - 31].*/

#define IFX_RX_MASK             (4)         /**< Receive antenna mask defines which antennas to activate. 
                                                         Multiple antennas can be activated by masking.*/
#define IFX_TX_MASK             (1)         /**< Transmitting antenna mask defines which antennas to activate. */


#define IFX_IF_GAIN_DB                  (33)        /**< This is the amplification factor that is
                                                         applied to the IF signal before sampling.
                                                         Valid range is [18 - 60]dB.*/

#define IFX_SAMPLE_RATE_HZ           (1000000U)  /**< Sample rate of the ADC.
                                                      Valid range is [100k - 2M]Hz.*/

#define IFX_NUM_SAMPLES_PER_CHIRP       (128)       /**< Number of samples per chirp.*/

#define IFX_NUM_CHIRPS_PER_FRAME        (32)        /**< Number of chirps in a frame.*/

#define IFX_LOWER_FREQUENCY_HZ         ((uint64_t)60500000000)  /**< Lower frequency of the bandwidth the chirps start at.*/

#define IFX_UPPER_FREQUENCY_HZ         ((uint64_t)61500000000)  /**< Higher frequency of the bandwidth the chirps end with.*/

#define IFX_CHIRP_REPETITION_TIME_S ((ifx_Float_t)5.014928e-4)   /**< The time between consecutive chirps in 100 picoseconds steps.*/

#define IFX_FRAME_REPETITION_TIME_S             ((ifx_Float_t)1 / IFX_FRAME_RATE_HZ)  /**< Frame-rate in Hz to period in microseconds calculation */

#endif /* RAW_DATA_DEFAULTS_H */

/**
* @}
*/