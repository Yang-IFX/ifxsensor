/**
* \copyright
* MIT License
*
* Copyright (c) 2021 Infineon Technologies AG
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
* SOFTWARE.
*
* \endcopyright
*
* \author Infineon Technologies AG
*
* \file DeviceControl.h
*
* \brief   \copybrief gr_devicecontrol
*
* For details refer to \ref gr_devicecontrol
*
*
* @{
*/


#ifndef IFX_RADAR_DEVICE_CONTROL_H
#define IFX_RADAR_DEVICE_CONTROL_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#include <stdbool.h>

#include "ifxBase/Forward.h"
#include "ifxBase/Error.h"

#include "ifxRadar/DeviceConfig.h"
#include "ifxRadar/Frame.h"

/*
==============================================================================
   2. DEFINITIONS
==============================================================================
*/

/*
==============================================================================
   3. TYPES
==============================================================================
*/

/**
 * @brief A handle for an instance of DeviceControl module, see DeviceControl.h.
 */
typedef struct ifx_Device_Controller_s* ifx_Device_Handle_t;

enum ifx_Device_Type
{
    IFX_DEVICE_ANY = 0,
    IFX_DEVICE_BGT60TR13 = 1,
    IFX_DEVICE_BGT60ATR24 = 2,
    IFX_DEVICE_BGT60TR11 = 3
};

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Device_Access
  * @{
  */

/** @defgroup gr_devicecontrol Device Control
  * @brief API for Radar device control operations
  * @{
  */

/**
 * \brief This function returns a list of available COM ports.
 *
 * The port list is compiled into a zero terminated string, where all
 * available ports are separated by a semicolon (';'). The string buffer to
 * hold the port list must be provided by the caller. If the buffer is not
 * sufficient to hold the names of all available ports, it is filled with as
 * much complete port names as possible. Port names will not be truncated.
 *
 * The function will return the number of available COM ports regardless of
 * the number of port names that have been written to the buffer. The caller
 * can count the number of names in the string buffer and compare with the
 * returned number to check if it is sensible to call this function again with
 * a bigger string buffer.
 *
 * \param[out] port_list    A pointer to the string buffer where the port
 *                          names will be written to.
 * \param[in]  buffer_size  The size of the string buffer in bytes.
 *
 * \return The function returns the number of available COM ports.
 */
IFX_DLL_PUBLIC
uint32_t ifx_device_get_list(char* port_list, size_t buffer_size);

/**
 * @brief Creates a device handle.
 *
 * This function searches for a BGT60TRxx sensor device connected to the host machine and
 * connects to the first found sensor device.
 *
 * Please note that this function is not thread-safe and must not accessed
 * at the same time by multiple threads.
 *
 * @return Handle to the newly created instance or NULL in case of failure.
 *
 */
IFX_DLL_PUBLIC
ifx_Device_Handle_t ifx_device_create(void);

/**
 * @brief Creates a device handle.
 *
 * This function opens the sensor device connected to the port given by port.
 *
 * Please note that this function is not thread-safe and must not accessed
 * at the same time by multiple threads.
 *
 * @param [in]  port    name of port to open
 * @return Handle to the newly created instance or NULL in case of failure.
 *
 */
IFX_DLL_PUBLIC
ifx_Device_Handle_t ifx_device_create_by_port(const char* port);

/**
 * @brief Creates a device handle.
 **
 * This function searches for a BGT60TRxx sensor device connected to the host machine and
 * connects to the sensor device with a matching UUID.
 *
 * Please note that this function is not thread-safe and must not accessed
 * at the same time by multiple threads.
 *
 * @param [in]     uuid       16-byte UUID
 *
 * @return Handle to the newly created instance or NULL in case of failure.
 *
 */
IFX_DLL_PUBLIC
ifx_Device_Handle_t ifx_device_create_by_uuid(const uint8_t uuid[16]);

/**
 * @brief Configures radar sensor device and starts acquisition of time domain data.
 * The board is configured according to the parameters provided through *config*
 * and acquisition of time domain data is started.
 *
 * If the function fails ifx_error_get() function will return one of the following error codes:
 *         - \ref IFX_ERROR_NO_DEVICE
 *         - \ref IFX_ERROR_DEVICE_BUSY
 *         - \ref IFX_ERROR_COMMUNICATION_ERROR
 *         - \ref IFX_ERROR_NUM_SAMPLES_OUT_OF_RANGE
 *         - \ref IFX_ERROR_RX_ANTENNA_COMBINATION_NOT_ALLOWED
 *         - \ref IFX_ERROR_TX_ANTENNA_MODE_NOT_ALLOWED
 *         - \ref IFX_ERROR_IF_GAIN_OUT_OF_RANGE
 *         - \ref IFX_ERROR_SAMPLERATE_OUT_OF_RANGE
 *         - \ref IFX_ERROR_RF_OUT_OF_RANGE
 *         - \ref IFX_ERROR_TX_POWER_OUT_OF_RANGE
 *         - \ref IFX_ERROR_CHIRP_RATE_OUT_OF_RANGE
 *         - \ref IFX_ERROR_FRAME_RATE_OUT_OF_RANGE
 *         - \ref IFX_ERROR_NUM_CHIRPS_NOT_ALLOWED
 *         - \ref IFX_ERROR_FRAME_SIZE_NOT_SUPPORTED
 *
 * @param [in]     handle    A handle to the radar device object
 * @param [in]     config    This struct contains the parameters for data acquisition.
 *                           The device in configured according to these parameters.
 */
IFX_DLL_PUBLIC
void ifx_device_set_config(ifx_Device_Handle_t handle,
                           const ifx_Device_Config_t* config);

/**
* @brief Reads the current configurations of the radar sensor device.
* If this function succeeds and time domain data is available, IFX_OK is returned and the
* configuration is copied to *config*. If readout fails IFX_ERROR_COMMUNICATION_ERROR will be
* returned.
*
* @param [in]     handle     A handle to the radar device object.
* @param [out]    config     The current configuration of the device.
*
*/
IFX_DLL_PUBLIC
void ifx_device_get_config(ifx_Device_Handle_t handle,
                           ifx_Device_Config_t* config);

/**
* @brief Reads the temperature of the radar sensor device.
* On success, the current temperature of the radar sensor is written to the variable
* temperature_celsius. The temperature is in units of degrees Celsius.
* The 'temperature' field of the device handle is updated with the millidegree Celcius
* value.
* If an error occurs, temperature_celsius is not accessed and the error code can
* be retrieved using the function \ref ifx_error_get
*
* @param [in]     handle                  A handle to the radar device object.
* @param [out]    temperature_celcius     The current configuration of the device.
*
*/
IFX_DLL_PUBLIC
void ifx_device_get_temperature(ifx_Device_Handle_t handle, ifx_Float_t* temperature_celcius);

/**
* @brief Derives a device configuration from specified feature space metrics.
*
* This functions calculates FMCW frequency range, number of samples per chirp, number of chirps
* per frame and chirp-to-chirp time needed to achieve the specified feature space metrics. Number
* of samples per chirp and number of chirps per frame are rounded up to the next power of two,
* because this is a usual constraint for range and Doppler transform. The resulting maximum range
* and maximum speed may therefore be larger than specified.
*
* The calculated values are written to the members of the *device_config* struct, which must be
* provided by the caller.
*
* If handle is a valid pointer the delays for the connected radar chip are used. If handle is NULL,
* default values for the delays are used.
*
* @param [in]     handle     The device handle.
* @param [in]     metrics    The desired feature space metrics to be converted.
* @param [out]    config     The struct where the parameters calculated from the metrics are
*                            written to.
*/
IFX_DLL_PUBLIC
void ifx_device_translate_metrics_to_config(const ifx_Device_Handle_t handle,
                                            const ifx_Device_Metrics_t* metrics,
                                            ifx_Device_Config_t* config);

/**
 * @brief Configures radar sensor device ADC settings.
 *
 * @param [in]     handle    A handle to the radar device object.
 * @param [in]     config    This struct contains the ADC parameters to apply.
 *                           The ADC in configured according to these parameters.
 *
 */
IFX_DLL_PUBLIC
void ifx_device_configure_adc(ifx_Device_Handle_t handle,
                              const ifx_Device_ADC_Config_t* config);

/**
 * @brief Initializes structures used for frame creation without real Device i.e. without commlib commands.
 *
 * @param [in]     device_config       This struct contains the parameters for data acquisition.
 *                                     The device in configured according to these parameters.
 *
 * @return Handle to the newly created instance or NULL in case of failure.
 *
 */
IFX_DLL_PUBLIC
ifx_Device_Handle_t ifx_device_create_dummy(ifx_Device_Config_t* device_config);

/**
 * @brief Closes the connection to the radar sensor device.
 *
 * This function stops acquisition of time domain data, closes the connection to the device and
 * destroys the handle. The handle becomes invalid and must not be used any more after this
 * function was called.
 *
 * Please note that this function is not thread-safe and must not accessed
 * at the same time by multiple threads.
 *
 * @param [in]     handle    A handle to the radar device object.
 *
 */
IFX_DLL_PUBLIC
void ifx_device_destroy(ifx_Device_Handle_t handle);

/**
 * @brief Frees the memory occupied by device handle.
 *
 * @param [in]     handle    A handle to the radar device object.
 *
 */
IFX_DLL_PUBLIC
void ifx_device_destroy_dummy(ifx_Device_Handle_t handle);

/**
 * @brief Creates a frame structure for time domain data acquisition using information
 * from a \ref ifx_Device_Handle_t
 *
 * This function checks the current configuration of the specified sensor device and initializes a
 * data structure (of type \ref ifx_Frame_R_t) that can hold a time domain data frame according
 * acquired through that device.
 * The device must be initialized before calling this function.
 *
 * If the configuration of the device is changed (e.g. by calling \ref ifx_device_set_config), the
 * frame handle is no longer valid. In that case you have to destroy the old frame using
 * \ref ifx_frame_destroy_r and create a new one with this function.
 *
 * Use \ref ifx_frame_destroy_r to destroy the frame handle and free the allocated memory.
 *
 * It is important to note that samples per chirp and chirps per frame has an impact on range spectrum and range doppler map
 * module. So, if these parameters are changed in new frame then user has to destroy and create new handles for
 * range spectrum and range doppler map modules.
 *
 * @param [in]     handle    A handle to the radar device object.
 *
 * @return Pointer to allocated frame \ref ifx_Frame_R_t or NULL if allocation failed.
 *
 */
IFX_DLL_PUBLIC
ifx_Frame_R_t* ifx_device_create_frame_from_device_handle(ifx_Device_Handle_t handle);

/**
 * @brief Retrieves the next frame of time domain data from a radar device (blocking).
 *
 * This function retrieves the next complete frame of time domain data from the connected device.
 * The samples from all chirps and all enabled RX antennas will be copied to the provided data
 * structure *frame*. It is assumed that *frame* was allocated beforehand with
 * \ref ifx_frame_create_r or \ref ifx_device_create_frame_from_device_handle.
 *
 * The function blocks until a full frame has been received or an error occured. Possible errors
 * are \ref IFX_ERROR_TIMEOUT or \ref IFX_ERROR_FIFO_OVERFLOW. The error \ref IFX_ERROR_TIMEOUT occurs
 * if the device does not reply within a given time span. The error \ref IFX_ERROR_FIFO_OVERFLOW
 * occurs if the internal buffer of the radar device is too small to store all data. In that case
 * radar data is lost. To prevent an overflow error, the function \ref ifx_device_get_next_frame
 * should be called more frequently.
 *
 * This function fetches the radar data in slices which are typically smaller than a frame.
 * For this reason it makes sense to call this function more often than after a full frame
 * is completed. In particular, to prevent \ref IFX_ERROR_FIFO_OVERFLOW errors this function
 * should be called as frequently as possible.
 *
 * For high values of the frame repetition time (10 seconds or larger, corresponding to a
 * frame rate of 0.1Hz or lower) the function might return the error \ref IFX_ERROR_TIMEOUT.
 * In that case, please use \ref ifx_device_get_next_frame_nonblock.
 *
 * See also \ref ifx_device_get_next_frame_nonblock for a non-blocking version of this function.
 *
 * @param [in]     handle    A handle to the radar device object.
 * @param [out]    frame     The frame structure where the time domain data should be copied to.
 *
 * @return If this function succeeds and time domain data is available, IFX_OK is returned and the
 *         data is copied to *frame*. If no data is available the \ref IFX_ERROR_TIMEOUT is
 *         returned. If data frames are acquired faster than they are retrieved by this function
 *         an internal buffer overflow on the sensor device will occur. In that case
 *         \ref IFX_ERROR_FIFO_OVERFLOW is returned.
 */
IFX_DLL_PUBLIC
ifx_Error_t ifx_device_get_next_frame(ifx_Device_Handle_t handle,
                                      ifx_Frame_R_t* frame);

/**
 * @brief Retrieves the next frame of time domain data from a radar device (non-blocking).
 *
 * This function retrieves the next frame of time domain data from the connected device.
 * The samples from all chirps and all enabled RX antennas will be copied to the provided data
 * structure *frame*. It is assumed that *frame* was allocated beforehand with
 * \ref ifx_frame_create_r or \ref ifx_device_create_frame_from_device_handle.
 *
 * This function fetches the radar data in slices which are typically smaller than a frame.
 * For this reason it makes sense to call this function more often than after a full frame
 * is completed. In particular, to prevent \ref IFX_ERROR_FIFO_OVERFLOW errors this function
 * should be called as frequently as possible.
 *
 * Unlike \ref ifx_device_get_next_frame the function does not block. If no no more samples
 * are currently available, the function returns \ref IFX_OK. The caller can check if
 * the frame is complete by calling \ref ifx_frame_complete. If the frame is incomplete,
 * the frame contains both time domain data from the last and the current frame. The
 * frame structure must not be read or written as long \ref ifx_frame_complete returns false.
 *
 * Here is a typical usage of this function:
 * @code
 *      while(1)
 *      {
 *          ifx_Error_t ret = ifx_device_get_next_frame_nonblock(device_handle, frame);
 *          if(ret != IFX_OK)
 *              // error handling
 *
 *          bool is_complete = ifx_frame_complete(frame);
 *          if(is_complete)
 *              // process or copy data
 *
 *          // do something else
 *      }
 * @endcode
 *
 * See also \ref ifx_device_get_next_frame for a blocking version of this function.
 *
 * @param [in]      handle                          A handle to the radar device object.
 * @param [out]     frame                           The frame structure where the time domain data should be copied to.
 * @return          IFX_OK                          if successful.
 * @return          error code                      if an error occured.
 */
IFX_DLL_PUBLIC
ifx_Error_t ifx_device_get_next_frame_nonblock(ifx_Device_Handle_t handle,
                                               ifx_Frame_R_t* frame);

/**
 * @brief Retrieves the register values of the device.
 *
 * @param [in]      handle                          A handle to the radar device object.
 * @param [out]     registers                       Array to save register values. Each array element
 *                                                  contains the register number in the upper 8 bit and
 *                                                  the register value in the lower 24 bit.
 * @param [in/out]  num_register                    Number of registers.
 * @return          IFX_OK                          if successful.
 * @return          error code                      if an error occured.
 */
IFX_DLL_PUBLIC
ifx_Error_t ifx_device_get_dumped_registers(ifx_Device_Handle_t handle,
                                            uint32_t* registers,
                                            uint8_t* num_register);

/**
 * @brief Retrieves the number of RX antennas available on the connected radar device.
 *
 * This function returns the number of receiving antennas that are available on the
 * radar device specified by the given device handle.
 *
 * @param [in]     handle    A handle to the radar device object.
 *
 * @return Number of available receiving antennas. If the input parameter is null
 *         the returned value would be 0
 */
IFX_DLL_PUBLIC
uint8_t ifx_device_get_num_rx_antennas(ifx_Device_Handle_t handle);

/**
 * @brief Retrieves the number of TX antennas available on the connected radar device.
 *
 * This function returns the number of transmitting antennas that are available on the
 * radar device specified by the given device handle.
 *
 * @param [in]     handle    A handle to the radar device object.
 *
 * @return Number of available transmitting antennas. If the input parameter is null
 *         the returned value would be 0
 */
IFX_DLL_PUBLIC
uint8_t ifx_device_get_num_tx_antennas(ifx_Device_Handle_t handle);

/**
 * @brief Retrieves the current RF transmission power from the sensor device.
 *
 * The power is returned for the specified antenna in units of dBm.
 *
 * tx_antenna must be smaller than the number of total TX antennas, see also
 * \ref ifx_device_get_num_tx_antennas.
 *
 * On errors NAN is returned and the error code can be retrieved using \ref ifx_error_get.
 *
 * @param [in]     handle       A handle to the radar device object.
 * @param [in]     tx_antenna   TX antenna.
 * @return TX power for specified antenna in units of dBm.
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_device_get_tx_power(ifx_Device_Handle_t handle, uint8_t tx_antenna);

/**
 * @brief Retrieves the unique id of the radar device (RF shield).
 *
 * This function returns the unique id of the connected RF shield.
 *
 * @param [in]     handle    A handle to the radar device object.
 * @param [out]    uuid      Unique id of radar shield.
 *
 * @return True on success.
 * @return False otherwise.
 */
IFX_DLL_PUBLIC
bool ifx_device_get_shield_uuid(ifx_Device_Handle_t handle, uint8_t uuid[16]);

/**
 * @brief Get the limits for the Metrics parameters
 *
 * Retrieve the limits (lower and upper bounds) for the parameters in
 * ifx_Device_Metrics_t.
 *
 * The outputted limits depend on the input metrics. Different inputs for
 * metrics will lead to different bounds in limits.
 *
 * The limits depend on:
 *   - sample_rate_Hz depends on the connected the device device.
 *   - rx_mask depends on the connected radar device.
 *   - tx_mask depends on the connected radar device.
 *   - tx_power_level on the connected radar device.
 *   - if_gain_dB on the connected radar device.
 *   - range_resolution_m depends on the maximum bandwidth of the connected radar device.
 *   - max_range_m depends on range_resolution_m.
 *   - max_speed_m_s depends on max_range_m and range_resolution_m.
 *   - speed_resolution_m_s depends on max_speed_m_s (and implicitly on range_resolution_m and max_range_m).
 *   - frame_repetition_time_s depends on range_resolution_m, max_range_m, max_speed_m_s, speed_resolution_m_s.
 *
 * If all values in metrics are within the bounds computed in limits, the function returns true.
 * That means the function returns true if
 * \code
 *      limits->sample_rate_Hz.min <= metrics->sample_rate_Hz <= limits->sample_rate_Hz.max
 * \endcode
 * and similar for all other other struct members of metrics. Otherwise the function returns false.
 *
 * @param [in]     handle    A handle to the radar device object.
 * @param [in]     metrics   Pointer to metrics structure.
 * @param [out]    limits    Pointer to limits structure.
 * @retval         true      if all members of metrics are within the corresponding min and max values of limits
 * @retval         false     otherwise
 */
IFX_DLL_PUBLIC
bool ifx_device_metrics_get_limits(const ifx_Device_Handle_t handle, const ifx_Device_Metrics_t* metrics, ifx_Device_Metrics_Limits_t* limits);


/**
 * @brief Computes the chirp time.
 *
 * The chirp time is the total chirp time consisting of the sampling time and
 * the sampling delay.
 *
 * @param [in]  handle      A handle to the radar device object.
 * @param [in]  config      The current configuration of the device.
 * @return      chirp time
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_device_get_chirp_time(const ifx_Device_Handle_t handle, const ifx_Device_Config_t* config);

/**
 * @brief Get information about the connected device.
 *
 * @param [in]  handle      A handle to the radar device object.
 * @return      pointer to \ref ifx_Device_Info_t structure
 *
 */
IFX_DLL_PUBLIC
const ifx_Device_Info_t* ifx_device_get_device_information(const ifx_Device_Handle_t handle);

/**
 * @brief Get information about the firmware version.
 *
 * @param [in]  handle      A handle to the radar device object.
 * @return      pointer to \ref ifx_Firmware_Info_t structure
 *
 */
IFX_DLL_PUBLIC
const ifx_Firmware_Info_t* ifx_device_get_firmware_information(const ifx_Device_Handle_t handle);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_RADAR_DEVICE_CONTROL_H */

/**
* @}
*/