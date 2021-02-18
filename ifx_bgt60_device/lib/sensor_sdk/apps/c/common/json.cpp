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
*
* @{
*/
#include <array>
#include <climits>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ifxRadar/SDK.h"
#include "ifxRadar/Util.h"


#include "json.hpp"
#include "json.h"

using std::string;
using std::vector;
using std::to_string;

// use ordered_json to preserve the insertion order.
using json = nlohmann::ordered_json;

#define IFX_DEVICE_DEFAULT_SAMPLE_RATE_HZ ((ifx_Float_t)1000000)



/**
 * @brief Convert antenna mask to a C++ vector of uint32
 *
 * @param [in] mask     antenna mask
 * @retval antenna vector
 */
static vector<uint32_t> mask_to_antenna_vector(uint32_t mask)
{
    vector<uint32_t> v;
    for (uint32_t i = 0; i < 32; i++)
    {
        if (mask & (1 << i))
            v.push_back(i+1);
    }
    return v;
}


/**
 * @brief Read sample rate
 *
 * If sample rate is not specified the function returns the default value
 * IFX_DEVICE_DEFAULT_SAMPLE_RATE_HZ.
 *
 * @retval sample_rate_Hz
 */
uint32_t ifxJsonConfiguration::get_sample_rate() const
{
    const uint32_t lower = IFX_DEVICE_SAMPLE_RATE_HZ_LOWER;
    const uint32_t upper = IFX_DEVICE_SAMPLE_RATE_HZ_UPPER;
    const double sample_rate = get_number("sample_rate_Hz", IFX_DEVICE_DEFAULT_SAMPLE_RATE_HZ);
    if (sample_rate < lower || sample_rate > upper)
        throw string("sample_rate_Hz must be between " + to_string(lower) + " and " + to_string(upper));

    return static_cast<uint32_t>(std::round(sample_rate));
}

/**
 * @brief Set active json object
 *
 * Set active json object to the path given by the vector path. This function
 * works similar to chdir for filesystems: All other get functions reference
 * relative from the active json object.
 *
 * The method throws an exception and sets m_error if the object does not
 * exist.
 *
 * @param [in] path     path to active json object
 */
void ifxJsonConfiguration::set_active_json_object(const vector<string>& path)
{
    const json* active = &m_json;

    if (!active)
        throw string("No json configuration loaded");

    string s_path = "";
    for (size_t i = 0; i < path.size() - 1; i++)
        s_path += path[i] + ".";
    s_path += path.back();

    if (path.size() == 0)
    {
        m_path = "";
        return;
    }

    for (const string& dir : path)
    {
        if (!active->contains(dir))
            throw string("Object " + s_path + " missing");
        active = &(active->at(dir));
    }

    m_path = s_path;
    m_active = active;
}

/**
 * @brief Get JSON object
 *
 * Relative to the active JSON object, get the object with the name given by
 * name. If the object does not exist an exception is thrown.
 *
 * @param [in] name     name of object
 * @retval json object
 */
const json& ifxJsonConfiguration::get_json_object(const string& name) const
{
    if (!m_active->contains(name))
        throw string("Property " + m_path + "." + name + " missing");

    return m_active->at(name);
}

/**
 * @brief Get number
 *
 * Get the number (of type ifx_Float_t) corresponding to name. If the object
 * does not exist or it is not a number, an exception is thrown.
 *
 * @param [in] name     name of property
 * @retval value
 */
ifx_Float_t ifxJsonConfiguration::get_number(const string& name) const
{
    auto property = get_json_object(name);
    if (!property.is_number())
        throw m_path + "." + name + " must be number";

    return property.get<ifx_Float_t>();
}

/**
 * @brief Get number or default value.
 *
 * Get the number (of type ifx_Float_t) corresponding to name. If the object
 * does not exist the value default_value is returned. If the value exists but
 * it is not a number, an exception is thrown.
 *
 * @param [in] name             name of property
 * @param [in] default_value    value returned if property is not present
 * @retval value
 */
ifx_Float_t ifxJsonConfiguration::get_number(const string& name, ifx_Float_t default_value) const
{
    if (!m_active->contains(name))
        return default_value;

    return get_number(name);
}

/**
 * @brief Get string
 *
 * Get the string corresponding to name. If the object does not exist or it is
 * not a string, an exception is thrown.
 *
 * @param [in] name     name of property
 * @retval value
 */
string ifxJsonConfiguration::get_string(const string& name) const
{
    auto property = get_json_object(name);
    if (!property.is_string())
        throw m_path + "." + name + " must be string";

    return property.get<string>();
}

/**
 * @brief Get string or default string
 *
 * Get the string corresponding to name. If the object does not exist the string
 * default_string is returned. If the object exists but it is not a string, an
 * exception is thrown.
 *
 * @param [in] name     name of property
 * @param [in] default_value    value returned if property is not present
 * @retval value
 */
string ifxJsonConfiguration::get_string(const string& name, const string& default_value) const
{
    if (!m_active->contains(name))
        return default_value;

    return get_string(name);
}

/**
 * @brief Get high-medium-low value
 *
 * Parse property with allowed values "high", "medium", "low" and return it
 * as enum class LowMedHigh. If the property is not present or it is not
 * an allowed value, an exception is thrown.
 *
 * @param [in]  name    name of property
 * @retval element of LowMedHigh class
 */
LowMedHigh ifxJsonConfiguration::get_low_med_high(const string& name) const
{
    string s = get_string(name);
    if (s == "low")
        return LowMedHigh::low;
    else if (s == "medium")
        return LowMedHigh::medium;
    else if (s == "high")
        return LowMedHigh::high;

    throw string(name + " must be either \"low\" or \"medium\" or \"high\"");
}

/**
 * @brief Get positive number or default value.
 *
 * Get the positive number (of type ifx_Float_t) corresponding to name. If the object
 * does not exist the value default_value is returned. If the value exists but it is
 * not a positive number, an exception is thrown.
 *
 * @param [in] name             name of property
 * @param [in] default_value    value returned if property is not present
 * @retval value
 */
ifx_Float_t ifxJsonConfiguration::get_positive_number(const string& name, ifx_Float_t default_value) const
{
    if (!m_active->contains(name))
        return default_value;

    return get_positive_number(name);
}

/**
 * @brief Get positive number
 *
 * Get the number (of type ifx_Float_t) corresponding to name. If the object
 * does not exist or it is not a positive number, an exception is thrown.
 *
 * @param [in] name     name of property
 * @retval value
 */
ifx_Float_t ifxJsonConfiguration::get_positive_number(const string& name) const
{
    const ifx_Float_t value = get_number(name);
    if (value <= 0)
        throw m_path + "." + name + " must be positive number";

    return value;
}

/**
 * @brief Get number and check bounds
 *
 * Get the number (ifx_Float_t) corresponding to name. If the object does not
 * exist or it is not a number, an exception is thrown. If the value is
 * outside [min, max] an exception is thrown as well.
 *
 * @param [in] name     name of property
 * @retval value
 */
ifx_Float_t ifxJsonConfiguration::get_number_bounds(const string& name, ifx_Float_t min, ifx_Float_t max) const
{
    const ifx_Float_t value = get_number(name);
    if (value < min || value > max)
        throw m_path + "." + name + " must be between " + to_string(min) + " and " + to_string(max);

    return value;
}

/**
 * @brief Get boolean
 *
 * Get the boolean corresponding to name. If the object does not exist or it is
 * not a boolean, an exception is thrown.
 *
 * @param [in] name     name of property
 * @retval value
 */
bool ifxJsonConfiguration::get_bool(const string& name) const
{
    auto property = get_json_object(name);
    if (!property.is_boolean())
        throw m_path + "." + name + " must be bool";

    return property.get<bool>();
}

/**
 * @brief Get boolean or default value.
 *
 * Get the boolean corresponding to name. If the object does not exist the
 * value default_value is returned. If the value exists but it is not a
 * boolean, an exception is thrown.
 *
 * @param [in] name             name of property
 * @param [in] default_value    value returned if property is not present
 * @retval value
 */
bool ifxJsonConfiguration::get_bool(const string& name, bool default_value) const
{
    if (!m_active->contains(name))
        return default_value;

    return get_bool(name);
}

/**
 * @brief Get uint32
 *
 * Get the uint32 corresponding to name. If the object does not exist or it is
 * not a uint32, an exception is thrown.
 *
 * @param [in] name     name of property
 * @retval value
 */
uint32_t ifxJsonConfiguration::get_uint32(const string& name) const
{
    const uint64_t value_u64 = get_uint64(name);
    if (value_u64 > UINT32_MAX)
        throw m_path + "." + name + " too big to store as 32bit integer";

    return static_cast<uint32_t>(value_u64);
}

/**
 * @brief Get uint32 or default value.
 *
 * Get the uint32 corresponding to name. If the object does not exist the
 * value default_value is returned. If the value exists but it is not a
 * uint32, an exception is thrown.
 *
 * @param [in] name             name of property
 * @param [in] default_value    value returned if property is not present
 * @retval value
 */
uint32_t ifxJsonConfiguration::get_uint32(const string& name, uint32_t default_value) const
{
    const uint64_t value_u64 = get_uint64(name, default_value);
    if (value_u64 > UINT32_MAX)
        throw m_path + "." + name + " too big to store as 32bit integer";

    return static_cast<uint32_t>(value_u64);
}

/**
 * @brief Get uint64
 *
 * Get the uint64 corresponding to name. If the object does not exist or it is
 * not a uint64, an exception is thrown.
 *
 * @param [in] name     name of property
 * @retval value
 */
uint64_t ifxJsonConfiguration::get_uint64(const string& name) const
{
    auto property = get_json_object(name);
    if (!property.is_number_unsigned())
        throw m_path + "." + name + " must be unsigned integer";

    return property.get<uint64_t>();
}

/**
 * @brief Get uint64 or default value.
 *
 * Get the uint64 corresponding to name. If the object does not exist the
 * value default_value is returned. If the value exists but it is not a
 * uint64, an exception is thrown.
 *
 * @param [in] name             name of property
 * @param [in] default_value    value returned if property is not present
 * @retval value
 */
uint64_t ifxJsonConfiguration::get_uint64(const string& name, uint64_t default_value) const
{
    if (!m_active->contains(name))
        return default_value;

    return get_uint64(name);
}

/**
 * @brief Get uint64 and check bounds
 *
 * Get the uint64 corresponding to name. If the object does not exist or it is
 * not a uint64, an exception is thrown. If the value is outside [min, max] an
 * exception is thrown as well.
 *
 * @param [in] name     name of property
 * @param [in] min      minimum allowed value
 * @param [in] max      maximum allowed value
 * @retval value
 */
uint64_t ifxJsonConfiguration::get_uint64_bounds(const string& name, uint64_t min, uint64_t max) const
{
    const uint64_t value = get_uint64(name);
    if (value < min || value > max)
        throw m_path + "." + name + " must be between " + to_string(min) + " and " + to_string(max);

    return value;
}

/**
 * @brief Get uint32 and check bounds
 *
 * Get the uint32 corresponding to name. If the object does not exist or it is
 * not a uint32, an exception is thrown. If the value is outside [min, max] an
 * exception is thrown as well.
 *
 * @param [in] name     name of property
 * @param [in] min      minimum allowed value
 * @param [in] max      maximum allowed value
 * @retval value
 */
uint32_t ifxJsonConfiguration::get_uint32_bounds(const string& name, uint32_t min, uint32_t max) const
{
    return static_cast<uint32_t>(get_uint64_bounds(name, min, max));
}

/**
 * @brief Get antenna mask
 *
 * Get the antenna mask corresponding to name. If the object does not exist or it is
 * not a valid antenna mask, an exception is thrown.
 *
 * @param [in] name     name of property
 * @retval value
 */
uint32_t ifxJsonConfiguration::get_antenna_mask(const string& name) const
{
    auto property = get_json_object(name);
    if (!property.is_array())
        throw m_path + "." + name + " must be array";

    uint32_t bitmask = 0;
    for (auto it : property)
    {
        if (!it.is_number_unsigned())
            throw m_path + "." + name + " must contain only positive integers";

        uint64_t value = it.get<uint64_t>();
        if (value < 1 || value > 32)
            throw m_path + "." + name + " must contain only integers in the range [1-32]";

        bitmask |= 1 << (value-1);
    }

    return bitmask;
}

/**
 * @brief Get float vector
 *
 * Get the float vector corresponding to name. If the object does not exist or
 * it is not a valid float vector, an exception is thrown.
 *
 * @param [in] name     name of property
 * @retval value
 */
ifx_Vector_R_t* ifxJsonConfiguration::get_vector_r(const string& name) const
{
    auto property = get_json_object(name);
    if (!property.is_array())
        throw m_path + "." + name + " must be array";

    vector<double> v;
    for (auto it : property)
    {
        double value;
        if (!it.is_number() || (value = it.get<double>()) <= 0)
            throw m_path + "." + name + " must contain only positive numbers";

        v.push_back(value);
    }

    if(v.size() < 1)
        throw m_path + "." + name + " must contain at least one positive number";
    if(v.size() > 32)
        throw m_path + "." + name + " has too many entries";

    ifx_Vector_R_t* vec = ifx_vec_create_r(static_cast<uint32_t>(v.size()));
    for (size_t i = 0; i < v.size(); i++)
        IFX_VEC_AT(vec, i) = static_cast<ifx_Float_t>(v[i]);

    return vec;
}

/**
 * @brief Get MIMO mode
 *
 * Get the MIMO mode. If the propery does not exist IFX_MIMO_OFF is
 * returned. If the property exists but is not an allowed value, an
 * exception is thrown.
 *
 * @retval mimo mode
 */
enum ifx_Device_MIMO_Mode ifxJsonConfiguration::get_mimo_mode() const
{
    const string name = "mimo_mode";
    if (!m_active->contains(name))
        return IFX_MIMO_OFF; // off

    const string value = get_string(name);
    if (value == "off")
        return IFX_MIMO_OFF;
    else if (value == "tdm")
        return IFX_MIMO_TDM;
    else
        throw string("mimo_mode must be \"off\" or \"tdm\"");
}

/**
 * @brief Constructor
 */
ifxJsonConfiguration::ifxJsonConfiguration()
{
}

/**
 * @brief Load configuration from file
 *
 * Load json configuration from file. If the json file cannot be opened or is
 * not a valid json file, an exception is thrown.
 *
 * @param [in]  filename    filename of configuration
 * @param [in]  type        type of radar sensor (currently ignored)
 * @param [in]  uuid        uuid of radar sensor (currently ignored)
 * @param true  if successful
 * @param false on errors
 */
void ifxJsonConfiguration::load_from_file(const string& filename, int type, const char* uuid)
{
    std::ifstream file;

    file.open(filename);
    if (!file.is_open())
        throw string("Cannot open file for reading");

    try
    {
        file >> m_json;
    }
    catch (...)
    {
        throw string("Error parsing JSON file");
    }
}

/**
 * @brief Save configuration to file
 *
 * Save configuration to JSON file. If the file cannot be written, an exception
 * is thrown.
 *
 * @param [in]  filename    filename of configuration
 * @param true  if successful
 * @param false on errors
 */
void ifxJsonConfiguration::save_to_file(const string& filename)
{
    std::ofstream file;

    file.open(filename);
    if (!file.is_open())
        throw string("Cannot open file for writing");

    try
    {
        file << std::setw(4) << m_json;
    }
    catch (...)
    {
        throw string("Error writing to file");
    }
}

bool ifxJsonConfiguration::has_device() const
{
    if (m_json.contains("device"))
        return true;
    else
        return false;
}

const vector<std::array<uint8_t, 16>> ifxJsonConfiguration::get_device_uuids() const
{
    vector<std::array<uint8_t, 16>> list;
    if (!has_device() || !m_json["device"].contains("uuids"))
        return list;

    const auto j = m_json["device"]["uuids"];
    if (!j.is_array())
        throw string("device.uuids must be a list");

    for (const string s : j)
    {
        std::array<uint8_t, 16> uuid;
        if (!ifx_util_string_to_uuid(s.c_str(), uuid.data()))
            throw string("device.uuids: " + s + " is not a valid uuid");
        list.push_back(uuid);
    }

    return list;
}

/**
 * @brief Return true if fmcw_scene configuration is present
 */
bool ifxJsonConfiguration::has_config_fmcw_scene() const
{
    if (m_json.contains("device_config") && m_json["device_config"].contains("fmcw_scene"))
        return true;
    else
        return false;
}

/**
 * @brief Set fmcw_scene configuration
 */
void ifxJsonConfiguration::set_config_fmcw_scene(const ifx_Device_Metrics_t* config)
{
    m_json["device_config"] = { {
        "fmcw_scene", {
            { "rx_antennas", mask_to_antenna_vector(config->rx_mask) },
            { "tx_antennas", mask_to_antenna_vector(config->rx_mask) },
            { "tx_power_level", config->tx_power_level },
            { "if_gain_dB", config->if_gain_dB },

            { "range_resolution_m", config->range_resolution_m },
            { "max_range_m", config->max_range_m },
            { "max_speed_m_s", config->max_speed_m_s },
            { "speed_resolution_m_s", config->speed_resolution_m_s },

            { "frame_repetition_time_s", config->frame_repetition_time_s }
        }
    } };

    /* write out the optional parameters only if they are set */
    if (config->sample_rate_Hz)
        m_json["device_config"]["fmcw_scene"]["sample_rate_Hz"] = config->sample_rate_Hz;
    if(config->center_frequency_Hz)
        m_json["device_config"]["fmcw_scene"]["center_frequency_Hz"] = config->center_frequency_Hz;
}

/**
 * @brief Get fmcw_scene configuration
 *
 * Write the fmcw_scene configuration to config_scene. If the configuration is
 * incorrect, an exception is thrown.
 *
 * @param [out]     config_scene    scene configuration
 */
void ifxJsonConfiguration::get_config_fmcw_scene(ifx_Device_Metrics_t* config_scene)
{
    ifx_Device_Metrics_t config = {};

    set_active_json_object({ "device_config", "fmcw_scene" });

    // required parameters
    config.range_resolution_m = get_positive_number("range_resolution_m");
    config.max_range_m = get_positive_number("max_range_m");
    config.max_speed_m_s = get_positive_number("max_speed_m_s");
    config.speed_resolution_m_s = get_positive_number("speed_resolution_m_s");
    config.frame_repetition_time_s = get_positive_number("frame_repetition_time_s");
    config.rx_mask = get_antenna_mask("rx_antennas");
    config.tx_mask = get_antenna_mask("tx_antennas");
    config.tx_power_level = get_uint32_bounds("tx_power_level", TX_POWER_LEVEL_LOWER, TX_POWER_LEVEL_UPPER);
    config.if_gain_dB = get_uint32_bounds("if_gain_dB", IF_GAIN_DB_LOWER, IF_GAIN_DB_UPPER);

    // optional parameters
    config.sample_rate_Hz = get_sample_rate();
    config.center_frequency_Hz = get_positive_number("center_frequency_Hz", 0);

    *config_scene = config;
}

/**
 * @brief Return true if presence sensing configuration is present
 */
bool ifxJsonConfiguration::has_config_presence_sensing() const
{
    return m_json.contains("presence_sensing");
}

/**
 * @brief Get presence sensing configuration
 *
 * Write the presence sensing configuration to config_presence_sensing. If the
 * configuration is incorrect, an exception is thrown.
 *
 * @param [in]      device_config   device configuration
 * @param [out]     config_scene    scene configuration
 */
void ifxJsonConfiguration::get_config_presence_sensing(const ifx_Device_Config_t* device_config, ifx_PresenceSensing_Config_t* config_presence_sensing)
{
    ifx_PresenceSensing_Config_t config = {};
    ifx_presence_sensing_get_config_defaults(device_config, &config);

    set_active_json_object({ "presence_sensing" });

    // required parameters
    config.min_detection_range_m = get_positive_number("min_detection_range_m");
    config.max_detection_range_m = get_positive_number("max_detection_range_m");
    config.range_hysteresis_percentage = get_number_bounds("range_hysteresis_percentage", 0, 100);
    config.absence_confirm_count = get_uint32("absence_confirm_count");
    config.presence_confirm_count = get_uint32("presence_confirm_count");

    *config_presence_sensing = config;
}

void ifxJsonConfiguration::set_config_presence_sensing(const ifx_PresenceSensing_Config_t* config)
{
    m_json["presence_sensing"] = { {
            { "min_detection_range_m", config->min_detection_range_m },
            { "max_detection_range_m", config->max_detection_range_m },
            { "range_hysteresis_percentage", config->range_hysteresis_percentage },
            { "absence_confirm_count", config->absence_confirm_count },
            { "presence_confirm_count", config->presence_confirm_count }
    } };
}

/**
 * @brief Return true if segmentation configuration is present
 */
bool ifxJsonConfiguration::has_config_segmentation() const
{
    return m_json.contains("segmentation");
}

/**
 * @brief Set segmentation configuration
 */
void ifxJsonConfiguration::set_config_segmentation(const ifx_Segmentation_Config_t* config)
{
    /* copy segments_degrees in a C++ vector */
    const uint32_t num_segments = IFX_VEC_LEN(config->segments_degrees);
    vector<ifx_Float_t> segments_degrees;
    for (uint32_t i = 0; i < num_segments; i++)
        segments_degrees.push_back(IFX_VEC_AT(config->segments_degrees, i));

    string static_target_removal, detection_sensitivity;
    switch (config->static_target_removal)
    {
    case IFX_STATIC_TARGET_CANCELATION_LOW: static_target_removal = "low"; break;
    case IFX_STATIC_TARGET_CANCELATION_MED: static_target_removal = "medium"; break;
    case IFX_STATIC_TARGET_CANCELATION_HIGH: static_target_removal = "high"; break;
    }

    switch (config->detection_sensitivity)
    {
    case IFX_DETECTION_SENSITIVITY_LOW: detection_sensitivity = "low"; break;
    case IFX_DETECTION_SENSITIVITY_MED: detection_sensitivity = "medium"; break;
    case IFX_DETECTION_SENSITIVITY_HIGH: detection_sensitivity = "high"; break;
    }

    m_json["segmentation"] = { {
            { "segments_degrees", segments_degrees },
            { "max_detection_range_m", config->max_detection_range_m },
            { "static_target_removal", config->static_target_removal },
            { "detection_sensitivity", detection_sensitivity },
            { "static_target_removal", static_target_removal },
            { "enable_hysteresis", config->enable_hysteresis },
            { "max_num_tracks", config->max_num_tracks }
    } };
}

/**
 * @brief Get segmentation configuration
 *
 * Write the segmentation configuration to config_segmentation. If the
 * configuration is incorrect, an exception is thrown.
 *
 * @param [in]      device_config   device configuration
 * @param [out]     config_scene    scene configuration
 */
void ifxJsonConfiguration::get_config_segmentation(const ifx_Device_Config_t* device_config, ifx_Segmentation_Config_t* config_segmentation)
{
    ifx_Segmentation_Config_t config = {};

    set_active_json_object({ "segmentation" });

    config.max_detection_range_m = get_positive_number("max_detection_range_m");

    switch (get_low_med_high("detection_sensitivity"))
    {
    case LowMedHigh::low:    config.detection_sensitivity = IFX_DETECTION_SENSITIVITY_LOW; break;
    case LowMedHigh::medium: config.detection_sensitivity = IFX_DETECTION_SENSITIVITY_MED; break;
    case LowMedHigh::high:   config.detection_sensitivity = IFX_DETECTION_SENSITIVITY_HIGH; break;
    }

    switch (get_low_med_high("static_target_removal"))
    {
    case LowMedHigh::low:    config.static_target_removal = IFX_STATIC_TARGET_CANCELATION_LOW; break;
    case LowMedHigh::medium: config.static_target_removal = IFX_STATIC_TARGET_CANCELATION_MED; break;
    case LowMedHigh::high:   config.static_target_removal = IFX_STATIC_TARGET_CANCELATION_HIGH; break;
    }

    config.enable_hysteresis = get_bool("enable_hysteresis");
    config.max_num_tracks = get_uint32("max_num_tracks");
    config.segments_degrees = get_vector_r("segments_degrees");

    // copy from device config
    config.num_samples_per_chirp = device_config->num_samples_per_chirp;
    config.num_chirps_per_frame = device_config->num_chirps_per_frame;
    config.bandwidth_Hz = ifx_device_get_bandwidth(device_config);
    config.chirp_repetition_time_s = ifx_device_get_chirp_repetition_time(device_config);
    config.center_frequency_Hz = ifx_device_get_center_frequency(device_config);

    *config_segmentation = config;
}

/**
 * @brief Return true if fmcw_single_shape configuration is present
 */
bool ifxJsonConfiguration::has_config_fmcw_single_shape() const
{
    if (m_json.contains("device_config") && m_json["device_config"].contains("fmcw_single_shape"))
        return true;
    else
        return false;
}

/**
 * @brief Set fmcw single shape configuration
 */
void ifxJsonConfiguration::set_config_fmcw_single_shape(const ifx_Device_Config_t* config_single_shape)
{
    m_json["device_config"] = { {
        "fmcw_single_shape", {
            { "rx_antennas", mask_to_antenna_vector(config_single_shape->rx_mask) },
            { "tx_antennas", mask_to_antenna_vector(config_single_shape->rx_mask) },
            { "tx_power_level", config_single_shape->tx_power_level },
            { "if_gain_dB", config_single_shape->if_gain_dB },
            { "lower_frequency_Hz", config_single_shape->lower_frequency_Hz },
            { "upper_frequency_Hz", config_single_shape->upper_frequency_Hz },
            { "num_chirps_per_frame", config_single_shape->num_chirps_per_frame },
            { "num_samples_per_chirp", config_single_shape->num_samples_per_chirp },
            { "chirp_repetition_time_s", config_single_shape->chirp_repetition_time_s },
            { "frame_repetition_time_s", config_single_shape->frame_repetition_time_s }
        }
    } };
}

/**
 * @brief Get fmcw_single_shape configuration
 *
 * Save the configuration to single_shape.
 *
 * @param [in]      single_shape    single shape configuration
 * @retval true     if successfull
 * @retval false    if an error occured
 */

/**
 * @brief Get fmcw_single_shape configuration
 *
 * Write the fmcw_single_shape configuration to single_shape. If the
 * configuration is incorrect, an exception is thrown.
 *
 * @param [in]      device_config   device configuration
 * @param [out]     config_scene    scene configuration
 */
void ifxJsonConfiguration::get_config_fmcw_single_shape(ifx_Device_Config_t* single_shape)
{
    ifx_Device_Config_t config = {};

    set_active_json_object({"device_config", "fmcw_single_shape"});

    // required parameters
    config.rx_mask = get_antenna_mask("rx_antennas");
    config.tx_mask = get_antenna_mask("tx_antennas");
    config.tx_power_level = get_uint32_bounds("tx_power_level", TX_POWER_LEVEL_LOWER, TX_POWER_LEVEL_UPPER);
    config.if_gain_dB = get_uint32_bounds("if_gain_dB", IF_GAIN_DB_LOWER, IF_GAIN_DB_UPPER);
    config.chirp_repetition_time_s = get_positive_number("chirp_repetition_time_s");
    config.frame_repetition_time_s = get_positive_number("frame_repetition_time_s");
    config.num_chirps_per_frame = get_uint32("num_chirps_per_frame");
    config.num_samples_per_chirp = get_uint32("num_samples_per_chirp");

    const double lower_frequency = get_number_bounds("lower_frequency_Hz", 0, 80e9);
    const double upper_frequency = get_number_bounds("upper_frequency_Hz", 0, 80e9);
    if (lower_frequency >= upper_frequency )
        throw string("upper_frequency_Hz must be larger than lower_frequency_Hz");
    config.lower_frequency_Hz = static_cast<uint64_t>(std::round(lower_frequency));
    config.upper_frequency_Hz = static_cast<uint64_t>(std::round(upper_frequency));

    // optional parameters
    config.sample_rate_Hz = get_sample_rate();
    config.mimo_mode = get_mimo_mode();

    // everything fine
    *single_shape = config;
}

/* ----------- *
 * C interface *
 * ----------- */

struct ifxJsonConfigurationC
{
    ifxJsonConfiguration obj;
    string error;
};

ifx_json_t* ifx_json_create(void)
{
    return new (std::nothrow) ifxJsonConfigurationC;
}

void ifx_json_destroy(ifx_json_t* j)
{
    delete j;
}

const char* ifx_json_get_error(const ifx_json_t* json)
{
    return json->error.c_str();
}

bool ifx_json_load_from_file(ifx_json_t* json, const char* filename)
{
    try
    {
        json->obj.load_from_file(filename);
        return true;
    }
    catch (string& error)
    {
        json->error = error;
        return false;
    }
}

bool ifx_json_save_to_file(ifx_json_t* json, const char* filename)
{
    try
    {
        json->obj.save_to_file(filename);
        return true;
    }
    catch (string& error)
    {
        json->error = error;
        return false;
    }
}

ifx_json_t* ifx_json_create_from_file(const char* filename)
{
    ifxJsonConfigurationC* json = ifx_json_create();
    if (json == nullptr)
        return nullptr;

    if (ifx_json_load_from_file(json, filename))
        return json;

    // loading was not successful
    ifx_json_destroy(json);
    return nullptr;
}

bool ifx_json_has_config_scene(const ifx_json_t* json)
{
    return json->obj.has_config_fmcw_scene();
}

void ifx_json_set_device_config_scene(ifx_json_t* json, const ifx_Device_Metrics_t* metrics)
{
    return json->obj.set_config_fmcw_scene(metrics);
}

bool ifx_json_get_device_config_scene(ifx_json_t* json, ifx_Device_Metrics_t* metrics)
{
    try
    {
        json->obj.get_config_fmcw_scene(metrics);
        return true;
    }
    catch (string& error)
    {
        json->error = error;
        return false;
    }
}

bool ifx_json_get_device_config_single_shape(ifx_json_t* json, ifx_Device_Config_t* config)
{
    try
    {
        json->obj.get_config_fmcw_single_shape(config);
        return true;
    }
    catch (string& error)
    {
        json->error = error;
        return false;
    }
}

bool ifx_json_has_config_single_shape(const ifx_json_t* json)
{
    return json->obj.has_config_fmcw_single_shape();
}

void ifx_json_set_device_config_single_shape(ifx_json_t* json, const ifx_Device_Config_t* config)
{
    json->obj.set_config_fmcw_single_shape(config);
}

void ifx_json_set_segmentation(ifx_json_t* json, const ifx_Segmentation_Config_t* segmentation_config)
{
    json->obj.set_config_segmentation(segmentation_config);
}

bool ifx_json_has_segmentation(const ifx_json_t* json)
{
    return json->obj.has_config_segmentation();
}

bool ifx_json_get_segmentation(ifx_json_t* json, const ifx_Device_Config_t* device_config, ifx_Segmentation_Config_t* config_segmentation)
{
    try
    {
        json->obj.get_config_segmentation(device_config, config_segmentation);
        return true;
    }
    catch (string& error)
    {
        json->error = error;
        return false;
    }
}

void ifx_json_set_presence_sensing(ifx_json_t* json, const ifx_PresenceSensing_Config_t* config_presence_sensing)
{
    json->obj.set_config_presence_sensing(config_presence_sensing);
}

bool ifx_json_has_presence_sensing(const ifx_json_t* json)
{
    return json->obj.has_config_presence_sensing();
}

bool ifx_json_get_presence_sensing(ifx_json_t* json, const ifx_Device_Config_t* device_config, ifx_PresenceSensing_Config_t* config_presence_sensing)
{
    try
    {
        json->obj.get_config_presence_sensing(device_config, config_presence_sensing);
        return true;
    }
    catch (string& error)
    {
        json->error = error;
        return false;
    }
}

/**
* @}
*/
