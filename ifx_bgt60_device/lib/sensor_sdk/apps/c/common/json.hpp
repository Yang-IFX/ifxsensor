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
*
* @{
*/

#ifndef APP_COMMON_JSON_HPP
#define APP_COMMON_JSON_HPP


/*
==============================================================================
    1. INCLUDE FILES
==============================================================================
*/

#include <array>
#include <string>
#include <vector>
#include <cstdint>

#include <nlohmann/json.hpp>

#include "ifxRadar/SDK.h"

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

enum class LowMedHigh { low = 0, medium = 1, high = 2 };

class ifxJsonConfiguration
{
private:
    nlohmann::ordered_json m_json;
    const nlohmann::ordered_json* m_active = nullptr;
    std::string m_path;

    void set_active_json_object(const std::vector<std::string>& path);
    const nlohmann::ordered_json& get_json_object(const std::string& name) const;

    bool get_bool(const std::string& name) const;
    bool get_bool(const std::string& name, bool default_value) const;

    std::string get_string(const std::string& name) const;
    std::string get_string(const std::string& name, const std::string& default_value) const;

    ifx_Float_t get_number(const std::string& name) const;
    ifx_Float_t get_number(const std::string& name, ifx_Float_t default_value) const;
    ifx_Float_t get_number_bounds(const std::string& name, ifx_Float_t min, ifx_Float_t max) const;

    ifx_Float_t get_positive_number(const std::string& name) const;
    ifx_Float_t get_positive_number(const std::string& name, ifx_Float_t default_value) const;

    uint32_t get_uint32(const std::string& name) const;
    uint32_t get_uint32(const std::string& name, uint32_t default_value) const;
    uint32_t get_uint32_bounds(const std::string& name, uint32_t min, uint32_t max) const;

    uint64_t get_uint64(const std::string& name) const;
    uint64_t get_uint64(const std::string& name, uint64_t default_value) const;
    uint64_t get_uint64_bounds(const std::string& name, uint64_t min, uint64_t max) const;

    uint32_t get_antenna_mask(const std::string& name) const;
    LowMedHigh get_low_med_high(const std::string& name) const;
    ifx_Vector_R_t* get_vector_r(const std::string& name) const;
    uint32_t get_sample_rate() const;

    enum ifx_Device_MIMO_Mode get_mimo_mode() const;

public:
    ifxJsonConfiguration();

    void load_from_file(const std::string& filename, int type = 0, const char* uuid = "");
    void save_to_file(const std::string& filename);

    bool has_device() const;
    const std::vector<std::array<uint8_t,16>> get_device_uuids() const;

    bool has_config_fmcw_scene() const;
    void get_config_fmcw_scene(ifx_Device_Metrics_t* config_scene);
    void set_config_fmcw_scene(const ifx_Device_Metrics_t* config);

    bool has_config_fmcw_single_shape() const;
    void get_config_fmcw_single_shape(ifx_Device_Config_t* config_single_shape);
    void set_config_fmcw_single_shape(const ifx_Device_Config_t* config);

    bool has_config_segmentation() const;
    void get_config_segmentation(const ifx_Device_Config_t* device_config, ifx_Segmentation_Config_t* config_segmentation);
    void set_config_segmentation(const ifx_Segmentation_Config_t* config);

    bool has_config_presence_sensing() const;
    void get_config_presence_sensing(const ifx_Device_Config_t* device_config, ifx_PresenceSensing_Config_t* config);
    void set_config_presence_sensing(const ifx_PresenceSensing_Config_t* config);
};


/*
==============================================================================
    4. FUNCTION PROTOTYPES
==============================================================================
*/



#endif // #ifndef APP_COMMON_JSON_HPP

/**
* @}
*/