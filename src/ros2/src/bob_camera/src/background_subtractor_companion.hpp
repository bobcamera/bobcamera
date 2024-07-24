#pragma once
#ifndef __BACKGROUND_SUBTRACTOR_COMPANION_H__
#define __BACKGROUND_SUBTRACTOR_COMPANION_H__

#include <string>
#include <map>
#include <json/json.h>

class Sensitivity 
{
public:
    int vibe_threshold;
    int vibe_bg_samples;
    int vibe_required_bg_samples;
    int vibe_learning_rate;
    bool wmv_enable_weight;
    bool wmv_enable_threshold;
    double wmv_threshold;
    double wmv_weight1;
    double wmv_weight2;
    double wmv_weight3;
    int blob_size_threshold;
    int blob_area_threshold;
    int blob_min_distance;
    int blob_max_blobs;
    bool median_filter;
};

class SensitivityConfig
{
public:
    std::string name;
    Sensitivity sensitivity;
};

class SensitivityConfigCollection
{
public:
    const auto& get_configs() const { return configs_; }

    void set_configs(const std::map<std::string, SensitivityConfig>& configs) 
    { 
        configs_ = configs; 
    }

    void set_configs(const std::string & json_string)
    { 
        from_json_string(json_string);
    }

private:
    // Method to parse JSON string into SensitivityConfigCollection object
    void from_json_string(const std::string& json_string) 
    {
        Json::Value root;
        Json::Reader reader;

        if (!reader.parse(json_string, root)) 
        {
            throw std::invalid_argument("Failed to parse JSON");
        }

        if (!root.isArray()) 
        {
            throw std::invalid_argument("JSON is not an array");
        }

        for (const auto& item : root) 
        {
            SensitivityConfig config;
            config.name = item["name"].asString();
            config.sensitivity.vibe_threshold = item["sensitivity"]["vibe"]["threshold"].asInt();
            config.sensitivity.vibe_bg_samples = item["sensitivity"]["vibe"]["bgSamples"].asInt();
            config.sensitivity.vibe_required_bg_samples = item["sensitivity"]["vibe"]["requiredBGSamples"].asInt();
            config.sensitivity.vibe_learning_rate = item["sensitivity"]["vibe"]["learningRate"].asInt();
            config.sensitivity.wmv_enable_weight = item["sensitivity"]["wmv"]["enableWeight"].asBool();
            config.sensitivity.wmv_enable_threshold = item["sensitivity"]["wmv"]["enableThreshold"].asBool();
            config.sensitivity.wmv_threshold = item["sensitivity"]["vibe"]["threshold"].asDouble();
            config.sensitivity.wmv_weight1 = item["sensitivity"]["wmv"]["weight1"].asDouble();
            config.sensitivity.wmv_weight2 = item["sensitivity"]["wmv"]["weight2"].asDouble();
            config.sensitivity.wmv_weight3 = item["sensitivity"]["wmv"]["weight3"].asDouble();
            config.sensitivity.blob_size_threshold = item["sensitivity"]["blob"]["sizeThreshold"].asInt();
            config.sensitivity.blob_area_threshold = item["sensitivity"]["blob"]["areaThreshold"].asInt();
            config.sensitivity.blob_min_distance = item["sensitivity"]["blob"]["minDistance"].asInt();
            config.sensitivity.blob_max_blobs = item["sensitivity"]["blob"]["maxBlobs"].asInt();
            config.sensitivity.median_filter = item["sensitivity"]["median_filter"].asBool();

            configs_[config.name] = config;
        }

    }

    std::map<std::string, SensitivityConfig> configs_;
};

#endif