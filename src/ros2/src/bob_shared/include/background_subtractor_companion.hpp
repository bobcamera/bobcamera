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
    int vibe_bgSamples;
    int vibe_requiredBGSamples;
    int vibe_learningRate;
    bool wmv_enableWeight;
    bool wmv_enableThreshold;
    double wmv_threshold;
    double wmv_weight1;
    double wmv_weight2;
    double wmv_weight3;
    int blob_sizeThreshold;
    int blob_areaThreshold;
    int blob_minDistance;
    int blob_maxBlobs;
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
    std::map<std::string, SensitivityConfig> configs;

    // Method to parse JSON string into SensitivityConfigCollection object
    static SensitivityConfigCollection fromJsonString(const std::string& jsonString) 
    {
        SensitivityConfigCollection collection;
        Json::Value root;
        Json::Reader reader;

        if (!reader.parse(jsonString, root)) 
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
            config.sensitivity.vibe_bgSamples = item["sensitivity"]["vibe"]["bgSamples"].asInt();
            config.sensitivity.vibe_requiredBGSamples = item["sensitivity"]["vibe"]["requiredBGSamples"].asInt();
            config.sensitivity.vibe_learningRate = item["sensitivity"]["vibe"]["learningRate"].asInt();
            config.sensitivity.wmv_enableWeight = item["sensitivity"]["wmv"]["enableWeight"].asBool();
            config.sensitivity.wmv_enableThreshold = item["sensitivity"]["wmv"]["enableThreshold"].asBool();
            config.sensitivity.wmv_threshold = item["sensitivity"]["vibe"]["threshold"].asDouble();
            config.sensitivity.wmv_weight1 = item["sensitivity"]["wmv"]["weight1"].asDouble();
            config.sensitivity.wmv_weight2 = item["sensitivity"]["wmv"]["weight2"].asDouble();
            config.sensitivity.wmv_weight3 = item["sensitivity"]["wmv"]["weight3"].asDouble();
            config.sensitivity.blob_sizeThreshold = item["sensitivity"]["blob"]["sizeThreshold"].asInt();
            config.sensitivity.blob_areaThreshold = item["sensitivity"]["blob"]["areaThreshold"].asInt();
            config.sensitivity.blob_minDistance = item["sensitivity"]["blob"]["minDistance"].asInt();
            config.sensitivity.blob_maxBlobs = item["sensitivity"]["blob"]["maxBlobs"].asInt();
            config.sensitivity.median_filter = item["sensitivity"]["median_filter"].asBool();

            collection.configs[config.name] = config;
        }

        return collection;
    }
};

#endif