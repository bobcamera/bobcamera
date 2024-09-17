#pragma once

#include <string>
#include <algorithm>

std::string to_lowercase(const std::string &str)
{
    std::string lower_str = str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);
    return lower_str;
}

template <typename T>
T string_to_number(const std::string &str, T default_value)
{
    T value;
    auto result = std::from_chars(str.data(), str.data() + str.size(), value);

    // Check if conversion was successful
    if (result.ec == std::errc())
    {
        return value;
    }
    else
    {
        // Return the default value if conversion fails
        return default_value;
    }
}

// Specialization for floating point numbers (float, double, etc.)
template <>
float string_to_number<float>(const std::string &str, float default_value)
{
    try
    {
        return std::stof(str);
    }
    catch (...)
    {
        return default_value;
    }
}

template <>
double string_to_number<double>(const std::string &str, double default_value)
{
    try
    {
        return std::stod(str);
    }
    catch (...)
    {
        return default_value;
    }
}

std::tuple<int, int, int> extract_rgb(const std::string &str)
{
    int r, g, b;
    char ch; // to skip non-numeric characters

    // Create a stringstream to read the values
    std::stringstream ss(str);

    // Parse the string in the format "{R, G, B}"
    ss >> ch >> r >> ch >> g >> ch >> b >> ch;

    // Return the extracted values as a tuple
    return {r, g, b};
}

std::map<std::string, std::string> parse_json_to_map(const std::string & jsonString)
{
    // if (jsonString.empty())
    // {
    //     return {};
    // }

    // Create a JSON root object to hold the parsed data
    Json::Value root;
    Json::CharReaderBuilder readerBuilder;
    std::string errs;

    // Parse the JSON string
    std::istringstream stream(jsonString);
    if (!Json::parseFromStream(readerBuilder, stream, &root, &errs))
    {
        std::cerr << "Text: \"" << jsonString << "\", Failed to parse JSON: " << errs << std::endl;
        return {};
    }

    // Create a std::map to hold the key-value pairs
    std::map<std::string, std::string> result;

    // Iterate over the JSON object's members
    for (const auto &key : root.getMemberNames())
    {
        const Json::Value &value = root[key];

        // Convert the JSON value to a string and insert into the map
        if (value.isString())
        {
            result[key] = value.asString();
        }
        else if (value.isNumeric())
        {
            result[key] = std::to_string(value.asFloat());
        }
        else if (value.isBool())
        {
            result[key] = value.asBool() ? "true" : "false";
        }
        else if (value.isObject())
        {
            result[key] = value.toStyledString();
        }
    }

    return result;
}