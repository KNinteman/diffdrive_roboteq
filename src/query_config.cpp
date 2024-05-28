#include "yaml-cpp/yaml.h"
#include <string>
#include <map>
#include <iostream>
#include <fstream>

class QueryConfig {
public:
    std::map<std::string, std::string> queries;
};

QueryConfig loadQueryConfig(const std::string& yaml_file_path) {
    QueryConfig config;

    // Check if the file exists
    std::ifstream infile(yaml_file_path);
    if (!infile.good()) {
        std::cerr << "Failed to load YAML config: file does not exist or is not readable: " << yaml_file_path << std::endl;
        return config;
    }

    try {
        YAML::Node yaml_config = YAML::LoadFile(yaml_file_path);
        for (const auto& item : yaml_config["query"]) {
            std::string topic = item.first.as<std::string>();
            std::string query = item.second.as<std::string>();
            config.queries[topic] = query;
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "Failed to load YAML config: " << e.what() << std::endl;
    }

    return config;
}
