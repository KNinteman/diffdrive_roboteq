#ifndef QUERY_CONFIG_HPP
#define QUERY_CONFIG_HPP

#include <map>
#include <string>
#include <iostream>

struct QueryConfig {
    std::map<std::string, std::string> queries;
};

QueryConfig loadQueryConfig(const std::string& yaml_file_path);

#endif  // QUERY_CONFIG_HPP