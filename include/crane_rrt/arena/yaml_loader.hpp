#ifndef ARENA_YAML_LOADER_HPP
#define ARENA_YAML_LOADER_HPP

#include <string>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <yaml-cpp/yaml.h>

namespace arena {

class YamlLoader {
public:
    static YAML::Node LoadFileWithDiagnostics(const std::string& path) {
        try {
            if (!std::filesystem::exists(path)) {
                std::cerr << "YAML load error: File not found: " << path << std::endl;

                std::filesystem::path parent = std::filesystem::path(path).parent_path();
                std::string target = std::filesystem::path(path).filename().string();

                if (std::filesystem::exists(parent) && std::filesystem::is_directory(parent)) {
                    std::vector<std::string> candidates;
                    for (const auto& entry : std::filesystem::directory_iterator(parent)) {
                        if (entry.is_regular_file()) {
                            std::string name = entry.path().filename().string();
                            if (IsYamlFile(name)) {
                                candidates.push_back(name);
                            }
                        }
                    }

                    if (!candidates.empty()) {
                        std::string suggestion = FindClosestName(target, candidates);
                        if (!suggestion.empty()) {
                            std::cerr << ">>> \033[3mDid you mean:\033[0m " << "\033[3;33m" << (parent / suggestion).string() << "\033[0m" << " ?" << std::endl;
                            // std::cerr << ">>> Did you mean: " << (parent / suggestion).string() << " ?" << std::endl;
                        }
                    }
                }

                throw std::runtime_error("Cannot find file: " + path);
            }

            return YAML::LoadFile(path);
        }
        catch (const std::exception& e) {
            std::cerr << "Failed to load YAML file: " << path << std::endl;
            std::cerr << "Reason: " << e.what() << std::endl;
            throw;
        }
    }

private:
     static bool IsYamlFile(const std::string& filename) {
        return EndsWith(filename, ".yaml") || EndsWith(filename, ".yml");
    }

    static bool EndsWith(const std::string& str, const std::string& suffix) {
        return str.size() >= suffix.size() &&
               str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
    }

    static std::string FindClosestName(const std::string& target, const std::vector<std::string>& candidates) {
        int minDist = std::numeric_limits<int>::max();
        std::string bestMatch;

        for (const auto& name : candidates) {
            int dist = LevenshteinDistance(target, name);
            if (dist < minDist) {
                minDist = dist;
                bestMatch = name;
            }
        }

        return (minDist <= 5) ? bestMatch : "";
    }

    // Classic Levenshtein distance (O(n*m), acceptable for small strings)
    static int LevenshteinDistance(const std::string& a, const std::string& b) {
        const size_t m = a.size();
        const size_t n = b.size();

        std::vector<std::vector<int>> dp(m + 1, std::vector<int>(n + 1));
        for (size_t i = 0; i <= m; ++i) dp[i][0] = static_cast<int>(i);
        for (size_t j = 0; j <= n; ++j) dp[0][j] = static_cast<int>(j);

        for (size_t i = 1; i <= m; ++i) {
            for (size_t j = 1; j <= n; ++j) {
                int cost = (a[i - 1] == b[j - 1]) ? 0 : 1;
                dp[i][j] = std::min({ 
                    dp[i - 1][j] + 1,      // deletion
                    dp[i][j - 1] + 1,      // insertion
                    dp[i - 1][j - 1] + cost // substitution
                });
            }
        }

        return dp[m][n];
    }
};

} // namespace arena

#endif // ARENA_YAML_LOADER_HPP
