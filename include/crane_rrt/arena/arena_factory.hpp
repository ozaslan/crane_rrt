#ifndef ARENA_ARENA_FACTORY_HPP
#define ARENA_ARENA_FACTORY_HPP

#include <string>
#include <memory>
#include <unordered_set>
#include <filesystem>
#include <yaml-cpp/yaml.h>

#include "arena.hpp"
#include "bundle_factory.hpp"
#include "arena_yaml_utils.hpp"
#include "yaml_loader.hpp"

namespace arena {

template<typename Scalar>
class ArenaFactory {
public:
    using ArenaType   = Arena<Scalar>;
    using ArenaPtr    = std::shared_ptr<ArenaType>;
    using Bundle      = PrimitiveBundle<Scalar>;
    using BundlePtr   = typename Bundle::Ptr;
    using Pose        = typename Bundle::Pose;
    using Point       = typename Bundle::Point;

    static ArenaPtr LoadFromFile(const std::string& path) {
        YAML::Node root;
        try {
            root = YamlLoader::LoadFileWithDiagnostics(path);
        } catch (const std::exception& e) {
            throw std::runtime_error("While loading arena file: " + path + "\n→ " + e.what());
        }

        if (!root || !root.IsMap())
            throw std::runtime_error("Invalid or empty YAML file: " + path);

        if (!root["arena"])
            throw std::runtime_error("Missing top-level 'arena' key in file: " + path);

        const YAML::Node& arenaNode = root["arena"];
        if (!arenaNode["name"] || !arenaNode["id"] || !arenaNode["bundles"])
            throw std::runtime_error("Missing one of required fields: 'name', 'id', or 'bundles' in arena file: " + path);

        std::string name = arenaNode["name"].as<std::string>();
        std::string id   = arenaNode["id"].as<std::string>();
        const YAML::Node& bundlesNode = arenaNode["bundles"];

        if (!bundlesNode.IsSequence())
            throw std::runtime_error("'bundles' field must be a list in file: " + path);

        // Context and cycle detection
        std::filesystem::path context = std::filesystem::path(path).parent_path();
        std::unordered_set<std::string> visited;

        // Root bundle holding all top-level children
        auto rootBundle = std::make_shared<Bundle>();
        rootBundle->setTag("root");

        for (std::size_t i = 0; i < bundlesNode.size(); ++i) {
            const auto& node = bundlesNode[i];

            Pose  pose  = LoadPose<Scalar>(node["pose"]);
            Point scale = LoadScale<Scalar>(node["scale"]);

            BundlePtr subBundle = nullptr;

            if (node["file"]) {
                std::string relPath = node["file"].as<std::string>();
                std::filesystem::path resolved = context / relPath;

                try {
                    subBundle = BundleFactory<Scalar>::LoadTree(resolved.string(), pose, scale, &visited);
                } catch (const std::exception& e) {
                    throw std::runtime_error("While loading an bundle file inside an the arena file: " + path + "\n→ " + e.what());
                }
            }
            else if (node["primitives"] || node["bundles"] || node["from_image"]) {
                try {
                    subBundle = BundleFactory<Scalar>::LoadTree(node, context, pose, scale, &visited);
                } catch (const std::exception& e) {
                    throw std::runtime_error("While parsing inline bundle at index " + std::to_string(i) + " in arena file: " + path + "\n→ " + e.what());
                }
            } else {
                throw std::runtime_error("Each bundle entry must contain either 'file:' or inline bundle content at index " + std::to_string(i));
            }

            rootBundle->addChild(subBundle);
        }

        return std::make_shared<ArenaType>(name, id, rootBundle);
    }
};

} // namespace arena

#endif // ARENA_ARENA_FACTORY_HPP
