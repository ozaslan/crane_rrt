#ifndef ARENA_PRIMITIVE_FACTORY_HPP
#define ARENA_PRIMITIVE_FACTORY_HPP

#include <string>
#include <memory>
#include <stdexcept>
#include <algorithm>
#include <cctype>
#include <yaml-cpp/yaml.h>

#include "primitive.hpp"
#include "sphere.hpp" 
#include "arena_yaml_utils.hpp"

namespace arena {

template<typename Scalar = double>
class PrimitiveFactory {
public:
    using Ptr   = typename Primitive<Scalar>::Ptr;
    using Point = typename Primitive<Scalar>::Point;
    using Color = typename Primitive<Scalar>::Color;
    using Pose  = typename Primitive<Scalar>::Pose;

    static Ptr LoadFromNode(const YAML::Node& node) {
        if (!node || !node.IsMap()) {
            throw std::runtime_error("Primitive node must be a map.");
        }

        if (!node["type"]) {
            throw std::runtime_error("Primitive node is missing required field: 'type'");
        }

        std::string type = node["type"].as<std::string>();
        Ptr primitive;

        if (type == "sphere") {
            primitive = std::make_shared<Sphere<Scalar>>();
        }
        // else if (type == "box") { ... }
        else {
            throw std::runtime_error("Unknown primitive type: '" + type + "'");
        }

        // Optional: tag
        if (node["tag"]) {
            primitive->setTag(node["tag"].as<std::string>());
        }

        // Optional: pose
        if (node["pose"]) {
            Pose pose = LoadPose<Scalar>(node["pose"]);
            primitive->setPose(pose);
        }

        // Optional: scale
        if (node["scale"]) {
            primitive->setScale(LoadScale<Scalar>(node["scale"]));
        }

        // Optional: color
        if (node["color"]) {
            primitive->setColor(LoadColor<Scalar>(node["color"]));
        }

        return primitive;
    }
};

} // namespace arena

#endif // ARENA_PRIMITIVE_FACTORY_HPP
