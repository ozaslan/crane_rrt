#ifndef ARENA_BUNDLE_FACTORY_HPP
#define ARENA_BUNDLE_FACTORY_HPP

#include <string>
#include <vector>
#include <memory>
#include <unordered_set>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include "primitive_bundle.hpp"
#include "primitive_factory.hpp"
#include "arena_yaml_utils.hpp"
#include "yaml_loader.hpp"
#include "image_to_circles.hpp"

namespace arena {

template<typename Scalar>
class BundleFactory {
public:
    using Primitive     = arena::Primitive<Scalar>;
    using PrimitivePtr  = typename Primitive::Ptr;
    using Pose          = typename Primitive::Pose;
    using Point         = typename Primitive::Point;
    using Bundle        = arena::PrimitiveBundle<Scalar>;
    using BundlePtr     = typename Bundle::Ptr;
    using PrimitiveList = std::vector<PrimitivePtr>;

    // Entry point: load a bundle from a file
    static BundlePtr LoadTree(
        const std::string& filepath,
        const Pose& parent_pose = Pose::Identity(),
        const Point& parent_scale = Point::Ones(),
        std::unordered_set<std::string>* visited = nullptr
    ) {

        std::filesystem::path abs_path = std::filesystem::weakly_canonical(filepath);
        std::string key = abs_path.string();

        std::unordered_set<std::string> owned_visited;
        if (!visited) visited = &owned_visited;

        if (visited->count(key)) {
            throw std::runtime_error(">>> \033[3mRecursive inclusion detected for bundle file:\033[0m \033[3;33m" + key + "\033[0m");
        }
        visited->insert(key);

        YAML::Node root;

        try {
            root = YamlLoader::LoadFileWithDiagnostics(abs_path);
        } catch (const std::exception& e) {
            throw std::runtime_error("While loading bundle file: " + abs_path.string() + "\n→ " + e.what());
        }
        
        if (!root["bundle"]) {
            throw std::runtime_error("Missing required top-level 'bundle' key in file: " + key);
        }

        const YAML::Node& bundle_node = root["bundle"];
        auto bundle = LoadTree(bundle_node, abs_path.parent_path(), parent_pose, parent_scale, visited);

        visited->erase(key);
        return bundle;
    }

    // Entry point: load a bundle from a node (inline or nested)
    static BundlePtr LoadTree(
        const YAML::Node& bundle_node,
        const std::filesystem::path& context,
        const Pose& parent_pose = Pose::Identity(),
        const Point& parent_scale = Point::Ones(),
        std::unordered_set<std::string>* visited = nullptr
    ) {

        if (!bundle_node || !bundle_node.IsMap()) {
            throw std::runtime_error("Invalid or empty bundle node.");
        }

        auto current_bundle = std::make_shared<Bundle>();

        if (bundle_node["tag"])
            current_bundle->setTag(bundle_node["tag"].as<std::string>());

        // Load primitives
        if (bundle_node["primitives"]) {
            for (const auto& prim_node : bundle_node["primitives"]) {
                auto prim = PrimitiveFactory<Scalar>::LoadFromNode(prim_node);

                Pose  local_pose   = prim->getPose();
                Pose  global_pose  = parent_pose * local_pose;

                Point local_scale  = prim->getScale();
                Point global_scale = parent_scale.cwiseProduct(local_scale);

                prim->setPose(global_pose);
                prim->setScale(global_scale);

                current_bundle->addChild(prim);
            }
        }

        if (bundle_node["from_image"]) {
            LoadFromImageNode(bundle_node["from_image"], context, current_bundle, parent_pose, parent_scale);
        }

        // Load nested bundles
        if (bundle_node["bundles"]) {
            for (const auto& child : bundle_node["bundles"]) {
                Pose  rel_pose  = LoadPose<Scalar>(child["pose"]);
                Point rel_scale = LoadScale<Scalar>(child["scale"]);

                Pose  global_pose  = parent_pose * rel_pose;
                Point global_scale = parent_scale.cwiseProduct(rel_scale);

                if (child["file"]) {
                    std::string subpath = child["file"].as<std::string>();
                    std::filesystem::path resolved = context / subpath;

                    auto child_bundle = LoadTree(resolved.string(), global_pose, global_scale, visited);
                    current_bundle->addChild(child_bundle);
                } else if (child["primitives"] || child["bundles"] || child["from_image"]) {
                    auto inline_bundle = LoadTree(child, context, global_pose, global_scale, visited);
                    current_bundle->addChild(inline_bundle);
                } else {
                    throw std::runtime_error("Invalid bundle entry: must contain either 'file:' or inline contents.");
                }
            }
        }

        current_bundle->setPose(parent_pose);
        current_bundle->setScale(parent_scale);
        return current_bundle;
    }

private:
    static void LoadFromImageNode(const YAML::Node& img_node, 
                                        const std::filesystem::path& context,
                                        BundlePtr& current_bundle,
                                        const Pose& parent_pose = Pose::Identity(),
                                        const Point& parent_scale = Point::Ones()
    ) {
        std::string image_path = (context / img_node["path"].as<std::string>()).string();
        double pixel_size_cm = img_node["pixel_size_cm"].as<double>();
        double min_radius_px = img_node["min_radius_px"] ? img_node["min_radius_px"].as<double>() : 5.0;
        double whiteness_thresh = img_node["whiteness_threshold"] ? img_node["whiteness_threshold"].as<double>() : 250.0;
        double z_plane = img_node["z_plane"] ? img_node["z_plane"].as<double>() : 0.0;
        bool   group_by_component = img_node["group_by_component"] ? img_node["group_by_component"].as<bool>() : true;

        Color<Scalar> color{1.0, 1.0, 1.0, 1.0};
        bool use_color = false;
        if (img_node["color"]) {
            auto rgba = img_node["color"].as<std::vector<Scalar>>();
            if (rgba.size() >= 3) {
                color.r = rgba[0];
                color.g = rgba[1];
                color.b = rgba[2];
                if (rgba.size() == 4) 
                    color.a = rgba[3];
                use_color = true;
            }
        }

        std::string tag_prefix = img_node["tag_prefix"] ? img_node["tag_prefix"].as<std::string>() : "circle";

        // Generate circles
        arena::ImageToCircles itc(min_radius_px, whiteness_thresh);
        if (!itc.loadImage(image_path, pixel_size_cm)) {
            throw std::runtime_error("Failed to load image for from_image bundle: " + image_path);
        }

        itc.process();
        const auto labelToCircles = itc.getLabeledCircles();
        const double pxToMeter = pixel_size_cm / 100.0;

        int global_circle_id = 0;
        for (const auto& [label, circles] : labelToCircles) {

            auto sub_bundle = std::make_shared<arena::PrimitiveBundle<Scalar>>();
            sub_bundle->setTag("component_" + std::to_string(label));

            for (const auto& c : circles) {
                float x_m = +(c.centerNorm.x - 0.5f) * itc.getImageSize().width * pxToMeter;
                float y_m = -(c.centerNorm.y - 0.5f) * itc.getImageSize().height * pxToMeter;
                float r_m = c.radiusNorm * std::min(itc.getImageSize().width, itc.getImageSize().height) * pxToMeter;

                auto s = std::make_shared<arena::Sphere<Scalar>>();

                Point rel_scale    = Point::Constant(2.0 * r_m);
                Point global_scale = parent_scale.cwiseProduct(rel_scale);

                Pose rel_pose = Pose::Identity();
                rel_pose.translation() = Point(x_m, y_m, z_plane);
                Pose  global_pose = parent_pose * rel_pose;

                s->setPose(global_pose);
                s->setScale(global_scale);

                s->setTag(tag_prefix + "_" + std::to_string(global_circle_id++));

                if (use_color)
                    s->setColor(color);

                if(group_by_component){
                    sub_bundle->addChild(s);    
                } else {
                    current_bundle->addChild(s);        
                }
                
            }

            if(group_by_component){
                current_bundle->addChild(sub_bundle);
            }
        }
    }
};

} // namespace arena

#endif // ARENA_BUNDLE_FACTORY_HPP
