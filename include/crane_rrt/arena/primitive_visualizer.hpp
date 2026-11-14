#ifndef ARENA_OBSTACLE_VISUALIZER_HPP
#define ARENA_OBSTACLE_VISUALIZER_HPP

#include <memory>
#include <vector>
#include <string>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "primitive.hpp"
#include "sphere.hpp"
#include "aabb_visualizer.hpp"

namespace arena {

template<typename Scalar>
class PrimitiveVisualizer {
public:
    using PrimitiveT    = Primitive<Scalar>;
    using PrimitivePtr  = typename PrimitiveT::Ptr;
    using Marker        = visualization_msgs::msg::Marker;
    using MarkerArray   = visualization_msgs::msg::MarkerArray;
    using PrimitiveList = std::vector<PrimitivePtr>;
    using AABBParams    = typename AABBVisualizer<Scalar>::Params;
    using ColorT        = Color<Scalar>;

    struct Params {
        std::string frame_id = "map";
        std::string ns = "primitives";
        rclcpp::Time stamp;
        double lifetime = 0.0;
    };

    // --- Single primitive: optional AABB ---
    static MarkerArray toMarkers(const PrimitiveT& prm,
                                 const Params& params,
                                 int& id,
                                 bool addAABB = false) {
        MarkerArray array;

        Marker m;
        if (convertPrimitive(prm, params, id, m)) {
            array.markers.push_back(std::move(m));
            array.markers.push_back(makeTextMarker(prm, params, id));
        }
        ++id;

        if (addAABB) {
            AABBParams aabbParams;
            aabbParams.frame_id = params.frame_id;
            aabbParams.stamp = params.stamp;
            aabbParams.lifetime = params.lifetime;
            aabbParams.line_width = 0.002;
            aabbParams.ns = "aabb";
            aabbParams.color = prm.getColor();

            MarkerArray box = AABBVisualizer<Scalar>::visualizeAABB(prm, aabbParams, id++);
            array.markers.insert(array.markers.end(), box.markers.begin(), box.markers.end());
        }

        return array;
    }

    // --- Single primitive: full AABB customization ---
    static MarkerArray toMarkers(const PrimitiveT& prm,
                                 const Params& params,
                                 const AABBParams& aabbParams,
                                 int& id) {
        MarkerArray array;

        Marker m;
        if (convertPrimitive(prm, params, id, m)) {
            array.markers.push_back(std::move(m));
            array.markers.push_back(makeTextMarker(prm, params, id));
        }
        ++id;

        MarkerArray box = AABBVisualizer<Scalar>::visualizeAABB(prm, aabbParams, id++);
        array.markers.insert(array.markers.end(), box.markers.begin(), box.markers.end());

        return array;
    }

    // --- List of primitives: optional AABB ---
    static MarkerArray toMarkers(const PrimitiveList& list,
                                 const Params& params,
                                 int& id,
                                 bool addAABB = false) {
        MarkerArray array;
        for (const auto& prm : list) {
            Marker m;
            if (convertPrimitive(*prm, params, id, m)) {
                array.markers.push_back(std::move(m));
                array.markers.push_back(makeTextMarker(*prm, params, id));
            }
            ++id;

            if (addAABB) {
                AABBParams aabbParams;
                aabbParams.frame_id = params.frame_id;
                aabbParams.stamp = params.stamp;
                aabbParams.lifetime = params.lifetime;
                aabbParams.line_width = 0.002;
                aabbParams.ns = "aabb";
                aabbParams.color = prm->getColor();

                MarkerArray box = AABBVisualizer<Scalar>::visualizeAABB(*prm, aabbParams, id++);
                array.markers.insert(array.markers.end(), box.markers.begin(), box.markers.end());
            }
        }

        return array;
    }

    // --- List of primitives: full AABB customization ---
    static MarkerArray toMarkers(const PrimitiveList& list,
                                 const Params& params,
                                 const AABBParams& aabbParams,
                                 int& id) {
        MarkerArray array;
        for (const auto& prm : list) {
            Marker m;
            if (convertPrimitive(*prm, params, id, m)) {
                array.markers.push_back(std::move(m));
                array.markers.push_back(makeTextMarker(*prm, params, id));
            }
            ++id;

            MarkerArray box = AABBVisualizer<Scalar>::visualizeAABB(*prm, aabbParams, id++);
            array.markers.insert(array.markers.end(), box.markers.begin(), box.markers.end());
        }
        return array;
    }

private:
    static bool convertPrimitive(const PrimitiveT& prm, const Params& params, int id, Marker& marker) {
        marker.header.frame_id = params.frame_id;
        marker.header.stamp = params.stamp;
        marker.ns = params.ns;
        marker.id = id;
        marker.action = Marker::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(params.lifetime);
        marker.pose = toRosPose(prm.getPosition(), prm.getOrientation());

        auto scale = prm.getScale();
        marker.scale.x = scale.x();
        marker.scale.y = scale.y();
        marker.scale.z = scale.z();

        const auto& c = prm.getColor();
        marker.color.r = c.r;
        marker.color.g = c.g;
        marker.color.b = c.b;
        marker.color.a = c.a;

        if (dynamic_cast<const Sphere<Scalar>*>(&prm) != nullptr) {
            marker.type = Marker::SPHERE;
            marker.points.clear();
            marker.colors.clear();
            marker.text.clear();
            marker.mesh_resource.clear();
            return true;
        }

        return false;
    }

    static Marker makeTextMarker(const PrimitiveT& prm, const Params& params, int id) {
        Marker text;
        text.header.frame_id = params.frame_id;
        text.header.stamp = params.stamp;
        text.ns = params.ns + "_tag";
        text.id = id;
        text.type = Marker::TEXT_VIEW_FACING;
        text.action = Marker::ADD;
        text.lifetime = rclcpp::Duration::from_seconds(params.lifetime);

        const auto& pos = prm.getPosition();
        text.pose.position.x = pos.x();
        text.pose.position.y = pos.y();
        text.pose.position.z = pos.z() + prm.getScale().z() / 2 + 0.1;  // Raise above primitive
        text.pose.orientation.w = 1.0;

        text.scale.z = 0.10; // Font size

        const auto& c = prm.getColor();
        text.color.r = c.r;
        text.color.g = c.g;
        text.color.b = c.b;
        text.color.a = 1.0;

        text.text = prm.getTag();

        return text;
    }

    static geometry_msgs::msg::Pose toRosPose(const Eigen::Matrix<Scalar, 3, 1>& pos,
                                              const Eigen::Matrix<Scalar, 3, 3>& rot) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = pos.x();
        pose.position.y = pos.y();
        pose.position.z = pos.z();

        Eigen::Quaternion<Scalar> q(rot);
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();

        return pose;
    }
};

} // namespace arena

#endif // ARENA_OBSTACLE_VISUALIZER_HPP
