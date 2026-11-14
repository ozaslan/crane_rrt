#ifndef ARENA_AABB_VISUALIZER_HPP
#define ARENA_AABB_VISUALIZER_HPP

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/time.hpp>
#include <Eigen/Dense>

#include "color.hpp"

namespace arena {

template<typename Scalar>
class AABBVisualizer {
public:
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Marker = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using ColorT = Color<Scalar>;

    struct Params {
        std::string frame_id = "map";
        std::string ns = "aabb";
        rclcpp::Time stamp;
        double lifetime = 0.0;
        float line_width = 0.01;
        float text_font_size = 0.07;
        ColorT color = ColorT::White(1.0);  // Default to white
    };

    static MarkerArray toMarker(const Vec3& min,
                            const Vec3& max,
                            const Params& params,
                            int id = 0,
                            const std::string& label = "") {
        MarkerArray array;

        // ----- AABB Marker -----
        Marker marker;
        marker.header.frame_id = params.frame_id;
        marker.header.stamp = params.stamp;
        marker.ns = params.ns;
        marker.id = id;
        marker.type = Marker::LINE_LIST;
        marker.action = Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = rclcpp::Duration::from_seconds(params.lifetime);
        marker.scale.x = params.line_width;

        auto c = params.color;
        c.clamp();
        marker.color.r = c.r;
        marker.color.g = c.g;
        marker.color.b = c.b;
        marker.color.a = c.a;

        Vec3 c8[8];
        c8[0] = {min.x(), min.y(), min.z()};
        c8[1] = {max.x(), min.y(), min.z()};
        c8[2] = {max.x(), max.y(), min.z()};
        c8[3] = {min.x(), max.y(), min.z()};
        c8[4] = {min.x(), min.y(), max.z()};
        c8[5] = {max.x(), min.y(), max.z()};
        c8[6] = {max.x(), max.y(), max.z()};
        c8[7] = {min.x(), max.y(), max.z()};

        int edges[12][2] = {
            {0, 1}, {1, 2}, {2, 3}, {3, 0},
            {4, 5}, {5, 6}, {6, 7}, {7, 4},
            {0, 4}, {1, 5}, {2, 6}, {3, 7}
        };

        for (auto& e : edges) {
            marker.points.push_back(toPoint(c8[e[0]]));
            marker.points.push_back(toPoint(c8[e[1]]));
        }

        array.markers.push_back(marker);

        // ----- Optional Text Marker -----
        if (!label.empty()) {
            Marker text;
            text.header.frame_id = params.frame_id;
            text.header.stamp = params.stamp;
            text.ns = params.ns + "_label";
            text.id = id + 1;
            text.type = Marker::TEXT_VIEW_FACING;
            text.action = Marker::ADD;
            text.lifetime = rclcpp::Duration::from_seconds(params.lifetime);

            text.pose.position = toPoint(c8[7]);
            text.pose.position.z += 0.1;  // raise above corner
            text.pose.orientation.w = 1.0;

            text.scale.z = params.text_font_size;

            text.color.r = c.r;
            text.color.g = c.g;
            text.color.b = c.b;
            text.color.a = 1.0;

            text.text = label;

            array.markers.push_back(text);
        }

        return array;
    }


    template<typename ObjectWithAABB>
    static MarkerArray visualizeAABB(const ObjectWithAABB& obj,
                                     const Params& params,
                                     int id = 0,
                                     const std::string& label = "") {
        auto aabb = obj.getAABB(); // expects min() and max()
        MarkerArray array;
        if (!aabb.isValid()) {
            return array;
        }
        MarkerArray box = toMarker(aabb.min(), aabb.max(), params, id, label);
        array.markers.insert(array.markers.end(), box.markers.begin(), box.markers.end());
        return array;
    }


private:
    static geometry_msgs::msg::Point toPoint(const Vec3& v) {
        geometry_msgs::msg::Point p;
        p.x = v.x(); p.y = v.y(); p.z = v.z();
        return p;
    }
};

} // namespace arena

#endif // ARENA_AABB_VISUALIZER_HPP
