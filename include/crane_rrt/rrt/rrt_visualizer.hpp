#ifndef GANTRY_RRT_VISUALIZER_HPP
#define GANTRY_RRT_VISUALIZER_HPP

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "crane_rrt/rrt/rrt_planner.hpp"

namespace gantry {
namespace rrt {

template <typename ScalarT = double>
class RRTVisualizer {
public:
    using Scalar      = ScalarT;
    using State       = CraneState<Scalar>;
    using NodeData    = ::rrt::CraneRRTData<Scalar>;
    using Tree        = ::rrt::RRTTree<Scalar, NodeData>;
    using Node        = typename Tree::Node;
    using NodePtr     = typename Tree::NodePtr;
    using Marker      = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Color       = std_msgs::msg::ColorRGBA;
    using Vec2        = Eigen::Matrix<Scalar, 2, 1>;
    using Vec3        = Eigen::Matrix<Scalar, 3, 1>;

    struct Parameters {
        std::string frame_id = "map";
        std::string ns = "rrt";
        int base_id = 5000;
        double rope_length = 1.0;
        double goal_tolerance = 0.35;
        double tree_line_width = 0.008;
        double path_line_width = 0.035;
        bool draw_payload_tree = true;
        bool draw_soft_corridor = false;
        double corridor_half_width = 1.25;
        Color tree_color;
        Color payload_tree_color;
        Color trolley_path_color;
        Color payload_path_color;
        Color start_color;
        Color goal_color;
        Color corridor_color;

        Parameters() {
            tree_color         = makeColor(0.2, 0.6, 1.0, 0.45);
            payload_tree_color = makeColor(1.0, 0.5, 0.2, 0.30);
            trolley_path_color = makeColor(0.0, 0.2, 1.0, 1.0);
            payload_path_color = makeColor(0.95, 0.1, 0.1, 1.0);
            start_color        = makeColor(0.1, 0.9, 0.2, 0.85);
            goal_color         = makeColor(1.0, 0.85, 0.0, 0.65);
            corridor_color     = makeColor(0.45, 0.15, 0.75, 0.75);
        }

        static Color makeColor(double r, double g, double b, double a = 1.0) {
            Color c;
            c.r = static_cast<float>(r);
            c.g = static_cast<float>(g);
            c.b = static_cast<float>(b);
            c.a = static_cast<float>(a);
            return c;
        }
    };

    explicit RRTVisualizer(const Parameters& params = Parameters())
        : mParams(params) {}

    void setParameters(const Parameters& params) { mParams = params; }
    const Parameters& getParameters() const { return mParams; }

    MarkerArray generateMarkers(
        const Tree& tree,
        const State& start_state,
        const State& goal_state,
        const std::vector<State>& planned_trajectory,
        const rclcpp::Time& stamp
    ) const {
        MarkerArray arr;
        int id = mParams.base_id;

        arr.markers.push_back(buildEndpointMarker(start_state, mParams.start_color, 0.22, stamp, id++, "start"));
        arr.markers.push_back(buildEndpointMarker(goal_state, mParams.goal_color, 2.0 * mParams.goal_tolerance, stamp, id++, "goal"));

        if (mParams.draw_soft_corridor) {
            arr.markers.push_back(buildCorridorMarker(start_state, goal_state, stamp, id++));
        }

        if (tree.size() >= 2) {
            arr.markers.push_back(buildTreeMarker(tree, false, mParams.tree_color, stamp, id++));
            if (mParams.draw_payload_tree) {
                arr.markers.push_back(buildTreeMarker(tree, true, mParams.payload_tree_color, stamp, id++));
            }
        }

        if (planned_trajectory.size() >= 2) {
            arr.markers.push_back(buildTrajectoryMarker(
                planned_trajectory, false, mParams.trolley_path_color, stamp, id++));
            arr.markers.push_back(buildTrajectoryMarker(
                planned_trajectory, true, mParams.payload_path_color, stamp, id++));
        }

        return arr;
    }

private:
    Parameters mParams;

    static geometry_msgs::msg::Point toPoint(const Vec3& v) {
        geometry_msgs::msg::Point p;
        p.x = static_cast<double>(v.x());
        p.y = static_cast<double>(v.y());
        p.z = static_cast<double>(v.z());
        return p;
    }

    Marker buildEndpointMarker(
        const State& state,
        const Color& color,
        double diameter,
        const rclcpp::Time& stamp,
        int id,
        const std::string& sub_ns
    ) const {
        Marker m;
        m.header.frame_id = mParams.frame_id;
        m.header.stamp = stamp;
        m.ns = mParams.ns + "_" + sub_ns;
        m.id = id;
        m.type = Marker::SPHERE;
        m.action = Marker::ADD;

        const Vec3 pos = state.getTrolleyPosition();
        m.pose.position = toPoint(pos);
        m.pose.orientation.w = 1.0;
        m.scale.x = diameter;
        m.scale.y = diameter;
        m.scale.z = diameter;
        m.color = color;
        return m;
    }

    Marker buildCorridorMarker(
        const State& start_state,
        const State& goal_state,
        const rclcpp::Time& stamp,
        int id
    ) const {
        Marker m;
        m.header.frame_id = mParams.frame_id;
        m.header.stamp = stamp;
        m.ns = mParams.ns + "_soft_corridor";
        m.id = id;
        m.type = Marker::LINE_LIST;
        m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.025;
        m.color = mParams.corridor_color;

        const Vec2 ps = start_state.getTrolleyPosition2D();
        const Vec2 pg = goal_state.getTrolleyPosition2D();
        const Vec2 chord = pg - ps;
        const Scalar len = chord.norm();
        if (len > Scalar(1e-6)) {
            const Vec2 e_par = chord / len;
            const Vec2 e_perp(-e_par.y(), e_par.x());
            const Scalar w = static_cast<Scalar>(mParams.corridor_half_width);
            for (Scalar sgn : {Scalar(-1), Scalar(1)}) {
                const Vec2 a = ps - Scalar(0.25) * e_par + sgn * w * e_perp;
                const Vec2 b = pg + Scalar(0.25) * e_par + sgn * w * e_perp;
                m.points.push_back(toPoint(Vec3(a.x(), a.y(), Scalar(0))));
                m.points.push_back(toPoint(Vec3(b.x(), b.y(), Scalar(0))));
            }
        }
        return m;
    }

    Marker buildTreeMarker(
        const Tree& tree,
        bool payload_space,
        const Color& color,
        const rclcpp::Time& stamp,
        int id
    ) const {
        Marker m;
        m.header.frame_id = mParams.frame_id;
        m.header.stamp = stamp;
        m.ns = payload_space ? (mParams.ns + "_payload_tree") : (mParams.ns + "_trolley_tree");
        m.id = id;
        m.type = Marker::LINE_LIST;
        m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = mParams.tree_line_width;
        m.color = color;

        const auto& nodes = tree.getNodes();
        m.points.reserve(nodes.size() * 2);

        const Scalar ell = static_cast<Scalar>(mParams.rope_length);
        for (const auto& node : nodes) {
            if (!node) continue;
            if (!node->getData().sst_active && node->getData().sst_children == 0) continue;
            auto parent = node->getParent();
            if (!parent) continue;

            const State& s0 = parent->getData().state;
            const State& s1 = node->getData().state;
            const Vec3 p0 = payload_space ? s0.getBallPosition(ell) : s0.getTrolleyPosition();
            const Vec3 p1 = payload_space ? s1.getBallPosition(ell) : s1.getTrolleyPosition();

            m.points.push_back(toPoint(p0));
            m.points.push_back(toPoint(p1));
        }

        return m;
    }

    Marker buildTrajectoryMarker(
        const std::vector<State>& traj,
        bool payload_space,
        const Color& color,
        const rclcpp::Time& stamp,
        int id
    ) const {
        Marker m;
        m.header.frame_id = mParams.frame_id;
        m.header.stamp = stamp;
        m.ns = payload_space ? (mParams.ns + "_payload_path") : (mParams.ns + "_trolley_path");
        m.id = id;
        m.type = Marker::LINE_STRIP;
        m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = mParams.path_line_width;
        m.color = color;
        m.points.reserve(traj.size());

        const Scalar ell = static_cast<Scalar>(mParams.rope_length);
        for (const auto& s : traj) {
            const Vec3 pt = payload_space ? s.getBallPosition(ell) : s.getTrolleyPosition();
            m.points.push_back(toPoint(pt));
        }
        return m;
    }
};

} // namespace rrt
} // namespace gantry

#endif // GANTRY_RRT_VISUALIZER_HPP
