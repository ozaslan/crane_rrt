#ifndef __GANTRY_VISUALIZER_HPP__
#define __GANTRY_VISUALIZER_HPP__

#include <memory>
#include <deque>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "crane_rrt/msg/gantry_crane_state.hpp"
#include "crane_rrt/crane_state.hpp"

namespace gantry {

class Visualizer {
public:
    using Self          = Visualizer;
    using Scalar        = double;
    using Vec3          = Eigen::Vector<Scalar, 3>;
    using Marker        = visualization_msgs::msg::Marker;
    using MarkerArray   = visualization_msgs::msg::MarkerArray;
    using Color         = std_msgs::msg::ColorRGBA;
    using Ptr           = std::shared_ptr<Self>;
    using ConstPtr      = std::shared_ptr<const Self>;
    using State         = CraneState<Scalar>;

    struct Parameters {
        std::string ns = "gantry";
        std::string frame_id = "world";
        int base_id = 1000;
        bool draw_trolley_velocity = false;
        bool draw_ball_velocity = false;
        bool draw_trolley_history = true;
        bool draw_ball_history = true;
        double trolley_size = 0.2;
        double ball_radius = 0.1;
        double rope_thickness = 0.02;
        double arrow_thickness = 0.04;
        double history_duration_limit_sec = 10.0;
        size_t history_count_limit = 500;
        double refresh_limit = 30.0;
        Color trolley_color;
        Color ball_color;
        Color rope_color;
        Color velocity_color;
        Color force_color;

        Parameters() {
            trolley_color = makeColor(0.0, 0.0, 1.0);
            ball_color = makeColor(1.0, 0.1, 0.1);
            rope_color = makeColor(0.0, 0.0, 0.0);
            velocity_color = makeColor(0.0, 1.0, 0.0);
            force_color = makeColor(1.0, 1.0, 0.0);
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

    explicit Visualizer(const Parameters& params = Parameters())
    : mParams(params) {}

    MarkerArray generateMarkers(
        const crane_rrt::msg::GantryCraneState& msg,
        const rclcpp::Time& stamp
    ) {
        State state;
        state.setPositionState(msg.position);
        state.setVelocityState(msg.velocity);

        Vec3 trolley = state.getTrolleyPosition();
        Vec3 ball = state.getBallPosition(msg.rope_length);
        Vec3 force;

        force(0) = msg.input[0];
        force(1) = msg.input[1];
        force(2) = 0.0;

        if (mParams.draw_trolley_history) {
            mTrolleyHistory.push_back(trolley);
            if (mTrolleyHistory.size() > mParams.history_count_limit)
                mTrolleyHistory.pop_front();
        } else {
            mTrolleyHistory.clear();
        }

        if (mParams.draw_ball_history) {
            mBallHistory.push_back(ball);
            if (mBallHistory.size() > mParams.history_count_limit)
                mBallHistory.pop_front();
        } else {
            mBallHistory.clear();
        }

        MarkerArray marker_array;
        int id = mParams.base_id;

        marker_array.markers.reserve(10);

        marker_array.markers.push_back(buildTrolleyMarker(trolley, stamp, id++));
        marker_array.markers.push_back(buildCrosshairMarkerX(trolley, stamp, id++));
        marker_array.markers.push_back(buildCrosshairMarkerY(trolley, stamp, id++));
        marker_array.markers.push_back(buildBallMarker(ball, stamp, id++));
        marker_array.markers.push_back(buildRopeMarker(trolley, ball, stamp, id++));
        marker_array.markers.push_back(buildForceMarker(trolley, force, stamp, id++));

        if (mParams.draw_trolley_velocity)
            marker_array.markers.push_back(buildVelocityMarker(trolley, state.getTrolleyVelocity(), stamp, id++));

        if (mParams.draw_ball_velocity)
            marker_array.markers.push_back(buildVelocityMarker(ball, state.getBallVelocity(msg.rope_length), stamp, id++));

        if (mParams.draw_trolley_history && mTrolleyHistory.size() >= 2)
            marker_array.markers.push_back(buildHistoryMarker(mTrolleyHistory, id++, mParams.trolley_color, stamp));

        if (mParams.draw_ball_history && mBallHistory.size() >= 2)
            marker_array.markers.push_back(buildHistoryMarker(mBallHistory, id++, mParams.ball_color, stamp));

        return marker_array;
    }

    Parameters getParameters() const {
        return this->mParams;
    }

    Self& setParameters(const Parameters& params) {
        mParams = params;
        return *this;
    }

    void clearHistory() {
        mTrolleyHistory.clear();
        mBallHistory.clear();
    }

private:
    Parameters mParams;
    std::deque<Vec3> mTrolleyHistory;
    std::deque<Vec3> mBallHistory;

    Marker buildTrolleyMarker(const Vec3& pos, const rclcpp::Time& stamp, int id) {
        Marker marker;
        marker.header.frame_id = mParams.frame_id;
        marker.header.stamp = stamp;
        marker.ns = mParams.ns;
        marker.id = id;
        marker.type = marker.CUBE;
        marker.action = marker.ADD;
        marker.pose.position.x = pos.x();
        marker.pose.position.y = pos.y();
        marker.pose.position.z = pos.z();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = mParams.trolley_size;
        marker.scale.y = mParams.trolley_size;
        marker.scale.z = mParams.trolley_size * 0.5;
        marker.color = mParams.trolley_color;
        return marker;
    }

    Marker buildCrosshairMarkerX(const Vec3& pos, const rclcpp::Time& stamp, int id) {
        auto marker = buildTrolleyMarker(pos, stamp, id);
        marker.scale.x = 0.6;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color = Parameters::makeColor(0.0, 0.0, 0.0);
        return marker;
    }

    Marker buildCrosshairMarkerY(const Vec3& pos, const rclcpp::Time& stamp, int id) {
        auto marker = buildTrolleyMarker(pos, stamp, id);
        marker.scale.x = 0.02;
        marker.scale.y = 0.6;
        marker.scale.z = 0.02;
        marker.color = Parameters::makeColor(0.0, 0.0, 0.0);
        return marker;
    }

    Marker buildBallMarker(const Vec3& pos, const rclcpp::Time& stamp, int id) {
        Marker marker;
        marker.header.frame_id = mParams.frame_id;
        marker.header.stamp = stamp;
        marker.ns = mParams.ns;
        marker.id = id;
        marker.type = marker.SPHERE;
        marker.action = marker.ADD;
        marker.pose.position.x = pos.x();
        marker.pose.position.y = pos.y();
        marker.pose.position.z = pos.z();
        marker.pose.orientation.w = 1.0;
        // RViz SPHERE scale represents diameter (2 * radius)
        marker.scale.x = marker.scale.y = marker.scale.z = 2.0 * mParams.ball_radius;
        marker.color = mParams.ball_color;
        return marker;
    }

    Marker buildRopeMarker(const Vec3& from, const Vec3& to, const rclcpp::Time& stamp, int id) {
        Marker marker;
        marker.header.frame_id = mParams.frame_id;
        marker.header.stamp = stamp;
        marker.ns = mParams.ns;
        marker.id = id;
        marker.type = marker.LINE_STRIP;
        marker.action = marker.ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = mParams.rope_thickness;
        marker.color = mParams.rope_color;
        geometry_msgs::msg::Point pt1, pt2;
        pt1.x = from.x(); pt1.y = from.y(); pt1.z = from.z();
        pt2.x = to.x();   pt2.y = to.y();   pt2.z = to.z();
        marker.points.push_back(pt1);
        marker.points.push_back(pt2);
        return marker;
    }

    Marker buildVelocityMarker(const Vec3& pos, const Vec3& vel, const rclcpp::Time& stamp, int id) {
        Marker marker;
        marker.header.frame_id = mParams.frame_id;
        marker.header.stamp = stamp;
        marker.ns = mParams.ns;
        marker.id = id;
        marker.type = marker.ARROW;
        marker.action = marker.ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = mParams.arrow_thickness;
        marker.scale.y = mParams.arrow_thickness * 1.5;
        marker.scale.z = mParams.arrow_thickness * 2.0;
        marker.color = mParams.velocity_color;
        geometry_msgs::msg::Point start, end;
        start.x = pos.x(); start.y = pos.y(); start.z = pos.z();
        if (vel.norm() < 1e-6) {
            marker.color.a = 0.0f;
            end.x = pos.x() + 1e-4; end.y = pos.y(); end.z = pos.z();
        } else {
            end.x = pos.x() + vel.x(); end.y = pos.y() + vel.y(); end.z = pos.z() + vel.z();
        }
        marker.points.push_back(start);
        marker.points.push_back(end);
        return marker;
    }

    Marker buildHistoryMarker(const std::deque<Vec3>& history, int id, const Color& color, const rclcpp::Time& stamp) {
        Marker marker;
        marker.header.frame_id = mParams.frame_id;
        marker.header.stamp = stamp;
        marker.ns = mParams.ns;
        marker.id = id;
        marker.type = marker.LINE_STRIP;
        marker.action = marker.ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.01;
        marker.color = color;
        marker.points.reserve(history.size());
        for (const auto& pt : history) {
            geometry_msgs::msg::Point p;
            p.x = pt.x(); p.y = pt.y(); p.z = pt.z();
            marker.points.push_back(p);
        }
        return marker;
    }

    Marker buildForceMarker(const Vec3& pos, const Vec3& force, const rclcpp::Time& stamp, int id) {
        Marker marker;
        marker.header.frame_id = mParams.frame_id;
        marker.header.stamp = stamp;
        marker.ns = mParams.ns;
        marker.id = id;
        marker.type = marker.ARROW;
        marker.action = marker.ADD;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = mParams.arrow_thickness;
        marker.scale.y = mParams.arrow_thickness * 1.5;
        marker.scale.z = mParams.arrow_thickness * 1.5;
        marker.color = mParams.force_color;

        geometry_msgs::msg::Point start, end;
        start.x = pos.x();
        start.y = pos.y();
        start.z = pos.z();
        if (force.norm() < 1e-6) {
            marker.color.a = 0.0f;
            end.x = pos.x() + 1e-4;
            end.y = pos.y();
            end.z = pos.z();
        } else {
            end.x = pos.x() + force.x();
            end.y = pos.y() + force.y();
            end.z = pos.z() + force.z();
        }

        marker.points.push_back(start);
        marker.points.push_back(end);

        return marker;
    }
};

} // namespace gantry

#endif  // __GANTRY_VISUALIZER_HPP__
