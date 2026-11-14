#ifndef __GANTRY_VISUALIZER_NODE_HPP__
#define __GANTRY_VISUALIZER_NODE_HPP__

#include <iostream>
#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "crane_rrt/visualizer.hpp"
#include "crane_rrt/msg/gantry_crane_state.hpp"
#include "crane_rrt/msg/collision_status.hpp"

namespace gantry {

class VisualizerNode : public rclcpp::Node {
public:
    VisualizerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("gantry_visualizer_node", options)
    {
        Visualizer::Parameters params = loadParameters();

        mVisualizer = std::make_unique<Visualizer>(params);

        mSub = this->create_subscription<crane_rrt::msg::GantryCraneState>(
            "/gantry_crane/state", 10,
            std::bind(&VisualizerNode::stateCallback, this, std::placeholders::_1)
        );

        mColSub = this->create_subscription<crane_rrt::msg::CollisionStatus>(
            "/gantry_crane/collision_status", 10,
            std::bind(&VisualizerNode::collisionCallback, this, std::placeholders::_1)
        );

        mCmdSub = this->create_subscription<std_msgs::msg::String>(
            "/gantry_crane/rrt_command", 10,
            std::bind(&VisualizerNode::commandCallback, this, std::placeholders::_1)
        );

        mPub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/gantry_crane/visualization", 10
        );

        mLastUpdateTime = this->now();
        mColStat.in_collision = false;

        RCLCPP_INFO(this->get_logger(), "VisualizerNode initialized.");
    }

private:
    rclcpp::Time mLastUpdateTime;
    Visualizer::Ptr mVisualizer;
    rclcpp::Subscription<crane_rrt::msg::GantryCraneState>::SharedPtr mSub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mPub;
    rclcpp::Subscription<crane_rrt::msg::CollisionStatus>::SharedPtr mColSub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mCmdSub;

    crane_rrt::msg::CollisionStatus mColStat;

    void collisionCallback(const crane_rrt::msg::CollisionStatus::SharedPtr msg) {
        mColStat = *msg;
    }

    void commandCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::istringstream iss(msg->data);
        std::string line;
        std::string cmd;
        auto params = mVisualizer->getParameters();
        bool params_changed = false;

        while (std::getline(iss, line)) {
            if (line.empty()) continue;
            auto pos = line.find('=');
            if (pos == std::string::npos) continue;
            std::string key = line.substr(0, pos);
            std::string val = line.substr(pos + 1);

            try {
                if (key == "command") {
                    cmd = val;
                } else if (key == "ball_radius") {
                    params.ball_radius = std::stod(val);
                    params_changed = true;
                } else if (key == "visualizer.trolley_size") {
                    params.trolley_size = std::stod(val);
                    params_changed = true;
                } else if (key == "visualizer.rope_thickness") {
                    params.rope_thickness = std::stod(val);
                    params_changed = true;
                } else if (key == "visualizer.arrow_thickness") {
                    params.arrow_thickness = std::stod(val);
                    params_changed = true;
                } else if (key == "visualizer.history_count_limit") {
                    params.history_count_limit = static_cast<size_t>(std::max(2, std::stoi(val)));
                    params_changed = true;
                } else if (key == "visualizer.draw_trolley_velocity") {
                    params.draw_trolley_velocity = (val == "1" || val == "true");
                    params_changed = true;
                } else if (key == "visualizer.draw_ball_velocity") {
                    params.draw_ball_velocity = (val == "1" || val == "true");
                    params_changed = true;
                } else if (key == "visualizer.draw_trolley_history") {
                    params.draw_trolley_history = (val == "1" || val == "true");
                    params_changed = true;
                } else if (key == "visualizer.draw_ball_history") {
                    params.draw_ball_history = (val == "1" || val == "true");
                    params_changed = true;
                }
            } catch (const std::exception&) {
            }
        }

        if (params_changed) {
            mVisualizer->setParameters(params);
        }

        if (cmd == "reset" || cmd == "apply_and_reset" ||
            cmd == "clear_history" || cmd == "restart_playback") {
            mVisualizer->clearHistory();
            visualization_msgs::msg::MarkerArray del;
            visualization_msgs::msg::Marker m;
            m.header.frame_id = params.frame_id;
            m.header.stamp = this->now();
            m.action = visualization_msgs::msg::Marker::DELETEALL;
            del.markers.push_back(m);
            mPub->publish(del);
        }
    }

    void stateCallback(const crane_rrt::msg::GantryCraneState::SharedPtr msg) {
        rclcpp::Time now = this->get_clock()->now();

        double refresh_limit = mVisualizer->getParameters().refresh_limit;
        rclcpp::Duration threshold_duration = rclcpp::Duration::from_seconds(1.0 / refresh_limit);

        if ((now - mLastUpdateTime) >= threshold_duration) {
            auto params = mVisualizer->getParameters();
            if (mColStat.in_collision) {
                params.ball_color = Visualizer::Parameters::makeColor(1, 0, 0);
            } else {
                params.ball_color = Visualizer::Parameters::makeColor(0, 1, 0);
            }
            mVisualizer->setParameters(params);

            auto markers = mVisualizer->generateMarkers(*msg, now);
            mPub->publish(markers);
            mLastUpdateTime = now;
        }
    }

    Visualizer::Parameters loadParameters(const Visualizer::Parameters& defaults = Visualizer::Parameters{}) {
        Visualizer::Parameters p = defaults;

        p.ns                         = this->get_parameter_or("ns",                         p.ns);
        p.frame_id                   = this->get_parameter_or("frame_id",                   p.frame_id);
        p.base_id                    = this->get_parameter_or("base_id",                    p.base_id);
        p.draw_trolley_velocity      = this->get_parameter_or("draw_trolley_velocity",      p.draw_trolley_velocity);
        p.draw_ball_velocity         = this->get_parameter_or("draw_ball_velocity",         p.draw_ball_velocity);
        p.draw_trolley_history       = this->get_parameter_or("draw_trolley_history",       p.draw_trolley_history);
        p.draw_ball_history          = this->get_parameter_or("draw_ball_history",          p.draw_ball_history);
        p.trolley_size               = this->get_parameter_or("trolley_size",               p.trolley_size);
        p.ball_radius                = this->get_parameter_or("ball_radius",                p.ball_radius);
        p.rope_thickness             = this->get_parameter_or("rope_thickness",             p.rope_thickness);
        p.arrow_thickness            = this->get_parameter_or("arrow_thickness",            p.arrow_thickness);
        p.history_duration_limit_sec = this->get_parameter_or("history_duration_limit_sec", p.history_duration_limit_sec);
        p.history_count_limit        = static_cast<size_t>(this->get_parameter_or("history_count_limit", static_cast<int>(p.history_count_limit)));
        p.refresh_limit              = this->get_parameter_or("refresh_limit",              p.refresh_limit);

        return p;
    }
};

} // namespace gantry

#endif // __GANTRY_VISUALIZER_NODE_HPP__
