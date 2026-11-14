#ifndef ARENA_VISUALIZATION_NODE_HPP
#define ARENA_VISUALIZATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <memory>
#include <sstream>
#include <string>

#include "crane_rrt/arena/arena.hpp"
#include "crane_rrt/arena/arena_visualizer.hpp"
#include "crane_rrt/arena/ecc_arenas.hpp"

namespace arena {

template <typename Scalar>
class ArenaVisualizationNode : public rclcpp::Node {
public:
    using ArenaType   = Arena<Scalar>;
    using ArenaPtr    = std::shared_ptr<ArenaType>;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Visualizer  = ArenaVisualizer<Scalar>;
    using VisParams   = typename PrimitiveVisualizer<Scalar>::Params;

    explicit ArenaVisualizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("arena_visualizer", options)
    {
        const std::string path = this->get_parameter_or("arena_file", std::string{"Arena1_ImageIndustrial"});
        mShowAABB = this->get_parameter_or("show_aabb", false);

        mPub = this->create_publisher<MarkerArray>("/arena/visualization", 10);

        mCmdSub = this->create_subscription<std_msgs::msg::String>(
            "/gantry_crane/rrt_command", 10,
            std::bind(&ArenaVisualizationNode::commandCallback, this, std::placeholders::_1)
        );

        loadArenaSpec(path);

        mTimer = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ArenaVisualizationNode::onTimer, this)
        );

        RCLCPP_INFO(get_logger(), "Arena visualization node initialized.");
    }

private:
    void clearMarkers() {
        if (!mPub) return;
        MarkerArray del;
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.action = visualization_msgs::msg::Marker::DELETEALL;
        del.markers.push_back(m);
        mPub->publish(del);
    }

    void loadArenaSpec(const std::string& arenaSpec) {
        if (arenaSpec == mCurrentArenaSpec && (arenaSpec.empty() || mArena != nullptr)) {
            return;
        }
        mCurrentArenaSpec = arenaSpec;
        clearMarkers();

        if (arenaSpec.empty() || arenaSpec == "none") {
            mArena = nullptr;
            RCLCPP_INFO(get_logger(), "Arena cleared (obstacle-free mode).");
            return;
        }

        try {
            mArena = loadArenaByNameOrPath<Scalar>(arenaSpec);
            RCLCPP_INFO(get_logger(), "Loaded arena for visualization: %s", arenaSpec.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to load arena '%s': %s", arenaSpec.c_str(), e.what());
            mArena = nullptr;
        }
    }

    void commandCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::istringstream iss(msg->data);
        std::string line;
        while (std::getline(iss, line)) {
            if (line.empty()) continue;
            auto pos = line.find('=');
            if (pos == std::string::npos) continue;
            std::string key = line.substr(0, pos);
            std::string val = line.substr(pos + 1);

            if (key == "arena_file") {
                loadArenaSpec(val);
            } else if (key == "arena.show_aabb") {
                bool newShow = (val == "1" || val == "true");
                if (newShow != mShowAABB) {
                    mShowAABB = newShow;
                    clearMarkers();
                }
            }
        }
    }

    void onTimer() {
        if (!mArena) return;

        VisParams params;
        params.frame_id = "map";
        params.ns = "arena";
        params.stamp = this->now();
        params.lifetime = 0.0;

        int id_start = 1000;
        auto msg = Visualizer::toMarkers(*mArena, params, id_start, mShowAABB);
        mPub->publish(msg);
    }

    rclcpp::Publisher<MarkerArray>::SharedPtr mPub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mCmdSub;
    rclcpp::TimerBase::SharedPtr mTimer;
    ArenaPtr mArena;
    std::string mCurrentArenaSpec;
    bool mShowAABB = false;
};

} // namespace arena

#endif // ARENA_VISUALIZATION_NODE_HPP

