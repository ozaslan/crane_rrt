#ifndef GANTRY_RRT_EXPLORATION_NODE_HPP
#define GANTRY_RRT_EXPLORATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <algorithm>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "crane_rrt/arena/ecc_arenas.hpp"
#include "crane_rrt/msg/collision_status.hpp"
#include "crane_rrt/msg/gantry_crane_state.hpp"
#include "crane_rrt/rrt/rrt_planner.hpp"
#include "crane_rrt/rrt/rrt_visualizer.hpp"

namespace gantry {

class RRTExplorationNode : public rclcpp::Node {
public:
    using Scalar       = double;
    using ArenaScalar  = float;
    using Planner      = rrt::RRTPlanner<Scalar, ArenaScalar>;
    using PlannerPtr   = std::shared_ptr<Planner>;
    using PlannerParam = typename Planner::Parameters;
    using Model        = typename Planner::Model;
    using ModelParams  = typename Model::Parameters;
    using Solver       = typename Planner::Solver;
    using SolverParams = typename Solver::Parameters;
    using State        = typename Planner::State;
    using Input        = typename Planner::Input;
    using Arena        = arena::Arena<ArenaScalar>;
    using ArenaPtr     = typename Arena::Ptr;
    using Visualizer   = rrt::RRTVisualizer<Scalar>;
    using MarkerArray  = visualization_msgs::msg::MarkerArray;
    using StateMsg     = crane_rrt::msg::GantryCraneState;
    using CollisionMsg = crane_rrt::msg::CollisionStatus;

    explicit RRTExplorationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("rrt_exploration_node", options)
    {
        ModelParams  modelParams  = loadModelParameters();
        SolverParams solverParams = loadSolverParameters();
        PlannerParam rrtParams    = loadRRTParameters();

        auto model  = std::make_shared<Model>();
        model->setParameters(modelParams);
        auto solver = std::make_shared<Solver>(model);
        solver->setParameters(solverParams);

        mCurrentArenaFile = this->get_parameter_or("arena_file", std::string{"Arena1_ImageIndustrial"});
        ArenaPtr arena = loadArena(mCurrentArenaFile);

        std::vector<double> defaultStart = {-3.8,  3.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> defaultGoal  = { 3.8, -3.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        const bool autoArenaPreset = this->get_parameter_or("rrt.auto_arena_preset", true);
        if (autoArenaPreset) {
            if (mCurrentArenaFile == "Arena2_SlalomCorridor" ||
                mCurrentArenaFile == "Arena4_PayloadClutterCorridor") {
                defaultStart = {-3.8, -3.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                defaultGoal  = { 3.8,  3.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            }
            if (mCurrentArenaFile == "Arena4_PayloadClutterCorridor") {
                rrtParams.enable_soft_corridor = true;
            }
        }

        State startState = autoArenaPreset &&
                (mCurrentArenaFile == "Arena2_SlalomCorridor" ||
                 mCurrentArenaFile == "Arena4_PayloadClutterCorridor")
            ? State::FromVector(defaultStart)
            : loadStateParam("rrt.start_state", defaultStart);
        State goalState = autoArenaPreset &&
                (mCurrentArenaFile == "Arena2_SlalomCorridor" ||
                 mCurrentArenaFile == "Arena4_PayloadClutterCorridor")
            ? State::FromVector(defaultGoal)
            : loadStateParam("rrt.goal_state", defaultGoal);

        mPlanner = std::make_shared<Planner>(model, solver, arena, rrtParams);

        typename Visualizer::Parameters visParams;
        visParams.frame_id            = this->get_parameter_or("rrt.frame_id", std::string{"map"});
        visParams.rope_length         = modelParams.ell;
        visParams.goal_tolerance      = rrtParams.goal_pos_tolerance;
        visParams.draw_payload_tree   = this->get_parameter_or("rrt.draw_payload_tree", true);
        visParams.draw_soft_corridor  = rrtParams.enable_soft_corridor;
        visParams.corridor_half_width = rrtParams.corridor_half_width;
        mVisualizer.setParameters(visParams);

        mStartupDelaySec = this->get_parameter_or("rrt.startup_delay_sec", 2.0);
        mBatchIterations = static_cast<std::size_t>(
            this->get_parameter_or("rrt.batch_iterations", 20));
        mLoopPlayback    = this->get_parameter_or("rrt.loop_playback", true);

        if (!mPlanner->initialize(startState, goalState)) {
            RCLCPP_WARN(get_logger(),
                "Start state is in collision or out of bounds! Check rrt.start_state and arena obstacles.");
        }

        mStatePub     = create_publisher<StateMsg>("/gantry_crane/state", 10);
        mCollisionPub = create_publisher<CollisionMsg>("/gantry_crane/collision_status", 10);
        mRRTMarkerPub = create_publisher<MarkerArray>("/gantry_crane/rrt_markers", 10);
        mStatusPub    = create_publisher<std_msgs::msg::String>("/gantry_crane/rrt_status", 10);

        mCmdSub = create_subscription<std_msgs::msg::String>(
            "/gantry_crane/rrt_command", 10,
            std::bind(&RRTExplorationNode::commandCallback, this, std::placeholders::_1)
        );

        mStartTime = get_clock()->now();
        recreateTimer(rrtParams.integration_dt);

        RCLCPP_INFO(get_logger(), "RRTExplorationNode initialized.\n%s", rrtParams.toString().c_str());
    }

private:
    PlannerPtr mPlanner;
    Visualizer mVisualizer;
    std::string mCurrentArenaFile;

    rclcpp::Time mStartTime;
    double mStartupDelaySec = 2.0;
    std::size_t mBatchIterations = 20;
    std::size_t mTotalIterations = 0;
    bool mPlanningFinished = false;
    bool mPaused = false;
    bool mLoopPlayback = true;

    std::vector<State> mPlannedStates;
    std::vector<Input> mPlannedInputs;
    std::size_t mPlaybackIndex = 0;
    std::size_t mMarkerTick = 0;
    State mLastPublishedState = State::Zero();
    Input mLastPublishedInput = Input::Zero();

    rclcpp::Publisher<StateMsg>::SharedPtr mStatePub;
    rclcpp::Publisher<CollisionMsg>::SharedPtr mCollisionPub;
    rclcpp::Publisher<MarkerArray>::SharedPtr mRRTMarkerPub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mStatusPub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mCmdSub;
    rclcpp::TimerBase::SharedPtr mTimer;

    void recreateTimer(double dt) {
        if (mTimer) {
            mTimer->cancel();
        }
        const double safe_dt = std::clamp(dt, 0.002, 0.2);
        mTimer = create_wall_timer(
            std::chrono::duration<double>(safe_dt),
            std::bind(&RRTExplorationNode::timerCallback, this)
        );
    }

    void clearRRTMarkersInRViz() {
        MarkerArray del;
        visualization_msgs::msg::Marker m;
        m.header.frame_id = mVisualizer.getParameters().frame_id;
        m.header.stamp = get_clock()->now();
        m.action = visualization_msgs::msg::Marker::DELETEALL;
        del.markers.push_back(m);
        mRRTMarkerPub->publish(del);
    }

    void resetExploration(const State& startState, const State& goalState) {
        mStartupDelaySec = 0.0;
        mTotalIterations = 0;
        mPlanningFinished = false;
        mPaused = false;
        mPlaybackIndex = 0;
        mPlannedStates.clear();
        mPlannedInputs.clear();

        clearRRTMarkersInRViz();

        if (!mPlanner->initialize(startState, goalState)) {
            RCLCPP_WARN(get_logger(),
                "Start state (%.2f, %.2f) is in collision or out of bounds!",
                startState.getX(), startState.getY());
        } else {
            RCLCPP_INFO(get_logger(),
                "RRT reset! Exploring from (%.2f, %.2f) to (%.2f, %.2f) in '%s' with seed %u...",
                startState.getX(), startState.getY(),
                goalState.getX(), goalState.getY(),
                mCurrentArenaFile.c_str(),
                mPlanner->getParameters().random_seed);
        }

        const auto now = get_clock()->now();
        publishCraneState(mPlanner->getStartState(), Input::Zero(), now);
        publishRRTMarkers(now);
        publishStatus();
    }

    void commandCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::istringstream iss(msg->data);
        std::string line;
        std::string cmd;

        PlannerParam rrtParams    = mPlanner->getParameters();
        ModelParams  modelParams  = mPlanner->getModel()->getParameters();
        SolverParams solverParams = mPlanner->getSolver()->getParameters();
        auto         visParams    = mVisualizer.getParameters();

        State startState = mPlanner->getStartState();
        State goalState  = mPlanner->getGoalState();
        double start_x = startState.getX();
        double start_y = startState.getY();
        double goal_x  = goalState.getX();
        double goal_y  = goalState.getY();
        std::string newArenaFile = mCurrentArenaFile;

        bool model_changed  = false;
        bool solver_changed = false;
        bool vis_changed    = false;
        bool dt_changed     = false;

        auto toBool = [](const std::string& v) {
            return (v == "1" || v == "true" || v == "True" || v == "TRUE");
        };

        while (std::getline(iss, line)) {
            if (line.empty()) continue;
            auto pos = line.find('=');
            if (pos == std::string::npos) continue;
            std::string key = line.substr(0, pos);
            std::string val = line.substr(pos + 1);

            try {
                if (key == "command") {
                    cmd = val;
                } else if (key == "rrt.start_x") {
                    start_x = std::stod(val);
                } else if (key == "rrt.start_y") {
                    start_y = std::stod(val);
                } else if (key == "rrt.goal_x") {
                    goal_x = std::stod(val);
                } else if (key == "rrt.goal_y") {
                    goal_y = std::stod(val);
                } else if (key == "rrt.x_min") {
                    rrtParams.x_min = std::stod(val);
                } else if (key == "rrt.x_max") {
                    rrtParams.x_max = std::stod(val);
                } else if (key == "rrt.y_min") {
                    rrtParams.y_min = std::stod(val);
                } else if (key == "rrt.y_max") {
                    rrtParams.y_max = std::stod(val);
                } else if (key == "rrt.max_velocity") {
                    rrtParams.max_velocity = std::stod(val);
                } else if (key == "rrt.max_swing_angle") {
                    rrtParams.max_swing_angle = std::stod(val);
                } else if (key == "rrt.max_force") {
                    rrtParams.max_force = std::stod(val);
                } else if (key == "rrt.min_duration") {
                    rrtParams.min_duration = std::stod(val);
                } else if (key == "rrt.max_duration") {
                    rrtParams.max_duration = std::stod(val);
                } else if (key == "rrt.enable_force_smoothing") {
                    rrtParams.enable_force_smoothing = toBool(val);
                } else if (key == "rrt.force_direction_bias") {
                    rrtParams.force_direction_bias = std::stod(val);
                } else if (key == "rrt.w_pos") {
                    rrtParams.w_pos = std::stod(val);
                } else if (key == "rrt.w_vel") {
                    rrtParams.w_vel = std::stod(val);
                } else if (key == "rrt.goal_bias") {
                    rrtParams.goal_bias = std::stod(val);
                } else if (key == "rrt.min_search_step") {
                    rrtParams.min_search_step = static_cast<std::size_t>(std::max(1, std::stoi(val)));
                } else if (key == "rrt.max_search_step") {
                    rrtParams.max_search_step = static_cast<std::size_t>(std::max(1, std::stoi(val)));
                } else if (key == "rrt.use_equal_budget_random_subset") {
                    rrtParams.use_equal_budget_random_subset = toBool(val);
                } else if (key == "rrt.force_history_nns_weight") {
                    rrtParams.force_history_nns_weight = std::stod(val);
                } else if (key == "rrt.use_sst_planner") {
                    rrtParams.use_sst_planner = toBool(val);
                } else if (key == "rrt.sst_delta_bn") {
                    rrtParams.sst_delta_bn = std::stod(val);
                } else if (key == "rrt.sst_delta_s") {
                    rrtParams.sst_delta_s = std::stod(val);
                } else if (key == "rrt.num_control_samples") {
                    rrtParams.num_control_samples = static_cast<std::size_t>(std::max(1, std::stoi(val)));
                } else if (key == "rrt.use_cell_partition_nns") {
                    rrtParams.use_cell_partition_nns = toBool(val);
                } else if (key == "rrt.cell_grid_radix") {
                    rrtParams.cell_grid_radix = static_cast<std::size_t>(std::max(2, std::stoi(val)));
                } else if (key == "rrt.use_pd_local_steering") {
                    rrtParams.use_pd_local_steering = toBool(val);
                } else if (key == "rrt.enable_near_goal_braking") {
                    rrtParams.enable_near_goal_braking = toBool(val);
                } else if (key == "rrt.near_goal_radius") {
                    rrtParams.near_goal_radius = std::stod(val);
                } else if (key == "rrt.near_goal_zero_force_prob") {
                    rrtParams.near_goal_zero_force_prob = std::stod(val);
                } else if (key == "rrt.enable_soft_corridor") {
                    rrtParams.enable_soft_corridor = toBool(val);
                    visParams.draw_soft_corridor = rrtParams.enable_soft_corridor;
                    vis_changed = true;
                } else if (key == "rrt.corridor_half_width") {
                    rrtParams.corridor_half_width = std::stod(val);
                    visParams.corridor_half_width = rrtParams.corridor_half_width;
                    vis_changed = true;
                } else if (key == "rrt.corridor_penalty_weight") {
                    rrtParams.corridor_penalty_weight = std::stod(val);
                } else if (key == "rrt.corridor_sample_prob") {
                    rrtParams.corridor_sample_prob = std::stod(val);
                } else if (key == "rrt.goal_pos_tolerance") {
                    rrtParams.goal_pos_tolerance = std::stod(val);
                    visParams.goal_tolerance = rrtParams.goal_pos_tolerance;
                    vis_changed = true;
                } else if (key == "rrt.goal_vel_tolerance") {
                    rrtParams.goal_vel_tolerance = std::stod(val);
                } else if (key == "rrt.goal_swing_tolerance") {
                    rrtParams.goal_swing_tolerance = std::stod(val);
                } else if (key == "rrt.goal_omega_tolerance") {
                    rrtParams.goal_omega_tolerance = std::stod(val);
                } else if (key == "rrt.require_goal_velocity_check") {
                    rrtParams.require_goal_velocity_check = toBool(val);
                } else if (key == "rrt.require_goal_swing_check") {
                    rrtParams.require_goal_swing_check = toBool(val);
                } else if (key == "rrt.integration_dt") {
                    double new_dt = std::stod(val);
                    if (std::abs(new_dt - rrtParams.integration_dt) > 1e-6) {
                        rrtParams.integration_dt = new_dt;
                        dt_changed = true;
                    }
                } else if (key == "ball_radius") {
                    rrtParams.ball_radius = std::stod(val);
                } else if (key == "rrt.cable_radius") {
                    rrtParams.cable_radius = std::stod(val);
                } else if (key == "rrt.check_rope_collision") {
                    rrtParams.check_rope_collision = toBool(val);
                } else if (key == "rrt.rope_collision_samples") {
                    rrtParams.rope_collision_samples = static_cast<std::size_t>(std::max(1, std::stoi(val)));
                } else if (key == "rrt.max_iterations") {
                    rrtParams.max_iterations = static_cast<std::size_t>(std::max(10, std::stoi(val)));
                } else if (key == "rrt.max_nodes") {
                    rrtParams.max_nodes = static_cast<std::size_t>(std::max(10, std::stoi(val)));
                } else if (key == "rrt.batch_iterations") {
                    mBatchIterations = static_cast<std::size_t>(std::max(1, std::stoi(val)));
                } else if (key == "rrt.loop_playback") {
                    mLoopPlayback = toBool(val);
                } else if (key == "rrt.random_seed") {
                    rrtParams.random_seed = static_cast<unsigned int>(std::max(0, std::stoi(val)));
                } else if (key == "rrt.draw_payload_tree") {
                    visParams.draw_payload_tree = toBool(val);
                    vis_changed = true;
                } else if (key == "model.m_x") {
                    modelParams.m_x = std::stod(val); model_changed = true;
                } else if (key == "model.m_y") {
                    modelParams.m_y = std::stod(val); model_changed = true;
                } else if (key == "model.m") {
                    modelParams.m = std::stod(val); model_changed = true;
                } else if (key == "model.g") {
                    modelParams.g = std::stod(val); model_changed = true;
                } else if (key == "model.ell") {
                    modelParams.ell = std::stod(val);
                    visParams.rope_length = modelParams.ell;
                    model_changed = true; vis_changed = true;
                } else if (key == "model.D_x") {
                    modelParams.D_x = std::stod(val); model_changed = true;
                } else if (key == "model.D_y") {
                    modelParams.D_y = std::stod(val); model_changed = true;
                } else if (key == "solver.use_boost") {
                    solverParams.use_boost = toBool(val); solver_changed = true;
                } else if (key == "solver.integration_method") {
                    solverParams.method = SolverParams::fromStringToMethod(val); solver_changed = true;
                } else if (key == "arena_file") {
                    newArenaFile = val;
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Failed parsing '%s=%s': %s", key.c_str(), val.c_str(), e.what());
            }
        }

        const bool allowParamMutation =
            cmd.empty() || cmd == "reset" || cmd == "apply_and_reset" || cmd == "apply_only";

        if (allowParamMutation) {
            if (model_changed && modelParams.isValid()) {
                mPlanner->getModel()->setParameters(modelParams);
            }
            if (solver_changed) {
                mPlanner->getSolver()->setParameters(solverParams);
            }
            if (vis_changed) {
                clearRRTMarkersInRViz();
                mVisualizer.setParameters(visParams);
            }
            if (newArenaFile != mCurrentArenaFile) {
                mCurrentArenaFile = newArenaFile;
                mPlanner->setArena((newArenaFile == "none" || newArenaFile.empty())
                    ? nullptr
                    : loadArena(newArenaFile));
            }
            if (dt_changed) {
                recreateTimer(rrtParams.integration_dt);
            }

            startState = State::FromTrolleyPhase(start_x, start_y, 0.0, 0.0);
            goalState  = State::FromTrolleyPhase(goal_x,  goal_y,  0.0, 0.0);
        }

        if (cmd == "reset" || cmd == "apply_and_reset") {
            mPlanner->setParameters(rrtParams);
            resetExploration(startState, goalState);
        } else if (cmd == "apply_only") {
            mPlanner->updateParametersInPlace(rrtParams);
            publishRRTMarkers(get_clock()->now());
            publishStatus();
        } else if (cmd == "pause") {
            mPaused = true;
            publishStatus();
        } else if (cmd == "resume") {
            mPaused = false;
            publishStatus();
        } else if (cmd == "toggle_pause") {
            mPaused = !mPaused;
            publishStatus();
        } else if (cmd == "restart_playback") {
            mPlaybackIndex = 0;
            mPaused = false;
            publishStatus();
        } else if (cmd == "clear_history") {
            publishStatus();
        } else if (cmd == "step_once") {
            mPaused = true;
            executeSingleStep(get_clock()->now());
            publishRRTMarkers(get_clock()->now());
            publishStatus();
        }
    }

    void executeSingleStep(const rclcpp::Time& now) {
        const auto& params = mPlanner->getParameters();
        if (!mPlanningFinished) {
            for (std::size_t i = 0; i < mBatchIterations; ++i) {
                if (mTotalIterations >= params.max_iterations ||
                    mPlanner->getActiveTreeSize() >= params.max_nodes ||
                    mPlanner->getGoalNode() != nullptr) {
                    finishPlanning();
                    break;
                }
                mPlanner->stepIteration();
                ++mTotalIterations;
            }
            if (mPlanner->getClosestNode()) {
                publishCraneState(mPlanner->getClosestNode()->getData().state,
                                  mPlanner->getClosestNode()->getData().input,
                                  now);
            }
        } else if (!mPlannedStates.empty()) {
            const State& s = mPlannedStates[mPlaybackIndex];
            const Input& u = (mPlaybackIndex < mPlannedInputs.size())
                ? mPlannedInputs[mPlaybackIndex]
                : Input::Zero();
            publishCraneState(s, u, now);
            if (mPlaybackIndex + 1 < mPlannedStates.size()) {
                ++mPlaybackIndex;
            } else if (mLoopPlayback) {
                mPlaybackIndex = 0;
            }
        }
    }

    void timerCallback() {
        const auto now = get_clock()->now();
        const auto& params = mPlanner->getParameters();

        if (++mMarkerTick % 5 == 0) {
            publishStatus();
        }

        if ((now - mStartTime).seconds() < mStartupDelaySec) {
            publishCraneState(mPlanner->getStartState(), Input::Zero(), now);
            if (mMarkerTick % 10 == 0) {
                publishRRTMarkers(now);
            }
            return;
        }

        if (mPaused) {
            return;
        }

        if (!mPlanningFinished) {
            for (std::size_t i = 0; i < mBatchIterations; ++i) {
                if (mTotalIterations >= params.max_iterations ||
                    mPlanner->getActiveTreeSize() >= params.max_nodes ||
                    mPlanner->getGoalNode() != nullptr) {
                    finishPlanning();
                    break;
                }
                mPlanner->stepIteration();
                ++mTotalIterations;
            }

            if (mPlanner->getClosestNode()) {
                publishCraneState(mPlanner->getClosestNode()->getData().state,
                                  mPlanner->getClosestNode()->getData().input,
                                  now);
            } else {
                publishCraneState(mPlanner->getStartState(), Input::Zero(), now);
            }

            if (mMarkerTick % 3 == 0 || mPlanningFinished) {
                publishRRTMarkers(now);
            }
            return;
        }

        if (!mPlannedStates.empty()) {
            const State& s = mPlannedStates[mPlaybackIndex];
            const Input& u = (mPlaybackIndex < mPlannedInputs.size())
                ? mPlannedInputs[mPlaybackIndex]
                : Input::Zero();

            publishCraneState(s, u, now);

            CollisionMsg cmsg;
            cmsg.stamp = now;
            cmsg.in_collision = !mPlanner->isStateValidAndCollisionFree(s);
            mCollisionPub->publish(cmsg);

            if (mPlaybackIndex + 1 < mPlannedStates.size()) {
                ++mPlaybackIndex;
            } else if (mLoopPlayback) {
                mPlaybackIndex = 0;
            }
        }

        if (mMarkerTick % 25 == 0) {
            publishRRTMarkers(now);
        }
    }

    void finishPlanning() {
        mPlanningFinished = true;
        mPlaybackIndex = 0;
        auto targetNode = mPlanner->getGoalNode()
            ? mPlanner->getGoalNode()
            : mPlanner->getClosestNode();

        if (targetNode) {
            auto nodePath = mPlanner->getTree()->extractNodePath(targetNode);
            Planner::extractFullTrajectory(nodePath, mPlannedStates, mPlannedInputs);
        }

        if (mPlanner->getGoalNode()) {
            RCLCPP_INFO(get_logger(),
                "RRT Goal Reached! Active tree size: %zu nodes, iterations: %zu, trajectory steps: %zu, final dist: %.4f m",
                mPlanner->getActiveTreeSize(),
                mTotalIterations,
                mPlannedStates.size(),
                mPlanner->getBestGoalDistance());
        } else {
            RCLCPP_WARN(get_logger(),
                "RRT finished without exact goal hit. Closest dist: %.4f m, active tree size: %zu nodes, trajectory steps: %zu",
                mPlanner->getBestGoalDistance(),
                mPlanner->getActiveTreeSize(),
                mPlannedStates.size());
        }
        publishStatus();
    }

    void publishCraneState(const State& state, const Input& input, const rclcpp::Time& stamp) {
        mLastPublishedState = state;
        mLastPublishedInput = input;

        const auto& x  = state.getPositionState();
        const auto& dx = state.getVelocityState();
        const auto ddq = mPlanner->getModel()->computeAcceleration(state, input);

        StateMsg msg;
        msg.stamp        = stamp;
        msg.position     = {x(0), x(1), x(2), x(3)};
        msg.velocity     = {dx(0), dx(1), dx(2), dx(3)};
        msg.acceleration = {ddq(0), ddq(1), ddq(2), ddq(3)};
        msg.input        = input.toArray();
        msg.iteration    = mPlaybackIndex;
        msg.sim_time     = state.getTime();
        msg.rope_length  = mPlanner->getModel()->getParameters().ell;

        mStatePub->publish(msg);
    }

    void publishRRTMarkers(const rclcpp::Time& stamp) {
        auto markers = mVisualizer.generateMarkers(
            *mPlanner->getTree(),
            mPlanner->getStartState(),
            mPlanner->getGoalState(),
            mPlannedStates,
            stamp
        );
        mRRTMarkerPub->publish(markers);
    }

    void publishStatus() {
        const auto& p = mPlanner->getParameters();
        std::string mode = "EXPLORING";
        if (mPaused) {
            mode = "PAUSED";
        } else if (mPlanningFinished) {
            mode = (mPlanner->getGoalNode() != nullptr) ? "GOAL_REACHED" : "FINISHED_CLOSEST";
        }

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4)
            << "mode=" << mode << "\n"
            << "paused=" << (mPaused ? 1 : 0) << "\n"
            << "goal_reached=" << (mPlanner->getGoalNode() ? 1 : 0) << "\n"
            << "iterations=" << mTotalIterations << "\n"
            << "max_iterations=" << p.max_iterations << "\n"
            << "nodes=" << mPlanner->getActiveTreeSize() << "\n"
            << "max_nodes=" << p.max_nodes << "\n"
            << "collisions=" << mPlanner->getCollisionRejections() << "\n"
            << "best_dist=" << mPlanner->getBestGoalDistance() << "\n"
            << "playback_idx=" << mPlaybackIndex << "\n"
            << "playback_len=" << mPlannedStates.size() << "\n"
            << "x=" << mLastPublishedState.getX() << "\n"
            << "y=" << mLastPublishedState.getY() << "\n"
            << "alpha_deg=" << (mLastPublishedState.getAlpha() * 180.0 / M_PI) << "\n"
            << "beta_deg=" << (mLastPublishedState.getBeta() * 180.0 / M_PI) << "\n"
            << "fx=" << mLastPublishedInput.getFx() << "\n"
            << "fy=" << mLastPublishedInput.getFy() << "\n";

        std_msgs::msg::String msg;
        msg.data = oss.str();
        mStatusPub->publish(msg);
    }

    ArenaPtr loadArena(const std::string& arenaSpec) {
        if (arenaSpec.empty() || arenaSpec == "none") {
            return nullptr;
        }
        try {
            auto arena = arena::loadArenaByNameOrPath<ArenaScalar>(arenaSpec);
            RCLCPP_INFO(get_logger(), "Loaded arena: %s", arenaSpec.c_str());
            return arena;
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Could not load arena '%s': %s", arenaSpec.c_str(), e.what());
            return nullptr;
        }
    }

    State loadStateParam(const std::string& name, const std::vector<double>& defaultVec) {
        std::vector<double> vec = this->get_parameter_or(name, defaultVec);
        if (vec.size() != 8) {
            RCLCPP_WARN(get_logger(), "Parameter '%s' does not have 8 elements; using default.", name.c_str());
            return State::FromVector(defaultVec);
        }
        return State::FromVector(vec);
    }

    ModelParams loadModelParameters() {
        ModelParams params;
        params.m_x = this->get_parameter_or("model.m_x", params.m_x);
        params.m_y = this->get_parameter_or("model.m_y", params.m_y);
        params.m   = this->get_parameter_or("model.m",   params.m);
        params.g   = this->get_parameter_or("model.g",   params.g);
        params.ell = this->get_parameter_or("model.ell", params.ell);
        params.D_x = this->get_parameter_or("model.D_x", params.D_x);
        params.D_y = this->get_parameter_or("model.D_y", params.D_y);
        return params;
    }

    SolverParams loadSolverParameters() {
        SolverParams params(false, Solver::RKMethod::ClassicalRK4);
        params.use_boost = this->get_parameter_or("solver.use_boost", params.use_boost);
        std::string methodStr = this->get_parameter_or("solver.integration_method", params.methodToString());
        try {
            params.method = SolverParams::fromStringToMethod(methodStr);
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "%s. Falling back to ClassicalRK4.", e.what());
            params.method = Solver::RKMethod::ClassicalRK4;
        }
        return params;
    }

    PlannerParam loadRRTParameters() {
        PlannerParam p;
        p.x_min = this->get_parameter_or("rrt.x_min", p.x_min);
        p.x_max = this->get_parameter_or("rrt.x_max", p.x_max);
        p.y_min = this->get_parameter_or("rrt.y_min", p.y_min);
        p.y_max = this->get_parameter_or("rrt.y_max", p.y_max);
        p.max_velocity = this->get_parameter_or("rrt.max_velocity", p.max_velocity);
        p.max_swing_angle = this->get_parameter_or("rrt.max_swing_angle", p.max_swing_angle);
        p.max_force = this->get_parameter_or("rrt.max_force", p.max_force);
        p.min_duration = this->get_parameter_or("rrt.min_duration", p.min_duration);
        p.max_duration = this->get_parameter_or("rrt.max_duration", p.max_duration);
        p.enable_force_smoothing = this->get_parameter_or("rrt.enable_force_smoothing", p.enable_force_smoothing);
        p.force_direction_bias = this->get_parameter_or("rrt.force_direction_bias", p.force_direction_bias);
        p.w_pos = this->get_parameter_or("rrt.w_pos", p.w_pos);
        p.w_vel = this->get_parameter_or("rrt.w_vel", p.w_vel);
        p.goal_bias = this->get_parameter_or("rrt.goal_bias", p.goal_bias);
        p.min_search_step = static_cast<std::size_t>(
            this->get_parameter_or("rrt.min_search_step", static_cast<int>(p.min_search_step)));
        p.max_search_step = static_cast<std::size_t>(
            this->get_parameter_or("rrt.max_search_step", static_cast<int>(p.max_search_step)));
        p.use_equal_budget_random_subset = this->get_parameter_or(
            "rrt.use_equal_budget_random_subset", p.use_equal_budget_random_subset);
        p.force_history_nns_weight = this->get_parameter_or(
            "rrt.force_history_nns_weight", p.force_history_nns_weight);
        p.use_sst_planner = this->get_parameter_or("rrt.use_sst_planner", p.use_sst_planner);
        p.sst_delta_bn = this->get_parameter_or("rrt.sst_delta_bn", p.sst_delta_bn);
        p.sst_delta_s = this->get_parameter_or("rrt.sst_delta_s", p.sst_delta_s);
        p.num_control_samples = static_cast<std::size_t>(
            this->get_parameter_or("rrt.num_control_samples", static_cast<int>(p.num_control_samples)));
        p.use_cell_partition_nns = this->get_parameter_or("rrt.use_cell_partition_nns", p.use_cell_partition_nns);
        p.cell_grid_radix = static_cast<std::size_t>(
            this->get_parameter_or("rrt.cell_grid_radix", static_cast<int>(p.cell_grid_radix)));
        p.use_pd_local_steering = this->get_parameter_or("rrt.use_pd_local_steering", p.use_pd_local_steering);
        p.enable_near_goal_braking = this->get_parameter_or("rrt.enable_near_goal_braking", p.enable_near_goal_braking);
        p.near_goal_radius = this->get_parameter_or("rrt.near_goal_radius", p.near_goal_radius);
        p.near_goal_zero_force_prob = this->get_parameter_or("rrt.near_goal_zero_force_prob", p.near_goal_zero_force_prob);
        p.enable_soft_corridor = this->get_parameter_or("rrt.enable_soft_corridor", p.enable_soft_corridor);
        p.corridor_half_width = this->get_parameter_or("rrt.corridor_half_width", p.corridor_half_width);
        p.corridor_penalty_weight = this->get_parameter_or("rrt.corridor_penalty_weight", p.corridor_penalty_weight);
        p.corridor_sample_prob = this->get_parameter_or("rrt.corridor_sample_prob", p.corridor_sample_prob);
        p.goal_pos_tolerance = this->get_parameter_or("rrt.goal_pos_tolerance", p.goal_pos_tolerance);
        p.goal_vel_tolerance = this->get_parameter_or("rrt.goal_vel_tolerance", p.goal_vel_tolerance);
        p.goal_swing_tolerance = this->get_parameter_or("rrt.goal_swing_tolerance", p.goal_swing_tolerance);
        p.goal_omega_tolerance = this->get_parameter_or("rrt.goal_omega_tolerance", p.goal_omega_tolerance);
        p.require_goal_velocity_check = this->get_parameter_or("rrt.require_goal_velocity_check", p.require_goal_velocity_check);
        p.require_goal_swing_check = this->get_parameter_or("rrt.require_goal_swing_check", p.require_goal_swing_check);
        p.integration_dt = this->get_parameter_or("rrt.integration_dt", p.integration_dt);
        p.ball_radius = this->get_parameter_or("ball_radius", p.ball_radius);
        p.cable_radius = this->get_parameter_or("rrt.cable_radius", p.cable_radius);
        p.check_rope_collision = this->get_parameter_or("rrt.check_rope_collision", p.check_rope_collision);
        p.rope_collision_samples = static_cast<std::size_t>(
            this->get_parameter_or("rrt.rope_collision_samples", static_cast<int>(p.rope_collision_samples)));
        p.max_iterations = static_cast<std::size_t>(
            this->get_parameter_or("rrt.max_iterations", static_cast<int>(p.max_iterations)));
        p.max_nodes = static_cast<std::size_t>(
            this->get_parameter_or("rrt.max_nodes", static_cast<int>(p.max_nodes)));
        p.random_seed = static_cast<unsigned int>(
            this->get_parameter_or("rrt.random_seed", static_cast<int>(p.random_seed)));
        return p;
    }
};

} // namespace gantry

#endif // GANTRY_RRT_EXPLORATION_NODE_HPP

