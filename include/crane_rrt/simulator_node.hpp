#ifndef __GANTRY_SIM_NODE_HPP__
#define __GANTRY_SIM_NODE_HPP__

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "crane_rrt/arena/ecc_arenas.hpp"
#include "crane_rrt/arena/sphere.hpp"
#include "crane_rrt/crane_input.hpp"
#include "crane_rrt/crane_state.hpp"
#include "crane_rrt/dynamic_model.hpp"
#include "crane_rrt/msg/collision_status.hpp"
#include "crane_rrt/msg/gantry_crane_state.hpp"
#include "crane_rrt/numerical_solver.hpp"
#include "crane_rrt/simulator.hpp"

namespace gantry {

class SimulatorNode : public rclcpp::Node {
public:
    using Self          = SimulatorNode;
    using Ptr           = std::shared_ptr<Self>;
    using ConstPtr      = std::shared_ptr<const Self>;
    using SimPtr        = typename Simulator::Ptr;
    using SimConstPtr   = typename Simulator::ConstPtr;
    using Model         = typename Simulator::Model;
    using ModelPtr      = typename Simulator::ModelPtr;
    using State         = typename Simulator::State;
    using Input         = typename Simulator::Input;
    using Solver        = typename Simulator::Solver;
    using SolverPtr     = typename Simulator::SolverPtr;
    using SimParams     = typename Simulator::Parameters;
    using SolverParams  = typename Simulator::SolverParams;
    using ModelParams   = typename Simulator::ModelParams;
    using ArenaScalar   = float;
    using ArenaType     = arena::Arena<ArenaScalar>;
    using ArenaPtr      = typename ArenaType::Ptr;
    using SphereType    = arena::Sphere<ArenaScalar>;
    using CollisionMsg  = crane_rrt::msg::CollisionStatus;

    explicit SimulatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("crane_sim_node", options),
          mMostRecentInput(Input::Zero())
    {
        ModelParams  modelParams  = loadModelParameters();
        SolverParams solverParams = loadSolverParameters();
        SimParams    simParams    = loadSimParameters();

        mArenaFile  = this->get_parameter_or("arena_file", std::string{"none"});
        mBallRadius = static_cast<ArenaScalar>(this->get_parameter_or("ball_radius", 0.12));
        mBall.setScale(Eigen::Matrix<ArenaScalar, 3, 1>::Constant(2.0f * mBallRadius));

        if (!mArenaFile.empty() && mArenaFile != "none") {
            try {
                mArena = arena::loadArenaByNameOrPath<ArenaScalar>(mArenaFile);
                RCLCPP_INFO(this->get_logger(), "Loaded obstacle arena: %s", mArenaFile.c_str());
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to load arena '%s': %s", mArenaFile.c_str(), e.what());
            }
        }

        mSimulator = std::make_shared<Simulator>(simParams, modelParams, solverParams);

        mPublisher = this->create_publisher<crane_rrt::msg::GantryCraneState>(
            "/gantry_crane/state", 10
        );
        mCollisionPub = this->create_publisher<CollisionMsg>(
            "/gantry_crane/collision_status", 10
        );

        mForceSub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gantry_force_input", 10,
            std::bind(&SimulatorNode::forceCallback, this, std::placeholders::_1)
        );

        mTimer = this->create_wall_timer(
            std::chrono::duration<double>(simParams.getPeriod()),
            std::bind(&SimulatorNode::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "SimulatorNode initialized at %.1f Hz.", simParams.frequency);
    }

    Simulator& getSimulator() { return *mSimulator; }
    const Simulator& getSimulator() const { return *mSimulator; }

protected:
    SimPtr mSimulator;
    Input mMostRecentInput;
    std::string mArenaFile;
    ArenaScalar mBallRadius = 0.12f;
    ArenaPtr mArena;
    SphereType mBall;

    rclcpp::Publisher<crane_rrt::msg::GantryCraneState>::SharedPtr mPublisher;
    rclcpp::Publisher<CollisionMsg>::SharedPtr mCollisionPub;
    rclcpp::TimerBase::SharedPtr mTimer;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mForceSub;

    void forceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            mMostRecentInput = Input::FromParts(msg->data[0], msg->data[1]);
        }
    }

    virtual void timerCallback() {
        mSimulator->step(mMostRecentInput);

        const auto& state = mSimulator->getState();
        const auto& x     = state.getPositionState();
        const auto& dx    = state.getVelocityState();
        const auto& u     = mSimulator->getInput();
        const auto  ddq   = mSimulator->getModel().computeAcceleration(state, u);
        const double ell  = mSimulator->getModel().getParameters().ell;
        const auto now    = this->get_clock()->now();

        crane_rrt::msg::GantryCraneState msg;
        msg.stamp        = now;
        msg.position     = {x(0), x(1), x(2), x(3)};
        msg.velocity     = {dx(0), dx(1), dx(2), dx(3)};
        msg.acceleration = {ddq(0), ddq(1), ddq(2), ddq(3)};
        msg.input        = u.toArray();
        msg.iteration    = mSimulator->getIterationCount();
        msg.sim_time     = mSimulator->getSimTime();
        msg.rope_length  = ell;
        mPublisher->publish(msg);

        if (mArena) {
            const auto pos_d = state.getBallPosition(ell);
            mBall.setPosition(Eigen::Matrix<ArenaScalar, 3, 1>(
                static_cast<ArenaScalar>(pos_d.x()),
                static_cast<ArenaScalar>(pos_d.y()),
                static_cast<ArenaScalar>(pos_d.z())));
            CollisionMsg cmsg;
            cmsg.stamp = now;
            cmsg.in_collision = mArena->checkCollision(mBall);
            mCollisionPub->publish(cmsg);
        }
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
            RCLCPP_WARN(this->get_logger(), "%s. Falling back to ClassicalRK4.", e.what());
            params.method = Solver::RKMethod::ClassicalRK4;
        }
        return params;
    }

    Simulator::Parameters loadSimParameters() {
        Simulator::Parameters params;
        double freq = this->get_parameter_or("sim.frequency", params.frequency);
        std::vector<double> vec = this->get_parameter_or(
            "sim.initial_state", params.initial_state.toStdVector());
        bool log_state = this->get_parameter_or("sim.log_state", params.logger_active);
        if (vec.size() != 8) {
            throw std::runtime_error("Initial state must have 8 elements.");
        }
        return Simulator::Parameters(freq, log_state, State::FromVector(vec));
    }
};

} // namespace gantry

#endif // __GANTRY_SIM_NODE_HPP__

