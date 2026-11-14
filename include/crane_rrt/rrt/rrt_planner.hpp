#ifndef GANTRY_RRT_PLANNER_HPP
#define GANTRY_RRT_PLANNER_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <utility>
#include <vector>

#include "crane_rrt/crane_state.hpp"
#include "crane_rrt/crane_input.hpp"
#include "crane_rrt/dynamic_model.hpp"
#include "crane_rrt/numerical_solver.hpp"
#include "crane_rrt/arena/arena.hpp"
#include "crane_rrt/arena/sphere.hpp"
#include "crane_rrt/rrt/rrt_node.hpp"
#include "crane_rrt/rrt/rrt_tree.hpp"

namespace gantry {
namespace rrt {

template <typename ScalarT = double, typename ArenaScalarT = float>
class RRTPlanner {
public:
    using Scalar      = ScalarT;
    using ArenaScalar = ArenaScalarT;
    using State       = CraneState<Scalar>;
    using Input       = CraneInput<Scalar>;
    using Model       = DynamicModel<Scalar>;
    using ModelPtr    = typename Model::Ptr;
    using Solver      = NumericalSolver<Scalar>;
    using SolverPtr   = typename Solver::Ptr;
    using NodeData    = ::rrt::CraneRRTData<Scalar>;
    using Tree        = ::rrt::RRTTree<Scalar, NodeData>;
    using TreePtr     = typename Tree::Ptr;
    using Node        = typename Tree::Node;
    using NodePtr     = typename Tree::NodePtr;
    using ArenaType   = arena::Arena<ArenaScalar>;
    using ArenaPtr    = typename ArenaType::Ptr;
    using SphereType  = arena::Sphere<ArenaScalar>;
    using Vec2        = Eigen::Matrix<Scalar, 2, 1>;
    using Vec3        = Eigen::Matrix<Scalar, 3, 1>;

    struct Parameters {
        // Workspace bounds [m]
        Scalar x_min = Scalar(-4.5);
        Scalar x_max = Scalar(4.5);
        Scalar y_min = Scalar(-4.5);
        Scalar y_max = Scalar(4.5);

        // Path-wise velocity & swing limits
        Scalar max_velocity    = Scalar(2.0);     // [m/s]
        Scalar max_swing_angle = Scalar(0.4363);  // [rad] (~25 deg)

        // Force sampling, sustained horizon & ZOH smoothing
        Scalar max_force       = Scalar(0.5);     // [N]
        Scalar min_duration    = Scalar(0.2);     // [s]
        Scalar max_duration    = Scalar(0.5);     // [s]
        bool   enable_force_smoothing = true;
        Scalar force_direction_bias   = Scalar(0.35);

        // Dimensionless 4D trolley phase-space metric weights
        Scalar w_pos = Scalar(0.85);
        Scalar w_vel = Scalar(0.15);

        // Goal biasing
        Scalar goal_bias = Scalar(0.15);

        // Candidate-set reduction & optional force-history parent penalty
        std::size_t min_search_step = 1;
        std::size_t max_search_step = 15;
        bool        use_equal_budget_random_subset = false; // M7-RandSub
        Scalar      force_history_nns_weight = Scalar(0.0); // M7-ForceHist w_u

        // Stable Sparse RRT (SST, Li et al. 2016) baseline parameters
        bool        use_sst_planner = false;
        Scalar      sst_delta_bn    = Scalar(0.45); // [m] Best-Near selection radius
        Scalar      sst_delta_s     = Scalar(0.18); // [m] Witness pruning radius
        double      max_planning_time_sec = 0.0;    // [s] wall-clock timeout (0 = disabled)

        // Domain-adapted RRT baselines (B1: LaValle Best-of-K, B2: Zhou 3x3 Cell-RRT, B3: PD Anti-Sway)
        std::size_t num_control_samples    = 1;
        bool        use_cell_partition_nns = false;
        std::size_t cell_grid_radix        = 3;
        bool        use_pd_local_steering  = false;

        // Near-goal PD braking & sway damping
        bool   enable_near_goal_braking  = true;
        Scalar near_goal_radius          = Scalar(0.85);
        Scalar near_goal_zero_force_prob = Scalar(0.60);

        // Soft crane corridor guidance
        bool   enable_soft_corridor    = false;
        Scalar corridor_half_width     = Scalar(1.25); // [m]
        Scalar corridor_penalty_weight = Scalar(0.50);
        Scalar corridor_sample_prob    = Scalar(0.88);

        // Payload-centric terminal safe set S_goal tolerances
        Scalar goal_pos_tolerance   = Scalar(0.35);  // [m]
        Scalar goal_vel_tolerance   = Scalar(0.55);  // [m/s]
        Scalar goal_swing_tolerance = Scalar(0.22);  // [rad] (~12.6 deg)
        Scalar goal_omega_tolerance = Scalar(0.55);  // [rad/s]
        bool   require_goal_velocity_check = true;
        bool   require_goal_swing_check    = true;

        // Rollout & 3D collision checking parameters
        Scalar       integration_dt         = Scalar(0.02); // [s]
        Scalar       ball_radius            = Scalar(0.12); // [m]
        Scalar       cable_radius           = Scalar(0.08); // [m]
        bool         check_rope_collision   = true;
        std::size_t  rope_collision_samples = 10;
        std::size_t  max_iterations         = 20000;
        std::size_t  max_nodes              = 6000;
        unsigned int random_seed            = 42;           // 0 = std::random_device

        bool isValid() const {
            return (x_max > x_min) && (y_max > y_min) &&
                   (max_velocity > Scalar(0)) && (max_swing_angle > Scalar(0)) &&
                   (max_force > Scalar(0)) && (min_duration > Scalar(0)) &&
                   (max_duration >= min_duration) && (integration_dt > Scalar(0)) &&
                   (ball_radius >= Scalar(0)) && (cable_radius >= Scalar(0)) &&
                   (goal_pos_tolerance > Scalar(0)) && (goal_vel_tolerance > Scalar(0)) &&
                   (goal_swing_tolerance > Scalar(0)) && (goal_omega_tolerance > Scalar(0));
        }

        std::string toString() const {
            std::ostringstream oss;
            oss << "RRTPlanner::Parameters {\n"
                << "  workspace: [" << x_min << ", " << x_max << "] x [" << y_min << ", " << y_max << "]\n"
                << "  max_velocity: " << max_velocity << " m/s, max_force: " << max_force << " N\n"
                << "  duration: [" << min_duration << ", " << max_duration << "] s, dt: " << integration_dt << " s\n"
                << "  enable_force_smoothing: " << (enable_force_smoothing ? "true" : "false") << "\n"
                << "  weights (w_pos, w_vel, w_u): (" << w_pos << ", " << w_vel << ", " << force_history_nns_weight << ")\n"
                << "  goal_bias: " << goal_bias << ", force_direction_bias: " << force_direction_bias << "\n"
                << "  nearest search step: [" << min_search_step << ", " << max_search_step << "]"
                << (use_equal_budget_random_subset ? " (RandSub)" : "")
                << (use_sst_planner ? " (SST)" : "") << "\n"
                << "  soft_corridor: " << (enable_soft_corridor ? "true" : "false")
                << " (W_corr = " << corridor_half_width << " m)\n"
                << "  near_goal_braking: " << (enable_near_goal_braking ? "true" : "false")
                << " (r = " << near_goal_radius << " m)\n"
                << "  goal_pos_tolerance: " << goal_pos_tolerance << " m\n"
                << "  max_iterations: " << max_iterations << ", max_nodes: " << max_nodes << "\n"
                << "}";
            return oss.str();
        }
    };

    struct PlanResult {
        bool success = false;
        std::size_t iterations = 0;
        std::size_t tree_size = 0;
        std::size_t sst_active_nodes = 0;
        std::size_t sst_tree_nodes = 0;
        std::size_t collision_rejections = 0;
        std::size_t total_distance_evals = 0;
        std::size_t valid_extensions = 0;
        double valid_extension_rate = 0.0;
        double dist_evals_per_valid_branch = 0.0;
        double planning_time_sec = 0.0;
        Scalar best_goal_distance = std::numeric_limits<Scalar>::infinity();
        NodePtr goal_node = nullptr;
        NodePtr closest_node = nullptr;
        std::vector<NodePtr> node_path{};
        std::vector<State> state_trajectory{};
        std::vector<Input> input_trajectory{};
    };

    RRTPlanner(ModelPtr model,
               SolverPtr solver,
               ArenaPtr arena = nullptr,
               const Parameters& params = Parameters())
        : mModel(std::move(model)),
          mSolver(std::move(solver)),
          mArena(std::move(arena)),
          mParams(params),
          mTree(std::make_shared<Tree>(params.max_nodes))
    {
        if (!mModel) {
            mModel = std::make_shared<Model>();
        }
        if (!mSolver) {
            mSolver = std::make_shared<Solver>(mModel);
        } else {
            mSolver->setModel(mModel);
        }
        reseed(mParams.random_seed);
    }

    void reseed(unsigned int seed) {
        if (seed == 0) {
            std::random_device rd;
            mRng.seed(rd());
        } else {
            mRng.seed(seed);
        }
    }

    void setParameters(const Parameters& params) {
        mParams = params;
        reseed(mParams.random_seed);
    }

    void updateParametersInPlace(const Parameters& params) {
        mParams = params;
    }

    const Parameters& getParameters() const { return mParams; }
    const TreePtr& getTree() const { return mTree; }
    const ModelPtr& getModel() const { return mModel; }
    const SolverPtr& getSolver() const { return mSolver; }
    const ArenaPtr& getArena() const { return mArena; }
    void setArena(ArenaPtr arena) { mArena = std::move(arena); }

    const State& getStartState() const { return mStartState; }
    const State& getGoalState() const { return mGoalState; }
    NodePtr getGoalNode() const { return mGoalNode; }
    NodePtr getClosestNode() const { return mClosestNode; }
    Scalar getBestGoalDistance() const { return mBestGoalDist; }
    std::size_t getIterations() const { return mIterations; }
    std::size_t getCollisionRejections() const { return mCollisionRejections; }
    std::size_t getTotalDistanceEvals() const { return mTotalDistanceEvals; }
    std::size_t getValidExtensions() const { return mValidExtensions; }
    std::size_t getActiveTreeSize() const {
        return mParams.use_sst_planner ? countSSTTreeNodes() : mTree->size();
    }

    bool initialize(const State& start_state, const State& goal_state) {
        mStartState = start_state;
        mGoalState = goal_state;
        mTree->clear();
        mTree->reserve(mParams.max_nodes);
        mGoalNode = nullptr;
        mClosestNode = nullptr;
        mIterations = 0;
        mCollisionRejections = 0;
        mTotalDistanceEvals = 0;
        mValidExtensions = 0;
        mSSTWitnesses.clear();

        if (!mParams.isValid() || !isStateValidAndCollisionFree(mStartState)) {
            return false;
        }

        NodeData rootData(mStartState, Input::Zero(mStartState.getTime()), Scalar(0), Scalar(0));
        rootData.sst_active = true;
        rootData.sst_children = 0;
        rootData.trajectory.push_back(mStartState);
        rootData.input_trajectory.push_back(Input::Zero(mStartState.getTime()));

        NodePtr root = mTree->addNode(std::move(rootData), nullptr);
        mClosestNode = root;
        mBestGoalDist = goalPositionDistance(mStartState);

        if (mParams.use_sst_planner) {
            mSSTWitnesses.push_back(SSTWitness{mStartState, root});
        }

        if (isGoalReached(mStartState)) {
            mGoalNode = root;
        }
        return true;
    }

    bool stepIteration() {
        if (mTree->empty()) {
            return false;
        }
        ++mIterations;

        // 1. Sample random target in 4D trolley phase space [x, y, x_dot, y_dot]
        const State x_rand = sampleRandomPhaseState();
        const NodeData query_data(x_rand);

        // 2. Select parent candidate via dimensionless 4D metric (+ optional force-history & corridor penalties)
        const Scalar L_0 = Scalar(2.0);
        const Scalar v_0 = std::max(mParams.max_velocity, Scalar(1e-3));
        auto dist_fn = [this, L_0, v_0](const NodeData& a, const NodeData& b) -> Scalar {
            ++mTotalDistanceEvals;
            Scalar base_d = a.state.phaseDistanceTo(b.state, mParams.w_pos, mParams.w_vel, L_0, v_0);
            if (mParams.force_history_nns_weight > Scalar(0)) {
                const Vec2 u_des = computeDeterministicReferenceForce(a.state, b.state);
                const Vec2 u_prev(a.input.getFx(), a.input.getFy());
                base_d += mParams.force_history_nns_weight *
                          ((u_des - u_prev).norm() / (Scalar(2) * mParams.max_force));
            }
            if (!mParams.enable_soft_corridor) {
                return base_d;
            }
            return base_d + softCorridorPenalty(a.state.getTrolleyPosition2D(), L_0);
        };

        NodePtr nearest_node = nullptr;
        if (mParams.use_sst_planner) {
            nearest_node = findNearestSSTBestFirst(query_data, dist_fn);
        } else if (mParams.use_cell_partition_nns) {
            nearest_node = findNearestCellPartitioned(query_data, dist_fn);
        } else if (mParams.use_equal_budget_random_subset) {
            nearest_node = mTree->findNearestRandomSubset(
                query_data,
                dist_fn,
                mRng,
                mParams.min_search_step,
                mParams.max_search_step
            );
        } else {
            nearest_node = mTree->findNearest(
                query_data,
                dist_fn,
                mRng,
                mParams.min_search_step,
                mParams.max_search_step
            );
        }
        if (!nearest_node) {
            return false;
        }

        const NodeData& parent_data = nearest_node->getData();
        const State& x_near = parent_data.state;
        const Input& u_near = parent_data.input;

        // 3 & 4. Sample control horizon & target force, propagate ZOH-smoothed RK4 rollout
        const std::size_t K_samples = std::max<std::size_t>(1, mParams.num_control_samples);
        const Scalar dt = mParams.integration_dt;

        std::vector<State> best_rollout_states;
        std::vector<Input> best_rollout_inputs;
        Scalar best_rollout_dist = std::numeric_limits<Scalar>::infinity();
        std::size_t best_num_steps = 0;

        for (std::size_t c_idx = 0; c_idx < K_samples; ++c_idx) {
            const Scalar duration = sampleForceDuration();
            const Input u_target = sampleTargetForce(parent_data, x_rand);

            const std::size_t num_steps = std::max<std::size_t>(
                1, static_cast<std::size_t>(std::llround(duration / dt)));

            std::vector<State> rollout_states;
            std::vector<Input> rollout_inputs;
            rollout_states.reserve(num_steps);
            rollout_inputs.reserve(num_steps);

            State x_curr = x_near;
            double t_curr = x_near.getTime();
            bool valid_branch = true;

            for (std::size_t k = 1; k <= num_steps; ++k) {
                const Scalar alpha = static_cast<Scalar>(k) / static_cast<Scalar>(num_steps);
                Input u_step = mParams.enable_force_smoothing
                    ? u_near.interpolate(u_target, alpha)
                    : u_target;
                u_step.setTime(t_curr + static_cast<double>(dt));

                State x_next;
                mSolver->step(x_curr, u_step, t_curr, static_cast<double>(dt), x_next);

                if (!isStateValidAndCollisionFree(x_next)) {
                    valid_branch = false;
                    ++mCollisionRejections;
                    break;
                }

                rollout_states.push_back(x_next);
                rollout_inputs.push_back(u_step);
                x_curr = x_next;
                t_curr += static_cast<double>(dt);

                if (isGoalReached(x_next)) {
                    break;
                }
            }

            if (!valid_branch || rollout_states.empty()) {
                continue;
            }

            const Scalar cand_dist = isGoalReached(rollout_states.back())
                ? Scalar(-1.0)
                : (rollout_states.back().phaseDistanceTo(x_rand, mParams.w_pos, mParams.w_vel, L_0, v_0) +
                   (mParams.enable_soft_corridor
                        ? softCorridorPenalty(rollout_states.back().getTrolleyPosition2D(), L_0)
                        : Scalar(0)));

            if (cand_dist < best_rollout_dist) {
                best_rollout_dist = cand_dist;
                best_rollout_states = std::move(rollout_states);
                best_rollout_inputs = std::move(rollout_inputs);
                best_num_steps = best_rollout_states.size();
                if (cand_dist < Scalar(0)) {
                    break;
                }
            }
        }

        if (best_rollout_states.empty()) {
            return false;
        }
        ++mValidExtensions;

        // 5. Commit branch to tree (with SST witness dominance pruning when active)
        const State& x_new = best_rollout_states.back();
        const Input& u_new = best_rollout_inputs.back();
        const Scalar step_dist = (x_new.getTrolleyPosition2D() - x_near.getTrolleyPosition2D()).norm();
        const Scalar branch_dur = static_cast<Scalar>(best_num_steps) * dt;
        const Scalar new_cost = mParams.use_sst_planner
            ? (parent_data.cost + branch_dur)
            : (parent_data.cost + step_dist);
        const bool reaches_goal = isGoalReached(x_new);

        if (mParams.use_sst_planner && !reaches_goal) {
            if (!checkSSTWitnessDominanceAndPrepare(x_new, new_cost)) {
                return false;
            }
        }

        NodeData new_data(x_new, u_new, branch_dur, new_cost);
        new_data.sst_active = true;
        new_data.sst_children = 0;
        new_data.trajectory = std::move(best_rollout_states);
        new_data.input_trajectory = std::move(best_rollout_inputs);

        NodePtr new_node = mTree->addNode(std::move(new_data), nearest_node);
        if (mParams.use_sst_planner && !reaches_goal) {
            commitSSTNodeAndPruneWitness(nearest_node, new_node);
        }

        // 6. Update closest-to-goal node & goal condition
        const Scalar goal_dist = goalPositionDistance(x_new);
        if (goal_dist < mBestGoalDist) {
            mBestGoalDist = goal_dist;
            mClosestNode = new_node;
        }

        if (!mGoalNode && reaches_goal) {
            mGoalNode = new_node;
        }

        return true;
    }

    PlanResult plan(const State& start_state, const State& goal_state) {
        const auto t_start = std::chrono::steady_clock::now();

        PlanResult result;
        if (!initialize(start_state, goal_state)) {
            return result;
        }

        while (mIterations < mParams.max_iterations &&
               (mParams.use_sst_planner ? (countSSTTreeNodes() < mParams.max_nodes) : (mTree->size() < mParams.max_nodes)) &&
               mGoalNode == nullptr) {
            if (mParams.max_planning_time_sec > 0.0 && (mIterations & 31u) == 0u) {
                const double elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t_start).count();
                if (elapsed >= mParams.max_planning_time_sec) {
                    break;
                }
            }
            stepIteration();
        }

        const auto t_end = std::chrono::steady_clock::now();
        result.planning_time_sec = std::chrono::duration<double>(t_end - t_start).count();
        result.iterations = mIterations;
        result.tree_size = mParams.use_sst_planner ? countSSTTreeNodes() : mTree->size();
        result.sst_active_nodes = mParams.use_sst_planner ? countSSTActiveNodes() : mTree->size();
        result.sst_tree_nodes = mParams.use_sst_planner ? countSSTTreeNodes() : mTree->size();
        result.collision_rejections = mCollisionRejections;
        result.total_distance_evals = mTotalDistanceEvals;
        result.valid_extensions = mValidExtensions;
        result.valid_extension_rate = (mIterations > 0)
            ? (static_cast<double>(mValidExtensions) / static_cast<double>(mIterations))
            : 0.0;
        result.dist_evals_per_valid_branch = (mValidExtensions > 0)
            ? (static_cast<double>(mTotalDistanceEvals) / static_cast<double>(mValidExtensions))
            : static_cast<double>(mTotalDistanceEvals);
        result.best_goal_distance = mBestGoalDist;
        result.goal_node = mGoalNode;
        result.closest_node = mClosestNode;
        result.success = (mGoalNode != nullptr);

        NodePtr target_node = mGoalNode ? mGoalNode : mClosestNode;
        if (target_node) {
            result.node_path = mTree->extractNodePath(target_node);
            extractFullTrajectory(result.node_path, result.state_trajectory, result.input_trajectory);
        }

        return result;
    }

    static void extractFullTrajectory(const std::vector<NodePtr>& node_path,
                                      std::vector<State>& states_out,
                                      std::vector<Input>& inputs_out) {
        states_out.clear();
        inputs_out.clear();
        for (const auto& node : node_path) {
            if (!node) continue;
            const auto& data = node->getData();
            for (const auto& s : data.trajectory) {
                states_out.push_back(s);
            }
            for (const auto& u : data.input_trajectory) {
                inputs_out.push_back(u);
            }
        }
    }

    bool isStateValidAndCollisionFree(const State& state) const {
        if (!state.isValid()) {
            return false;
        }

        const Scalar x = state.getX();
        const Scalar y = state.getY();
        if (x < mParams.x_min || x > mParams.x_max ||
            y < mParams.y_min || y > mParams.y_max) {
            return false;
        }

        if (std::abs(state.getXdot()) > mParams.max_velocity ||
            std::abs(state.getYdot()) > mParams.max_velocity) {
            return false;
        }

        if (state.getCompositeSwingAngle() > mParams.max_swing_angle) {
            return false;
        }

        const Scalar ell = mModel->getParameters().ell;
        const Vec3 ball_pos = state.getBallPosition(ell);
        if (ball_pos.x() < mParams.x_min || ball_pos.x() > mParams.x_max ||
            ball_pos.y() < mParams.y_min || ball_pos.y() > mParams.y_max) {
            return false;
        }

        if (!mArena) {
            return true;
        }

        SphereType ball;
        ball.setScale(Eigen::Matrix<ArenaScalar, 3, 1>::Constant(
            static_cast<ArenaScalar>(Scalar(2) * mParams.ball_radius)));
        ball.setPosition(Eigen::Matrix<ArenaScalar, 3, 1>(
            static_cast<ArenaScalar>(ball_pos.x()),
            static_cast<ArenaScalar>(ball_pos.y()),
            static_cast<ArenaScalar>(ball_pos.z())));

        if (mArena->checkCollision(ball)) {
            return false;
        }

        if (mParams.check_rope_collision && mParams.rope_collision_samples > 0) {
            SphereType cable_sphere;
            cable_sphere.setScale(Eigen::Matrix<ArenaScalar, 3, 1>::Constant(
                static_cast<ArenaScalar>(Scalar(2) * mParams.cable_radius)));
            const Vec3 trolley_pos = state.getTrolleyPosition();
            for (std::size_t i = 0; i < mParams.rope_collision_samples; ++i) {
                const Scalar ratio = static_cast<Scalar>(i) /
                                     static_cast<Scalar>(mParams.rope_collision_samples);
                const Vec3 pt = (Scalar(1) - ratio) * trolley_pos + ratio * ball_pos;
                cable_sphere.setPosition(Eigen::Matrix<ArenaScalar, 3, 1>(
                    static_cast<ArenaScalar>(pt.x()),
                    static_cast<ArenaScalar>(pt.y()),
                    static_cast<ArenaScalar>(pt.z())));
                if (mArena->checkCollision(cable_sphere)) {
                    return false;
                }
            }
        }

        return true;
    }

    Scalar goalPositionDistance(const State& state) const {
        const Scalar ell = mModel->getParameters().ell;
        return (state.getBallPosition2D(ell) - mGoalState.getBallPosition2D(ell)).norm();
    }

    bool isGoalReached(const State& state) const {
        const Scalar pos_err = goalPositionDistance(state);
        if (pos_err > mParams.goal_pos_tolerance) {
            return false;
        }

        if (mParams.require_goal_velocity_check) {
            const Scalar ell = mModel->getParameters().ell;
            const Scalar load_speed = state.getBallVelocity(ell).norm();
            const Scalar trolley_speed = state.getTrolleyVelocity2D().norm();
            if (load_speed > mParams.goal_vel_tolerance ||
                trolley_speed > mParams.goal_vel_tolerance) {
                return false;
            }
        }

        if (mParams.require_goal_swing_check) {
            const Scalar swing_mag = state.getCompositeSwingAngle();
            const Scalar omega_mag = state.getAngularSwingSpeed();
            if (swing_mag > mParams.goal_swing_tolerance ||
                omega_mag > mParams.goal_omega_tolerance) {
                return false;
            }
        }

        return true;
    }

private:
    ModelPtr mModel;
    SolverPtr mSolver;
    ArenaPtr mArena;
    Parameters mParams;
    TreePtr mTree;

    State mStartState = State::Zero();
    State mGoalState = State::Zero();
    NodePtr mGoalNode = nullptr;
    NodePtr mClosestNode = nullptr;
    Scalar mBestGoalDist = std::numeric_limits<Scalar>::infinity();
    std::size_t mIterations = 0;
    std::size_t mCollisionRejections = 0;
    std::size_t mTotalDistanceEvals = 0;
    std::size_t mValidExtensions = 0;

    struct SSTWitness {
        State sample = State::Zero();
        NodePtr rep = nullptr;
    };
    std::vector<SSTWitness> mSSTWitnesses{};
    std::size_t mPendingSSTWitnessIdx = 0;

    std::mt19937 mRng;

    template <typename DistFn>
    NodePtr findNearestCellPartitioned(const NodeData& query, DistFn&& dist_fn) const {
        if (mTree->empty()) {
            return nullptr;
        }
        const auto& nodes = mTree->getNodes();
        if (nodes.size() == 1) {
            return nodes.front();
        }

        const std::size_t radix = std::max<std::size_t>(2, mParams.cell_grid_radix);
        const Scalar span_x = std::max(mParams.x_max - mParams.x_min, Scalar(1e-3));
        const Scalar span_y = std::max(mParams.y_max - mParams.y_min, Scalar(1e-3));

        auto getCellCoords = [&](const State& s) -> std::pair<int, int> {
            int cx = static_cast<int>(((s.getX() - mParams.x_min) / span_x) * static_cast<Scalar>(radix));
            int cy = static_cast<int>(((s.getY() - mParams.y_min) / span_y) * static_cast<Scalar>(radix));
            cx = std::clamp(cx, 0, static_cast<int>(radix) - 1);
            cy = std::clamp(cy, 0, static_cast<int>(radix) - 1);
            return {cx, cy};
        };

        const auto [q_cx, q_cy] = getCellCoords(query.state);

        NodePtr best_in_cell = nullptr;
        Scalar best_cell_dist = std::numeric_limits<Scalar>::infinity();
        NodePtr best_adj = nullptr;
        Scalar best_adj_dist = std::numeric_limits<Scalar>::infinity();

        for (std::size_t i = 0; i < nodes.size(); ++i) {
            const auto [nx, ny] = getCellCoords(nodes[i]->getData().state);
            const int dx = std::abs(nx - q_cx);
            const int dy = std::abs(ny - q_cy);
            if (dx == 0 && dy == 0) {
                const Scalar d = dist_fn(nodes[i]->getData(), query);
                if (d < best_cell_dist) {
                    best_cell_dist = d;
                    best_in_cell = nodes[i];
                }
            } else if (best_in_cell == nullptr && dx <= 1 && dy <= 1) {
                const Scalar d = dist_fn(nodes[i]->getData(), query);
                if (d < best_adj_dist) {
                    best_adj_dist = d;
                    best_adj = nodes[i];
                }
            }
        }

        if (best_in_cell) {
            return best_in_cell;
        }
        if (best_adj) {
            return best_adj;
        }
        return mTree->findNearest(query, std::forward<DistFn>(dist_fn));
    }

    Vec2 computeDeterministicReferenceForce(const State& s_cand, const State& s_query) const {
        const Scalar L_0 = Scalar(2.0);
        const Scalar v_0 = std::max(mParams.max_velocity, Scalar(1e-3));
        const Scalar ell = mModel->getParameters().ell;
        const Vec3 v_load3 = s_cand.getBallVelocity(ell);
        const Vec2 v_sway = (Vec2(v_load3.x(), v_load3.y()) - s_cand.getTrolleyVelocity2D()) / v_0;

        const Vec2 dp = (s_query.getTrolleyPosition2D() - s_cand.getTrolleyPosition2D()) / L_0;
        const Vec2 dv = (s_query.getTrolleyVelocity2D() - s_cand.getTrolleyVelocity2D()) / v_0;
        const Vec2 dir_raw = mParams.w_pos * dp + mParams.w_vel * (dv + Scalar(0.50) * v_sway);
        const Scalar norm = dir_raw.norm();
        if (norm <= Scalar(1e-6)) {
            return Vec2::Zero();
        }
        return (Scalar(0.65) * mParams.max_force / norm) * dir_raw;
    }

    template <typename DistFn>
    NodePtr findNearestSSTBestFirst(const NodeData& query, DistFn&& dist_fn) const {
        if (mTree->empty()) {
            return nullptr;
        }
        const Scalar L_0 = Scalar(2.0);
        const auto& nodes = mTree->getNodes();
        NodePtr best_in_ball = nullptr;
        Scalar best_cost_in_ball = std::numeric_limits<Scalar>::infinity();
        NodePtr closest_active = nullptr;
        Scalar closest_dist = std::numeric_limits<Scalar>::infinity();

        for (std::size_t i = 0; i < nodes.size(); ++i) {
            const NodeData& nd = nodes[i]->getData();
            if (!nd.sst_active) {
                continue;
            }
            const Scalar d = L_0 * dist_fn(nd, query);
            if (d < closest_dist) {
                closest_dist = d;
                closest_active = nodes[i];
            }
            if (d <= mParams.sst_delta_bn) {
                if (nd.cost < best_cost_in_ball) {
                    best_cost_in_ball = nd.cost;
                    best_in_ball = nodes[i];
                }
            }
        }
        return best_in_ball ? best_in_ball : (closest_active ? closest_active : nodes.front());
    }

    bool checkSSTWitnessDominanceAndPrepare(const State& x_new, Scalar new_cost) {
        const Scalar L_0 = Scalar(2.0);
        const Scalar v_0 = std::max(mParams.max_velocity, Scalar(1e-3));
        std::size_t best_w_idx = mSSTWitnesses.size();
        Scalar best_w_dist = std::numeric_limits<Scalar>::infinity();

        for (std::size_t i = 0; i < mSSTWitnesses.size(); ++i) {
            ++mTotalDistanceEvals;
            const Scalar d = L_0 * x_new.phaseDistanceTo(
                mSSTWitnesses[i].sample, mParams.w_pos, mParams.w_vel, L_0, v_0);
            if (d < best_w_dist) {
                best_w_dist = d;
                best_w_idx = i;
            }
        }

        if (best_w_idx == mSSTWitnesses.size() || best_w_dist > mParams.sst_delta_s) {
            mSSTWitnesses.push_back(SSTWitness{x_new, nullptr});
            mPendingSSTWitnessIdx = mSSTWitnesses.size() - 1;
            return true;
        }

        const NodePtr& rep = mSSTWitnesses[best_w_idx].rep;
        if (!rep || new_cost < rep->getData().cost) {
            mPendingSSTWitnessIdx = best_w_idx;
            return true;
        }
        return false;
    }

    void commitSSTNodeAndPruneWitness(const NodePtr& parent_node, const NodePtr& new_node) {
        if (parent_node) {
            ++parent_node->getData().sst_children;
        }
        if (mPendingSSTWitnessIdx >= mSSTWitnesses.size()) {
            return;
        }
        NodePtr old_rep = mSSTWitnesses[mPendingSSTWitnessIdx].rep;
        mSSTWitnesses[mPendingSSTWitnessIdx].rep = new_node;

        if (old_rep && old_rep != new_node) {
            old_rep->getData().sst_active = false;
            NodePtr curr = old_rep;
            while (curr && !curr->getData().sst_active &&
                   curr->getData().sst_children == 0 && !curr->isRoot()) {
                NodePtr p = curr->getParent();
                if (p && p->getData().sst_children > 0) {
                    --p->getData().sst_children;
                }
                curr = p;
            }
        }
    }

    std::size_t countSSTActiveNodes() const {
        const auto& nodes = mTree->getNodes();
        std::size_t count = 0;
        for (const auto& n : nodes) {
            if (n && n->getData().sst_active) {
                ++count;
            }
        }
        return count;
    }

    std::size_t countSSTTreeNodes() const {
        const auto& nodes = mTree->getNodes();
        std::size_t count = 0;
        for (const auto& n : nodes) {
            if (n && (n->getData().sst_active || n->getData().sst_children > 0 || n->isRoot())) {
                ++count;
            }
        }
        return count;
    }

    Scalar lateralCorridorOffset(const Vec2& p, Vec2* e_perp_out = nullptr, Vec2* e_par_out = nullptr) const {
        const Vec2 p_start = mStartState.getTrolleyPosition2D();
        const Vec2 p_goal  = mGoalState.getTrolleyPosition2D();
        const Vec2 chord   = p_goal - p_start;
        const Scalar len   = chord.norm();
        if (len <= Scalar(1e-6)) {
            if (e_perp_out) *e_perp_out = Vec2(Scalar(0), Scalar(1));
            if (e_par_out)  *e_par_out  = Vec2(Scalar(1), Scalar(0));
            return Scalar(0);
        }
        const Vec2 e_par  = chord / len;
        const Vec2 e_perp(-e_par.y(), e_par.x());
        if (e_perp_out) *e_perp_out = e_perp;
        if (e_par_out)  *e_par_out  = e_par;
        return e_perp.dot(p - p_start);
    }

    Scalar softCorridorPenalty(const Vec2& p, Scalar L_0) const {
        if (!mParams.enable_soft_corridor || mParams.corridor_penalty_weight <= Scalar(0)) {
            return Scalar(0);
        }
        const Scalar d_lat = std::abs(lateralCorridorOffset(p));
        const Scalar excess = std::max(Scalar(0), d_lat - mParams.corridor_half_width);
        return mParams.corridor_penalty_weight * (excess / L_0);
    }

    State sampleRandomPhaseState() {
        std::uniform_real_distribution<Scalar> unit_dist(Scalar(0), Scalar(1));
        const Vec2 p_goal = mGoalState.getTrolleyPosition2D();

        if (unit_dist(mRng) < mParams.goal_bias) {
            if (mParams.w_vel > Scalar(0) && mBestGoalDist > mParams.near_goal_radius) {
                const Vec2 p_ref = mClosestNode
                    ? mClosestNode->getData().state.getTrolleyPosition2D()
                    : mStartState.getTrolleyPosition2D();
                const Vec2 d_goal = p_goal - p_ref;
                const Scalar norm = d_goal.norm();
                if (norm > Scalar(1e-6)) {
                    const Vec2 v_flow = Scalar(0.40) * mParams.max_velocity * (d_goal / norm);
                    return State::FromTrolleyPhase(p_goal.x(), p_goal.y(), v_flow.x(), v_flow.y());
                }
            }
            return mGoalState;
        }

        std::uniform_real_distribution<Scalar> x_dist(mParams.x_min, mParams.x_max);
        std::uniform_real_distribution<Scalar> y_dist(mParams.y_min, mParams.y_max);
        Scalar rx = x_dist(mRng);
        Scalar ry = y_dist(mRng);
        if (mParams.enable_soft_corridor && unit_dist(mRng) < mParams.corridor_sample_prob) {
            const Vec2 p_start = mStartState.getTrolleyPosition2D();
            const Vec2 chord   = p_goal - p_start;
            const Scalar len   = chord.norm();
            if (len > Scalar(1e-6)) {
                const Vec2 e_par  = chord / len;
                const Vec2 e_perp(-e_par.y(), e_par.x());
                std::uniform_real_distribution<Scalar> s_dist(Scalar(-0.25), len + Scalar(0.25));
                std::uniform_real_distribution<Scalar> d_dist(-mParams.corridor_half_width, mParams.corridor_half_width);
                const Vec2 p_corr = p_start + s_dist(mRng) * e_par + d_dist(mRng) * e_perp;
                rx = std::clamp(p_corr.x(), mParams.x_min, mParams.x_max);
                ry = std::clamp(p_corr.y(), mParams.y_min, mParams.y_max);
            }
        }
        const Scalar uv1 = unit_dist(mRng);
        const Scalar uv2 = unit_dist(mRng);

        const Vec2 p_sample(rx, ry);
        const Vec2 to_goal = p_goal - p_sample;
        const Scalar dist_goal = to_goal.norm();
        if (dist_goal <= Scalar(1e-6)) {
            return State::FromTrolleyPhase(rx, ry, Scalar(0), Scalar(0));
        }

        const Scalar heading = std::atan2(to_goal.y(), to_goal.x());
        const Scalar delta_angle = (Scalar(2) * uv2 - Scalar(1)) * Scalar(M_PI / 3.0);
        Scalar speed_mag = (Scalar(0.20) + Scalar(0.40) * uv1) * mParams.max_velocity;
        if (mParams.near_goal_radius > Scalar(1e-6) && dist_goal < mParams.near_goal_radius) {
            speed_mag *= (dist_goal / mParams.near_goal_radius);
        }

        const Scalar vx = speed_mag * std::cos(heading + delta_angle);
        const Scalar vy = speed_mag * std::sin(heading + delta_angle);
        return State::FromTrolleyPhase(rx, ry, vx, vy);
    }

    Scalar sampleForceDuration() {
        if (mParams.max_duration <= mParams.min_duration) {
            return mParams.min_duration;
        }
        std::uniform_real_distribution<Scalar> dur_dist(mParams.min_duration, mParams.max_duration);
        return dur_dist(mRng);
    }

    Input clampForceNorm(Scalar fx, Scalar fy) const {
        const Scalar norm = std::hypot(fx, fy);
        if (norm > mParams.max_force && norm > Scalar(1e-12)) {
            const Scalar scale = mParams.max_force / norm;
            fx *= scale;
            fy *= scale;
        }
        return Input::FromParts(fx, fy);
    }

    Input sampleTargetForce(const NodeData& parent_data, const State& x_rand) {
        const State& x_near = parent_data.state;
        std::uniform_real_distribution<Scalar> unit_dist(Scalar(0), Scalar(1));

        if (mParams.enable_near_goal_braking) {
            const Scalar dist_to_goal = goalPositionDistance(x_near);
            if (dist_to_goal <= mParams.near_goal_radius &&
                unit_dist(mRng) < mParams.near_goal_zero_force_prob) {
                const Scalar ell = mModel->getParameters().ell;
                const Vec2 dp_goal = Scalar(0.5) * (mGoalState.getTrolleyPosition2D() - x_near.getTrolleyPosition2D())
                                   + Scalar(0.5) * (mGoalState.getBallPosition2D(ell) - x_near.getBallPosition2D(ell));
                const Vec2 v_near  = x_near.getTrolleyVelocity2D();
                const Vec3 v_load3 = x_near.getBallVelocity(ell);
                const Vec2 v_load(v_load3.x(), v_load3.y());
                const Vec2 u_brake = Scalar(0.40) * dp_goal - Scalar(0.35) * v_near - Scalar(0.15) * v_load;
                return clampForceNorm(u_brake.x(), u_brake.y());
            }
        }

        std::uniform_real_distribution<Scalar> f_dist(-mParams.max_force, mParams.max_force);
        const Scalar u_steer = unit_dist(mRng);
        const Scalar u_mag   = unit_dist(mRng);
        Scalar fx_rand = f_dist(mRng);
        Scalar fy_rand = f_dist(mRng);

        Vec2 u_corr_restore = Vec2::Zero();
        if (mParams.enable_soft_corridor && mParams.corridor_penalty_weight > Scalar(0)) {
            Vec2 e_perp = Vec2::Zero();
            const Scalar d_lat = lateralCorridorOffset(x_near.getTrolleyPosition2D(), &e_perp);
            const Scalar abs_lat = std::abs(d_lat);
            if (abs_lat > mParams.corridor_half_width) {
                const Scalar ratio = std::min(
                    Scalar(1),
                    (abs_lat - mParams.corridor_half_width) / std::max(mParams.corridor_half_width, Scalar(0.25)));
                const Scalar sign_lat = (d_lat >= Scalar(0)) ? Scalar(1) : Scalar(-1);
                u_corr_restore = -sign_lat * (Scalar(0.35) * ratio * mParams.max_force) * e_perp;
                fx_rand += u_corr_restore.x();
                fy_rand += u_corr_restore.y();
            }
        }

        if (mParams.use_pd_local_steering) {
            const Scalar ell = mModel->getParameters().ell;
            const Vec2 dp = x_rand.getTrolleyPosition2D() - x_near.getTrolleyPosition2D();
            const Vec2 v_near = x_near.getTrolleyVelocity2D();
            const Vec2 sway_disp = x_near.getBallPosition2D(ell) - x_near.getTrolleyPosition2D();
            const Vec2 u_pd = Scalar(0.35) * dp - Scalar(0.28) * v_near + Scalar(0.45) * sway_disp + u_corr_restore;
            return clampForceNorm(u_pd.x() + Scalar(0.12) * fx_rand,
                                  u_pd.y() + Scalar(0.12) * fy_rand);
        }

        if (mParams.force_direction_bias > Scalar(0) &&
            u_steer < mParams.force_direction_bias) {
            const Scalar L_0 = Scalar(2.0);
            const Scalar v_0 = std::max(mParams.max_velocity, Scalar(1e-3));
            const Scalar ell = mModel->getParameters().ell;
            const Vec3 v_load3 = x_near.getBallVelocity(ell);
            const Vec2 v_sway = (Vec2(v_load3.x(), v_load3.y()) - x_near.getTrolleyVelocity2D()) / v_0;

            const Vec2 dp = (x_rand.getTrolleyPosition2D() - x_near.getTrolleyPosition2D()) / L_0;
            const Vec2 dv = (x_rand.getTrolleyVelocity2D() - x_near.getTrolleyVelocity2D()) / v_0;
            const Vec2 dir_raw = mParams.w_pos * dp + mParams.w_vel * (dv + Scalar(0.50) * v_sway);
            const Scalar norm = dir_raw.norm();
            if (norm > Scalar(1e-6)) {
                const Vec2 dir = dir_raw / norm;
                const Scalar mag = (Scalar(0.30) + Scalar(0.70) * u_mag) * mParams.max_force;
                const Scalar fx = dir.x() * mag + Scalar(0.25) * fx_rand + u_corr_restore.x();
                const Scalar fy = dir.y() * mag + Scalar(0.25) * fy_rand + u_corr_restore.y();
                return clampForceNorm(fx, fy);
            }
        }

        return clampForceNorm(fx_rand, fy_rand);
    }
};

} // namespace rrt
} // namespace gantry

#endif // GANTRY_RRT_PLANNER_HPP

