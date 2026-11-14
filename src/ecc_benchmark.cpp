#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "crane_rrt/arena/ecc_arenas.hpp"
#include "crane_rrt/arena/sphere.hpp"
#include "crane_rrt/dynamic_model.hpp"
#include "crane_rrt/numerical_solver.hpp"
#include "crane_rrt/rrt/rrt_planner.hpp"
#include "crane_rrt/simulator.hpp"

namespace {

using Planner   = gantry::rrt::RRTPlanner<double, float>;
using Model     = Planner::Model;
using Solver    = Planner::Solver;
using State     = Planner::State;
using Input     = Planner::Input;
using ArenaSpec = arena::ECCArenaSpec<float>;

Model::Parameters makeNominalCraneParams() {
    Model::Parameters mp;
    mp.m_x = 0.1;
    mp.m_y = 0.1;
    mp.m   = 0.1;
    mp.g   = 9.81;
    mp.ell = 1.0;
    mp.D_x = 0.05;
    mp.D_y = 0.05;
    return mp;
}

Planner::Parameters makeBaseRRTParams() {
    Planner::Parameters p;
    p.x_min = -4.5;
    p.x_max =  4.5;
    p.y_min = -4.5;
    p.y_max =  4.5;
    p.max_velocity    = 2.0;
    p.max_swing_angle = 0.4363; // 25 deg
    p.max_force       = 0.5;
    p.min_duration    = 0.20;
    p.max_duration    = 0.50;
    p.integration_dt  = 0.02;
    p.ball_radius     = 0.12;
    p.cable_radius    = 0.08;
    p.check_rope_collision   = true;
    p.rope_collision_samples = 10;
    p.enable_force_smoothing = true;
    p.force_direction_bias   = 0.35;
    p.w_pos = 0.85;
    p.w_vel = 0.15;
    p.goal_bias       = 0.15;
    p.min_search_step = 1;
    p.max_search_step = 15;
    p.enable_near_goal_braking  = true;
    p.near_goal_radius          = 0.85;
    p.near_goal_zero_force_prob = 0.60;
    p.goal_pos_tolerance   = 0.35;
    p.goal_vel_tolerance   = 0.55;
    p.goal_swing_tolerance = 0.22; // ~12.6 deg
    p.goal_omega_tolerance = 0.55;
    p.require_goal_velocity_check = true;
    p.require_goal_swing_check    = true;
    p.max_iterations = 20000;
    p.max_nodes      = 6000;
    p.max_planning_time_sec = 0.0;
    return p;
}

struct TrajectoryMetrics {
    double max_swing_deg = 0.0;
    double rms_swing_deg = 0.0;
    double term_swing_deg = 0.0;
    double term_omega_rads = 0.0;
    double term_speed_ms = 0.0;
    double term_load_speed_ms = 0.0;
    double path_length_m = 0.0;
    double traj_duration_s = 0.0;
    double rms_force_jerk = 0.0;
};

TrajectoryMetrics computeTrajectoryMetrics(
    const std::vector<State>& states,
    const std::vector<Input>& inputs,
    double dt,
    double ell = 1.0)
{
    TrajectoryMetrics m;
    if (states.empty()) {
        return m;
    }

    double sum_sq_swing = 0.0;
    for (std::size_t i = 0; i < states.size(); ++i) {
        const double tilt_deg = states[i].getCompositeSwingAngle() * 180.0 / M_PI;
        m.max_swing_deg = std::max(m.max_swing_deg, tilt_deg);
        sum_sq_swing += tilt_deg * tilt_deg;
        if (i > 0) {
            m.path_length_m += (states[i].getTrolleyPosition2D() -
                                states[i - 1].getTrolleyPosition2D()).norm();
        }
    }
    m.rms_swing_deg = std::sqrt(sum_sq_swing / static_cast<double>(states.size()));

    const auto& last = states.back();
    m.term_swing_deg     = last.getCompositeSwingAngle() * 180.0 / M_PI;
    m.term_omega_rads    = last.getAngularSwingSpeed();
    m.term_speed_ms      = last.getTrolleyVelocity2D().norm();
    m.term_load_speed_ms = last.getBallVelocity(ell).norm();
    m.traj_duration_s    = static_cast<double>(states.size()) * dt;

    if (inputs.size() > 1) {
        double sum_sq_du = 0.0;
        std::size_t count_du = 0;
        for (std::size_t i = 1; i < inputs.size(); ++i) {
            const double dfx = (inputs[i].getFx() - inputs[i - 1].getFx()) / dt;
            const double dfy = (inputs[i].getFy() - inputs[i - 1].getFy()) / dt;
            sum_sq_du += (dfx * dfx + dfy * dfy);
            ++count_du;
        }
        if (count_du > 0) {
            m.rms_force_jerk = std::sqrt(sum_sq_du / static_cast<double>(count_du));
        }
    }
    return m;
}

struct FineValidationResult {
    bool all_clear = false;
    std::size_t total_violations = 0;
    std::size_t collision_violations = 0;
    std::size_t speed_or_bounds_violations = 0;
    std::size_t swing_violations = 0;
    double max_fine_swing_deg = 0.0;
    double min_cable_tension_n = std::numeric_limits<double>::infinity();
};

FineValidationResult checkFineRollout10x(
    const Planner& planner,
    const Planner::PlanResult& res,
    double dt,
    const std::shared_ptr<Solver>& fine_solver)
{
    FineValidationResult out;
    if (!res.success || res.state_trajectory.size() < 2) {
        return out;
    }

    const auto& S = res.state_trajectory;
    const auto& U = res.input_trajectory;
    const auto& p = planner.getParameters();
    const auto& model = *planner.getModel();
    const int substeps = 10;
    const double h_sub = dt / static_cast<double>(substeps);

    for (std::size_t k = 0; k + 1 < S.size() && k + 1 < U.size(); ++k) {
        State xs = S[k];
        double t_sub = xs.getTime();
        for (int j = 0; j < substeps; ++j) {
            State xn;
            fine_solver->step(xs, U[k + 1], t_sub, h_sub, xn);

            const double sw_rad = xn.getCompositeSwingAngle();
            out.max_fine_swing_deg = std::max(out.max_fine_swing_deg, sw_rad * 180.0 / M_PI);
            out.min_cable_tension_n = std::min(
                out.min_cable_tension_n, model.computeCableTensionClosedForm(xn, U[k + 1]));

            const bool bounds_ok =
                (xn.getX() >= p.x_min && xn.getX() <= p.x_max &&
                 xn.getY() >= p.y_min && xn.getY() <= p.y_max &&
                 xn.getTrolleyVelocity2D().norm() <= p.max_velocity);
            const bool swing_ok = (sw_rad <= p.max_swing_angle);
            const bool full_ok  = planner.isStateValidAndCollisionFree(xn);

            if (!full_ok) {
                ++out.total_violations;
                if (!bounds_ok) {
                    ++out.speed_or_bounds_violations;
                } else if (!swing_ok) {
                    ++out.swing_violations;
                } else {
                    ++out.collision_violations;
                }
            }
            xs = xn;
            t_sub += h_sub;
        }
    }
    out.all_clear = (out.total_violations == 0);
    return out;
}

bool verifyPhysicsAndSimulator() {
    std::cout << "[Check 1] Verifying 4-DOF Crane Dynamics, Lemma 1 Bounds, Cable Tension, and RK4..." << std::endl;

    const auto mp = makeNominalCraneParams();
    Solver::Parameters sp(true, Solver::RKMethod::ClassicalRK4);
    gantry::Simulator::Parameters simp(100.0, false, State::Zero());
    gantry::Simulator sim(simp, mp, sp);

    auto cloned = sim.clone(false);
    if (!cloned || !cloned->getState().isApprox(sim.getState())) {
        std::cerr << "  [FAIL] Simulator::clone(false) failed." << std::endl;
        return false;
    }

    for (int i = 0; i < 50; ++i) {
        sim.step(Input::FromParts(0.5, 0.25));
    }
    const State& s = sim.getState();
    if (s.getX() <= 0.05 || s.getY() <= 0.02 || std::abs(s.getTime() - 0.5) > 1e-6) {
        std::cerr << "  [FAIL] Simulator state progression mismatch: " << s << std::endl;
        return false;
    }

    Solver::Parameters sp_adaptive(true, Solver::RKMethod::CashKarp54);
    gantry::Simulator sim_adaptive(simp, mp, sp_adaptive);
    for (int i = 0; i < 50; ++i) {
        sim_adaptive.step(Input::FromParts(0.5, 0.25));
    }
    if (!s.isApprox(sim_adaptive.getState(), 1e-3)) {
        std::cerr << "  [FAIL] CashKarp54 diverged from ClassicalRK4." << std::endl;
        return false;
    }

    auto model = std::make_shared<Model>();
    model->setParameters(mp);
    Solver solver(model);
    solver.setParameters(Solver::Parameters(false, Solver::RKMethod::ClassicalRK4));

    auto u_tv = [](double t) -> Input {
        return Input::FromParts(0.40 * std::cos(1.5 * t), 0.35 * std::sin(1.2 * t), t);
    };
    auto integrate_tv = [&](double dt) -> State {
        State x_cur = State::Zero();
        const std::size_t N = static_cast<std::size_t>(std::llround(1.0 / dt));
        for (std::size_t k = 0; k < N; ++k) {
            State x_next;
            solver.stepTimeVarying(x_cur, u_tv, k * dt, dt, x_next);
            x_cur = x_next;
        }
        return x_cur;
    };
    const State x_tv_ref = integrate_tv(0.0005);
    const double err_02 = (integrate_tv(0.02).toVector() - x_tv_ref.toVector()).norm();
    const double err_01 = (integrate_tv(0.01).toVector() - x_tv_ref.toVector()).norm();
    if (err_02 / std::max(err_01, 1e-15) < 12.0) {
        std::cerr << "  [FAIL] stepTimeVarying did not exhibit 4th-order convergence." << std::endl;
        return false;
    }

    // Lemma 1 spectral bounds, planar Schur complement identity, and positive cable tension check
    double lmin_env = 1e9, lmax_env = 0.0, kappa_env = 0.0, max_schur_err = 0.0, max_tension_err = 0.0;
    const double theta_max = 25.0 * M_PI / 180.0;
    const double t_min_analytic = model->computeMinGuaranteedCableTension(theta_max, 2.0, 0.5);
    double t_min_sampled = 1e9;

    std::mt19937 rng(20261108);
    std::uniform_real_distribution<double> ang_dist(-theta_max, theta_max);
    std::uniform_real_distribution<double> vel_dist(-2.0, 2.0);
    std::uniform_real_distribution<double> omg_dist(-1.7, 1.7);
    std::uniform_real_distribution<double> frc_dist(-0.5, 0.5);

    for (int i = 0; i < 4000; ++i) {
        const double a = ang_dist(rng);
        const double b = ang_dist(rng);
        if (std::cos(a) * std::cos(b) < std::cos(theta_max)) {
            continue;
        }
        const Eigen::Matrix4d M = model->evaluateMassMatrix(a, b);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(M);
        const double lmin = es.eigenvalues()(0);
        const double lmax = es.eigenvalues()(3);
        lmin_env  = std::min(lmin_env, lmin);
        lmax_env  = std::max(lmax_env, lmax);
        kappa_env = std::max(kappa_env, lmax / lmin);

        const Eigen::Matrix2d Mt = model->computePlanarEffectiveMass(a, b);
        const Eigen::Matrix2d G_schur = M.inverse().topLeftCorner<2, 2>();
        max_schur_err = std::max(max_schur_err, (Mt.inverse() - G_schur).norm());

        State st = State::Zero();
        st.setAlpha(a); st.setBeta(b);
        st.setXdot(vel_dist(rng)); st.setYdot(vel_dist(rng));
        st.setAlphaDot(omg_dist(rng)); st.setBetaDot(omg_dist(rng));
        Input u = Input::FromParts(frc_dist(rng), frc_dist(rng));
        const double t_num = model->computeCableTension(st, u);
        const double t_cf  = model->computeCableTensionClosedForm(st, u);
        max_tension_err = std::max(max_tension_err, std::abs(t_num - t_cf));
        t_min_sampled   = std::min(t_min_sampled, t_cf);
        if (t_cf <= 0.0 || t_cf < t_min_analytic - 1e-12) {
            std::cerr << "  [FAIL] Cable tension positivity bound violated: T = " << t_cf << std::endl;
            return false;
        }
    }
    if (max_schur_err > 1e-12 || max_tension_err > 1e-12) {
        std::cerr << "  [FAIL] Schur complement or closed-form cable tension mismatch." << std::endl;
        return false;
    }

    std::cout << "  [PASS] Lemma 1 envelope (theta <= 25 deg): lambda_min >= "
              << std::fixed << std::setprecision(4) << lmin_env
              << ", lambda_max <= " << lmax_env
              << ", max cond kappa_2(M) = " << kappa_env
              << " | min cable tension >= " << t_min_analytic << " N (sampled min = " << t_min_sampled << " N)"
              << std::endl;
    return true;
}

bool verifySphereAndEllipsoidCollisions() {
    using Scalar = float;
    using Sphere = arena::Sphere<Scalar>;
    using Point  = typename Sphere::Point;

    std::cout << "[Check 2] Verifying 3D Sphere and Ellipsoid Collision Primitives..." << std::endl;

    Sphere s1, s2;
    s1.setPosition(Point(0.0f, 0.0f, 0.0f));
    s1.setScale(Point(1.0f, 1.0f, 1.0f));
    s2.setPosition(Point(1.2f, 0.0f, 0.0f));
    s2.setScale(Point(1.0f, 1.0f, 1.0f));
    if (s1.checkCollision(s2)) {
        std::cerr << "  [FAIL] Separated spheres reported false collision." << std::endl;
        return false;
    }

    Sphere ell;
    ell.setPosition(Point(1.2f, 0.0f, 0.0f));
    ell.setScale(Point(1.0f, 2.0f, 1.0f));
    if (s1.checkCollision(ell)) {
        std::cerr << "  [FAIL] Separated sphere vs ellipsoid reported false collision." << std::endl;
        return false;
    }

    ell.setPosition(Point(0.9f, 0.0f, 0.0f));
    if (!s1.checkCollision(ell)) {
        std::cerr << "  [FAIL] Overlapping sphere vs ellipsoid missed collision." << std::endl;
        return false;
    }

    std::cout << "  [PASS] 3D collision geometry checks passed." << std::endl;
    return true;
}

template <typename TaskFn>
void runParallelTasks(int total_tasks, unsigned int num_threads, const Model::Parameters& mp, TaskFn&& fn) {
    std::atomic<int> next_idx{0};
    const int n_workers = std::max(1, std::min(static_cast<int>(num_threads), total_tasks));
    std::vector<std::thread> workers;
    workers.reserve(n_workers);
    for (int w = 0; w < n_workers; ++w) {
        workers.emplace_back([&]() {
            auto tl_model = std::make_shared<Model>();
            tl_model->setParameters(mp);
            auto tl_solver = std::make_shared<Solver>(tl_model);
            tl_solver->setParameters(Solver::Parameters(false, Solver::RKMethod::ClassicalRK4));
            auto tl_fine = std::make_shared<Solver>(tl_model);
            tl_fine->setParameters(Solver::Parameters(false, Solver::RKMethod::ClassicalRK4));
            while (true) {
                const int i = next_idx.fetch_add(1, std::memory_order_relaxed);
                if (i >= total_tasks) {
                    break;
                }
                fn(i, tl_model, tl_solver, tl_fine);
            }
        });
    }
    for (auto& th : workers) {
        th.join();
    }
}

void runRK4ConvergenceStudy(const std::string& csv_path) {
    std::cout << "\n=== RK4 Step-Size Convergence, Energy Balance & Timing Study ===" << std::endl;

    const auto mp = makeNominalCraneParams();
    auto model = std::make_shared<Model>();
    model->setParameters(mp);

    State x0 = State::Zero();
    x0.setAlpha(0.15);
    x0.setBeta(-0.12);

    const double T_horizon = 5.0;
    const Input u_const = Input::FromParts(0.40, 0.35, 0.0);

    Solver ref_solver(model);
    ref_solver.setParameters(Solver::Parameters(false, Solver::RKMethod::ClassicalRK4));
    State x_ref = x0;
    const double dt_ref = 0.0001;
    const std::size_t N_ref = static_cast<std::size_t>(std::llround(T_horizon / dt_ref));
    for (std::size_t k = 0; k < N_ref; ++k) {
        State x_next;
        ref_solver.step(x_ref, u_const, k * dt_ref, dt_ref, x_next);
        x_ref = x_next;
    }

    const std::vector<double> step_sizes = {0.001, 0.002, 0.005, 0.010, 0.020, 0.050, 0.100, 0.200};
    std::ofstream ofs(csv_path);
    ofs << "dt_s,steps,pos_err_m,swing_err_deg,vel_err_ms,energy_res_j,min_tension_n,max_tension_n,cpu_us_per_step\n";

    Solver rk4_solver(model);
    rk4_solver.setParameters(Solver::Parameters(false, Solver::RKMethod::ClassicalRK4));

    for (double dt : step_sizes) {
        const std::size_t N = static_cast<std::size_t>(std::llround(T_horizon / dt));
        State x_cur = x0;
        const double E0 = model->computeTotalEnergy(x0);
        double work_diss_integral = 0.0;
        double min_tension = 1e9, max_tension = 0.0;

        for (std::size_t k = 0; k < N; ++k) {
            const double t = k * dt;
            State x_mid, x_next;
            rk4_solver.step(x_cur, u_const, t, 0.5 * dt, x_mid);
            rk4_solver.step(x_cur, u_const, t, dt, x_next);
            const double P0 = model->computeMechanicalPower(x_cur, u_const);
            const double Pm = model->computeMechanicalPower(x_mid, u_const);
            const double P1 = model->computeMechanicalPower(x_next, u_const);
            work_diss_integral += (dt / 6.0) * (P0 + 4.0 * Pm + P1);
            const double Tc = model->computeCableTensionClosedForm(x_cur, u_const);
            min_tension = std::min(min_tension, Tc);
            max_tension = std::max(max_tension, Tc);
            x_cur = x_next;
        }

        // Dedicated timing pass over pure RK4 steps
        const std::size_t timing_steps = std::max<std::size_t>(5000, N);
        State x_bench = x0;
        const auto t0 = std::chrono::steady_clock::now();
        for (std::size_t k = 0; k < timing_steps; ++k) {
            State x_next;
            rk4_solver.step(x_bench, u_const, 0.0, dt, x_next);
            x_bench = x_next;
        }
        const auto t1 = std::chrono::steady_clock::now();
        if (x_bench.getX() < -1e12) {
            std::cerr << x_bench.getX();
        }
        const double us_per_step =
            std::chrono::duration<double, std::micro>(t1 - t0).count() / static_cast<double>(timing_steps);

        const double ET = model->computeTotalEnergy(x_cur);
        const double energy_res = std::abs((ET - E0) - work_diss_integral);
        const double pos_err = (x_cur.getTrolleyPosition2D() - x_ref.getTrolleyPosition2D()).norm();
        const double swing_err_deg = std::hypot(x_cur.getAlpha() - x_ref.getAlpha(),
                                                x_cur.getBeta()  - x_ref.getBeta()) * 180.0 / M_PI;
        const double vel_err = (x_cur.getTrolleyVelocity2D() - x_ref.getTrolleyVelocity2D()).norm();

        std::cout << "  h = " << std::setw(6) << std::fixed << std::setprecision(3) << dt
                  << " s | pos_err = " << std::scientific << std::setprecision(2) << pos_err
                  << " m | swing_err = " << swing_err_deg << " deg"
                  << " | E_res = " << energy_res << " J"
                  << " | step_time = " << std::fixed << std::setprecision(2) << us_per_step << " us\n";

        ofs << dt << "," << N << "," << pos_err << "," << swing_err_deg << ","
            << vel_err << "," << energy_res << "," << min_tension << "," << max_tension << ","
            << us_per_step << "\n";
    }
    std::cout << "  Saved CSV: " << csv_path << std::endl;
}

bool runQuickSmokeTest(const std::vector<ArenaSpec>& arenas, unsigned int num_threads) {
    std::cout << "\n=== Quick 4-Arena Smoke Test (M7_ProposedFull, Seed 1001, h_fine = 0.002 s) ===" << std::endl;
    const auto mp = makeNominalCraneParams();
    std::atomic<bool> all_ok{true};

    std::vector<std::string> lines(arenas.size());
    runParallelTasks(static_cast<int>(arenas.size()), num_threads, mp,
        [&](int idx, const auto& tl_model, const auto& tl_solver, const auto& tl_fine) {
            const auto& ar = arenas[static_cast<std::size_t>(idx)];
            auto p = makeBaseRRTParams();
            p.random_seed = 1001;
            if (ar.use_soft_corridor) {
                p.enable_soft_corridor = true;
                p.corridor_half_width  = ar.corridor_half_width;
            }
            Planner planner(tl_model, tl_solver, ar.arena, p);
            const auto res = planner.plan(ar.start, ar.goal);
            const auto tm  = computeTrajectoryMetrics(res.state_trajectory, res.input_trajectory, p.integration_dt, mp.ell);
            const auto fv  = checkFineRollout10x(planner, res, p.integration_dt, tl_fine);
            if (!res.success || !fv.all_clear) {
                all_ok.store(false);
            }
            std::ostringstream oss;
            oss << "  " << std::left << std::setw(30) << ar.id
                << " | Succ: " << (res.success ? "YES" : "NO ")
                << " | Fine(2ms): " << (fv.all_clear ? "CLEAR" : "VIOL ")
                << " | Nodes: " << std::right << std::setw(4) << res.tree_size
                << " | Time: " << std::setw(6) << std::fixed << std::setprecision(1) << (res.planning_time_sec * 1000.0) << " ms"
                << " | MaxSwing: " << std::setw(5) << std::setprecision(2) << tm.max_swing_deg << " deg"
                << " | MinTension: " << std::setw(5) << std::setprecision(3) << fv.min_cable_tension_n << " N";
            lines[static_cast<std::size_t>(idx)] = oss.str();
        });

    for (const auto& ln : lines) {
        std::cout << ln << "\n";
    }
    return all_ok.load();
}

void runHeldOutTuning(const std::vector<ArenaSpec>& arenas,
                      unsigned int /*num_threads*/,
                      const std::string& csv_path)
{
    std::cout << "\n=== Held-Out Hyperparameter Tuning (Seeds 2001..2005 x 4 Arenas = 20 Runs, Single-Thread Per Trial) ===" << std::endl;
    const auto mp = makeNominalCraneParams();
    std::ofstream ofs(csv_path);
    ofs << "study,param_label,succ_out_of_20,mean_nodes_succ,mean_t_succ_ms\n";

    std::cout << "  [1] Tuning force-history weight w_u for M7_ForceHist (20,000-iter / 6,000-node budget):" << std::endl;
    const std::vector<double> wu_grid = {0.02, 0.05, 0.10, 0.15, 0.20};
    double best_wu = 0.05;
    int best_wu_succ = -1;
    double best_wu_nodes = 1e18;
    double best_wu_time = 1e18;

    for (double wu : wu_grid) {
        std::vector<int> succ(20, 0);
        std::vector<double> nodes(20, 0.0);
        std::vector<double> t_ms(20, 0.0);
        runParallelTasks(20, 1u, mp, [&](int idx, const auto& tl_model, const auto& tl_solver, const auto&) {
            const auto& ar = arenas[static_cast<std::size_t>(idx / 5)];
            auto p = makeBaseRRTParams();
            p.random_seed = static_cast<unsigned int>(2001 + (idx % 5));
            p.force_history_nns_weight = wu;
            if (ar.use_soft_corridor) {
                p.enable_soft_corridor = true;
                p.corridor_half_width  = ar.corridor_half_width;
            }
            Planner planner(tl_model, tl_solver, ar.arena, p);
            const auto res = planner.plan(ar.start, ar.goal);
            if (res.success) {
                succ[static_cast<std::size_t>(idx)]  = 1;
                nodes[static_cast<std::size_t>(idx)] = static_cast<double>(res.tree_size);
                t_ms[static_cast<std::size_t>(idx)]  = res.planning_time_sec * 1000.0;
            }
        });
        int succ_cnt = 0;
        double sum_n = 0.0;
        double sum_t = 0.0;
        for (int i = 0; i < 20; ++i) {
            succ_cnt += succ[static_cast<std::size_t>(i)];
            sum_n    += nodes[static_cast<std::size_t>(i)];
            sum_t    += t_ms[static_cast<std::size_t>(i)];
        }
        const double mean_n = (succ_cnt > 0) ? (sum_n / succ_cnt) : 99999.0;
        const double mean_t = (succ_cnt > 0) ? (sum_t / succ_cnt) : 9999.0;
        std::cout << "      w_u = " << std::fixed << std::setprecision(2) << wu
                  << " -> Succ: " << succ_cnt << "/20 | Mean Nodes: " << std::setprecision(1) << mean_n
                  << " | Mean T_succ: " << std::setprecision(2) << mean_t << " ms\n";
        ofs << "M7_ForceHist_wu,w_u=" << std::setprecision(2) << wu << ","
            << succ_cnt << "," << std::setprecision(2) << mean_n << "," << mean_t << "\n";
        if (succ_cnt > best_wu_succ ||
            (succ_cnt == best_wu_succ && (mean_n < best_wu_nodes - 1e-9 ||
             (std::abs(mean_n - best_wu_nodes) <= 1e-9 && mean_t < best_wu_time)))) {
            best_wu_succ  = succ_cnt;
            best_wu_nodes = mean_n;
            best_wu_time  = mean_t;
            best_wu       = wu;
        }
    }
    std::cout << "      Selected w_u = " << std::fixed << std::setprecision(2) << best_wu << "\n";

    std::cout << "  [2] Tuning (delta_bn, delta_s) for B4_SST (3.0 s wall-clock budget, single-thread per trial):" << std::endl;
    struct Pair { double bn; double s; };
    const std::vector<Pair> sst_grid = {{0.35, 0.15}, {0.45, 0.18}, {0.55, 0.22}, {0.65, 0.25}, {0.80, 0.30}};
    Pair best_sst{0.45, 0.18};
    int best_sst_succ = -1;
    double best_sst_time = 1e18;

    for (const auto& gp : sst_grid) {
        std::vector<int> succ(20, 0);
        std::vector<double> nodes(20, 0.0);
        std::vector<double> t_ms(20, 0.0);
        runParallelTasks(20, 1u, mp, [&](int idx, const auto& tl_model, const auto& tl_solver, const auto&) {
            const auto& ar = arenas[static_cast<std::size_t>(idx / 5)];
            auto p = makeBaseRRTParams();
            p.random_seed = static_cast<unsigned int>(2001 + (idx % 5));
            p.use_sst_planner = true;
            p.sst_delta_bn = gp.bn;
            p.sst_delta_s  = gp.s;
            p.w_pos = 1.0;
            p.w_vel = 0.0;
            p.force_direction_bias = 0.0;
            p.min_search_step = 1;
            p.max_search_step = 1;
            p.max_iterations  = 25000;
            p.max_nodes       = 6000;
            p.max_planning_time_sec = 3.0;
            if (ar.use_soft_corridor) {
                p.enable_soft_corridor = true;
                p.corridor_half_width  = ar.corridor_half_width;
            }
            Planner planner(tl_model, tl_solver, ar.arena, p);
            const auto res = planner.plan(ar.start, ar.goal);
            if (res.success) {
                succ[static_cast<std::size_t>(idx)]  = 1;
                nodes[static_cast<std::size_t>(idx)] = static_cast<double>(res.tree_size);
                t_ms[static_cast<std::size_t>(idx)]  = res.planning_time_sec * 1000.0;
            }
        });
        int succ_cnt = 0;
        double sum_n = 0.0;
        double sum_t = 0.0;
        for (int i = 0; i < 20; ++i) {
            succ_cnt += succ[static_cast<std::size_t>(i)];
            sum_n    += nodes[static_cast<std::size_t>(i)];
            sum_t    += t_ms[static_cast<std::size_t>(i)];
        }
        const double mean_n = (succ_cnt > 0) ? (sum_n / succ_cnt) : 99999.0;
        const double mean_t = (succ_cnt > 0) ? (sum_t / succ_cnt) : 9999.0;
        std::cout << "      (delta_bn=" << std::fixed << std::setprecision(2) << gp.bn
                  << ", delta_s=" << gp.s << ") -> Succ: " << succ_cnt
                  << "/20 | Mean T_succ: " << std::setprecision(1) << mean_t << " ms\n";
        ofs << "B4_SST_radii,bn=" << std::setprecision(2) << gp.bn << "_s=" << gp.s << ","
            << succ_cnt << "," << mean_n << "," << mean_t << "\n";
        if (succ_cnt > best_sst_succ || (succ_cnt == best_sst_succ && mean_t < best_sst_time)) {
            best_sst_succ = succ_cnt;
            best_sst_time = mean_t;
            best_sst      = gp;
        }
    }
    std::cout << "      Selected (delta_bn, delta_s) = ("
              << std::fixed << std::setprecision(2) << best_sst.bn << " m, " << best_sst.s << " m)\n";
    std::cout << "  Saved CSV: " << csv_path << std::endl;
}

struct MethodSpec {
    std::string id;
    std::string table_label;
    Planner::Parameters params;
};

std::vector<MethodSpec> buildTable2Methods() {
    std::vector<MethodSpec> methods;

    {
        auto p = makeBaseRRTParams();
        p.min_duration = 0.10; p.max_duration = 0.12;
        p.enable_force_smoothing = false;
        p.force_direction_bias = 0.0;
        p.w_pos = 1.0; p.w_vel = 0.0;
        p.goal_bias = 0.0;
        p.min_search_step = 1; p.max_search_step = 1;
        p.enable_near_goal_braking = false;
        methods.push_back({"M1_Vanilla", "M1: Vanilla RRT", p});
    }
    {
        auto p = makeBaseRRTParams();
        p.enable_force_smoothing = false;
        p.force_direction_bias = 0.0;
        p.w_pos = 1.0; p.w_vel = 0.0;
        p.goal_bias = 0.0;
        p.min_search_step = 1; p.max_search_step = 1;
        p.enable_near_goal_braking = false;
        methods.push_back({"M2_SustainedTf", "M2: + Horizon Tf", p});
    }
    {
        auto p = makeBaseRRTParams();
        p.force_direction_bias = 0.0;
        p.w_pos = 1.0; p.w_vel = 0.0;
        p.goal_bias = 0.0;
        p.min_search_step = 1; p.max_search_step = 1;
        p.enable_near_goal_braking = false;
        methods.push_back({"M3_ForceSmooth", "M3: + ZOH Smooth", p});
    }
    {
        auto p = makeBaseRRTParams();
        p.force_direction_bias = 0.0;
        p.w_pos = 1.0; p.w_vel = 0.0;
        p.min_search_step = 1; p.max_search_step = 1;
        methods.push_back({"M4_GoalBiasBrake", "M4: + Goal & Brake", p});
    }
    {
        auto p = makeBaseRRTParams();
        p.min_search_step = 1; p.max_search_step = 1;
        methods.push_back({"M5_4DPhaseOnly", "M5: + 4D (s=1)", p});
    }
    {
        auto p = makeBaseRRTParams();
        p.force_direction_bias = 0.0;
        p.w_pos = 1.0; p.w_vel = 0.0;
        methods.push_back({"M6_StrideOnly", "M6: + Stride (w_v=0)", p});
    }
    {
        auto p = makeBaseRRTParams();
        methods.push_back({"M7_ProposedFull", "M7: Proposed Full", p});
    }
    {
        auto p = makeBaseRRTParams();
        p.use_equal_budget_random_subset = true;
        methods.push_back({"M7_RandSub", "M7-RandSub (Equal-Kq)", p});
    }
    {
        auto p = makeBaseRRTParams();
        p.force_history_nns_weight = 0.05;
        methods.push_back({"M7_ForceHist", "M7-ForceHist (w_u=0.05)", p});
    }
    {
        auto p = makeBaseRRTParams();
        p.enable_force_smoothing = false;
        p.force_direction_bias = 0.0;
        p.w_pos = 1.0; p.w_vel = 0.0;
        p.min_search_step = 1; p.max_search_step = 1;
        p.num_control_samples = 5;
        p.enable_near_goal_braking = false;
        methods.push_back({"B1_LaValleBestK", "B1: Kinodynamic RRT (K=5)", p});
    }
    {
        auto p = makeBaseRRTParams();
        p.force_direction_bias = 0.0;
        p.w_pos = 1.0; p.w_vel = 0.0;
        p.min_search_step = 1; p.max_search_step = 1;
        p.use_cell_partition_nns = true;
        p.cell_grid_radix = 3;
        methods.push_back({"B2_ZhouCellRRT", "B2: Cell-RRT (3x3)", p});
    }
    {
        auto p = makeBaseRRTParams();
        p.force_direction_bias = 0.0;
        p.w_pos = 1.0; p.w_vel = 0.0;
        p.min_search_step = 1; p.max_search_step = 1;
        p.use_pd_local_steering = true;
        methods.push_back({"B3_PDSteerRRT", "B3: PD Anti-Sway", p});
    }
    {
        auto p = makeBaseRRTParams();
        p.use_sst_planner = true;
        p.sst_delta_bn = 0.45;
        p.sst_delta_s  = 0.18;
        p.w_pos = 1.0; p.w_vel = 0.0;
        p.force_direction_bias = 0.0;
        p.min_search_step = 1; p.max_search_step = 1;
        p.max_iterations  = 25000;
        p.max_nodes       = 6000;
        p.max_planning_time_sec = 3.0;
        methods.push_back({"B4_SST", "B4: SST (Euclidean)", p});
    }
    {
        auto p = makeBaseRRTParams();
        p.use_sst_planner = true;
        p.sst_delta_bn = 0.45;
        p.sst_delta_s  = 0.18;
        p.w_pos = 0.85; p.w_vel = 0.15;
        p.force_direction_bias = 0.35;
        p.min_search_step = 1; p.max_search_step = 1;
        p.max_iterations  = 25000;
        p.max_nodes       = 6000;
        p.max_planning_time_sec = 3.0;
        methods.push_back({"B4_SST_4D", "B4-4D: SST + 4D Steer", p});
    }
    return methods;
}

struct TrialRecord {
    std::size_t arena_idx = 0;
    bool success = false;
    bool fine_clear = false;
    std::size_t fine_col_viol = 0;
    std::size_t fine_vel_viol = 0;
    std::size_t fine_sw_viol = 0;
    double max_fine_swing_deg = 0.0;
    double min_cable_tension_n = 0.0;
    double nodes = 0.0;
    double time_ms = 0.0;
    double traj_dur_s = 0.0;
    double ver = 0.0;
    double depvb = 0.0;
    double max_sw = 0.0;
    double term_sw = 0.0;
    double term_vl = 0.0;
    double force_rate = 0.0;
    std::string csv_row;
};

void runTable2Benchmark(const std::vector<ArenaSpec>& all_arenas,
                        const std::set<std::string>& method_filter,
                        const std::set<std::string>& arena_filter,
                        int num_seeds,
                        unsigned int num_threads,
                        const std::string& csv_path)
{
    std::vector<ArenaSpec> arenas;
    for (const auto& ar : all_arenas) {
        if (arena_filter.empty() || arena_filter.count(ar.id) > 0) {
            arenas.push_back(ar);
        }
    }
    std::vector<MethodSpec> methods;
    for (const auto& m : buildTable2Methods()) {
        if (method_filter.empty() || method_filter.count(m.id) > 0) {
            methods.push_back(m);
        }
    }
    if (arenas.empty() || methods.empty()) {
        std::cerr << "No matching arenas or methods for --table2 filter." << std::endl;
        return;
    }

    std::cout << "\n=== ECC 2027 Table II Ablation & Baseline Benchmark ("
              << methods.size() << " methods x " << arenas.size() << " arenas x "
              << num_seeds << " seeds) ===" << std::endl;
    std::cout << "  [Aggregation: N_all, VER, DE/VB = mean over all runs; Force (RMS d||u||/dt) and T_succ = mean over converged runs]" << std::endl;

    const auto mp = makeNominalCraneParams();
    std::ofstream ofs(csv_path);
    ofs << "arena,method_id,method_label,seed,success,fine_2ms_clear,fine_col_viol,fine_vel_viol,fine_sw_viol,"
        << "max_fine_swing_deg,min_cable_tension_n,iterations,tree_nodes,sst_active_nodes,collisions,"
        << "time_ms,traj_dur_s,path_len_m,max_swing_deg,rms_swing_deg,term_swing_deg,term_omega_rads,"
        << "term_speed_ms,term_load_speed_ms,rms_force_jerk,total_distance_evals,valid_extensions,"
        << "valid_extension_rate,dist_evals_per_valid_branch\n";

    std::cout << "  " << std::left << std::setw(26) << "Method"
              << " | " << std::setw(23) << "Arena Succ (per arena)"
              << " | " << std::setw(12) << "Total Succ"
              << " | " << std::setw(4)  << "Fine"
              << " | " << std::setw(5)  << "N_all"
              << " | " << std::setw(6)  << "VER"
              << " | " << std::setw(6)  << "DE/VB"
              << " | " << std::setw(6)  << "Force"
              << " | " << "T_succ\n";
    std::cout << "  " << std::string(110, '-') << "\n";

    for (const auto& m : methods) {
        const int total_runs = static_cast<int>(arenas.size()) * num_seeds;
        std::vector<TrialRecord> recs(static_cast<std::size_t>(total_runs));

        runParallelTasks(total_runs, num_threads, mp,
            [&](int idx, const auto& tl_model, const auto& tl_solver, const auto& tl_fine) {
                const std::size_t a_idx = static_cast<std::size_t>(idx / num_seeds);
                const int s = idx % num_seeds;
                const auto& ar = arenas[a_idx];

                Planner::Parameters p = m.params;
                p.random_seed = static_cast<unsigned int>(1001 + s);
                if (ar.use_soft_corridor) {
                    p.enable_soft_corridor = true;
                    p.corridor_half_width  = ar.corridor_half_width;
                }

                Planner planner(tl_model, tl_solver, ar.arena, p);
                const auto res = planner.plan(ar.start, ar.goal);
                const auto tm  = computeTrajectoryMetrics(
                    res.state_trajectory, res.input_trajectory, p.integration_dt, mp.ell);
                const auto fv  = checkFineRollout10x(planner, res, p.integration_dt, tl_fine);
                const double time_ms = res.planning_time_sec * 1000.0;

                TrialRecord tr;
                tr.arena_idx           = a_idx;
                tr.success             = res.success;
                tr.fine_clear          = fv.all_clear;
                tr.fine_col_viol       = fv.collision_violations;
                tr.fine_vel_viol       = fv.speed_or_bounds_violations;
                tr.fine_sw_viol        = fv.swing_violations;
                tr.max_fine_swing_deg  = fv.max_fine_swing_deg;
                tr.min_cable_tension_n = res.success ? fv.min_cable_tension_n : 0.0;
                tr.nodes               = static_cast<double>(res.tree_size);
                tr.time_ms             = time_ms;
                tr.traj_dur_s          = tm.traj_duration_s;
                tr.ver                 = res.valid_extension_rate;
                tr.depvb               = res.dist_evals_per_valid_branch;
                tr.max_sw              = tm.max_swing_deg;
                tr.term_sw             = tm.term_swing_deg;
                tr.term_vl             = tm.term_load_speed_ms;
                tr.force_rate          = tm.rms_force_jerk;

                std::ostringstream oss;
                oss << ar.id << "," << m.id << ",\"" << m.table_label << "\"," << p.random_seed << ","
                    << (res.success ? 1 : 0) << "," << (fv.all_clear ? 1 : 0) << ","
                    << fv.collision_violations << "," << fv.speed_or_bounds_violations << ","
                    << fv.swing_violations << "," << fv.max_fine_swing_deg << ","
                    << tr.min_cable_tension_n << ","
                    << res.iterations << "," << res.tree_size << "," << res.sst_active_nodes << ","
                    << res.collision_rejections << "," << time_ms << "," << tm.traj_duration_s << ","
                    << tm.path_length_m << "," << tm.max_swing_deg << "," << tm.rms_swing_deg << ","
                    << tm.term_swing_deg << "," << tm.term_omega_rads << ","
                    << tm.term_speed_ms << "," << tm.term_load_speed_ms << "," << tm.rms_force_jerk << ","
                    << res.total_distance_evals << "," << res.valid_extensions << ","
                    << res.valid_extension_rate << "," << res.dist_evals_per_valid_branch << "\n";
                tr.csv_row = oss.str();
                recs[static_cast<std::size_t>(idx)] = std::move(tr);
            });

        std::vector<int> arena_succ(arenas.size(), 0);
        int succ_total = 0, fine_total = 0;
        std::size_t sum_col_viol = 0, sum_vel_viol = 0;
        double min_tension_succ = 1e9, max_fine_sw_succ = 0.0;
        double sum_nodes = 0.0, sum_ver = 0.0, sum_depvb = 0.0;
        double sum_force_succ = 0.0, sum_t_succ = 0.0;

        for (const auto& tr : recs) {
            ofs << tr.csv_row;
            sum_nodes += tr.nodes;
            sum_ver   += tr.ver;
            sum_depvb += tr.depvb;
            if (tr.success) {
                ++succ_total;
                ++arena_succ[tr.arena_idx];
                if (tr.fine_clear) {
                    ++fine_total;
                }
                sum_col_viol += tr.fine_col_viol;
                sum_vel_viol += tr.fine_vel_viol;
                min_tension_succ = std::min(min_tension_succ, tr.min_cable_tension_n);
                max_fine_sw_succ = std::max(max_fine_sw_succ, tr.max_fine_swing_deg);
                sum_force_succ  += tr.force_rate;
                sum_t_succ      += tr.time_ms;
            }
        }

        std::ostringstream a_str;
        for (std::size_t a = 0; a < arenas.size(); ++a) {
            if (a > 0) a_str << " ";
            a_str << std::setw(2) << arena_succ[a] << "/" << num_seeds;
        }
        const double denom_all  = static_cast<double>(total_runs);
        const double denom_succ = (succ_total > 0) ? static_cast<double>(succ_total) : 1.0;
        const double succ_pct   = 100.0 * static_cast<double>(succ_total) / denom_all;

        std::ostringstream s_str;
        s_str << std::setw(3) << succ_total << " (" << std::fixed << std::setprecision(1) << std::setw(4) << succ_pct << "%)";

        std::cout << "  " << std::left << std::setw(26) << m.table_label
                  << " | " << std::setw(23) << a_str.str()
                  << " | " << std::right << std::setw(12) << s_str.str()
                  << " | " << std::setw(4) << fine_total
                  << " | " << std::setw(5) << std::fixed << std::setprecision(0) << (sum_nodes / denom_all)
                  << " | " << std::setw(5) << std::setprecision(1) << (100.0 * sum_ver / denom_all) << "%"
                  << " | " << std::setw(6) << std::setprecision(0) << (sum_depvb / denom_all)
                  << " | " << std::setw(6) << std::setprecision(2) << (succ_total > 0 ? (sum_force_succ / denom_succ) : 0.0)
                  << " | " << std::setw(5) << std::setprecision(0) << (succ_total > 0 ? (sum_t_succ / denom_succ) : 0.0) << " ms\n";

        if (m.id == "M7_ProposedFull" && succ_total > 0) {
            std::cout << "      [M7 Fine-Grid 0.002s Post-Hoc Check] obstacle/cable contacts = " << sum_col_viol
                      << ", speed violations = " << sum_vel_viol
                      << ", min cable tension = " << std::fixed << std::setprecision(3) << min_tension_succ << " N"
                      << ", peak fine swing = " << std::setprecision(3) << max_fine_sw_succ << " deg\n";
        }
    }
    std::cout << "  Saved CSV: " << csv_path << std::endl;
}

void runTable3Sensitivity(const std::vector<ArenaSpec>& arenas,
                          int num_seeds,
                          unsigned int num_threads,
                          const std::string& csv_path)
{
    struct QuiescenceProfile {
        std::string id;
        std::string label;
        double r_goal;
        double theta_goal_rad;
        double v_goal_max;
        double omega_goal_max;
    };
    const std::vector<QuiescenceProfile> profiles = {
        {"P1_Nominal", "Nominal (12.6 deg, 0.55 m/s, 0.55 rad/s)", 0.35, 0.220000, 0.55, 0.55},
        {"P2_Tighter", "Tighter (5.0 deg, 0.20 m/s, 0.20 rad/s)",  0.35, 0.087266, 0.20, 0.20},
        {"P3_Strict",  "Strict (2.0 deg, 0.10 m/s, 0.10 rad/s)",   0.35, 0.034907, 0.10, 0.10}
    };

    const std::set<std::string> keep = {"M7_ProposedFull", "M7_RandSub", "M5_4DPhaseOnly", "B4_SST"};
    std::vector<MethodSpec> methods;
    for (const auto& m : buildTable2Methods()) {
        if (keep.count(m.id) > 0) {
            methods.push_back(m);
        }
    }
    // Match Table III row ordering: M7, M7-RandSub, M5, B4
    std::sort(methods.begin(), methods.end(), [](const MethodSpec& a, const MethodSpec& b) {
        auto rank = [](const std::string& id) {
            if (id == "M7_ProposedFull") return 0;
            if (id == "M7_RandSub")      return 1;
            if (id == "M5_4DPhaseOnly")  return 2;
            return 3;
        };
        return rank(a.id) < rank(b.id);
    });

    std::cout << "\n=== ECC 2027 Table III Terminal Quiescence Sensitivity Study ===" << std::endl;
    const auto mp = makeNominalCraneParams();
    std::ofstream ofs(csv_path);
    ofs << "profile_id,profile_label,arena,method_id,seed,success,fine_2ms_clear,"
        << "iterations,nodes,time_ms,traj_dur_s,max_swing_deg,term_swing_deg,"
        << "term_omega_rads,term_speed_ms,term_load_speed_ms\n";

    for (const auto& prof : profiles) {
        std::cout << "\n  --- " << prof.id << " [" << prof.label << "] ---\n";
        for (const auto& m : methods) {
            const int total_runs = static_cast<int>(arenas.size()) * num_seeds;
            std::vector<TrialRecord> recs(static_cast<std::size_t>(total_runs));

            runParallelTasks(total_runs, num_threads, mp,
                [&](int idx, const auto& tl_model, const auto& tl_solver, const auto& tl_fine) {
                    const std::size_t a_idx = static_cast<std::size_t>(idx / num_seeds);
                    const int s = idx % num_seeds;
                    const auto& ar = arenas[a_idx];

                    Planner::Parameters p = m.params;
                    p.random_seed          = static_cast<unsigned int>(1001 + s);
                    p.goal_pos_tolerance   = prof.r_goal;
                    p.goal_swing_tolerance = prof.theta_goal_rad;
                    p.goal_vel_tolerance   = prof.v_goal_max;
                    p.goal_omega_tolerance = prof.omega_goal_max;
                    if (ar.use_soft_corridor) {
                        p.enable_soft_corridor = true;
                        p.corridor_half_width  = ar.corridor_half_width;
                    }

                    Planner planner(tl_model, tl_solver, ar.arena, p);
                    const auto res = planner.plan(ar.start, ar.goal);
                    const auto tm  = computeTrajectoryMetrics(
                        res.state_trajectory, res.input_trajectory, p.integration_dt, mp.ell);
                    const auto fv  = checkFineRollout10x(planner, res, p.integration_dt, tl_fine);
                    const double time_ms = res.planning_time_sec * 1000.0;

                    TrialRecord tr;
                    tr.success    = res.success;
                    tr.fine_clear = fv.all_clear;
                    tr.time_ms    = time_ms;
                    tr.traj_dur_s = tm.traj_duration_s;
                    tr.term_sw    = tm.term_swing_deg;
                    tr.term_vl    = tm.term_load_speed_ms;

                    std::ostringstream oss;
                    oss << prof.id << ",\"" << prof.label << "\"," << ar.id << ","
                        << m.id << "," << p.random_seed << ","
                        << (res.success ? 1 : 0) << "," << (fv.all_clear ? 1 : 0) << ","
                        << res.iterations << "," << res.tree_size << ","
                        << time_ms << "," << tm.traj_duration_s << ","
                        << tm.max_swing_deg << "," << tm.term_swing_deg << ","
                        << tm.term_omega_rads << "," << tm.term_speed_ms << ","
                        << tm.term_load_speed_ms << "\n";
                    tr.csv_row = oss.str();
                    recs[static_cast<std::size_t>(idx)] = std::move(tr);
                });

            int succ = 0, fine_ok = 0;
            double sum_t = 0.0, sum_traj = 0.0, sum_tsw = 0.0, sum_tvl = 0.0;
            for (const auto& tr : recs) {
                ofs << tr.csv_row;
                if (tr.success) {
                    ++succ;
                    if (tr.fine_clear) ++fine_ok;
                    sum_t    += tr.time_ms;
                    sum_traj += tr.traj_dur_s;
                    sum_tsw  += tr.term_sw;
                    sum_tvl  += tr.term_vl;
                }
            }
            const double pct = 100.0 * static_cast<double>(succ) / static_cast<double>(total_runs);
            std::cout << "    " << std::left << std::setw(24) << m.table_label
                      << " | Succ: " << std::right << std::setw(3) << succ << "/" << total_runs
                      << " (" << std::setw(4) << std::fixed << std::setprecision(1) << pct << "%)"
                      << " | Fine: " << std::setw(3) << fine_ok;
            if (succ > 0) {
                const double ds = static_cast<double>(succ);
                std::cout << " | T_succ: " << std::setw(5) << std::setprecision(0) << (sum_t / ds) << " ms"
                          << " | T_traj: " << std::setw(5) << std::setprecision(1) << (sum_traj / ds) << " s"
                          << " | theta(T): " << std::setw(5) << std::setprecision(2) << (sum_tsw / ds) << " deg"
                          << " | ||v_load(T)||: " << std::setw(4) << std::setprecision(2) << (sum_tvl / ds) << " m/s\n";
            } else {
                std::cout << " | T_succ:   --- | T_traj:   --- | theta(T):   --- | ||v_load(T)||:  ---\n";
            }
        }
    }
    std::cout << "  Saved CSV: " << csv_path << std::endl;
}

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n\n"
              << "Modes (default: --quick):\n"
              << "  --quick             Run physics/Lemma-1/collision checks + 4-arena M7 smoke test\n"
              << "  --rk4               Run RK4 step-size convergence, energy & timing study (Table I)\n"
              << "  --tune              Run held-out hyperparameter tuning on seeds 2001..2005\n"
              << "  --table2            Run 14-method x 4-arena x 30-seed ablation & baseline suite (Table II)\n"
              << "  --table3            Run terminal quiescence sensitivity suite across 3 profiles (Table III)\n"
              << "  --all               Run checks, --rk4, --tune, --table2, and --table3\n\n"
              << "Optional filters & settings:\n"
              << "  --method <ID>       Filter --table2 to specific method ID(s) (repeatable)\n"
              << "  --arena  <ID>       Filter --table2 to specific arena ID(s) (repeatable)\n"
              << "  --seeds  <N>        Number of test seeds starting at 1001 (default: 30)\n"
              << "  --threads <N>       Worker threads (default: min(4, hardware_concurrency))\n"
              << "  --out-dir <DIR>     Output directory for CSV files (default: results)\n"
              << "  -h, --help          Show this help message\n";
}

} // namespace

int main(int argc, char** argv) {
    bool do_quick  = false;
    bool do_rk4    = false;
    bool do_tune   = false;
    bool do_table2 = false;
    bool do_table3 = false;
    int num_seeds  = 30;
    unsigned int num_threads = std::min(4u, std::max(1u, std::thread::hardware_concurrency()));
    std::string out_dir = "results";
    std::set<std::string> method_filter;
    std::set<std::string> arena_filter;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--quick") {
            do_quick = true;
        } else if (arg == "--rk4") {
            do_rk4 = true;
        } else if (arg == "--tune") {
            do_tune = true;
        } else if (arg == "--table2") {
            do_table2 = true;
        } else if (arg == "--table3") {
            do_table3 = true;
        } else if (arg == "--all") {
            do_rk4 = do_tune = do_table2 = do_table3 = true;
        } else if (arg == "--method" && i + 1 < argc) {
            method_filter.insert(argv[++i]);
        } else if (arg == "--arena" && i + 1 < argc) {
            arena_filter.insert(argv[++i]);
        } else if (arg == "--seeds" && i + 1 < argc) {
            num_seeds = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--threads" && i + 1 < argc) {
            num_threads = static_cast<unsigned int>(std::max(1, std::stoi(argv[++i])));
        } else if (arg == "--out-dir" && i + 1 < argc) {
            out_dir = argv[++i];
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    if (!do_quick && !do_rk4 && !do_tune && !do_table2 && !do_table3) {
        do_quick = true;
    }

    if (!verifyPhysicsAndSimulator() || !verifySphereAndEllipsoidCollisions()) {
        return 1;
    }

    std::vector<ArenaSpec> arenas;
    try {
        arenas = arena::buildECCArenas<float>();
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Could not load ECC benchmark arenas: " << e.what() << std::endl;
        return 1;
    }

    if (do_rk4 || do_tune || do_table2 || do_table3) {
        try {
            std::filesystem::create_directories(out_dir);
        } catch (const std::exception& e) {
            std::cerr << "[FAIL] Cannot create output directory '" << out_dir << "': " << e.what() << std::endl;
            return 1;
        }
    }

    if (do_quick) {
        if (!runQuickSmokeTest(arenas, num_threads)) {
            return 1;
        }
    }
    if (do_rk4) {
        runRK4ConvergenceStudy((std::filesystem::path(out_dir) / "rk4_convergence.csv").string());
    }
    if (do_tune) {
        runHeldOutTuning(arenas, num_threads, (std::filesystem::path(out_dir) / "tuning_summary.csv").string());
    }
    if (do_table2) {
        runTable2Benchmark(
            arenas, method_filter, arena_filter, num_seeds, num_threads,
            (std::filesystem::path(out_dir) / "table2_ablation_baselines.csv").string());
    }
    if (do_table3) {
        runTable3Sensitivity(
            arenas, num_seeds, num_threads,
            (std::filesystem::path(out_dir) / "table3_terminal_sensitivity.csv").string());
    }

    return 0;
}

