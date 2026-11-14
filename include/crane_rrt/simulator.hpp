#ifndef __GANTRY_SIMULATOR_HPP__
#define __GANTRY_SIMULATOR_HPP__

#include <memory>
#include <deque>
#include <string>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include "crane_rrt/logger.hpp"
#include "crane_rrt/dynamic_model.hpp"
#include "crane_rrt/numerical_solver.hpp"

namespace gantry {

class Simulator {
public:
    using Self          = Simulator;
    using Ptr           = std::shared_ptr<Self>;
    using ConstPtr      = std::shared_ptr<const Self>;
    using Scalar        = double;
    using Model         = DynamicModel<Scalar>;
    using ModelPtr      = typename Model::Ptr;
    using State         = typename Model::State;
    using Input         = typename Model::Input;
    using Solver        = NumericalSolver<Scalar>;
    using SolverPtr     = typename Solver::Ptr;
    using SolverParams  = typename Solver::Parameters;
    using ModelParams   = typename Model::Parameters;

    struct Parameters {
        double frequency = 100.0;
        State initial_state = State::Zero();
        bool logger_active = false;
        std::size_t max_history_size = 10000;

        Parameters() = default;

        explicit Parameters(double freq, bool log = false)
        : frequency(freq), initial_state(State::Zero()), logger_active(log) {
            validate();
        }

        Parameters(double freq, bool log, const State& init)
        : frequency(freq), initial_state(init), logger_active(log) {
            validate();
        }

        void validate() const {
            if (frequency <= 0.0 || !std::isfinite(frequency))
                throw std::invalid_argument("Simulation frequency must be finite and positive.");
            if (!initial_state.isValid())
                throw std::invalid_argument("Initial state must contain only finite values.");
        }

        double getPeriod() const {
            if (frequency <= 0.0)
                throw std::runtime_error("Frequency must be positive to compute period.");
            return 1.0 / frequency;
        }

        friend std::ostream& operator<<(std::ostream& os, const Parameters& p) {
            os << "Simulator::Parameters {\n";
            os << "  frequency: " << p.frequency << "\n";
            os << "  logger active: " << (p.logger_active ? "TRUE" : "FALSE") << "\n";
            os << "  initial_state: [";
            auto stateVec = p.initial_state.toVector();
            for (int i = 0; i < stateVec.size(); ++i) {
                os << stateVec(i);
                if (i < stateVec.size() - 1) os << ", ";
            }
            os << "]\n}";
            return os;
        }
    };

    // ============================================================================== //

    // === Main Constructor ===
    Simulator(const Parameters& simParams,
              const ModelParams& modelParams,
              const SolverParams& solverParams)
    : mSimParams(simParams),
      mModel(std::make_shared<Model>()),
      mSolver(std::make_shared<Solver>()),
      mLogger(nullptr),
      mCurrentTime(simParams.initial_state.getTime()),
      mIteration(0)
    {
        mSimParams.validate();
        mModel->setParameters(modelParams);
        mSolver->setParameters(solverParams);
        mSolver->setModel(mModel);
        mStateHistory.push_back(mSimParams.initial_state);
        mInputHistory.push_back(Input::Zero(mCurrentTime));
        if (mSimParams.logger_active) {
            mLogger = std::make_unique<Logger>();
            logState();
        }
    }

    // === Copy Constructor (with optional history copy) ===
    Simulator(const Simulator& other, bool copy_history = false)
    : mSimParams(other.mSimParams),
      mModel(std::make_shared<Model>(*other.mModel)),
      mSolver(std::make_shared<Solver>(*other.mSolver)),
      mLogger(nullptr),
      mCurrentTime(other.mCurrentTime),
      mIteration(other.mIteration)
    {
        // Cloned simulators disable file logging by default to avoid creating duplicate CSV files
        mSimParams.logger_active = false;
        mSolver->setModel(mModel);
        if (copy_history) {
            mStateHistory = other.mStateHistory;
            mInputHistory = other.mInputHistory;
        } else {
            mStateHistory.push_back(other.getState());
            mInputHistory.push_back(other.getInput());
        }
    }

    // === Clone ===
    std::unique_ptr<Simulator> clone(bool copy_history = false) const {
        return std::make_unique<Simulator>(*this, copy_history);
    }

    // === Reset state for fast rollout reuse ===
    void reset(const State& state, const Input& input = Input::Zero()) {
        mStateHistory.clear();
        mInputHistory.clear();
        mCurrentTime = state.getTime();
        mIteration = 0;
        mStateHistory.push_back(state);
        Input init_input = input;
        init_input.setTime(mCurrentTime);
        mInputHistory.push_back(init_input);
    }

    void step() {
        step(Input::Zero(mCurrentTime));
    }

    void step(const Input& input) {
        State newState;
        const double dt = mSimParams.getPeriod();

        mSolver->step(
            getState(),     // current state
            input,          // input
            mCurrentTime,   // time
            dt,             // timestep
            newState        // output
        );

        mCurrentTime += dt;
        ++mIteration;

        Input timedInput = input;
        timedInput.setTime(mCurrentTime);

        mStateHistory.push_back(newState);
        mInputHistory.push_back(timedInput);

        if (mSimParams.max_history_size > 0) {
            while (mStateHistory.size() > mSimParams.max_history_size) {
                mStateHistory.pop_front();
            }
            while (mInputHistory.size() > mSimParams.max_history_size) {
                mInputHistory.pop_front();
            }
        }

        logState();
    }

    // === Accessors ===
    const Model& getModel() const { return *mModel; }
    Model& getModel() { return *mModel; }

    const Solver& getSolver() const { return *mSolver; }
    Solver& getSolver() { return *mSolver; }

    const Parameters& getParameters() const { return mSimParams; }
    double getSimTime() const { return mCurrentTime; }
    std::size_t getIterationCount() const { return mIteration; }

    const std::deque<State>& getStateHistory() const {
        return mStateHistory;
    }

    const std::deque<Input>& getInputHistory() const {
        return mInputHistory;
    }

    const State& getState() const {
        if (mStateHistory.empty())
            throw std::runtime_error("[Simulator::getState] mStateHistory is empty.");
        return mStateHistory.back();
    }

    const Input& getInput() const {
        if (mInputHistory.empty())
            throw std::runtime_error("[Simulator::getInput] mInputHistory is empty.");
        return mInputHistory.back();
    }

private:
    Parameters mSimParams;
    ModelPtr mModel;
    SolverPtr mSolver;
    std::unique_ptr<Logger> mLogger;

    std::deque<State> mStateHistory;
    std::deque<Input> mInputHistory;
    double mCurrentTime = 0.0;
    std::size_t mIteration = 0;

    void logState() {
        if (!mSimParams.logger_active || !mLogger)
            return;
        mLogger->log(mCurrentTime, mIteration, getState(), getInput());
    }
};

} // namespace gantry

#endif // __GANTRY_SIMULATOR_HPP__
