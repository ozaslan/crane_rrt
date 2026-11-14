#ifndef __GANTRY_NUMERICAL_SOLVER_HPP__
#define __GANTRY_NUMERICAL_SOLVER_HPP__

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <array>
#include <cctype>
#include <stdexcept>

#include <boost/numeric/odeint/stepper/euler.hpp>
#include <boost/numeric/odeint/stepper/modified_midpoint.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_fehlberg78.hpp>
#include <boost/numeric/odeint/stepper/controlled_runge_kutta.hpp>
#include <boost/numeric/odeint/stepper/bulirsch_stoer.hpp>
#include <boost/numeric/odeint/integrate/integrate_adaptive.hpp>

#include "crane_rrt/dynamic_model.hpp"

namespace gantry {

template<typename ScalarT>
class NumericalSolver {
public:
    using Scalar      = ScalarT;
    using Self        = NumericalSolver<Scalar>;
    using Ptr         = std::shared_ptr<Self>;
    using ConstPtr    = std::shared_ptr<const Self>;
    using Model       = DynamicModel<Scalar>;
    using ModelPtr    = typename Model::Ptr;
    using State       = typename Model::State;
    using Input       = typename Model::Input;
    using Derivative  = typename Model::Derivative;
    using StateVector = typename Model::StateVector;
    using OdeArray    = std::array<Scalar, 8>;

    // Enum listing supported Runge-Kutta integration methods
    enum class RKMethod {
        Euler,                 // Explicit Euler method (1st order)
        ModifiedMidpoint,      // Modified Midpoint method (2nd order)
        ClassicalRK4,          // Classical Runge-Kutta method (4th order)
        CashKarp54,            // Cash-Karp RK5(4) method with error estimation
        DormandPrince5,        // Dormand-Prince RK5(4) method with dense output
        Fehlberg78,            // Fehlberg RK7(8) method with error estimation
        BulirschStoer          // Bulirsch-Stoer method with adaptive step size and order
    };

    struct Parameters {
        bool use_boost = true;
        RKMethod method = RKMethod::ClassicalRK4;

        Parameters() = default;

        Parameters(bool use_boost_, RKMethod method_)
        : use_boost(use_boost_), method(method_) {}

        Parameters(bool use_boost_, const std::string& method_str)
        : use_boost(use_boost_) {
            try {
                method = fromStringToMethod(method_str);
            } catch (const std::invalid_argument& e) {
                std::cerr << "[NumericalSolver::Parameters] " << e.what()
                          << ". Falling back to ClassicalRK4." << std::endl;
                method = RKMethod::ClassicalRK4;
            }
        }

        std::string methodToString() const {
            switch (method) {
                case RKMethod::Euler:              return "Euler";
                case RKMethod::ModifiedMidpoint:   return "ModifiedMidpoint";
                case RKMethod::ClassicalRK4:       return "ClassicalRK4";
                case RKMethod::CashKarp54:         return "CashKarp54";
                case RKMethod::DormandPrince5:     return "DormandPrince5";
                case RKMethod::Fehlberg78:         return "Fehlberg78";
                case RKMethod::BulirschStoer:      return "BulirschStoer";
                default:                           return "Unknown";
            }
        }

        static RKMethod fromStringToMethod(const std::string& methodStr) {
            std::string lower;
            lower.reserve(methodStr.size());
            for (unsigned char c : methodStr)
                lower.push_back(static_cast<char>(std::tolower(c)));

            if (lower == "euler")              return RKMethod::Euler;
            if (lower == "modifiedmidpoint")   return RKMethod::ModifiedMidpoint;
            if (lower == "classicalrk4" || lower == "rk4") return RKMethod::ClassicalRK4;
            if (lower == "cashkarp54")         return RKMethod::CashKarp54;
            if (lower == "dormandprince5")     return RKMethod::DormandPrince5;
            if (lower == "fehlberg78")         return RKMethod::Fehlberg78;
            if (lower == "bulirschstoer")      return RKMethod::BulirschStoer;

            throw std::invalid_argument("Unknown Runge-Kutta method string: " + methodStr);
        }

        friend std::ostream& operator<<(std::ostream& os, const Parameters& p) {
            os << "NumericalSolver::Parameters { " << std::endl;
            os << " use_boost = " << std::boolalpha << p.use_boost << std::endl;
            os << " method = " << p.methodToString() << std::endl;
            os << "}";
            return os;
        }
    };

    NumericalSolver(ModelPtr model = nullptr)
        : mModel(std::move(model)) {}

    // Main integration interface for piecewise-constant input over [t, t + dt]
    void step(const State& x, const Input& u, double t, double dt, State& x_next) {
        if (!mModel) {
            throw std::runtime_error("[NumericalSolver::step] DynamicModel pointer is null.");
        }
        if (mParams.use_boost) {
            stepBoost(x, u, t, dt, x_next, mParams.method);
        } else {
            stepManual(x, u, t, dt, x_next, mParams.method);
        }
    }

    // Fourth-order RK4 integration for time-varying input u(t) evaluated at stage times t, t+h/2, t+h
    template <typename InputFunc>
    void stepTimeVarying(const State& x, InputFunc&& u_fn, double t, double dt, State& x_next) {
        if (!mModel) {
            throw std::runtime_error("[NumericalSolver::stepTimeVarying] DynamicModel pointer is null.");
        }
        const double t_half = t + 0.5 * dt;
        const double t_end  = t + dt;
        const Scalar h      = static_cast<Scalar>(dt);

        const Input u1 = u_fn(t);
        const Input u2 = u_fn(t_half);
        const Input u4 = u_fn(t_end);

        auto k1 = (*mModel)(x, u1);
        auto k2 = (*mModel)(State::FromVector(x.toVector() + Scalar(0.5) * h * k1), u2);
        auto k3 = (*mModel)(State::FromVector(x.toVector() + Scalar(0.5) * h * k2), u2);
        auto k4 = (*mModel)(State::FromVector(x.toVector() + h * k3), u4);
        x_next = State::FromVector(
            x.toVector() + (h / Scalar(6)) * (k1 + Scalar(2) * k2 + Scalar(2) * k3 + k4),
            t_end);
    }

    void setModel(ModelPtr model) {
        mModel = std::move(model);
    }

    const Model& getModel() const {
        if (!mModel) {
            throw std::runtime_error("[NumericalSolver::getModel] DynamicModel pointer is null.");
        }
        return *mModel;
    }

    Model& getModel() {
        if (!mModel) {
            throw std::runtime_error("[NumericalSolver::getModel] DynamicModel pointer is null.");
        }
        return *mModel;
    }

    void setParameters(const Parameters& params) {
        mParams = params;
    }

    Parameters getParameters() const {
        return mParams;
    }

private:
    Parameters mParams{};
    ModelPtr mModel;

    static OdeArray toOdeArray(const StateVector& v) {
        OdeArray arr;
        for (std::size_t i = 0; i < 8; ++i) {
            arr[i] = v(static_cast<Eigen::Index>(i));
        }
        return arr;
    }

    static StateVector fromOdeArray(const OdeArray& arr) {
        StateVector v;
        for (std::size_t i = 0; i < 8; ++i) {
            v(static_cast<Eigen::Index>(i)) = arr[i];
        }
        return v;
    }

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
    // Integration using Boost.Odeint methods
    void stepBoost(const State& x, const Input& u, double t, double dt, State& x_next, RKMethod method) {
        using namespace boost::numeric::odeint;

        auto rhs = [this, &u](const OdeArray& x_arr, OdeArray& dxdt_arr, double /*t*/) {
            StateVector x_vec = fromOdeArray(x_arr);
            Derivative dxdt_vec = (*mModel)(x_vec, u);
            dxdt_arr = toOdeArray(dxdt_vec);
        };

        OdeArray x_temp = toOdeArray(x.toVector());
        const double t_end = t + dt;

        switch (method) {
            case RKMethod::Euler: {
                euler<OdeArray> stepper;
                stepper.do_step(rhs, x_temp, t, dt);
                break;
            }
            case RKMethod::ModifiedMidpoint: {
                modified_midpoint<OdeArray> stepper;
                stepper.do_step(rhs, x_temp, t, dt);
                break;
            }
            case RKMethod::ClassicalRK4: {
                runge_kutta4<OdeArray> stepper;
                stepper.do_step(rhs, x_temp, t, dt);
                break;
            }
            case RKMethod::CashKarp54: {
                using ErrorStepper = runge_kutta_cash_karp54<OdeArray>;
                controlled_runge_kutta<ErrorStepper> controlled_stepper;
                integrate_adaptive(controlled_stepper, rhs, x_temp, t, t_end, dt);
                break;
            }
            case RKMethod::DormandPrince5: {
                using ErrorStepper = runge_kutta_dopri5<OdeArray>;
                controlled_runge_kutta<ErrorStepper> controlled_stepper;
                integrate_adaptive(controlled_stepper, rhs, x_temp, t, t_end, dt);
                break;
            }
            case RKMethod::Fehlberg78: {
                using ErrorStepper = runge_kutta_fehlberg78<OdeArray>;
                controlled_runge_kutta<ErrorStepper> controlled_stepper;
                integrate_adaptive(controlled_stepper, rhs, x_temp, t, t_end, dt);
                break;
            }
            case RKMethod::BulirschStoer: {
                bulirsch_stoer<OdeArray> stepper;
                integrate_adaptive(stepper, rhs, x_temp, t, t_end, dt);
                break;
            }
            default: {
                throw std::runtime_error(
                    "[NumericalSolver::stepBoost] Unimplemented RKMethod encountered.");
            }
        }

        x_next = State::FromVector(fromOdeArray(x_temp), t_end);
    }
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

    // Manual implementation of fixed-step RK methods
    void stepManual(const State& x, const Input& u, double t, double dt, State& x_next, RKMethod method) {
        const double t_end = t + dt;
        const Scalar h = static_cast<Scalar>(dt);

        if (method == RKMethod::Euler) {
            auto dx = (*mModel)(x, u);
            x_next = State::FromVector(x.toVector() + h * dx, t_end);
        } else if (method == RKMethod::ModifiedMidpoint) {
            auto k1 = (*mModel)(x, u);
            auto midpoint = State::FromVector(x.toVector() + Scalar(0.5) * h * k1);
            auto k2 = (*mModel)(midpoint, u);
            x_next = State::FromVector(x.toVector() + h * k2, t_end);
        } else {
            // ClassicalRK4 (and fallback for adaptive methods when use_boost is false)
            auto k1 = (*mModel)(x, u);
            auto k2 = (*mModel)(State::FromVector(x.toVector() + Scalar(0.5) * h * k1), u);
            auto k3 = (*mModel)(State::FromVector(x.toVector() + Scalar(0.5) * h * k2), u);
            auto k4 = (*mModel)(State::FromVector(x.toVector() + h * k3), u);
            x_next = State::FromVector(
                x.toVector() + (h / Scalar(6)) * (k1 + Scalar(2) * k2 + Scalar(2) * k3 + k4),
                t_end);
        }
    }
};

} // namespace gantry

#endif // __GANTRY_NUMERICAL_SOLVER_HPP__
