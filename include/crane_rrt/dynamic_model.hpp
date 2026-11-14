#ifndef __GANTRY_DYNAMIC_MODEL_HPP__
#define __GANTRY_DYNAMIC_MODEL_HPP__

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "crane_rrt/crane_state.hpp"
#include "crane_rrt/crane_input.hpp"

namespace gantry {

template <typename Scalar>
class DynamicModel {
public:
    using Self        = DynamicModel<Scalar>;
    using State       = CraneState<Scalar>;
    using Input       = CraneInput<Scalar>;
    using MassMatrix  = Eigen::Matrix<Scalar, 4, 4>;
    using Vector4     = Eigen::Matrix<Scalar, 4, 1>;
    using Vector3     = Eigen::Matrix<Scalar, 3, 1>;
    using Vector2     = Eigen::Matrix<Scalar, 2, 1>;
    using Derivative  = Eigen::Matrix<Scalar, 8, 1>;
    using StateVector = Eigen::Matrix<Scalar, 8, 1>;
    using Ptr         = std::shared_ptr<Self>;
    using ConstPtr    = std::shared_ptr<const Self>;

    struct Parameters {
        Scalar m_x = Scalar(0.1);
        Scalar m_y = Scalar(0.1);
        Scalar m   = Scalar(0.1);
        Scalar g   = Scalar(9.81);
        Scalar ell = Scalar(1.0);
        Scalar D_x = Scalar(0.0);
        Scalar D_y = Scalar(0.0);

        std::string toString() const {
            std::ostringstream oss;
            oss << "DynamicModel::Parameters: {\n"
                << "  m_x  = " << m_x << "\n"
                << "  m_y  = " << m_y << "\n"
                << "  m    = " << m << "\n"
                << "  g    = " << g << "\n"
                << "  ell  = " << ell << "\n"
                << "  D_x  = " << D_x << "\n"
                << "  D_y  = " << D_y << "\n"
                << "}";
            return oss.str();
        }

        friend std::ostream& operator<<(std::ostream& os, const Parameters& p) {
            return os << p.toString();
        }

        bool isValid() const {
            return m_x > Scalar(0) && m_y > Scalar(0) && m > Scalar(0) &&
                   g > Scalar(0) && ell > Scalar(0) &&
                   D_x >= Scalar(0) && D_y >= Scalar(0);
        }
    };

    struct TrigCache {
        Scalar sa = Scalar(0), ca = Scalar(1);
        Scalar sb = Scalar(0), cb = Scalar(1);
        Scalar s2b = Scalar(0);

        TrigCache() = default;
        TrigCache(Scalar alpha, Scalar beta) {
            update(alpha, beta);
        }
        void update(Scalar alpha, Scalar beta) {
            sa = std::sin(alpha);
            ca = std::cos(alpha);
            sb = std::sin(beta);
            cb = std::cos(beta);
            s2b = std::sin(Scalar(2) * beta);
        }
    };

    DynamicModel() = default;

    Self& setParameters(const Parameters& params) {
        if (!params.isValid()) {
            throw std::invalid_argument("[DynamicModel::setParameters] Invalid model parameters.");
        }
        mParams = params;
        return *this;
    }

    const Parameters& getParameters() const { return mParams; }

    Vector4 computeAcceleration(const State& x, const Input& u) {
        mTrigCache.update(x.getAlpha(), x.getBeta());

        computeMassMatrix();
        computeGravityVector();
        computeCoriolisDamping(x);

        Vector4 Bu = Vector4::Zero();
        Bu(0) = u.getFx();
        Bu(1) = u.getFy();

        const Vector4 rhs = Bu - mGravityVector - mCoriolisVector;
        Eigen::LDLT<MassMatrix> ldlt(mMassMatrix);
        if (ldlt.info() == Eigen::Success) {
            return ldlt.solve(rhs);
        }
        return mMassMatrix.colPivHouseholderQr().solve(rhs);
    }

    Derivative operator()(const StateVector& x, const Input& u) {
        return (*this)(State::FromVector(x), u);
    }

    Derivative operator()(const State& x, const Input& u) {
        Vector4 dq  = x.getVelocityState();
        Vector4 ddq = computeAcceleration(x, u);

        Derivative dxdt;
        dxdt << dq, ddq;
        return dxdt;
    }

    Derivative getSystemRhs(const State& x, const Input& u) {
        return (*this)(x, u);
    }

    const MassMatrix& getMassMatrix() const { return mMassMatrix; }
    const Vector4& getGravityVector() const { return mGravityVector; }
    const Vector4& getCoriolisVector() const { return mCoriolisVector; }

    MassMatrix evaluateMassMatrix(Scalar alpha, Scalar beta) {
        mTrigCache.update(alpha, beta);
        computeMassMatrix();
        return mMassMatrix;
    }

    Vector2 computeHorizontalCableProjection(Scalar alpha, Scalar beta) const {
        return Vector2(std::sin(alpha) * std::cos(beta), -std::sin(beta));
    }

    Eigen::Matrix<Scalar, 2, 2> computePlanarEffectiveMass(Scalar alpha, Scalar beta) const {
        const Vector2 w = computeHorizontalCableProjection(alpha, beta);
        Eigen::Matrix<Scalar, 2, 2> Mt = Eigen::Matrix<Scalar, 2, 2>::Zero();
        Mt(0, 0) = mParams.m_x;
        Mt(1, 1) = mParams.m_y;
        Mt.noalias() += mParams.m * (w * w.transpose());
        return Mt;
    }

    Scalar computeKineticEnergy(const State& x) const {
        const Scalar vx = x.getXdot();
        const Scalar vy = x.getYdot();
        const Vector3 v_load = x.getBallVelocity(mParams.ell);
        return Scalar(0.5) * (mParams.m_x * vx * vx + mParams.m_y * vy * vy +
                              mParams.m * v_load.squaredNorm());
    }

    Scalar computePotentialEnergy(const State& x) const {
        return -mParams.m * mParams.g * mParams.ell *
               std::cos(x.getAlpha()) * std::cos(x.getBeta());
    }

    Scalar computeTotalEnergy(const State& x) const {
        return computeKineticEnergy(x) + computePotentialEnergy(x);
    }

    Scalar computeMechanicalPower(const State& x, const Input& u) const {
        const Scalar vx = x.getXdot();
        const Scalar vy = x.getYdot();
        return u.getFx() * vx + u.getFy() * vy -
               mParams.D_x * vx * vx - mParams.D_y * vy * vy;
    }

    Scalar computeCableTension(const State& x, const Input& u) {
        const Vector4 ddq = computeAcceleration(x, u);
        const Scalar sa = mTrigCache.sa, ca = mTrigCache.ca;
        const Scalar sb = mTrigCache.sb, cb = mTrigCache.cb;
        const Scalar da = x.getAlphaDot(), db = x.getBetaDot();
        const Scalar xdd = ddq(0), ydd = ddq(1);
        return mParams.m * (mParams.g * ca * cb +
                            mParams.ell * (da * da * cb * cb + db * db) +
                            xdd * sa * cb - ydd * sb);
    }

    Scalar computeCableTensionClosedForm(const State& x, const Input& u) const {
        const Scalar sa = std::sin(x.getAlpha()), ca = std::cos(x.getAlpha());
        const Scalar sb = std::sin(x.getBeta()),  cb = std::cos(x.getBeta());
        const Scalar da = x.getAlphaDot(),        db = x.getBetaDot();
        const Vector2 w(sa * cb, -sb);
        const Vector2 f_net(u.getFx() - mParams.D_x * x.getXdot(),
                            u.getFy() - mParams.D_y * x.getYdot());
        const Scalar T_0 = mParams.m * (mParams.g * ca * cb +
                                        mParams.ell * (da * da * cb * cb + db * db));
        const Scalar w_M0inv_f = (w.x() * f_net.x()) / mParams.m_x +
                                 (w.y() * f_net.y()) / mParams.m_y;
        const Scalar w_M0inv_w = (w.x() * w.x()) / mParams.m_x +
                                 (w.y() * w.y()) / mParams.m_y;
        return (T_0 + mParams.m * w_M0inv_f) / (Scalar(1) + mParams.m * w_M0inv_w);
    }

    Scalar computeMinGuaranteedCableTension(Scalar theta_max,
                                            Scalar v_max,
                                            Scalar F_max) const {
        const Scalar sth = std::sin(theta_max);
        const Scalar cth = std::cos(theta_max);
        const Scalar m_min = std::min(mParams.m_x, mParams.m_y);
        const Scalar f_net_max = F_max + std::hypot(mParams.D_x, mParams.D_y) * v_max;
        const Scalar num = mParams.m * mParams.g * cth - (mParams.m / m_min) * f_net_max * sth;
        const Scalar den = Scalar(1) + (mParams.m / m_min) * sth * sth;
        return num / den;
    }

private:
    void computeMassMatrix() {
        mMassMatrix.setZero();
        const Scalar l = mParams.ell;
        const Scalar m = mParams.m;
        const auto& sa = mTrigCache.sa;
        const auto& ca = mTrigCache.ca;
        const auto& sb = mTrigCache.sb;
        const auto& cb = mTrigCache.cb;

        auto& M = mMassMatrix;
        M(0,0) = m + mParams.m_x;
        M(1,1) = m + mParams.m_y;
        M(0,2) = -l * m * ca * cb;
        M(0,3) =  l * m * sa * sb;
        M(1,3) =  l * m * cb;
        M(2,0) = M(0,2);
        M(2,2) = l * l * m * cb * cb;
        M(3,0) = M(0,3);
        M(3,1) = M(1,3);
        M(3,3) = l * l * m;
    }

    void computeGravityVector() {
        mGravityVector.setZero();
        const Scalar l = mParams.ell;
        const Scalar m = mParams.m;
        const Scalar g = mParams.g;
        const auto& sa = mTrigCache.sa;
        const auto& ca = mTrigCache.ca;
        const auto& sb = mTrigCache.sb;
        const auto& cb = mTrigCache.cb;

        mGravityVector(2) = g * l * m * sa * cb;
        mGravityVector(3) = g * l * m * sb * ca;
    }

    void computeCoriolisDamping(const State& state) {
        mCoriolisVector.setZero();

        const Scalar dx = state.getXdot(), dy = state.getYdot();
        const Scalar da = state.getAlphaDot(), db = state.getBetaDot();
        const Scalar l = mParams.ell;
        const Scalar m = mParams.m;
        const Scalar D_x = mParams.D_x;
        const Scalar D_y = mParams.D_y;

        const auto& sa = mTrigCache.sa;
        const auto& ca = mTrigCache.ca;
        const auto& sb = mTrigCache.sb;
        const auto& cb = mTrigCache.cb;
        const auto& s2b = mTrigCache.s2b;

        const Scalar lms = l * m;
        const Scalar lms_sacb = lms * sa * cb;
        const Scalar lms_sbca = lms * sb * ca;
        const Scalar l2m = l * l * m;

        mCoriolisVector(0) = D_x * dx + lms_sacb * (da * da + db * db) + Scalar(2) * lms_sbca * da * db;
        mCoriolisVector(1) = D_y * dy - lms * sb * db * db;
        mCoriolisVector(2) = -l2m * s2b * da * db;
        mCoriolisVector(3) = Scalar(0.5) * l2m * s2b * da * da;
    }

    MassMatrix mMassMatrix = MassMatrix::Zero();
    Vector4 mGravityVector = Vector4::Zero();
    Vector4 mCoriolisVector = Vector4::Zero();
    Parameters mParams{};
    TrigCache mTrigCache{};
};

} // namespace gantry

#endif // __GANTRY_DYNAMIC_MODEL_HPP__

