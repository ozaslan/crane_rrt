#ifndef GANTRY_CRANE_STATE_HPP
#define GANTRY_CRANE_STATE_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <array>
#include <vector>
#include <stdexcept>

namespace gantry {

template <typename ScalarT>
class CraneState {
public:
    using Scalar    = ScalarT;
    using Vector    = Eigen::Matrix<Scalar, 8, 1>;
    using Vector8   = Vector;
    using Vector4   = Eigen::Matrix<Scalar, 4, 1>;
    using Vector3   = Eigen::Matrix<Scalar, 3, 1>;
    using Vector2   = Eigen::Matrix<Scalar, 2, 1>;
    using Array4    = std::array<Scalar, 4>;
    using Self      = CraneState<Scalar>;
    using Ptr       = std::shared_ptr<Self>;
    using ConstPtr  = std::shared_ptr<const Self>;
    using StdVector = std::vector<Scalar>;

    enum class Index : int {
        X = 0, Y = 1, ALPHA = 2, BETA = 3,
        XDOT = 4, YDOT = 5, ALPHADOT = 6, BETADOT = 7
    };

    CraneState() : mVec(Vector::Zero()), mTime(Scalar(0)) {}
    explicit CraneState(const Vector& vec, Scalar time = Scalar(0))
        : mVec(vec), mTime(time) {}

    // --- Getters ---
    Scalar get(Index i) const { return mVec[static_cast<int>(i)]; }

    Scalar getX()        const { return get(Index::X); }
    Scalar getY()        const { return get(Index::Y); }
    Scalar getAlpha()    const { return get(Index::ALPHA); }
    Scalar getBeta()     const { return get(Index::BETA); }
    Scalar getXdot()     const { return get(Index::XDOT); }
    Scalar getYdot()     const { return get(Index::YDOT); }
    Scalar getAlphaDot() const { return get(Index::ALPHADOT); }
    Scalar getBetaDot()  const { return get(Index::BETADOT); }

    Vector4 getPositionState() const {
        return mVec.template segment<4>(0);
    }

    Vector4 getVelocityState() const {
        return mVec.template segment<4>(4);
    }

    Vector2 getTrolleyPosition2D() const {
        return Vector2(getX(), getY());
    }

    Vector2 getTrolleyVelocity2D() const {
        return Vector2(getXdot(), getYdot());
    }

    const Vector& toVector() const { return mVec; }
    Vector& toVector() { return mVec; }

    StdVector toStdVector() const {
        StdVector retval(static_cast<std::size_t>(mVec.size()));
        for (Eigen::Index i = 0; i < mVec.size(); ++i) {
            retval[static_cast<std::size_t>(i)] = mVec(i);
        }
        return retval;
    }

    Scalar getTime() const { return mTime; }
    void setTime(Scalar t) { mTime = t; }

    // --- Setters ---
    void set(Index i, Scalar value) { mVec[static_cast<int>(i)] = value; }

    void setX(Scalar v)        { set(Index::X, v); }
    void setY(Scalar v)        { set(Index::Y, v); }
    void setAlpha(Scalar v)    { set(Index::ALPHA, v); }
    void setBeta(Scalar v)     { set(Index::BETA, v); }
    void setXdot(Scalar v)     { set(Index::XDOT, v); }
    void setYdot(Scalar v)     { set(Index::YDOT, v); }
    void setAlphaDot(Scalar v) { set(Index::ALPHADOT, v); }
    void setBetaDot(Scalar v)  { set(Index::BETADOT, v); }

    static Self FromVector(const Vector& vec, Scalar time = Scalar(0)) {
        return Self(vec, time);
    }

    static Self FromVector(const StdVector& vec, Scalar time = Scalar(0)) {
        if (vec.size() != 8) {
            throw std::invalid_argument("CraneState::FromVector requires a vector of 8 elements.");
        }
        Self retval;
        retval.mVec = Eigen::Map<const Vector>(vec.data());
        retval.mTime = time;
        return retval;
    }

    Self& setPositionState(const Vector4& pos) {
        mVec.template segment<4>(0) = pos;
        return *this;
    }

    Self& setPositionState(const Array4& pos) {
        for (std::size_t i = 0; i < 4; ++i) {
            mVec(static_cast<Eigen::Index>(i)) = pos[i];
        }
        return *this;
    }

    Self& setVelocityState(const Vector4& vel) {
        mVec.template segment<4>(4) = vel;
        return *this;
    }

    Self& setVelocityState(const Array4& vel) {
        for (std::size_t i = 0; i < 4; ++i) {
            mVec(static_cast<Eigen::Index>(i + 4)) = vel[i];
        }
        return *this;
    }

    // --- Derived Quantities ---
    Vector3 getTrolleyPosition() const {
        return Vector3(getX(), getY(), Scalar(0));
    }

    Vector3 getBallPosition(Scalar ell) const {
        Vector3 rod_local(Scalar(0), Scalar(0), -ell);
        Eigen::AngleAxis<Scalar> Rx(getBeta(), Vector3::UnitX());
        Eigen::AngleAxis<Scalar> Ry(getAlpha(), Vector3::UnitY());
        Eigen::Matrix<Scalar, 3, 3> R = (Ry * Rx).toRotationMatrix();
        return getTrolleyPosition() + R * rod_local;
    }

    Vector3 getTrolleyVelocity() const {
        return Vector3(getXdot(), getYdot(), Scalar(0));
    }

    Vector3 getBallVelocity(Scalar ell) const {
        const Scalar dx = getXdot(), dy = getYdot();
        const Scalar da = getAlphaDot(), db = getBetaDot();
        const Scalar sa = std::sin(getAlpha());
        const Scalar ca = std::cos(getAlpha());
        const Scalar sb = std::sin(getBeta());
        const Scalar cb = std::cos(getBeta());

        Scalar vx = ell * sa * sb * db - ell * ca * cb * da + dx;
        Scalar vy = ell * cb * db + dy;
        Scalar vz = ell * sa * cb * da + ell * sb * ca * db;

        return Vector3(vx, vy, vz);
    }

    Vector2 getBallPosition2D(Scalar ell) const {
        const Vector3 p = getBallPosition(ell);
        return Vector2(p.x(), p.y());
    }

    Scalar getCompositeSwingAngle() const {
        const Scalar prod = std::clamp(std::cos(getAlpha()) * std::cos(getBeta()), Scalar(-1), Scalar(1));
        return std::acos(prod);
    }

    Scalar getAngularSwingSpeed() const {
        return std::hypot(getAlphaDot(), getBetaDot());
    }

    // --- Distance Metrics ---
    // Dimensionless phase-space distance metric on 4D trolley state [x, y, xdot, ydot]:
    //   dist = w_pos * (||p_xy - other.p_xy|| / L_0) + w_vel * (||v_xy - other.v_xy|| / v_0)
    Scalar phaseDistanceTo(const Self& other,
                           Scalar w_pos = Scalar(1.0),
                           Scalar w_vel = Scalar(0.0),
                           Scalar L_0 = Scalar(2.0),
                           Scalar v_0 = Scalar(2.0)) const {
        const Scalar dp = (getTrolleyPosition2D() - other.getTrolleyPosition2D()).norm() / L_0;
        const Scalar dv = (getTrolleyVelocity2D() - other.getTrolleyVelocity2D()).norm() / v_0;
        return w_pos * dp + w_vel * dv;
    }

    // Weighted distance across trolley position and velocity
    Scalar distanceTo(const Self& other,
                      Scalar w_pos = Scalar(1.0),
                      Scalar w_vel = Scalar(0.0)) const {
        return phaseDistanceTo(other, w_pos, w_vel);
    }

    // Full 8D state weighted Euclidean distance
    Scalar fullStateDistanceTo(const Self& other,
                               Scalar w_pos = Scalar(1.0),
                               Scalar w_vel = Scalar(0.0)) const {
        const Scalar dp2 = (getPositionState() - other.getPositionState()).squaredNorm();
        const Scalar dv2 = (getVelocityState() - other.getVelocityState()).squaredNorm();
        return std::sqrt(w_pos * dp2 + w_vel * dv2);
    }

    // --- Static Factory Methods ---
    static Self Zero() {
        return Self(Vector::Zero());
    }

    static Self FromParts(const Vector4& pos, const Vector4& vel, Scalar time = Scalar(0)) {
        Vector v;
        v << pos, vel;
        return Self(v, time);
    }

    static Self FromTrolleyPhase(Scalar x, Scalar y, Scalar xdot = Scalar(0), Scalar ydot = Scalar(0), Scalar time = Scalar(0)) {
        Vector v = Vector::Zero();
        v(static_cast<int>(Index::X))    = x;
        v(static_cast<int>(Index::Y))    = y;
        v(static_cast<int>(Index::XDOT)) = xdot;
        v(static_cast<int>(Index::YDOT)) = ydot;
        return Self(v, time);
    }

    // --- Interpolation ---
    static Self interpolate(const Self& a, const Self& b, Scalar alpha, bool use_slerp = false) {
        Vector result;

        // Linear for x, y
        result(static_cast<int>(Index::X)) =
            (Scalar(1) - alpha) * a.getX() + alpha * b.getX();
        result(static_cast<int>(Index::Y)) =
            (Scalar(1) - alpha) * a.getY() + alpha * b.getY();

        // Angular interpolation
        if (use_slerp) {
            result(static_cast<int>(Index::ALPHA)) =
                slerpAngle(a.getAlpha(), b.getAlpha(), alpha);
            result(static_cast<int>(Index::BETA)) =
                slerpAngle(a.getBeta(), b.getBeta(), alpha);
        } else {
            result(static_cast<int>(Index::ALPHA)) =
                (Scalar(1) - alpha) * a.getAlpha() + alpha * b.getAlpha();
            result(static_cast<int>(Index::BETA)) =
                (Scalar(1) - alpha) * a.getBeta() + alpha * b.getBeta();
        }

        // Linear for derivatives
        for (int i = static_cast<int>(Index::XDOT); i <= static_cast<int>(Index::BETADOT); ++i) {
            result(i) = (Scalar(1) - alpha) * a.mVec(i) + alpha * b.mVec(i);
        }

        Scalar t_interp = (Scalar(1) - alpha) * a.getTime() + alpha * b.getTime();
        return Self(result, t_interp);
    }

    // --- Comparison ---
    bool operator==(const Self& other) const {
        return mVec == other.mVec && mTime == other.mTime;
    }

    bool isApprox(const Self& other, Scalar tol = Scalar(1e-6)) const {
        return mVec.isApprox(other.mVec, tol) &&
               std::abs(mTime - other.mTime) < tol;
    }

    bool isValid() const {
        if (!std::isfinite(mTime)) {
            return false;
        }
        for (int i = 0; i < mVec.size(); ++i) {
            const Scalar val = mVec(i);
            if (!std::isfinite(val)) {
                return false;
            }
        }
        return true;
    }

    // --- CSV / YAML output ---
    std::string toCSV() const {
        std::ostringstream oss;
        for (int i = 0; i < 8; ++i) {
            oss << mVec(i) << ",";
        }
        oss << mTime;
        return oss.str();
    }

    std::string toYAML(const std::string& prefix = "") const {
        std::ostringstream oss;
        oss << prefix << "x: "        << getX()        << "\n"
            << prefix << "y: "        << getY()        << "\n"
            << prefix << "alpha: "    << getAlpha()    << "\n"
            << prefix << "beta: "     << getBeta()     << "\n"
            << prefix << "xdot: "     << getXdot()     << "\n"
            << prefix << "ydot: "     << getYdot()     << "\n"
            << prefix << "alphadot: " << getAlphaDot() << "\n"
            << prefix << "betadot: "  << getBetaDot()  << "\n"
            << prefix << "time: "     << mTime         << "\n";
        return oss.str();
    }

    // --- Debug print ---
    friend std::ostream& operator<<(std::ostream& os, const Self& s) {
        os << std::fixed << std::setprecision(4)
           << "CraneState: ["
           << s.getX() << ", " << s.getY() << ", "
           << s.getAlpha() << ", " << s.getBeta() << ", "
           << s.getXdot() << ", " << s.getYdot() << ", "
           << s.getAlphaDot() << ", " << s.getBetaDot() << ", "
           << "t=" << s.getTime()
           << "]";
        return os;
    }

private:
    static Scalar slerpAngle(Scalar theta1, Scalar theta2, Scalar alpha) {
        Scalar x1 = std::cos(theta1), y1 = std::sin(theta1);
        Scalar x2 = std::cos(theta2), y2 = std::sin(theta2);
        Scalar x_interp = (Scalar(1) - alpha) * x1 + alpha * x2;
        Scalar y_interp = (Scalar(1) - alpha) * y1 + alpha * y2;
        return std::atan2(y_interp, x_interp);
    }

    Vector mVec;
    Scalar mTime;
};

} // namespace gantry

#endif // GANTRY_CRANE_STATE_HPP
