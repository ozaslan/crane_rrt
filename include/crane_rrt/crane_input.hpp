#ifndef GANTRY_CRANE_INPUT_HPP
#define GANTRY_CRANE_INPUT_HPP

#include <Eigen/Core>
#include <memory>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <array>
#include <algorithm>

namespace gantry {

template <typename ScalarT>
class CraneInput {
public:
    using Scalar   = ScalarT;
    using Vector   = Eigen::Matrix<Scalar, 2, 1>;
    using Self     = CraneInput<Scalar>;
    using Array2   = std::array<Scalar, 2>;
    using Ptr      = std::shared_ptr<Self>;
    using ConstPtr = std::shared_ptr<const Self>;

    enum class Index : int {
        FX = 0,
        FY = 1
    };

    CraneInput() : mVec(Vector::Zero()), mTime(Scalar(0)) {}
    explicit CraneInput(const Vector& vec, Scalar time = Scalar(0))
        : mVec(vec), mTime(time) {}

    // --- Getters ---
    Scalar get(Index i) const { return mVec[static_cast<int>(i)]; }
    Scalar getFx() const { return get(Index::FX); }
    Scalar getFy() const { return get(Index::FY); }

    Scalar operator[](int i) const { return mVec(i); }
    Scalar& operator[](int i) { return mVec(i); }

    Scalar norm() const { return mVec.norm(); }

    const Vector& toVector() const { return mVec; }
    Vector& toVector() { return mVec; }

    Array2 toArray() const { return Array2{mVec(0), mVec(1)}; }

    Scalar getTime() const { return mTime; }

    // --- Setters ---
    void set(Index i, Scalar value) { mVec[static_cast<int>(i)] = value; }
    void setFx(Scalar fx) { set(Index::FX, fx); }
    void setFy(Scalar fy) { set(Index::FY, fy); }
    void setVector(const Vector& vec) { mVec = vec; }
    void setTime(Scalar t) { mTime = t; }

    // --- Factory ---
    static Self Zero(Scalar t = Scalar(0)) {
        return Self(Vector::Zero(), t);
    }

    static Self FromParts(Scalar fx, Scalar fy, Scalar t = Scalar(0)) {
        return Self(Vector(fx, fy), t);
    }

    // --- Interpolation ---
    static Self interpolate(const Self& a, const Self& b, Scalar alpha) {
        const Scalar clamped = std::clamp(alpha, Scalar(0), Scalar(1));
        Vector result = (Scalar(1) - clamped) * a.mVec + clamped * b.mVec;
        Scalar t_interp = (Scalar(1) - clamped) * a.mTime + clamped * b.mTime;
        return Self(result, t_interp);
    }

    Self interpolate(const Self& other, Scalar alpha) const {
        return interpolate(*this, other, alpha);
    }

    // --- Comparison ---
    bool operator==(const Self& other) const {
        return mVec == other.mVec && mTime == other.mTime;
    }

    bool isApprox(const Self& other, Scalar tol = Scalar(1e-6)) const {
        return mVec.isApprox(other.mVec, tol) &&
               std::abs(mTime - other.mTime) < tol;
    }

    // --- CSV / YAML ---
    std::string toCSV() const {
        std::ostringstream oss;
        oss << mVec(0) << "," << mVec(1) << "," << mTime;
        return oss.str();
    }

    std::string toYAML(const std::string& prefix = "") const {
        std::ostringstream oss;
        oss << prefix << "fx: "   << getFx()   << "\n"
            << prefix << "fy: "   << getFy()   << "\n"
            << prefix << "time: " << mTime     << "\n";
        return oss.str();
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

    // --- Debug print ---
    friend std::ostream& operator<<(std::ostream& os, const Self& u) {
        os << std::fixed << std::setprecision(4)
           << "CraneInput: ["
           << u.getFx() << ", " << u.getFy() << ", "
           << "t=" << u.getTime() << "]";
        return os;
    }

private:
    Vector mVec;
    Scalar mTime;
};

} // namespace gantry

#endif // GANTRY_CRANE_INPUT_HPP
