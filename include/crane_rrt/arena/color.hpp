#ifndef ARENA_COLOR_HPP
#define ARENA_COLOR_HPP

#include <algorithm>    // std::clamp
#include <iostream>     // std::ostream
#include <Eigen/Core>   // Eigen::Matrix

namespace arena {

template<typename Scalar = double>
struct Color {
    Scalar r = 1.0;
    Scalar g = 1.0;
    Scalar b = 1.0;
    Scalar a = 1.0;

    Color() = default;

    Color(Scalar r_, Scalar g_, Scalar b_, Scalar a_ = Scalar(1))
        : r(r_), g(g_), b(b_), a(a_) {}

    Color(const Eigen::Matrix<Scalar, 4, 1> &color)
        : r(color(0)), g(color(1)), b(color(2)), a(color(3)) {}

    Eigen::Matrix<Scalar, 4, 1> toVector() const {
        return Eigen::Matrix<Scalar, 4, 1>(r, g, b, a);
    }

    void clamp() {
        r = std::clamp(r, Scalar(0), Scalar(1));
        g = std::clamp(g, Scalar(0), Scalar(1));
        b = std::clamp(b, Scalar(0), Scalar(1));
        a = std::clamp(a, Scalar(0), Scalar(1));
    }

    static Color Red(Scalar alpha = 1.0)     { return {1.0, 0.0, 0.0, alpha}; }
    static Color Green(Scalar alpha = 1.0)   { return {0.0, 1.0, 0.0, alpha}; }
    static Color Blue(Scalar alpha = 1.0)    { return {0.0, 0.0, 1.0, alpha}; }
    static Color White(Scalar alpha = 1.0)   { return {1.0, 1.0, 1.0, alpha}; }
    static Color Black(Scalar alpha = 1.0)   { return {0.0, 0.0, 0.0, alpha}; }
    static Color Grey(Scalar alpha = 1.0)   { return {0.4, 0.4, 0.4, alpha}; }
    static Color Gray(Scalar alpha = 1.0)   { return Grey(); }
    static Color Transparent()              { return {0.0, 0.0, 0.0, 0.0}; }

    bool operator==(const Color& other) const {
        return r == other.r && g == other.g &&
               b == other.b && a == other.a;
    }

    bool operator!=(const Color& other) const {
        return !(*this == other);
    }
};

// Stream output operator
template<typename Scalar>
std::ostream& operator<<(std::ostream& os, const Color<Scalar>& c) {
    os << "rgba(" << c.r << ", " << c.g << ", " << c.b << ", " << c.a << ")";
    return os;
}

} // namespace arena

#endif