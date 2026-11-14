#ifndef ARENA_YAML_UTILS_HPP
#define ARENA_YAML_UTILS_HPP

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdexcept>

namespace arena {

template<typename Scalar>
Eigen::Transform<Scalar, 3, Eigen::Affine> LoadPose(const YAML::Node& node) {
    using Pose = Eigen::Transform<Scalar, 3, Eigen::Affine>;

    Pose T = Pose::Identity();
    if (!node || !node.IsMap()) return T;

    if (node["position"]) {
        if (!node["position"].IsSequence() || node["position"].size() != 3)
            throw std::runtime_error("'pose.position' must be a sequence of 3 numbers.");
        for (int i = 0; i < 3; ++i)
            T.translation()[i] = node["position"][i].as<Scalar>();
    }

    if (node["orientation"]) {
        if (!node["orientation"].IsSequence() || node["orientation"].size() != 4)
            throw std::runtime_error("'pose.orientation' must be a sequence of 4 numbers.");
        Eigen::Quaternion<Scalar> q;
        for (int i = 0; i < 4; ++i)
            q.coeffs()[i] = node["orientation"][i].as<Scalar>();
        T.linear() = q.normalized().toRotationMatrix();
    }

    return T;
}

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> LoadScale(const YAML::Node& node) {
    using Point = Eigen::Matrix<Scalar, 3, 1>;

    Point s = Point::Ones();
    if (!node || !node.IsSequence()) return s;
    if (node.size() != 3)
        throw std::runtime_error("'scale' must be a sequence of 3 numbers.");
    for (int i = 0; i < 3; ++i)
        s[i] = node[i].as<Scalar>();
    return s;
}

template<typename Scalar>
Eigen::Matrix<Scalar, 4, 1> LoadColor(const YAML::Node& node) {
    using Color = Eigen::Matrix<Scalar, 4, 1>;
    if (!node || !node.IsSequence() || node.size() != 4)
        throw std::runtime_error("'color' must be a list of 4 numbers (RGBA).");

    Color color;
    for (int i = 0; i < 4; ++i) {
        Scalar value = node[i].as<Scalar>();
        if (value < Scalar(0.0) || value > Scalar(1.0)) {
            throw std::runtime_error(
                "'color[" + std::to_string(i) + "]' must be in [0.0, 1.0], got " + std::to_string(value));
        }
        color[i] = value;
    }
    return color;
}

} // namespace arena

#endif // ARENA_YAML_UTILS_HPP
