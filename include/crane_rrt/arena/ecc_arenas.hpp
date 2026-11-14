#ifndef CRANE_RRT_ECC_ARENAS_HPP
#define CRANE_RRT_ECC_ARENAS_HPP

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "crane_rrt/arena/arena.hpp"
#include "crane_rrt/arena/arena_factory.hpp"
#include "crane_rrt/arena/color.hpp"
#include "crane_rrt/arena/primitive_bundle.hpp"
#include "crane_rrt/arena/sphere.hpp"
#include "crane_rrt/crane_state.hpp"

namespace arena {

template <typename Scalar = float>
struct ECCArenaSpec {
    std::string id;
    typename Arena<Scalar>::Ptr arena;
    gantry::CraneState<double> start;
    gantry::CraneState<double> goal;
    bool use_soft_corridor = false;
    double corridor_half_width = 1.25;
};

namespace detail {

template <typename Scalar>
inline void addObstacleColumn(const typename PrimitiveBundle<Scalar>::Ptr& root,
                              float cx,
                              float cy,
                              float radius,
                              const Color<Scalar>& color) {
    using SphereT = Sphere<Scalar>;
    using Point   = typename SphereT::Point;
    for (float z : {-1.1f, -0.75f, -0.4f, -0.05f}) {
        auto s = std::make_shared<SphereT>();
        s->setPosition(Point(static_cast<Scalar>(cx),
                             static_cast<Scalar>(cy),
                             static_cast<Scalar>(z)));
        s->setScale(Point(static_cast<Scalar>(2.0f * radius),
                          static_cast<Scalar>(2.0f * radius),
                          static_cast<Scalar>(2.0f * radius)));
        s->setColor(color);
        root->addChild(s);
    }
}

template <typename Scalar>
inline void addPayloadLevelObstacle(const typename PrimitiveBundle<Scalar>::Ptr& root,
                                    float cx,
                                    float cy,
                                    float radius,
                                    const Color<Scalar>& color,
                                    float z_center = -0.98f) {
    using SphereT = Sphere<Scalar>;
    using Point   = typename SphereT::Point;
    auto s = std::make_shared<SphereT>();
    s->setPosition(Point(static_cast<Scalar>(cx),
                         static_cast<Scalar>(cy),
                         static_cast<Scalar>(z_center)));
    s->setScale(Point(static_cast<Scalar>(2.0f * radius),
                      static_cast<Scalar>(2.0f * radius),
                      static_cast<Scalar>(2.0f * radius)));
    s->setColor(color);
    root->addChild(s);
}

} // namespace detail

template <typename Scalar = float>
inline typename Arena<Scalar>::Ptr createSlalomCorridorArena() {
    auto root = std::make_shared<PrimitiveBundle<Scalar>>();
    const Color<Scalar> col(Scalar(0.20), Scalar(0.49), Scalar(0.66), Scalar(0.45));
    for (float y = -4.2f; y <= 0.9f; y += 0.65f) {
        detail::addObstacleColumn<Scalar>(root, -1.3f, y, 0.42f, col);
    }
    for (float y = -0.9f; y <= 4.2f; y += 0.65f) {
        detail::addObstacleColumn<Scalar>(root, 1.3f, y, 0.42f, col);
    }
    return std::make_shared<Arena<Scalar>>("slalom_corridor", "arena_slalom", root);
}

template <typename Scalar = float>
inline typename Arena<Scalar>::Ptr createDensePillarYardArena() {
    auto root = std::make_shared<PrimitiveBundle<Scalar>>();
    const Color<Scalar> col(Scalar(0.24), Scalar(0.56), Scalar(0.44), Scalar(0.45));
    const std::vector<std::pair<float, float>> pillars = {
        {-2.2f,  2.0f}, { 0.0f,  2.3f}, { 2.2f,  2.0f},
        {-2.0f,  0.0f}, { 0.0f,  0.0f}, { 2.0f,  0.0f},
        {-2.2f, -2.0f}, { 0.0f, -2.3f}, { 2.2f, -2.0f}
    };
    for (const auto& [cx, cy] : pillars) {
        detail::addObstacleColumn<Scalar>(root, cx, cy, 0.58f, col);
    }
    return std::make_shared<Arena<Scalar>>("dense_pillar_yard", "arena_dense", root);
}

template <typename Scalar = float>
inline typename Arena<Scalar>::Ptr createPayloadClutterCorridorArena() {
    auto root = std::make_shared<PrimitiveBundle<Scalar>>();
    const Color<Scalar> col(Scalar(0.76), Scalar(0.39), Scalar(0.20), Scalar(0.50));
    const float ex = 0.7355399f, ey = 0.6774814f;
    const float nx = -ey,        ny = ex;
    auto addAtCorridorCoord = [&](float s_lon, float d_perp, float radius) {
        const float cx = s_lon * ex + d_perp * nx;
        const float cy = s_lon * ey + d_perp * ny;
        if (cx >= -4.2f && cx <= 4.2f && cy >= -4.2f && cy <= 4.2f) {
            detail::addPayloadLevelObstacle<Scalar>(root, cx, cy, radius, col, -0.98f);
        }
    };

    const std::vector<float> odd_s  = {-3.0f, -1.0f, 1.0f, 3.0f};
    const std::vector<float> odd_d  = {-2.36f, -1.18f, 0.00f, 1.18f, 2.36f};
    for (float s_lon : odd_s) {
        for (float d_perp : odd_d) {
            addAtCorridorCoord(s_lon, d_perp, 0.27f);
        }
    }
    const std::vector<float> even_s = {-2.0f, 0.0f, 2.0f};
    const std::vector<float> even_d = {-2.95f, -1.77f, -0.59f, 0.59f, 1.77f, 2.95f};
    for (float s_lon : even_s) {
        for (float d_perp : even_d) {
            addAtCorridorCoord(s_lon, d_perp, 0.27f);
        }
    }
    return std::make_shared<Arena<Scalar>>("payload_clutter_corridor", "arena_payload_clutter", root);
}

inline std::string resolveArenaYamlPath(const std::string& relOrAbs) {
    if (std::filesystem::exists(relOrAbs)) {
        return relOrAbs;
    }
    try {
        const std::string shareDir = ament_index_cpp::get_package_share_directory("crane_rrt");
        const auto candidate = std::filesystem::path(shareDir) / relOrAbs;
        if (std::filesystem::exists(candidate)) {
            return candidate.string();
        }
    } catch (const std::exception&) {
    }
    return relOrAbs;
}

template <typename Scalar = float>
inline typename Arena<Scalar>::Ptr loadArenaByNameOrPath(const std::string& spec) {
    if (spec.empty() || spec == "none") {
        return nullptr;
    }
    if (spec == "Arena2_SlalomCorridor") {
        return createSlalomCorridorArena<Scalar>();
    }
    if (spec == "Arena3_DensePillarYard") {
        return createDensePillarYardArena<Scalar>();
    }
    if (spec == "Arena4_PayloadClutterCorridor") {
        return createPayloadClutterCorridorArena<Scalar>();
    }
    const std::string yamlRel = (spec == "Arena1_ImageIndustrial")
        ? "arenas/arena_from_images.arena.yaml"
        : spec;
    const std::string resolved = resolveArenaYamlPath(yamlRel);
    return ArenaFactory<Scalar>::LoadFromFile(resolved);
}

template <typename Scalar = float>
inline std::vector<ECCArenaSpec<Scalar>> buildECCArenas(const std::string& arena1YamlPath = "arenas/arena_from_images.arena.yaml") {
    using State = gantry::CraneState<double>;
    const std::string resolved1 = resolveArenaYamlPath(arena1YamlPath);
    auto arena1 = ArenaFactory<Scalar>::LoadFromFile(resolved1);
    std::vector<ECCArenaSpec<Scalar>> arenas = {
        {"Arena1_ImageIndustrial", arena1,
         State::FromTrolleyPhase(-3.8,  3.5, 0.0, 0.0),
         State::FromTrolleyPhase( 3.8, -3.5, 0.0, 0.0), false, 1.25},
        {"Arena2_SlalomCorridor", createSlalomCorridorArena<Scalar>(),
         State::FromTrolleyPhase(-3.8, -3.5, 0.0, 0.0),
         State::FromTrolleyPhase( 3.8,  3.5, 0.0, 0.0), false, 1.25},
        {"Arena3_DensePillarYard", createDensePillarYardArena<Scalar>(),
         State::FromTrolleyPhase(-3.8,  3.5, 0.0, 0.0),
         State::FromTrolleyPhase( 3.8, -3.5, 0.0, 0.0), false, 1.25},
        {"Arena4_PayloadClutterCorridor", createPayloadClutterCorridorArena<Scalar>(),
         State::FromTrolleyPhase(-3.8, -3.5, 0.0, 0.0),
         State::FromTrolleyPhase( 3.8,  3.5, 0.0, 0.0), true,  1.25}
    };
    for (const auto& ar : arenas) {
        if (ar.arena) {
            (void)ar.arena->getAABB();
        }
    }
    return arenas;
}

} // namespace arena

#endif // CRANE_RRT_ECC_ARENAS_HPP

