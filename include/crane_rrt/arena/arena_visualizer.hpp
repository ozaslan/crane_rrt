#ifndef ARENA_VISUALIZER_HPP
#define ARENA_VISUALIZER_HPP

#include "arena.hpp"
#include "bundle_visualizer.hpp"

namespace arena {

template<typename Scalar>
class ArenaVisualizer {
public:
    using ArenaType     = Arena<Scalar>;
    using Params        = typename PrimitiveVisualizer<Scalar>::Params;
    using AABBParams    = typename AABBVisualizer<Scalar>::Params;
    using MarkerArray   = visualization_msgs::msg::MarkerArray;
    using Bundle        = PrimitiveBundle<Scalar>;

    // --- With optional AABB flag ---
    static MarkerArray toMarkers(const ArenaType& arena,
                                 const Params& params,
                                 int& id_start,
                                 bool addAABB = false) {
        return BundleVisualizer<Scalar>::toMarkers(*arena.getRoot(), params, ++id_start, addAABB);
    }

    // --- With full AABB customization ---
    static MarkerArray toMarkers(const ArenaType& arena,
                                 const Params& params,
                                 const AABBParams& aabbParams,
                                 int& id_start) {
        return BundleVisualizer<Scalar>::toMarkers(*arena.getRoot(), params, aabbParams, ++id_start);
    }
};

} // namespace arena

#endif // ARENA_VISUALIZER_HPP
