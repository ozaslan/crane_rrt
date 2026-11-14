#ifndef ARENA_PRIMITIVE_BUNDLE_VISUALIZER_HPP
#define ARENA_PRIMITIVE_BUNDLE_VISUALIZER_HPP

#include "primitive_bundle.hpp"
#include "primitive_visualizer.hpp"
#include "aabb_visualizer.hpp"

namespace arena {

template<typename Scalar>
class BundleVisualizer {
public:
    using Bundle         = PrimitiveBundle<Scalar>;
    using PrimitiveT     = Primitive<Scalar>;
    using PrimitivePtr   = typename PrimitiveT::Ptr;
    using Marker         = visualization_msgs::msg::Marker;
    using MarkerArray    = visualization_msgs::msg::MarkerArray;
    using Params         = typename PrimitiveVisualizer<Scalar>::Params;
    using AABBParams     = typename AABBVisualizer<Scalar>::Params;
    using ColorT         = Color<Scalar>;

    // --- With optional AABB flag ---
    static MarkerArray toMarkers(const Bundle& bundle,
                                 const Params& params,
                                 int& id,
                                 bool addAABB = false) {
        MarkerArray array;
        appendMarkersRecursive(bundle, params, addAABB, array, ++id);
        return array;
    }

    // --- With full AABB customization ---
    static MarkerArray toMarkers(const Bundle& bundle,
                                 const Params& params,
                                 const AABBParams& aabbParams,
                                 int& id) {
        MarkerArray array;
        appendMarkersRecursive(bundle, params, aabbParams, array, ++id);
        return array;
    }

    // --- Delete all markers ---
    static MarkerArray deleteAllMarkers(const Params& params) {
        MarkerArray array;
        Marker marker;
        marker.action = Marker::DELETEALL;
        marker.header.frame_id = params.frame_id;
        marker.header.stamp = params.stamp;
        array.markers.push_back(marker);
        return array;
    }

private:
    static void appendMarkersRecursive(const Bundle& bundle,
                                       const Params& params,
                                       bool addAABB,
                                       MarkerArray& out_array,
                                       int& id) {
        if (addAABB) {
            AABBParams bundleAABBParams;
            bundleAABBParams.frame_id = params.frame_id;
            bundleAABBParams.stamp = params.stamp;
            bundleAABBParams.lifetime = params.lifetime;
            bundleAABBParams.line_width = 0.02;
            bundleAABBParams.ns = "bundle_aabb";
            bundleAABBParams.color = bundle.getColor();

            auto aabbMarkers = AABBVisualizer<Scalar>::visualizeAABB(bundle, bundleAABBParams, ++id, bundle.getTag());
            out_array.markers.insert(out_array.markers.end(),
                                     aabbMarkers.markers.begin(), aabbMarkers.markers.end());
        }

        for (const auto& child : bundle.getPrimitives()) {
            if (auto subbundle = std::dynamic_pointer_cast<Bundle>(child)) {
                appendMarkersRecursive(*subbundle, params, addAABB, out_array, ++id);
            } else {
                auto markers = PrimitiveVisualizer<Scalar>::toMarkers(*child, params, ++id, addAABB);
                out_array.markers.insert(out_array.markers.end(),
                                         markers.markers.begin(), markers.markers.end());
            }
        }
    }

    static void appendMarkersRecursive(const Bundle& bundle,
                                       const Params& params,
                                       const AABBParams& aabbParams,
                                       MarkerArray& out_array,
                                       int& id) {
        auto bundleMarkers = AABBVisualizer<Scalar>::visualizeAABB(bundle, aabbParams, ++id, bundle.getTag());
        out_array.markers.insert(out_array.markers.end(),
                                 bundleMarkers.markers.begin(), bundleMarkers.markers.end());

        for (const auto& child : bundle.getPrimitives()) {
            if (auto subbundle = std::dynamic_pointer_cast<Bundle>(child)) {
                appendMarkersRecursive(*subbundle, params, aabbParams, out_array, ++id);
            } else {
                auto markers = PrimitiveVisualizer<Scalar>::toMarkers(*child, params, aabbParams, ++id);
                out_array.markers.insert(out_array.markers.end(),
                                         markers.markers.begin(), markers.markers.end());
            }
        }
    }
};

} // namespace arena

#endif // ARENA_PRIMITIVE_BUNDLE_VISUALIZER_HPP
