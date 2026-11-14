#ifndef ARENA_ARENA_HPP
#define ARENA_ARENA_HPP

#include <string>
#include <memory>
#include <vector>

#include "primitive_bundle.hpp"

namespace arena {

template<typename Scalar>
class Arena {
public:
    using Self           = Arena<Scalar>;
    using Primitive      = arena::Primitive<Scalar>;
    using PrimitivePtr   = typename Primitive::Ptr;
    using Bundle         = arena::PrimitiveBundle<Scalar>;
    using BundlePtr      = typename Bundle::Ptr;
    using PrimitiveList  = std::vector<PrimitivePtr>;
    using AABB           = arena::AABB<Scalar>;
    using Point          = typename Primitive::Point;
    using Ptr            = std::shared_ptr<Self>;

    Arena(std::string name, std::string id, BundlePtr root)
        : mName(std::move(name)), mId(std::move(id)), mRoot(std::move(root)) {}

    const std::string& getName() const { return mName; }
    const std::string& getId() const { return mId; }

    BundlePtr getRoot() const { return mRoot; }

    AABB getAABB() const {
        return mRoot ? mRoot->getAABB() : AABB();
    }

    // Return a flattened list of all non-bundle primitives
    PrimitiveList flatten() const {
        PrimitiveList result;
        flattenRecursive(mRoot, result);
        return result;
    }

    int countPrimitives() const {
        return getRoot()->countPrimitives();
    }

    bool checkCollision(const Primitive& other) const {
        return mRoot->checkCollision(other);
    }

    bool checkCollision(const Bundle& other) const {
        return mRoot->checkCollision(other);
    }

private:
    std::string mName;
    std::string mId;
    BundlePtr   mRoot;

    static void flattenRecursive(const PrimitivePtr& node, PrimitiveList& out) {
        auto bundle = std::dynamic_pointer_cast<Bundle>(node);
        if (bundle) {
            for (const auto& child : bundle->getPrimitives()) {
                flattenRecursive(child, out);
            }
        } else {
            out.push_back(node);
        }
    }
};

} // namespace arena

#endif // ARENA_ARENA_HPP
