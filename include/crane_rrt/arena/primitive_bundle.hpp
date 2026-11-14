#ifndef ARENA_PRIMITIVE_BUNDLE_HPP
#define ARENA_PRIMITIVE_BUNDLE_HPP

#include <vector>
#include <memory>

#include "primitive.hpp"

namespace arena {

template<typename Scalar>
class PrimitiveBundle : public Primitive<Scalar> {
public:
    using Self     = PrimitiveBundle<Scalar>;
    using Base     = Primitive<Scalar>;
    using Ptr      = typename std::shared_ptr<Self>;
    using Point    = typename Base::Point;
    using AABB     = typename Base::AABB;
    using PrimitiveList = std::vector<typename Base::Ptr>;
    using Color   = typename Base::Color;


    PrimitiveBundle() = default;

    explicit PrimitiveBundle(const PrimitiveList& prm)
        : mPrims(prm), mAABBDirty(true) {}

    Color getColor() const override {
        if(mPrims.empty()){
            return Color::Black();
        } else {
            return mPrims.back()->getColor();
        }
    }

    Self& addChild(const Ptr& prm) {
        mPrims.emplace_back(prm);
        mAABBDirty = true;
        return *this;
    }

    Self& addChild(const typename Base::Ptr& prm) {
        mPrims.emplace_back(prm);
        mAABBDirty = true;
        return *this;
    }

    const PrimitiveList& getPrimitives() const {
        return mPrims;
    }

    void invalidateAABB() const {
        mAABBDirty = true;
    }

    AABB getAABB() const override {
        if (mAABBDirty) {
            AABB box;
            for (const auto& prm : mPrims) {
                box.expandToInclude(prm->getAABB());
            }
            mCachedAABB = box;
            mAABBDirty = false;
        }
        return mCachedAABB;
    }

    typename Base::Ptr clone() const override {
        PrimitiveList clones;
        for (const auto& prm : mPrims) {
            clones.push_back(prm->clone());
        }
        return std::make_shared<Self>(clones);
    }

    bool checkRoughCollision(const Base& other) const {
        return getAABB().checkCollision(other.getAABB());
    }

    bool checkRoughCollision(const Self& other) const {
        return getAABB().checkCollision(other.getAABB());
    }

    bool checkCollision(const Base& other) const {

        if(!checkRoughCollision(other)){
            return false;
        }

        for (const auto& prm : mPrims) {
            if (prm->checkCollision(other)) {
                return true;
            }
        }

        return false;
    }

    bool checkCollision(const Self& other) const {

        if(!checkRoughCollision(other)){
            return false;
        }

        for (const auto& a : mPrims) {
            for (const auto& b : other.mPrims) {
                if (a->checkCollision(*b)) {
                    return true;
                }
            }
        }
        return false;
    }

    size_t countPrimitives() const {
        size_t count = 0;
        for (const auto& prm : mPrims) {
            auto subbundle = std::dynamic_pointer_cast<Self>(prm);
            if (subbundle) {
                count += subbundle->countPrimitives(); // recurse
            } else {
                ++count; // count leaf primitive
            }
        }
        return count;
    }

    void clear() override {
        for(auto p : mPrims){
            p->clear();
        }

        mPrims.clear();
        mAABBDirty = true;
    }


private:
    PrimitiveList mPrims;
    mutable bool mAABBDirty = true;
    mutable AABB mCachedAABB{};
};

} // namespace arena

#endif // ARENA_OBSTACLE_BUNDLE_HPP
