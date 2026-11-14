#ifndef RRT_NODE_HPP
#define RRT_NODE_HPP

#include <cstddef>
#include <memory>
#include <vector>
#include "crane_rrt/crane_state.hpp"
#include "crane_rrt/crane_input.hpp"

namespace rrt {

template <typename ScalarT = double>
struct CraneRRTData {
    using Scalar = ScalarT;
    using State  = gantry::CraneState<Scalar>;
    using Input  = gantry::CraneInput<Scalar>;

    State state = State::Zero();
    Input input = Input::Zero();
    Scalar duration = Scalar(0);
    Scalar cost = Scalar(0);
    std::vector<State> trajectory{};
    std::vector<Input> input_trajectory{};

    // SST (Stable Sparse RRT, Li et al. 2016) active flag and active-child counter
    bool sst_active = true;
    std::size_t sst_children = 0;

    CraneRRTData() = default;

    CraneRRTData(const State& s, const Input& u = Input::Zero(), Scalar dur = Scalar(0), Scalar c = Scalar(0))
        : state(s), input(u), duration(dur), cost(c) {}
};

template <typename ScalarT, typename DataTypeT = CraneRRTData<ScalarT>>
class RRTNode {
public:
    using Scalar    = ScalarT;
    using DataType  = DataTypeT;
    using Self      = RRTNode<Scalar, DataType>;
    using Ptr       = std::shared_ptr<Self>;
    using ConstPtr  = std::shared_ptr<const Self>;
    using WeakPtr   = std::weak_ptr<Self>;

    RRTNode() = default;

    RRTNode(std::size_t index, const DataType& data, Ptr parent = nullptr)
        : mIndex(index), mData(data), mParent(parent) {}

    RRTNode(std::size_t index, DataType&& data, Ptr parent = nullptr)
        : mIndex(index), mData(std::move(data)), mParent(parent) {}

    std::size_t getIndex() const { return mIndex; }
    const DataType& getData() const { return mData; }
    DataType& getData() { return mData; }
    Ptr getParent() const { return mParent.lock(); }
    bool isRoot() const { return mParent.expired(); }

    void setParent(Ptr parent) { mParent = parent; }
    void setIndex(std::size_t index) { mIndex = index; }
    void setData(const DataType& data) { mData = data; }

private:
    std::size_t mIndex = 0;
    DataType mData{};
    WeakPtr mParent{};
};

} // namespace rrt

#endif // RRT_NODE_HPP

