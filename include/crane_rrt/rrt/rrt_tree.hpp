#ifndef RRT_TREE_HPP
#define RRT_TREE_HPP

#include <vector>
#include <memory>
#include <stdexcept>
#include <limits>
#include <algorithm>
#include <functional>
#include <random>

#include "rrt_node.hpp"

namespace rrt {

template <typename ScalarT, typename DataTypeT = CraneRRTData<ScalarT>>
class RRTTree {
public:
    using Scalar       = ScalarT;
    using DataType     = DataTypeT;
    using Node         = RRTNode<Scalar, DataType>;
    using NodePtr      = typename Node::Ptr;
    using NodeConstPtr = typename Node::ConstPtr;
    using Self         = RRTTree<Scalar, DataType>;
    using Ptr          = std::shared_ptr<Self>;
    using ConstPtr     = std::shared_ptr<const Self>;
    using DistanceFunc = std::function<Scalar(const DataType&, const DataType&)>;

    // Default constructor
    RRTTree() = default;

    // Constructor with preallocation
    explicit RRTTree(std::size_t reserved_count) {
        reserve(reserved_count);
    }

    // Add a node with optional parent (copy or move)
    NodePtr addNode(const DataType& data, NodePtr parent = nullptr) {
        std::size_t index = mNodes.size();
        NodePtr newNode = std::make_shared<Node>(index, data, parent);
        mNodes.push_back(newNode);
        mScratchIndices.push_back(index);
        return newNode;
    }

    NodePtr addNode(DataType&& data, NodePtr parent = nullptr) {
        std::size_t index = mNodes.size();
        NodePtr newNode = std::make_shared<Node>(index, std::move(data), parent);
        mNodes.push_back(newNode);
        mScratchIndices.push_back(index);
        return newNode;
    }

    // Reserve storage
    void reserve(std::size_t count) {
        mNodes.reserve(count);
        mScratchIndices.reserve(count);
    }

    // Clear all nodes
    void clear() {
        mNodes.clear();
        mScratchIndices.clear();
    }

    // Accessors
    bool empty() const { return mNodes.empty(); }
    std::size_t size() const { return mNodes.size(); }

    const std::vector<NodePtr>& getNodes() const { return mNodes; }

    NodePtr getNode(std::size_t index) const {
        if (index >= mNodes.size())
            throw std::out_of_range("RRTTree: index out of range");
        return mNodes[index];
    }

    NodePtr getRoot() const {
        if (mNodes.empty()) return nullptr;
        return mNodes.front();
    }

    // Nearest-neighbor search with optional randomized index skipping
    // (Paper Section V-B5: increments node index by random integer in [min_step, max_step])
    template <typename DistFn, typename RNG>
    NodePtr findNearest(const DataType& query,
                        DistFn&& dist_fn,
                        RNG& rng,
                        std::size_t min_step = 1,
                        std::size_t max_step = 1) const {
        if (mNodes.empty()) {
            return nullptr;
        }
        if (min_step < 1) min_step = 1;
        if (max_step < min_step) max_step = min_step;

        NodePtr best_node = mNodes.front();
        Scalar best_dist = dist_fn(best_node->getData(), query);

        if (mNodes.size() == 1) {
            return best_node;
        }

        if (min_step == 1 && max_step == 1) {
            for (std::size_t i = 1; i < mNodes.size(); ++i) {
                const Scalar d = dist_fn(mNodes[i]->getData(), query);
                if (d < best_dist) {
                    best_dist = d;
                    best_node = mNodes[i];
                }
            }
            return best_node;
        }

        std::uniform_int_distribution<std::size_t> step_dist(min_step, max_step);

        std::size_t idx = step_dist(rng);
        if (idx >= mNodes.size()) {
            idx = mNodes.size() - 1;
        }

        while (idx < mNodes.size()) {
            const Scalar d = dist_fn(mNodes[idx]->getData(), query);
            if (d < best_dist) {
                best_dist = d;
                best_node = mNodes[idx];
            }
            idx += step_dist(rng);
        }

        return best_node;
    }

    // Equal-budget uniform random subset nearest-neighbor search (Priority 1 ablation):
    // Determines the exact candidate count K_q that strided search would evaluate for the current
    // tree size N, then draws K_q distinct candidate nodes uniformly at random without replacement
    // in O(K_q) time via partial Fisher-Yates swaps on mScratchIndices.
    template <typename DistFn, typename RNG>
    NodePtr findNearestRandomSubset(const DataType& query,
                                    DistFn&& dist_fn,
                                    RNG& rng,
                                    std::size_t min_step = 1,
                                    std::size_t max_step = 15) const {
        if (mNodes.empty()) {
            return nullptr;
        }
        const std::size_t N = mNodes.size();
        if (N == 1) {
            (void)dist_fn(mNodes.front()->getData(), query);
            return mNodes.front();
        }
        if (min_step < 1) min_step = 1;
        if (max_step < min_step) max_step = min_step;

        std::uniform_int_distribution<std::size_t> step_dist(min_step, max_step);
        std::size_t K_q = 1; // Root node (index 0) is always evaluated in striding
        std::size_t idx = step_dist(rng);
        if (idx >= N) {
            idx = N - 1;
        }
        while (idx < N) {
            ++K_q;
            idx += step_dist(rng);
        }
        if (K_q > N) {
            K_q = N;
        }

        if (mScratchIndices.size() != N) {
            mScratchIndices.resize(N);
            for (std::size_t i = 0; i < N; ++i) {
                mScratchIndices[i] = i;
            }
        }

        NodePtr best_node = nullptr;
        Scalar best_dist = std::numeric_limits<Scalar>::infinity();
        for (std::size_t j = 0; j < K_q; ++j) {
            std::uniform_int_distribution<std::size_t> pick_dist(j, N - 1);
            const std::size_t r = pick_dist(rng);
            std::swap(mScratchIndices[j], mScratchIndices[r]);
            const NodePtr& cand = mNodes[mScratchIndices[j]];
            const Scalar d = dist_fn(cand->getData(), query);
            if (d < best_dist) {
                best_dist = d;
                best_node = cand;
            }
        }
        return best_node;
    }

    // Standard exhaustive nearest-neighbor search
    template <typename DistFn>
    NodePtr findNearest(const DataType& query, DistFn&& dist_fn) const {
        if (mNodes.empty()) {
            return nullptr;
        }
        NodePtr best_node = mNodes.front();
        Scalar best_dist = dist_fn(best_node->getData(), query);

        for (std::size_t i = 1; i < mNodes.size(); ++i) {
            const Scalar d = dist_fn(mNodes[i]->getData(), query);
            if (d < best_dist) {
                best_dist = d;
                best_node = mNodes[i];
            }
        }
        return best_node;
    }

    // Reconstruct path from root to target node
    std::vector<NodePtr> extractNodePath(NodePtr target) const {
        std::vector<NodePtr> path;
        NodePtr curr = target;
        while (curr != nullptr) {
            path.push_back(curr);
            curr = curr->getParent();
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

private:
    std::vector<NodePtr> mNodes;
    mutable std::vector<std::size_t> mScratchIndices;
};

} // namespace rrt

#endif // RRT_TREE_HPP
