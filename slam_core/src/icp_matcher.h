#ifndef ICP_MATCHER_H
#define ICP_MATCHER_H

#include "types.h"
#include <Eigen/Dense>
#include <vector>

namespace slam {

struct CorrespondencePair {
    Eigen::Vector2d source;  // Point from source scan (P)
    Eigen::Vector2d target;  // Corresponding point from target scan (Q)
    double weight;           // Correspondence weight (1.0 for uniform)
    
    CorrespondencePair(const Eigen::Vector2d& src, const Eigen::Vector2d& tgt, double w = 1.0)
        : source(src), target(tgt), weight(w) {}
};

// YOUR TASK: Implement this core function
// Given corresponding point pairs with weights, compute the optimal transformation
// that minimizes the weighted sum of squared distances:
//   E(R, t) = Σ w_i * ||R*p_i + t - q_i||²
//
// STEPS TO IMPLEMENT:
// 1. Compute weighted centroids of source and target points
// 2. Center both point clouds by subtracting their respective centroids
// 3. Build the cross-covariance matrix H = Σ w_i * (p_i - p̄) * (q_i - q̄)ᵀ
// 4. Compute SVD of H: H = U * Σ * Vᵀ
// 5. Extract rotation: R = V * Uᵀ (check that det(R) = +1)
// 6. Compute translation: t = q̄ - R * p̄
// 7. Return Transform2D(R, t)
Transform2D estimateTransform(const std::vector<CorrespondencePair>& correspondences);

} // namespace slam

#endif // ICP_MATCHER_H
