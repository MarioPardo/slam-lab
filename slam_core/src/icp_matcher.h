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

Transform2D estimateTransform(const std::vector<CorrespondencePair>& correspondences);

std::vector<Eigen::Vector2d> transformPointCloud(
    const std::vector<Eigen::Vector2d>& points,
    const Transform2D& transform);

std::vector<CorrespondencePair> findCorrespondencesPointToPoint(
    const std::vector<Eigen::Vector2d>& source,
    const std::vector<Eigen::Vector2d>& target,
    double max_distance);

//TODO try point to plane (line) correspondence matching

struct ICPResult {
    Transform2D transform;
    int iterations;
    double final_error;
    bool converged;
};

ICPResult alignPointClouds(
    const std::vector<Eigen::Vector2d>& source,
    const std::vector<Eigen::Vector2d>& target,
    const Transform2D& initial_guess,
    int max_iterations,
    double convergence_epsilon,
    double correspondence_distance);

} // namespace slam

#endif // ICP_MATCHER_H
