#include "icp_matcher.h"
#include <iostream>

namespace slam {

Transform2D estimateTransform(const std::vector<CorrespondencePair>& correspondences) 
{
    //Soln R: R = V * Uᵀ where VU is from SVD of H
        //H = Sum(Yn-Y0)(Xn-X0)ᵀPn

    //Soln T: T = Y0 - R * X0

    //1: Compute centroids Y0, X0
    Eigen::Vector2d Y0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d X0 = Eigen::Vector2d::Zero();
    float weightSum = 0;

    for(const CorrespondencePair& pair : correspondences){
        X0 += pair.source * pair.weight;
        Y0 += pair.target * pair.weight;
        weightSum += pair.weight;
    }

    Y0 = Y0 / weightSum;
    X0 = X0 / weightSum;


    //2: Compute covariance matrix H
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    
    for(const CorrespondencePair& pair : correspondences)
    {
        Eigen::Vector2d source_centered = pair.source - X0;
        Eigen::Vector2d target_centered = pair.target - Y0;
    
        H += pair.weight * source_centered * target_centered.transpose();
    }

    //3: SVD
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();

    //Get Results
    Eigen::Matrix2d RotationMatrix = V*U.transpose();
    Eigen::Vector2d TranslationVector = Y0 - RotationMatrix*X0;
    
    
    return Transform2D(RotationMatrix, TranslationVector);
}

} // namespace slam
