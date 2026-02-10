//DISCLAIMER: Made using Claude Code to help development


#include "../src/icp_matcher.h"
#include "../src/types.h"
#include <iostream>
#include <cmath>

using namespace slam;

void printResult(const ICPResult& result, const std::string& test_name) {
    std::cout << "\n=== " << test_name << " ===\n";
    std::cout << "Converged: " << (result.converged ? "Yes" : "No") << "\n";
    std::cout << "Iterations: " << result.iterations << "\n";
    std::cout << "Final error: " << result.final_error << "\n";
    std::cout << "Rotation:\n" << result.transform.rotation << "\n";
    std::cout << "Translation: [" << result.transform.translation.x() 
              << ", " << result.transform.translation.y() << "]\n";
    double angle = std::atan2(result.transform.rotation(1, 0), result.transform.rotation(0, 0));
    std::cout << "Rotation angle: " << angle * 180.0 / M_PI << " degrees\n";
}

void printVerification(const std::vector<Eigen::Vector2d>& source, 
                       const std::vector<Eigen::Vector2d>& target,
                       const Transform2D& transform) {
    std::cout << "Verification (applying found transform to source):\n";
    for (size_t i = 0; i < std::min(source.size(), (size_t)3); ++i) {
        Eigen::Vector2d transformed = transform.rotation * source[i] + transform.translation;
        std::cout << "  Source[" << i << "]: (" << source[i].x() << ", " << source[i].y() << ")";
        std::cout << " -> (" << transformed.x() << ", " << transformed.y() << ")";
        if (i < target.size()) {
            std::cout << " | Target: (" << target[i].x() << ", " << target[i].y() << ")";
            double error = (transformed - target[i]).norm();
            std::cout << " | Error: " << error;
        }
        std::cout << "\n";
    }
}

void testIdentity() {
    std::vector<Eigen::Vector2d> cloud = {
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(1, 0),
        Eigen::Vector2d(1, 1),
        Eigen::Vector2d(0, 1)
    };
    
    Transform2D initial_guess;
    ICPResult result = alignPointClouds(cloud, cloud, initial_guess, 50, 1e-6, 0.5);
    
    printResult(result, "Test 1: Identity");
    std::cout << "Expected: Identity transform, 0 iterations\n";
}

void testPureTranslation() {
    std::vector<Eigen::Vector2d> source = {
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(1, 0),
        Eigen::Vector2d(1, 1),
        Eigen::Vector2d(0, 1)
    };
    
    Eigen::Vector2d translation(2.0, 1.5);
    std::vector<Eigen::Vector2d> target;
    for (const auto& p : source) {
        target.push_back(p + translation);
    }
    
    Transform2D initial_guess;
    ICPResult result = alignPointClouds(source, target, initial_guess, 50, 1e-6, 3.0);
    
    printResult(result, "Test 2: Pure Translation");
    std::cout << "Expected: Translation [2, 1.5], ~1 iteration\n";
}

void testPureRotation() {
    std::vector<Eigen::Vector2d> source = {
        Eigen::Vector2d(1, 0),
        Eigen::Vector2d(0, 1),
        Eigen::Vector2d(-1, 0),
        Eigen::Vector2d(0, -1)
    };
    
    double angle = M_PI / 4.0;
    Eigen::Matrix2d R;
    R << std::cos(angle), -std::sin(angle),
         std::sin(angle),  std::cos(angle);
    
    std::vector<Eigen::Vector2d> target;
    for (const auto& p : source) {
        target.push_back(R * p);
    }
    
    Transform2D initial_guess;
    ICPResult result = alignPointClouds(source, target, initial_guess, 50, 1e-6, 1.0);
    
    printResult(result, "Test 3: Pure Rotation (45 degrees)");
    std::cout << "Expected: 45 degree rotation, few iterations\n";
}

void testCombinedTransform() {
    std::vector<Eigen::Vector2d> source = {
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(2, 0),
        Eigen::Vector2d(1, 1.5)
    };
    
    double angle = M_PI / 6.0;
    Eigen::Matrix2d R;
    R << std::cos(angle), -std::sin(angle),
         std::sin(angle),  std::cos(angle);
    Eigen::Vector2d t(1.5, -2.0);
    
    std::vector<Eigen::Vector2d> target;
    for (const auto& p : source) {
        target.push_back(R * p + t);
    }
    
    Transform2D initial_guess;
    ICPResult result = alignPointClouds(source, target, initial_guess, 50, 1e-6, 1.5);
    
    printResult(result, "Test 4: Combined Transform (30° + translation)");
    printVerification(source, target, result.transform);
    std::cout << "Expected: 30° rotation + translation [1.5, -2]\n";
}

void testPartialOverlap() {
    std::vector<Eigen::Vector2d> source;
    for (int i = 0; i < 10; ++i) {
        source.push_back(Eigen::Vector2d(i * 0.5, 0));
    }
    
    Eigen::Vector2d t(1.0, 0.5);
    std::vector<Eigen::Vector2d> target;
    for (int i = 5; i < 15; ++i) {
        target.push_back(Eigen::Vector2d(i * 0.5, 0) + t);
    }
    
    Transform2D initial_guess;
    ICPResult result = alignPointClouds(source, target, initial_guess, 50, 1e-6, 1.0);
    
    printResult(result, "Test 5: Partial Overlap");
    std::cout << "Expected: Should converge with overlapping points\n";
}

void testWithNoise() {
    std::vector<Eigen::Vector2d> source = {
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(1, 0),
        Eigen::Vector2d(1, 1),
        Eigen::Vector2d(0, 1)
    };
    
    Eigen::Vector2d t(0.5, 0.5);
    std::vector<Eigen::Vector2d> target;
    for (const auto& p : source) {
        target.push_back(p + t);
    }
    target.push_back(Eigen::Vector2d(10, 10));
    target.push_back(Eigen::Vector2d(-5, -5));
    
    Transform2D initial_guess;
    ICPResult result = alignPointClouds(source, target, initial_guess, 50, 1e-6, 0.8);
    
    printResult(result, "Test 6: With Outliers");
    std::cout << "Expected: Distance threshold should reject outliers\n";
}

int main() {
    std::cout << "========================================\n";
    std::cout << "    Full ICP Algorithm Tests\n";
    std::cout << "========================================\n";
    
    testIdentity();
    testPureTranslation();
    testPureRotation();
    testCombinedTransform();
    testPartialOverlap();
    testWithNoise();
    
    std::cout << "\n=== All tests complete ===\n\n";
    return 0;
}
