#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "util/random.h"

#include "interface/pnp_solver.h"

int main(int argc, char** argv) {
    int num_test = 1000;

    for (int i = 0; i != num_test; ++i) {
        int num_correspondences = colmap::RandomInteger(0, 1000);
        std::vector<Eigen::Vector2d> points2D(num_correspondences);
        std::vector<Eigen::Vector3d> points3D(num_correspondences);
        std::vector<double> priors(num_correspondences);
        for (int j = 0; j != num_correspondences; ++j) {
            points2D[j][0] = colmap::RandomReal<double>(0, 600);
            points2D[j][1] = colmap::RandomReal<double>(0, 600);
            points3D[j][0] = colmap::RandomReal<double>(0, 1);
            points3D[j][1] = colmap::RandomReal<double>(0, 1);
            points3D[j][2] = colmap::RandomReal<double>(0, 1);
            priors[j] = colmap::RandomReal<double>(0, 1);
        }
        std::string model_name = "SIMPLE_RADIAL";
        std::vector<double> params = {800.0, 300.0, 300.0, 0.001};

        Eigen::Vector4d qvec;
        Eigen::Vector3d tvec;
        std::vector<char> mask;

        size_t num_inliers = 0;
        if (sovle_pnp_ransac(points2D, points3D, model_name, params, qvec, tvec,
                 num_inliers, 8.0, 0.1, 0.9999, 1000, &mask,
                colpnp::LORANSAC, colpnp::WEIGHT_SAMPLE, &priors) ){
            std::cout << "Inlier: " << num_inliers << std::endl;
        } else {
            std::cout << "Failed" << std::endl;
        }
    }
    return 0;
}
