#ifndef PNPSOLVER_PNP_SOLVER_H
#define PNPSOLVER_PNP_SOLVER_H

#include <vector>
#include <Eigen/Core>

namespace colpnp {

// Sampling method
enum Sampler {
    RANDOM_SAMPLE = 1,
    WEIGHT_SAMPLE = 2,
};

// RANSAC method
enum Robustor { RANSAC = 1, LORANSAC = 2 };

// Estimate the camera pose from 2D-3D correspondences and refine pose with all
// inliers
//
// @param points2D       2D image points
// @param points3D       3D world points
// @param camera_model   camera model name, for example: SIMPLE_PINHOLE,
//                       SIMPLE_RADIAL, etc.
// @param params         The focal length, principal point, and extra
// parameters.
// @param qvec           Quaternion (qw, qx, qy, qz) from world to camera
// @param tvec           Translation (x, y, z) from world to camera
// @param error_thres    RANSAC max error thres
// @param inlier_ratio   RANSAC min inlier ratio
// @param confidence     RANSAC confidence
// @param max_iter       RANSAC max iteration
// @param mask           Inlier mask
// @param robustor       RANSAC(with p3p) or LORANSAC(p3p and epnp)
// @param sampler        RANSAC sampling method: RANDOM, WEIGHT
//
// @param priors         When using weighted sampler
//
// @return               Whether the solution is usable.
bool sovle_pnp_ransac(const std::vector<Eigen::Vector2d> &points2D,
                      const std::vector<Eigen::Vector3d> &points3D,
                      const std::string &camera_model,
                      const std::vector<double> &params, Eigen::Vector4d &qvec,
                      Eigen::Vector3d &tvec, size_t &num_inlier,
                      double error_thres = 8.0, double inlier_ratio = 0.1,
                      double confidence = 0.99, size_t max_iter = 10000,
                      std::vector<char> *mask = nullptr,
                      Robustor robustor = RANSAC,
                      Sampler sampler = RANDOM_SAMPLE,
                      std::vector<double> *priors = nullptr);

}  // namespace colpnp

#endif  // PNPSOLVER_PNP_SOLVER_H
