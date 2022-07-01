#include "pnp_solver.h"

#include "base/camera.h"
#include "estimators/pose.h"

namespace colpnp {

bool sovle_pnp_ransac(const std::vector<Eigen::Vector2d> &points2D,
                      const std::vector<Eigen::Vector3d> &points3D,
                      const std::string &camera_model,
                      const std::vector<double> &params, Eigen::Vector4d &qvec,
                      Eigen::Vector3d &tvec, size_t &num_inlier,
                      double error_thres, double inlier_ratio,
                      double confidence, size_t max_iter,
                      std::vector<char> *mask, Robustor robustor,
                      Sampler sampler, std::vector<double> *priors) {
    // CHECK(points2D.size() == points3D.size());
    if (points2D.size() < 4) {
        return false;
    }

    colmap::AbsolutePoseEstimationOptions options;
    options.estimate_focal_length = false;
    options.ransac_options.max_error = error_thres;
    options.ransac_options.min_inlier_ratio = inlier_ratio;
    options.ransac_options.confidence = confidence;
    options.ransac_options.max_num_trials = max_iter;

    colmap::Camera camera;
    camera.SetModelIdFromName(camera_model);
    camera.SetParams(params);

    num_inlier = 0;

    colmap::RansacSampler abs_pose_sampler;
    if (sampler == RANDOM_SAMPLE) {
        abs_pose_sampler = colmap::RANDOM_SAMPLE;
        // CHECK(priors == nullptr);
    } else if (sampler == WEIGHT_SAMPLE) {
        abs_pose_sampler = colmap::WEIGHT_SAMPLE;
        // CHECK(priors->size() == points3D.size());
    }

    colmap::RansacRobustor abs_pose_robustor;
    if (robustor == RANSAC) {
        abs_pose_robustor = colmap::ROBUSTRER_RANSAC;
    } else if (robustor == LORANSAC) {
        abs_pose_robustor = colmap::ROBUSTER_LORANSAC;
    }

    auto success = colmap::EstimateAbsolutePose(
        options, points2D, points3D, &qvec, &tvec, &camera, &num_inlier, mask,
        abs_pose_robustor, abs_pose_sampler, *priors);

    if (!success) {
        return false;
    }

    colmap::AbsolutePoseRefinementOptions refine_options;
    refine_options.refine_focal_length = false;
    refine_options.refine_extra_params = false;
    refine_options.max_num_iterations = 200;
    refine_options.print_summary = false;

    return colmap::RefineAbsolutePose(refine_options, *mask, points2D, points3D,
                                      &qvec, &tvec, &camera);
}

}  // namespace colpnp