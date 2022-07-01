// Copyright (c) 2021, ZJU and SenseTime
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//  * Neither the name of ZJU and SenseTime nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Hailin Yu

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
                             colpnp::LORANSAC, colpnp::WEIGHT_SAMPLE,
                             &priors)) {
            std::cout << "Inlier: " << num_inliers << std::endl;
        } else {
            std::cout << "Failed" << std::endl;
        }
    }
    return 0;
}
