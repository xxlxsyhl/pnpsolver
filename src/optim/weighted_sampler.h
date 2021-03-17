#ifndef PNPSOLVER_WEIGHTED_SAMPLER_H
#define PNPSOLVER_WEIGHTED_SAMPLER_H

#include <vector>

#include "optim/sampler.h"

namespace colmap {
// Weighted random sampler for Weighted RANSAC-based methods
//
// Note that a discrete probability distribution should be given
class WeigthedRandomSampler : public Sampler {
public:
    explicit WeigthedRandomSampler(const size_t num_sample);

    void Initialize(const size_t total_num_samples) override;

    void SetPriors(const std::vector<double> &prob) override;

    size_t MaxNumSamples() override;

    std::vector<size_t> Sample() override;

private:
    std::vector<double> priors_;
    const size_t num_samples_;
};

} // namespace colmap

#endif //PNPSOLVER_WEIGHTED_SAMPLER_H
