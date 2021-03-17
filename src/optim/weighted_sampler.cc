#include "weighted_sampler.h"

#include <numeric>

#include "util/random.h"
#include "util/misc.h"

namespace colmap {

WeigthedRandomSampler::WeigthedRandomSampler(const size_t num_samples)
        : num_samples_(num_samples){}

void WeigthedRandomSampler::Initialize(const size_t total_num_samples) {
    // pass
}

void WeigthedRandomSampler::SetPriors(const std::vector<double> &prob) {
    priors_.assign(prob.begin(), prob.end());
}

size_t WeigthedRandomSampler::MaxNumSamples() {
    return std::numeric_limits<size_t>::max();
}

std::vector<size_t> WeigthedRandomSampler::Sample() {
    return WeightedRandomSample(priors_, num_samples_);
}

} // namespace colmap