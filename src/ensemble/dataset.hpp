#pragma once

#include "sailroute/ensemble.hpp"

#include <vector>

namespace sailroute {

struct EnsembleDataset::Impl {
    EnsembleRunMetadata metadata;
    std::vector<EnsembleMemberMetadata> members;
    EnsembleAlignmentStatus alignment;
    std::vector<WeatherDataset> weather;
    std::vector<RoutingEnvironment> environments;
};

}  // namespace sailroute
