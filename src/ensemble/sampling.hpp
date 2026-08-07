#pragma once

#include "sailroute/ensemble.hpp"

#include <string>
#include <vector>

namespace sailroute {

struct EnsembleSampler::Impl {
    std::vector<std::string> identifiers;
    std::vector<WeatherSampler> samplers;
};

}  // namespace sailroute
