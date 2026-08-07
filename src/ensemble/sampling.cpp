#include "dataset.hpp"
#include "sampling.hpp"

#include <algorithm>
#include <utility>

namespace sailroute {

EnsembleSampler::EnsembleSampler() noexcept = default;
EnsembleSampler::~EnsembleSampler() = default;
EnsembleSampler::EnsembleSampler(const EnsembleSampler&) = default;
EnsembleSampler::EnsembleSampler(EnsembleSampler&&) noexcept = default;
EnsembleSampler& EnsembleSampler::operator=(const EnsembleSampler&) = default;
EnsembleSampler& EnsembleSampler::operator=(EnsembleSampler&&) noexcept = default;

std::size_t EnsembleSampler::member_count() const noexcept {
    return impl_ ? impl_->samplers.size() : 0U;
}

bool EnsembleSampler::valid() const noexcept {
    return impl_ && std::all_of(
        impl_->samplers.begin(),
        impl_->samplers.end(),
        [](const WeatherSampler& sampler) { return sampler.valid(); });
}

std::vector<EnsembleMemberWindSample> EnsembleSampler::sample(
    Coordinate coordinate) const {
    if (!impl_) {
        return {};
    }
    std::vector<EnsembleMemberWindSample> samples;
    samples.reserve(impl_->samplers.size());
    for (std::size_t index = 0U; index < impl_->samplers.size(); ++index) {
        samples.push_back(EnsembleMemberWindSample{
            impl_->identifiers[index],
            impl_->samplers[index].sample(coordinate)});
    }
    return samples;
}

Result<std::vector<EnsembleMemberWindSample>> EnsembleSampler::sample(
    std::span<const Coordinate> coordinates) const {
    if (!impl_) {
        return Error{
            ErrorCode::incomplete_forecast,
            "ensemble sampler is empty"};
    }
    if (coordinates.size() != impl_->samplers.size()) {
        return Error{
            ErrorCode::invalid_argument,
            "member-local coordinate count must match ensemble member count"};
    }
    std::vector<EnsembleMemberWindSample> samples;
    samples.reserve(impl_->samplers.size());
    for (std::size_t index = 0U; index < impl_->samplers.size(); ++index) {
        samples.push_back(EnsembleMemberWindSample{
            impl_->identifiers[index],
            impl_->samplers[index].sample(coordinates[index])});
    }
    return samples;
}

Result<EnsembleSampler> EnsembleDataset::sampler_at(TimePoint time) const {
    std::vector<TimePoint> member_times(member_count(), time);
    return sampler_at(member_times);
}

Result<EnsembleSampler> EnsembleDataset::sampler_at(
    std::span<const TimePoint> member_times) const {
    if (!impl_ || impl_->members.empty()) {
        return Error{
            ErrorCode::incomplete_forecast,
            "ensemble dataset is empty"};
    }
    if (member_times.size() != member_count()) {
        return Error{
            ErrorCode::invalid_argument,
            "member-local time count must match ensemble member count"};
    }

    auto sampler_impl = std::make_shared<EnsembleSampler::Impl>();
    sampler_impl->identifiers.reserve(member_count());
    sampler_impl->samplers.reserve(member_count());
    for (std::size_t index = 0U; index < member_count(); ++index) {
        auto sampler = impl_->weather[index].sampler_at(member_times[index]);
        if (!sampler) {
            return Error{
                sampler.error().code,
                "ensemble member '" + impl_->members[index].identifier +
                    "': " + sampler.error().message};
        }
        sampler_impl->identifiers.push_back(impl_->members[index].identifier);
        sampler_impl->samplers.push_back(std::move(sampler.value()));
    }

    EnsembleSampler result;
    result.impl_ = std::move(sampler_impl);
    return result;
}

}  // namespace sailroute
