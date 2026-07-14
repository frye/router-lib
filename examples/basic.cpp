#include "sailroute/sailroute.hpp"

#include <iostream>

int main() {
    auto weather = sailroute::WeatherDataset::load("forecast.grib");
    if (!weather) {
        std::cerr << weather.error().message << '\n';
        return 1;
    }

    sailroute::Router router{
        std::move(weather.value()),
        sailroute::VesselPolar::default_racer_cruiser_45ft()};
    sailroute::RouteRequest request{
        .start = {37.7749, -122.4194},
        .destination = {21.3069, -157.8583},
    };

    auto route = router.optimize(request);
    if (!route) {
        std::cerr << route.error().message << '\n';
        return 1;
    }

    auto json = sailroute::route_to_json(route.value());
    if (json) {
        std::cout << json.value();
    }
}
