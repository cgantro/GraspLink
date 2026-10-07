#include "simulation/RandomScenario.h"

#include <cmath>
#include <random>

namespace grasplink::simulation::scenario
{
namespace
{
std::mt19937& Generator()
{
    static std::mt19937 generator{std::random_device{}()};
    return generator;
}

std::array<float, 3> previousBoxPosition{};
std::array<float, 3> previousPlacementPosition{};
bool hasPreviousBoxPosition = false;
bool hasPreviousPlacementPosition = false;

std::array<float, 3> SamplePosition(float heightMeters, float minimumRadiusMeters,
    float maximumRadiusMeters, float minimumSeparationMeters,
    std::array<float, 3>& previousPosition, bool& hasPreviousPosition)
{
    std::uniform_real_distribution<float> coordinate{-1.05F, 1.05F};
    std::array<float, 3> position{};
    float radius = 0.0F;
    do
    {
        position = {coordinate(Generator()), heightMeters, coordinate(Generator())};
        radius = std::hypot(position[0], position[2]);
    } while (radius < minimumRadiusMeters || radius > maximumRadiusMeters ||
        (hasPreviousPosition && std::hypot(position[0] - previousPosition[0],
            position[2] - previousPosition[2]) < minimumSeparationMeters));

    previousPosition = position;
    hasPreviousPosition = true;
    return position;
}
}

void SeedRandom(std::uint32_t seed)
{
    Generator().seed(seed);
    hasPreviousBoxPosition = false;
    hasPreviousPlacementPosition = false;
}

std::array<float, 3> SampleBoxPosition()
{
    return SamplePosition(0.026F, 0.50F, 1.05F, 0.035F,
        previousBoxPosition, hasPreviousBoxPosition);
}

std::array<float, 3> SamplePlacementPosition()
{
    return SamplePosition(0.006F, 0.50F, 1.00F, 0.05F,
        previousPlacementPosition, hasPreviousPlacementPosition);
}

float SamplePlanarRotation()
{
    std::uniform_real_distribution<float> angle{-1.57079632679F, 1.57079632679F};
    return angle(Generator());
}

} // namespace grasplink::simulation::scenario
