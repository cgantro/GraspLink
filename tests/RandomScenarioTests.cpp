#include "simulation/RandomScenario.h"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
struct ScenarioSample
{
    std::array<float, 3> box;
    std::array<float, 3> placement;
    float boxRotation;
    float placementRotation;
};

ScenarioSample SampleSequence()
{
    using namespace grasplink::simulation::scenario;
    return {SampleBoxPosition(), SamplePlacementPosition(), SamplePlanarRotation(), SamplePlanarRotation()};
}
}

TEST(RandomScenarioTests, SeedRestoresTheSameSequenceAndPositionHistory)
{
    using namespace grasplink::simulation::scenario;
    SeedRandom(13542);
    const auto first = SampleSequence();
    const auto second = SampleSequence();

    SeedRandom(13542);

    EXPECT_EQ(SampleSequence().box, first.box);
    const auto replayedFirst = SampleSequence();
    EXPECT_EQ(replayedFirst.box, second.box);
    EXPECT_EQ(replayedFirst.placement, second.placement);
    EXPECT_FLOAT_EQ(replayedFirst.boxRotation, second.boxRotation);
    EXPECT_FLOAT_EQ(replayedFirst.placementRotation, second.placementRotation);
}

TEST(RandomScenarioTests, SamplesCoverBothCoordinateSignsWithinConfiguredReach)
{
    using namespace grasplink::simulation::scenario;
    SeedRandom(8061);
    bool negativeX = false;
    bool positiveX = false;
    bool negativeZ = false;
    bool positiveZ = false;
    std::array<float, 3> previousBox{};
    std::array<float, 3> previousPlacement{};

    for (int sample = 0; sample < 128; ++sample)
    {
        const auto box = SampleBoxPosition();
        const auto placement = SamplePlacementPosition();
        const float boxRadius = std::hypot(box[0], box[2]);
        const float placementRadius = std::hypot(placement[0], placement[2]);
        EXPECT_GE(boxRadius, 0.50F);
        EXPECT_LE(boxRadius, 1.05F);
        EXPECT_GE(placementRadius, 0.50F);
        EXPECT_LE(placementRadius, 1.00F);
        EXPECT_FLOAT_EQ(box[1], 0.026F);
        EXPECT_FLOAT_EQ(placement[1], 0.006F);
        EXPECT_GE(std::hypot(box[0] - previousBox[0], box[2] - previousBox[2]), 0.035F);
        EXPECT_GE(std::hypot(placement[0] - previousPlacement[0], placement[2] - previousPlacement[2]), 0.05F);
        previousBox = box;
        previousPlacement = placement;

        const float boxRotation = SamplePlanarRotation();
        EXPECT_GE(boxRotation, -1.5707964F);
        EXPECT_LE(boxRotation, 1.5707964F);
        negativeX = negativeX || box[0] < 0.0F;
        positiveX = positiveX || box[0] > 0.0F;
        negativeZ = negativeZ || box[2] < 0.0F;
        positiveZ = positiveZ || box[2] > 0.0F;
    }

    EXPECT_TRUE(negativeX);
    EXPECT_TRUE(positiveX);
    EXPECT_TRUE(negativeZ);
    EXPECT_TRUE(positiveZ);
}
