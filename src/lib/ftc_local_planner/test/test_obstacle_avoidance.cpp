#include <gtest/gtest.h>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>

#include "ftc_local_planner/obstacle_avoidance.h"

using ftc_local_planner::avoidance::avoidObstacles;
using ftc_local_planner::avoidance::isBlocked;
using ftc_local_planner::avoidance::isFree;
using ftc_local_planner::avoidance::lastBlockedIndex;
using ftc_local_planner::avoidance::Params;
using ftc_local_planner::avoidance::Result;
using Plan = std::vector<geometry_msgs::PoseStamped>;

namespace
{

// The first poses are taken as already being followed by the planner (current_index + 2).
constexpr size_t kFirstEditable = 2;

// 4 x 4 m costmap centred on the origin, 5 cm cells, everything free.
costmap_2d::Costmap2D emptyCostmap()
{
    return costmap_2d::Costmap2D(80, 80, 0.05, -2.0, -2.0, costmap_2d::FREE_SPACE);
}

// Marks every cell whose centre satisfies `inside(x, y)` as lethal.
template <typename Inside>
void markLethal(costmap_2d::Costmap2D &costmap, Inside inside)
{
    for (unsigned int mx = 0; mx < costmap.getSizeInCellsX(); ++mx)
    {
        for (unsigned int my = 0; my < costmap.getSizeInCellsY(); ++my)
        {
            double x, y;
            costmap.mapToWorld(mx, my, x, y);
            if (inside(x, y))
                costmap.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
        }
    }
}

void markDisk(costmap_2d::Costmap2D &costmap, double cx, double cy, double radius)
{
    markLethal(costmap, [&](double x, double y) { return std::hypot(x - cx, y - cy) <= radius; });
}

// Straight plan along the x axis from -1.5 m to 1.5 m, 10 cm apart, heading +x. Like
// FTCPlanner::setPlan(), the last pose is duplicated.
Plan straightPlan()
{
    Plan plan;
    tf2::Quaternion heading;
    heading.setRPY(0.0, 0.0, 0.0);
    for (int i = 0; i <= 30; ++i)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = -1.5 + 0.1 * i;
        pose.pose.orientation = tf2::toMsg(heading);
        plan.push_back(pose);
    }
    plan.push_back(plan.back());
    return plan;
}

// L-shaped plan: along +x from (-1.5, 0) to the corner at (0, 0), then along +y to (0, 1.5).
Plan cornerPlan()
{
    Plan plan;
    auto add = [&plan](double x, double y, double yaw) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        tf2::Quaternion heading;
        heading.setRPY(0.0, 0.0, yaw);
        pose.pose.orientation = tf2::toMsg(heading);
        plan.push_back(pose);
    };
    for (int i = 0; i <= 15; ++i)
        add(-1.5 + 0.1 * i, 0.0, 0.0);
    for (int i = 1; i <= 15; ++i)
        add(0.0, 0.1 * i, M_PI / 2);
    plan.push_back(plan.back());
    return plan;
}

bool samePosition(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b)
{
    return a.pose.position.x == b.pose.position.x && a.pose.position.y == b.pose.position.y;
}

bool samePlan(const Plan &a, const Plan &b)
{
    if (a.size() != b.size())
        return false;
    for (size_t i = 0; i < a.size(); ++i)
    {
        if (!samePosition(a[i], b[i]))
            return false;
    }
    return true;
}

// Largest sideways deviation from the x axis; positive is to the left.
double largestOffset(const Plan &plan)
{
    double offset = 0.0;
    for (const auto &pose : plan)
    {
        if (std::abs(pose.pose.position.y) > std::abs(offset))
            offset = pose.pose.position.y;
    }
    return offset;
}

}  // namespace

TEST(ObstacleAvoidance, ClearPathIsLeftAlone)
{
    const auto costmap = emptyCostmap();
    Plan plan = straightPlan();

    EXPECT_EQ(avoidObstacles(plan, kFirstEditable, costmap, costmap, Params()), Result::PATH_CLEAR);
    EXPECT_TRUE(samePlan(plan, straightPlan()));
}

TEST(ObstacleAvoidance, MapBoundaryNextToPathIsNotAnObstacle)
{
    // Outline paths run right next to the area boundary, which is lethal in the combined
    // costmap. Only the obstacle layer decides whether the path is blocked.
    const auto obstacles = emptyCostmap();
    auto combined = emptyCostmap();
    markLethal(combined, [](double, double y) { return y > 0.15; });
    Plan plan = straightPlan();

    EXPECT_EQ(avoidObstacles(plan, kFirstEditable, obstacles, combined, Params()), Result::PATH_CLEAR);
    EXPECT_TRUE(samePlan(plan, straightPlan()));
}

TEST(ObstacleAvoidance, ObstacleBeyondLookaheadIsIgnoredForNow)
{
    auto costmap = emptyCostmap();
    markDisk(costmap, 1.3, 0.0, 0.1);
    Plan plan = straightPlan();

    EXPECT_EQ(avoidObstacles(plan, kFirstEditable, costmap, costmap, Params()), Result::PATH_CLEAR);
    EXPECT_TRUE(samePlan(plan, straightPlan()));
}

TEST(ObstacleAvoidance, ObstacleOnPathGetsDetour)
{
    auto costmap = emptyCostmap();
    markDisk(costmap, 0.2, 0.0, 0.15);
    const Params params;
    const Plan original = straightPlan();
    Plan plan = original;

    ASSERT_EQ(avoidObstacles(plan, kFirstEditable, costmap, costmap, params), Result::DETOUR_PLANNED);

    // Same number of poses, so plan indices still mean the same thing.
    ASSERT_EQ(plan.size(), original.size());
    // Poses already being followed, and the goal poses, did not move.
    for (size_t i = 0; i < kFirstEditable; ++i)
        EXPECT_TRUE(samePosition(plan[i], original[i])) << "pose " << i;
    EXPECT_TRUE(samePosition(plan[plan.size() - 1], original[original.size() - 1]));
    EXPECT_TRUE(samePosition(plan[plan.size() - 2], original[original.size() - 2]));
    // Every pose now keeps its clearance, and the detour goes left (tried first).
    for (size_t i = 0; i < plan.size(); ++i)
        EXPECT_TRUE(isFree(costmap, plan[i].pose.position.x, plan[i].pose.position.y, params.clearance)) << "pose " << i;
    EXPECT_GT(largestOffset(plan), 0.0);
    EXPECT_LE(largestOffset(plan), params.max_offset);
}

TEST(ObstacleAvoidance, DetourAvoidsMapBoundary)
{
    // Obstacle on the path with the area boundary just to its left: the detour has to go right.
    auto obstacles = emptyCostmap();
    markDisk(obstacles, 0.2, 0.0, 0.15);
    auto combined = obstacles;
    markLethal(combined, [](double, double y) { return y > 0.45; });
    const Params params;
    Plan plan = straightPlan();

    ASSERT_EQ(avoidObstacles(plan, kFirstEditable, obstacles, combined, params), Result::DETOUR_PLANNED);
    EXPECT_LT(largestOffset(plan), 0.0);
    for (size_t i = 0; i < plan.size(); ++i)
        EXPECT_TRUE(isFree(combined, plan[i].pose.position.x, plan[i].pose.position.y, params.clearance)) << "pose " << i;
}

TEST(ObstacleAvoidance, WallAcrossPathMeansNoDetour)
{
    auto costmap = emptyCostmap();
    markLethal(costmap, [](double x, double) { return std::abs(x - 0.2) < 0.1; });
    Plan plan = straightPlan();

    EXPECT_EQ(avoidObstacles(plan, kFirstEditable, costmap, costmap, Params()), Result::NO_DETOUR);
    EXPECT_TRUE(samePlan(plan, straightPlan()));
}

TEST(ObstacleAvoidance, SecondCallOnDetourFindsPathClear)
{
    // The planner calls this every cycle; once the detour is in place it must stay put.
    auto costmap = emptyCostmap();
    markDisk(costmap, 0.2, 0.0, 0.15);
    Plan plan = straightPlan();
    ASSERT_EQ(avoidObstacles(plan, kFirstEditable, costmap, costmap, Params()), Result::DETOUR_PLANNED);

    const Plan detour = plan;
    EXPECT_EQ(avoidObstacles(plan, kFirstEditable, costmap, costmap, Params()), Result::PATH_CLEAR);
    EXPECT_TRUE(samePlan(plan, detour));
}

TEST(ObstacleAvoidance, SecondObstacleOnDetourGetsAnotherDetour)
{
    // Go around one obstacle, then have a second one appear on the detour itself.
    auto costmap = emptyCostmap();
    markDisk(costmap, 0.2, 0.0, 0.15);
    const Params params;
    Plan plan = straightPlan();
    ASSERT_EQ(avoidObstacles(plan, kFirstEditable, costmap, costmap, params), Result::DETOUR_PLANNED);

    // Put the second obstacle where the detour passes the first one.
    const auto beside_first = *std::min_element(plan.begin(), plan.end(), [](const auto &a, const auto &b) {
        return std::abs(a.pose.position.x - 0.2) < std::abs(b.pose.position.x - 0.2);
    });
    markDisk(costmap, beside_first.pose.position.x, beside_first.pose.position.y, 0.15);
    const size_t size_before = plan.size();

    ASSERT_EQ(avoidObstacles(plan, kFirstEditable, costmap, costmap, params), Result::DETOUR_PLANNED);
    EXPECT_EQ(plan.size(), size_before);
    for (size_t i = 0; i < plan.size(); ++i)
        EXPECT_TRUE(isFree(costmap, plan[i].pose.position.x, plan[i].pose.position.y, params.clearance)) << "pose " << i;
}

TEST(ObstacleAvoidance, ObstacleOnCornerGetsDetour)
{
    auto costmap = emptyCostmap();
    markDisk(costmap, 0.0, 0.0, 0.15);
    const Params params;
    Plan plan = cornerPlan();

    ASSERT_EQ(avoidObstacles(plan, kFirstEditable, costmap, costmap, params), Result::DETOUR_PLANNED);
    for (size_t i = 0; i < plan.size(); ++i)
        EXPECT_TRUE(isFree(costmap, plan[i].pose.position.x, plan[i].pose.position.y, params.clearance)) << "pose " << i;
}

TEST(ObstacleAvoidance, LastBlockedIndexIsTheEndOfTheObstacle)
{
    auto costmap = emptyCostmap();
    markDisk(costmap, 0.2, 0.0, 0.15);
    const Params params;
    const Plan plan = straightPlan();

    const int last = lastBlockedIndex(plan, 0, plan.size() - 1, costmap, params.clearance);
    ASSERT_GE(last, 0);
    ASSERT_LT(last + 1, static_cast<int>(plan.size()));
    // The returned pose is blocked and the one after it is not: resuming there clears the obstacle.
    const auto &end = plan[last].pose.position;
    const auto &after = plan[last + 1].pose.position;
    EXPECT_TRUE(isBlocked(costmap, end.x, end.y, params.clearance));
    EXPECT_FALSE(isBlocked(costmap, after.x, after.y, params.clearance));
    EXPECT_GT(after.x, 0.2 + params.clearance);
}

TEST(ObstacleAvoidance, LastBlockedIndexWhenNothingIsBlocked)
{
    auto costmap = emptyCostmap();
    const Plan plan = straightPlan();
    EXPECT_EQ(lastBlockedIndex(plan, 0, plan.size() - 1, costmap, Params().clearance), -1);

    // An obstacle behind `from` does not count.
    markDisk(costmap, -1.0, 0.0, 0.15);
    EXPECT_EQ(lastBlockedIndex(plan, 15, plan.size() - 1, costmap, Params().clearance), -1);
}

TEST(ObstacleAvoidance, LastBlockedIndexIgnoresObstaclesStartingPastTheWindow)
{
    // Obstacle starts being in the way around pose 13 (x = -0.2 m).
    auto costmap = emptyCostmap();
    markDisk(costmap, 0.2, 0.0, 0.15);
    const Plan plan = straightPlan();

    EXPECT_EQ(lastBlockedIndex(plan, 0, 5, costmap, Params().clearance), -1);
    // A window that reaches the obstacle reports the end of its whole run, past the window.
    EXPECT_GT(lastBlockedIndex(plan, 8, 14, costmap, Params().clearance), 14);
}

TEST(ObstacleAvoidance, LastBlockedIndexWhenAlreadyInsideTheObstacle)
{
    // E.g. after a bump: the poses the robot is on are already blocked.
    auto costmap = emptyCostmap();
    markDisk(costmap, -1.3, 0.0, 0.15);
    const Plan plan = straightPlan();

    const int last = lastBlockedIndex(plan, 2, 7, costmap, Params().clearance);
    ASSERT_GE(last, 2);
    const auto &after = plan[last + 1].pose.position;
    EXPECT_FALSE(isBlocked(costmap, after.x, after.y, Params().clearance));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
