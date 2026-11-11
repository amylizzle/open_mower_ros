#include "ftc_local_planner/obstacle_avoidance.h"

#include <costmap_2d/cost_values.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>

namespace ftc_local_planner
{
namespace avoidance
{
namespace
{

using Plan = std::vector<geometry_msgs::PoseStamped>;

// FTCPlanner::checkCollision() treats any cost above this as a possible collision, so a
// detour has to stay at or below it to be followed.
constexpr unsigned char kMaxDrivableCost = 127;

// An inclusive range of plan indices.
struct Section
{
    size_t first;
    size_t last;
};

// Calls visit(cost) for every cell within `radius` of (x, y), reporting cells outside the
// costmap as NO_INFORMATION. Returns false as soon as visit() returns false.
template <typename Visit>
bool allCellsInDisk(const costmap_2d::Costmap2D &costmap, double x, double y, double radius, Visit visit)
{
    const double resolution = costmap.getResolution();
    const int cells = static_cast<int>(std::ceil(radius / resolution));
    for (int dy = -cells; dy <= cells; ++dy)
    {
        for (int dx = -cells; dx <= cells; ++dx)
        {
            if (std::hypot(dx, dy) * resolution > radius)
                continue;
            unsigned int mx, my;
            unsigned char cost = costmap_2d::NO_INFORMATION;
            if (costmap.worldToMap(x + dx * resolution, y + dy * resolution, mx, my))
                cost = costmap.getCost(mx, my);
            if (!visit(cost))
                return false;
        }
    }
    return true;
}

double distanceBetween(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b)
{
    return std::hypot(b.pose.position.x - a.pose.position.x, b.pose.position.y - a.pose.position.y);
}

bool poseBlocked(const costmap_2d::Costmap2D &obstacles, const geometry_msgs::PoseStamped &pose, double radius)
{
    return isBlocked(obstacles, pose.pose.position.x, pose.pose.position.y, radius);
}

bool poseFree(const costmap_2d::Costmap2D &costmap, const geometry_msgs::PoseStamped &pose, double radius)
{
    return isFree(costmap, pose.pose.position.x, pose.pose.position.y, radius);
}

// Finds the first run of consecutive blocked poses in plan[first..last] that starts within the
// lookahead of plan[first].
bool findBlockedSection(const Plan &plan, size_t first, size_t last,
                        const costmap_2d::Costmap2D &obstacles, const Params &params, Section &blocked)
{
    double distance_ahead = 0.0;
    for (size_t i = first; i <= last; ++i)
    {
        if (i > first)
            distance_ahead += distanceBetween(plan[i - 1], plan[i]);
        if (distance_ahead > params.lookahead)
            return false;
        if (poseBlocked(obstacles, plan[i], params.clearance))
        {
            blocked = {i, i};
            while (blocked.last < last && poseBlocked(obstacles, plan[blocked.last + 1], params.clearance))
                ++blocked.last;
            return true;
        }
    }
    return false;
}

// Grows `blocked` by `distance` of arc length on both sides, without leaving [min_index, max_index].
Section widenSection(const Plan &plan, Section blocked, double distance, size_t min_index, size_t max_index)
{
    Section widened = blocked;
    double before = 0.0;
    while (widened.first > min_index && before < distance)
    {
        before += distanceBetween(plan[widened.first - 1], plan[widened.first]);
        --widened.first;
    }
    double after = 0.0;
    while (widened.last < max_index && after < distance)
    {
        after += distanceBetween(plan[widened.last], plan[widened.last + 1]);
        ++widened.last;
    }
    return widened;
}

// Fraction of the full offset applied to each pose of `section`: rises linearly from 0 at
// section.first to 1 at the first blocked pose, stays at 1 alongside the blocked poses, and
// falls linearly back to 0 at section.last.
std::vector<double> rampWeights(const Plan &plan, Section section, Section blocked)
{
    // Arc length of each pose, measured from section.first.
    std::vector<double> s(section.last - section.first + 1, 0.0);
    for (size_t i = 1; i < s.size(); ++i)
        s[i] = s[i - 1] + distanceBetween(plan[section.first + i - 1], plan[section.first + i]);

    const double full_offset_from = s[blocked.first - section.first];
    const double full_offset_until = s[blocked.last - section.first];
    const double total = s.back();

    std::vector<double> weights(s.size(), 1.0);
    for (size_t i = 0; i < s.size(); ++i)
    {
        if (s[i] < full_offset_from)
            weights[i] = s[i] / full_offset_from;
        else if (s[i] > full_offset_until)
            weights[i] = (total - s[i]) / (total - full_offset_until);
    }
    return weights;
}

// Copy of plan[section] with each pose moved sideways by `offset` times its weight, and
// re-oriented along the new path.
//
// All poses move in the same direction: to the left (negative offset: right) of the line from
// the fixed pose before the section to the fixed pose after it. Moving each pose along its own
// normal instead would pull neighbouring poses apart wherever the path bends (corners, or the
// ramps of an earlier detour), leaving gaps an obstacle could hide in.
Plan shiftedSection(const Plan &plan, Section section, const std::vector<double> &weights, double offset)
{
    const auto &from = plan[section.first - 1].pose.position;
    const auto &to = plan[section.last + 1].pose.position;
    double along = std::atan2(to.y - from.y, to.x - from.x);
    if (std::hypot(to.x - from.x, to.y - from.y) < 1e-3)
        along = tf2::getYaw(plan[section.first].pose.orientation);  // section starts and ends in the same place
    const double left_x = -std::sin(along);
    const double left_y = std::cos(along);

    Plan shifted(plan.begin() + section.first, plan.begin() + section.last + 1);
    for (size_t i = 0; i < shifted.size(); ++i)
    {
        shifted[i].pose.position.x += left_x * offset * weights[i];
        shifted[i].pose.position.y += left_y * offset * weights[i];
    }

    // Point each pose from its predecessor towards its successor.
    for (size_t i = 0; i < shifted.size(); ++i)
    {
        const auto &previous = (i > 0) ? shifted[i - 1].pose.position : plan[section.first - 1].pose.position;
        const auto &next = (i + 1 < shifted.size()) ? shifted[i + 1].pose.position : plan[section.last + 1].pose.position;
        const double dx = next.x - previous.x;
        const double dy = next.y - previous.y;
        if (std::hypot(dx, dy) < 1e-6)
            continue;  // duplicate points, keep the original heading
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, std::atan2(dy, dx));
        shifted[i].pose.orientation = tf2::toMsg(q);
    }
    return shifted;
}

// True if the robot can drive before -> shifted... -> after. Only the poses themselves are
// checked against the costmap, so consecutive poses must also be close enough together that
// no obstacle can hide between them.
bool isDrivable(const geometry_msgs::PoseStamped &before, const Plan &shifted, const geometry_msgs::PoseStamped &after,
                const costmap_2d::Costmap2D &combined, double clearance)
{
    const geometry_msgs::PoseStamped *previous = &before;
    for (const auto &pose : shifted)
    {
        if (distanceBetween(*previous, pose) > clearance || !poseFree(combined, pose, clearance))
            return false;
        previous = &pose;
    }
    return distanceBetween(*previous, after) <= clearance;
}

}  // namespace

bool isBlocked(const costmap_2d::Costmap2D &obstacles, double x, double y, double radius)
{
    return !allCellsInDisk(obstacles, x, y, radius,
                           [](unsigned char cost) { return cost != costmap_2d::LETHAL_OBSTACLE; });
}

bool isFree(const costmap_2d::Costmap2D &costmap, double x, double y, double radius)
{
    return allCellsInDisk(costmap, x, y, radius, [](unsigned char cost) { return cost <= kMaxDrivableCost; });
}

int lastBlockedIndex(const std::vector<geometry_msgs::PoseStamped> &plan, size_t from, size_t to,
                     const costmap_2d::Costmap2D &obstacles, double clearance)
{
    if (plan.empty())
        return -1;
    to = std::min(to, plan.size() - 1);
    for (size_t i = from; i <= to; ++i)
    {
        if (poseBlocked(obstacles, plan[i], clearance))
        {
            size_t last = i;
            while (last + 1 < plan.size() && poseBlocked(obstacles, plan[last + 1], clearance))
                ++last;
            return static_cast<int>(last);
        }
    }
    return -1;
}

Result avoidObstacles(std::vector<geometry_msgs::PoseStamped> &plan, size_t first_editable,
                      const costmap_2d::Costmap2D &obstacles, const costmap_2d::Costmap2D &combined,
                      const Params &params)
{
    // Every edited section keeps an unmoved pose on either side of it, and the two goal
    // poses at the end of the plan are never moved.
    first_editable = std::max<size_t>(first_editable, 1);
    if (plan.size() < 3 || first_editable > plan.size() - 3)
        return Result::PATH_CLEAR;
    const size_t last_editable = plan.size() - 3;

    Section blocked;
    if (!findBlockedSection(plan, first_editable, last_editable, obstacles, params, blocked))
        return Result::PATH_CLEAR;
    if (params.offset_step <= 0.0)
        return Result::NO_DETOUR;

    const Section section = widenSection(plan, blocked, params.ramp_distance, first_editable, last_editable);
    const std::vector<double> weights = rampWeights(plan, section, blocked);

    // Try the smallest offsets first, left before right.
    for (int step = 1; step * params.offset_step <= params.max_offset + 1e-9; ++step)
    {
        for (const double offset : {step * params.offset_step, -step * params.offset_step})
        {
            const Plan shifted = shiftedSection(plan, section, weights, offset);
            if (isDrivable(plan[section.first - 1], shifted, plan[section.last + 1], combined, params.clearance))
            {
                std::copy(shifted.begin(), shifted.end(), plan.begin() + section.first);
                return Result::DETOUR_PLANNED;
            }
        }
    }
    return Result::NO_DETOUR;
}

}  // namespace avoidance
}  // namespace ftc_local_planner
