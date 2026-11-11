#ifndef FTC_LOCAL_PLANNER_OBSTACLE_AVOIDANCE_H_
#define FTC_LOCAL_PLANNER_OBSTACLE_AVOIDANCE_H_

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

#include <cstddef>
#include <vector>

/**
 * Obstacle avoidance for the FTC planner.
 *
 * When the plan ahead runs into an obstacle, the blocked stretch of the plan is moved
 * sideways: it ramps out, runs alongside the obstacle at a constant offset, and ramps back
 * in. Poses are only moved, never added or removed, so plan indices (and therefore
 * planner progress reporting) keep their meaning.
 *
 * These are free functions without ROS node state so they can be unit tested against a
 * hand-built costmap.
 */
namespace ftc_local_planner
{
namespace avoidance
{

struct Params
{
    double lookahead = 1.5;      // [m] how far along the plan to look for blocked poses
    double clearance = 0.25;     // [m] radius around each pose that must be free (robot half-width)
    double max_offset = 1.0;     // [m] largest sideways offset tried
    double offset_step = 0.1;    // [m] offset increment between attempts
    double ramp_distance = 0.5;  // [m] arc length used to steer out before, and back in after, the obstacle
};

enum class Result
{
    PATH_CLEAR,      // nothing ahead is blocked, plan unchanged
    DETOUR_PLANNED,  // plan was edited to go around the obstacle
    NO_DETOUR,       // something ahead is blocked but no free detour exists, plan unchanged
};

/**
 * @brief Edit `plan` to go around obstacles ahead, if needed and possible.
 *
 * @param plan            plan being followed; only poses in [first_editable, plan.size() - 3]
 *                        are modified (the last two are the goal poses duplicated by setPlan())
 * @param first_editable  first pose that may be moved (poses before it are already being followed)
 * @param obstacles       costmap holding only detected obstacles; decides whether the plan is blocked
 * @param combined        full costmap (map + obstacles); a detour must be free in this one
 * @param params          tuning, see Params
 */
Result avoidObstacles(std::vector<geometry_msgs::PoseStamped> &plan, size_t first_editable,
                      const costmap_2d::Costmap2D &obstacles, const costmap_2d::Costmap2D &combined,
                      const Params &params);

/**
 * @brief Where the path ahead stops being blocked.
 *
 * Finds the first blocked pose in plan[from..to] (blocked: an obstacle within `clearance`), then
 * follows the run of blocked poses after it, which may continue past `to`.
 *
 * @return index of the last pose of that run, or -1 if no pose in plan[from..to] is blocked
 */
int lastBlockedIndex(const std::vector<geometry_msgs::PoseStamped> &plan, size_t from, size_t to,
                     const costmap_2d::Costmap2D &obstacles, double clearance);

/// True if a lethal obstacle lies within `radius` of (x, y). Cells outside the costmap are ignored.
bool isBlocked(const costmap_2d::Costmap2D &obstacles, double x, double y, double radius);

/// True if every cell within `radius` of (x, y) is inside the costmap and cheap enough to drive on.
bool isFree(const costmap_2d::Costmap2D &costmap, double x, double y, double radius);

}  // namespace avoidance
}  // namespace ftc_local_planner

#endif
