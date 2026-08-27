// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

geometry_msgs::msg::PoseStamped makePose(double x, double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

// Distance from a point to the route polyline
double distanceToRoute(
  const geometry_msgs::msg::Point & point,
  const std::vector<geometry_msgs::msg::PoseStamped> & route)
{
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < route.size(); i++) {
    const double ax = route[i].pose.position.x;
    const double ay = route[i].pose.position.y;
    const double dx = route[i + 1].pose.position.x - ax;
    const double dy = route[i + 1].pose.position.y - ay;
    const double length_sq = dx * dx + dy * dy;
    double t = 0.0;
    if (length_sq > 0.0) {
      t = std::clamp(((point.x - ax) * dx + (point.y - ay) * dy) / length_sq, 0.0, 1.0);
    }
    best = std::min(best, std::hypot(point.x - (ax + t * dx), point.y - (ay + t * dy)));
  }
  return best;
}

double pathLength(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  double length = 0.0;
  for (size_t i = 1; i < poses.size(); i++) {
    length += std::hypot(
      poses[i].pose.position.x - poses[i - 1].pose.position.x,
      poses[i].pose.position.y - poses[i - 1].pose.position.y);
  }
  return length;
}

class CorridorPlanningTest : public ::testing::Test
{
protected:
  void configurePlanner(double max_route_deviation)
  {
    node_ = std::make_shared<nav2::LifecycleNode>("navfn_corridor_test");
    node_->declare_parameter("test.max_route_deviation", max_route_deviation);

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    costmap_ = costmap_ros_->getCostmap();

    // Start from a fully known, empty world so the tests only exercise the corridor
    for (unsigned int i = 0; i < costmap_->getSizeInCellsX(); i++) {
      for (unsigned int j = 0; j < costmap_->getSizeInCellsY(); j++) {
        costmap_->setCost(i, j, nav2_costmap_2d::FREE_SPACE);
      }
    }

    planner_ = std::make_unique<nav2_navfn_planner::NavfnPlanner>();
    auto tf = nav2::create_transform_buffer(node_);
    planner_->configure(node_, "test", tf, costmap_ros_);
    planner_->activate();
  }

  void TearDown() override
  {
    if (planner_) {
      planner_->deactivate();
      planner_->cleanup();
      planner_.reset();
    }
  }

  // Draw a lethal wall across a row of cells, spanning the given world x range
  void blockRow(double y, double from_x, double to_x)
  {
    unsigned int mx_from, mx_to, my;
    ASSERT_TRUE(costmap_->worldToMap(from_x, y, mx_from, my));
    ASSERT_TRUE(costmap_->worldToMap(to_x, y, mx_to, my));
    for (unsigned int i = mx_from; i <= mx_to; i++) {
      costmap_->setCost(i, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  static std::function<bool()> noCancel() {return []() {return false;};}

  std::shared_ptr<nav2::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;
  std::unique_ptr<nav2_navfn_planner::NavfnPlanner> planner_;
};

}  // namespace

// A radius is what turns viapoint support on, so planner_server knows to hand over the route
TEST_F(CorridorPlanningTest, testViapointSupportFollowsRadius)
{
  configurePlanner(0.0);
  EXPECT_FALSE(planner_->supportsViapoints());

  TearDown();
  configurePlanner(0.6);
  EXPECT_TRUE(planner_->supportsViapoints());
}

// Every pose of the returned path stays inside the capsule around the route, including around
// the bend where the two legs meet.
TEST_F(CorridorPlanningTest, testPathStaysInsideCorridor)
{
  const double radius = 0.6;
  configurePlanner(radius);

  auto start = makePose(1.0, 1.0);
  auto goal = makePose(3.5, 3.5);
  std::vector<geometry_msgs::msg::PoseStamped> viapoints = {makePose(1.0, 3.5)};

  auto path = planner_->createPlan(start, goal, viapoints, noCancel());
  ASSERT_FALSE(path.poses.empty());

  std::vector<geometry_msgs::msg::PoseStamped> route = {start, viapoints.front(), goal};
  for (const auto & pose : path.poses) {
    EXPECT_LE(distanceToRoute(pose.pose.position, route), radius)
      << "pose (" << pose.pose.position.x << ", " << pose.pose.position.y
      << ") left the corridor";
  }

  EXPECT_NEAR(path.poses.back().pose.position.x, goal.pose.position.x, 0.5);
  EXPECT_NEAR(path.poses.back().pose.position.y, goal.pose.position.y, 0.5);

  // Guard against the check above passing for free: over this route an unconstrained plan cuts
  // the corner diagonally and leaves a corridor this narrow, so the bound is doing work
  auto unconstrained = planner_->createPlan(start, goal, {}, noCancel());
  ASSERT_FALSE(unconstrained.poses.empty());
  double worst = 0.0;
  for (const auto & pose : unconstrained.poses) {
    worst = std::max(worst, distanceToRoute(pose.pose.position, route));
  }
  EXPECT_GT(worst, radius);
}

// The viapoints only guide the route: with a corridor wide enough to contain the shortcut, the
// path cuts the corner instead of driving through them.
TEST_F(CorridorPlanningTest, testViapointsAreNotForced)
{
  configurePlanner(1.0);

  auto start = makePose(1.0, 1.0);
  auto goal = makePose(3.0, 1.0);
  // A detour: going through the viapoint is longer than going straight
  std::vector<geometry_msgs::msg::PoseStamped> viapoints = {makePose(2.0, 1.8)};

  std::vector<geometry_msgs::msg::PoseStamped> route = {start, viapoints.front(), goal};
  const double route_length = pathLength(route);

  auto path = planner_->createPlan(start, goal, viapoints, noCancel());
  ASSERT_FALSE(path.poses.empty());

  EXPECT_LT(pathLength(path.poses), route_length)
    << "path was not allowed to cut the corner past the viapoint";
  for (const auto & pose : path.poses) {
    EXPECT_LE(distanceToRoute(pose.pose.position, route), 1.0);
  }
}

// A wall across the corridor cannot be driven around without leaving it, so the plan fails with
// the corridor-specific error rather than detouring.
TEST_F(CorridorPlanningTest, testBlockedCorridorThrows)
{
  configurePlanner(0.5);

  auto start = makePose(1.0, 1.0);
  auto goal = makePose(1.0, 3.5);

  // Wall wider than the corridor, so there is no way through or around inside it
  blockRow(2.0, 0.2, 2.5);

  EXPECT_THROW(
    planner_->createPlan(start, goal, {}, noCancel()),
    nav2_core::CorridorInfeasible);
}

// With no radius the planner behaves exactly as before, viapoints or not
TEST_F(CorridorPlanningTest, testUnconstrainedPlanUnaffected)
{
  configurePlanner(0.0);

  auto start = makePose(1.0, 1.0);
  auto goal = makePose(3.5, 3.5);

  auto without = planner_->createPlan(start, goal, {}, noCancel());
  auto with_ignored = planner_->createPlan(start, goal, {makePose(1.0, 3.5)}, noCancel());

  ASSERT_FALSE(without.poses.empty());
  ASSERT_EQ(without.poses.size(), with_ignored.poses.size());
  for (size_t i = 0; i < without.poses.size(); i++) {
    EXPECT_DOUBLE_EQ(without.poses[i].pose.position.x, with_ignored.poses[i].pose.position.x);
    EXPECT_DOUBLE_EQ(without.poses[i].pose.position.y, with_ignored.poses[i].pose.position.y);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
