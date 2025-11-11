#include "planning/idm.hpp"

#include <cmath>
#include <gtest/gtest.h>

TEST( IdmModel, FreeRoadAcceleratesUpToMax )
{
  // Free road scenario: far from goal and object, starting from standstill
  const double dist_goal     = 100.0;
  const double dist_object   = 100.0;
  const double max_speed     = 30.0;
  const double time_headway  = 1.5;
  const double dist_headway  = 2.0;
  const double current_speed = 0.0;
  const double max_acc       = 2.0;
  const double speed_object  = 0.0;

  const double acc = adore::planner::idm::calculate_idm_acc( dist_goal, dist_object, max_speed, time_headway, dist_headway, current_speed,
                                                             max_acc, speed_object );

  // Should accelerate forward, but not exceed max_acc
  EXPECT_GT( acc, 0.0 );
  EXPECT_LE( acc, max_acc + 1e-9 );
}

TEST( IdmModel, BrakesWhenApproachingSlowerObject )
{
  // Following a slower object at a finite distance
  const double dist_goal     = 100.0;
  const double dist_object   = 20.0;
  const double max_speed     = 30.0;
  const double time_headway  = 1.5;
  const double dist_headway  = 2.0;
  const double current_speed = 15.0;
  const double max_acc       = 2.0;
  const double speed_object  = 10.0;

  const double acc = adore::planner::idm::calculate_idm_acc( dist_goal, dist_object, max_speed, time_headway, dist_headway, current_speed,
                                                             max_acc, speed_object );

  // When closing in on a slower lead vehicle, IDM should command braking
  EXPECT_LT( acc, 0.0 );
}
