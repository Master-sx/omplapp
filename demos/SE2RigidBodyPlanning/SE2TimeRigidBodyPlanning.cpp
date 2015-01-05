/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#include <omplapp/apps/SE2TimeRigidBodyPlanning.h>
#include <omplapp/config.h>
#include <ompl/geometric/planners/rrt/RRT.h>

using namespace ompl;

int main()
{
    // plan in SE2
    app::SE2TimeRigidBodyPlanning setup;
    geometric::RRT *rrtplanner = new geometric::RRT(setup.getSpaceInformation ());
    rrtplanner->setGoalBias (0.03);

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/car1_planar_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/Maze_planar_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());

    // define starting state
    base::ScopedState<base::SE2TimeStateSpace> start(setup.getSpaceInformation());
    start->setX(-20.99);
    start->setY(-0.15);

    // define goal state
    base::ScopedState<base::SE2TimeStateSpace> goal(start);
    goal->setX(15.01);
    goal->setY(-52.00);

    base::RealVectorBounds bounds(2);
    bounds.low[0] = -55.00;
    bounds.low[1] = -55.00;
 
    bounds.high[0] = 55.00;
    bounds.high[1] = 55.00;

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    setup.getSpaceInformation ()->getStateSpace ()->as <base::SE2TimeStateSpace> ()->setBounds (bounds);

    // attempt to solve the problem, and print it to screen if a solution is found
    if (setup.solve())
        setup.getSolutionPath().print(std::cout);

    return 0;
}
