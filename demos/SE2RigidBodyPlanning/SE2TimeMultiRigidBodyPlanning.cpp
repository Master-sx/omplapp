/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2012, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ryan Luna */

#include <omplapp/apps/SE2MultiRigidBodyPlanning.h>
#include <omplapp/config.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <limits>
#include <boost/math/constants/constants.hpp>

namespace ob = ompl::base;
namespace oc = ompl::control;
using namespace std;

void propagate(const oc::SpaceInformation *si, const ob::State *state,
    const oc::Control* control, const double duration, ob::State *result)
{
    static double timeStep = .01;
    int nsteps = ceil(duration / timeStep);
    double dt = duration / nsteps;
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    ob::CompoundStateSpace::StateType& s = *result->as<ob::CompoundStateSpace::StateType>();
    ob::SE2StateSpace::StateType& se2 = *s.as<ob::SE2StateSpace::StateType>(0);
    ob::RealVectorStateSpace::StateType& velocity = *s.as<ob::RealVectorStateSpace::StateType>(1);
    // ob::DiscreteStateSpace::StateType& gear = *s.as<ob::DiscreteStateSpace::StateType>(2);
    ob::TimeStateSpace::StateType& timeSpace = *s.as<ob::TimeStateSpace::StateType>(2);

    si->getStateSpace()->copyState(result, state);
    for(int i = 0; i < nsteps; i++)
    {
        se2.setX(se2.getX() + dt * velocity.values[0] * cos(se2.getYaw()));
        se2.setY(se2.getY() + dt * velocity.values[0] * sin(se2.getYaw()));
        se2.setYaw(se2.getYaw() + dt * u[0]);
        // velocity.values[0] = velocity.values[0] + dt * (u[1]*gear.value);
        velocity.values[0] = velocity.values[0] + dt * u[1];
        timeSpace.position = timeSpace.position + duration;

        // 'guards' - conditions to change gears
        // if (gear.value > 0)
        // {
        //     if (gear.value < 3 && velocity.values[0] > 10*(gear.value + 1))
        //         gear.value++;
        //     else if (gear.value > 1 && velocity.values[0] < 10*gear.value)
        //         gear.value--;
        // }

        if (!si->satisfiesBounds(result))
            return;
    }
}

bool canPropagateBackward ()
{
    return false;
}

using namespace ompl;

int main()
{
    // plan for two bodies in SE2
    app::SE2MultiRigidBodyPlanning setup(2);

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/car1_planar_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/Maze_planar_env.dae";
    setup.setRobotMesh(robot_fname.c_str());    // The first mesh should use setRobotMesh.
    setup.addRobotMesh(robot_fname.c_str());    // Subsequent robot meshes MUST use addRobotMesh!
    setup.setEnvironmentMesh(env_fname.c_str());

    // constructing start and goal states
    base::ScopedState<base::CompoundStateSpace> start(setup.getSpaceInformation());
    base::ScopedState<base::CompoundStateSpace> goal(setup.getSpaceInformation());

    // define starting state for robot 1
    base::SE2StateSpace::StateType* start1 = start.get()->as<base::SE2StateSpace::StateType>(0);
    start1->setXY(0., 0.);
    start1->setYaw(0.);
    // define goal state for robot 1
    base::SE2StateSpace::StateType* goal1 = goal.get()->as<base::SE2StateSpace::StateType>(0);
    goal1->setXY(26., 0.);
    goal1->setYaw(0.);

    // define starting state for robot 2
    base::SE2StateSpace::StateType* start2 = start.get()->as<base::SE2StateSpace::StateType>(1);
    start2->setXY(26., 0.);
    start2->setYaw(0.);
    // define goal state for robot 2
    base::SE2StateSpace::StateType* goal2 = goal.get()->as<base::SE2StateSpace::StateType>(1);
    goal2->setXY(-30., 0.);
    goal2->setYaw(0.);

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // use RRTConnect for planning
    setup.setPlanner (base::PlannerPtr(new geometric::RRTConnect(setup.getSpaceInformation())));
    setup.setStatePropagator(boost::bind(&propagate, setup.getSpaceInformation().get(), _1, _2, _3, _4));

    setup.setup();
    setup.print(std::cout);
    // attempt to solve the problem, and print it to screen if a solution is found
    if (setup.solve(60))
    {
        setup.simplifySolution();
        setup.getSolutionPath().printAsMatrix(std::cout);
    }

    return 0;
}
