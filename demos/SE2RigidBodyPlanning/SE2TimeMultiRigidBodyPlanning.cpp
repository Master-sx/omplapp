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

#include <omplapp/apps/SE2TimeMultiRigidBodyPlanning.h>
#include <omplapp/config.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

using namespace ompl;

class timeStateValidityChecker : public base::StateValidityChecker
{
public:
        timeStateValidityChecker(const base::SpaceInformationPtr &si) :
            base::StateValidityChecker(si)
                {

                }

        virtual bool isValid(const base::State *state) const
        {
            return true;
        }
};

class timeMotionValidator : public base::MotionValidator
{
public:
    virtual bool checkMotion(const base::State *s1, const base::State *s2) const
    {
        return true;
    }
};

int main()
{
    // plan for two bodies in SE2
    app::SE2TimeMultiRigidBodyPlanning setup(2);

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
    
    // set the bounds for the R^2 part of SE(2)
    base::RealVectorBounds bounds(2);
    bounds.setLow(-30.0);
    bounds.setHigh(30.0);

    setup.getSpaceInformation()->getStateSpace()->as<base::SE2StateSpace>()->setBounds(bounds);

    // set state validity checker
    base::SpaceInformationPtr si(setup.getSpaceInformation());
    si->setStateValidityChecker(base::StateValidityCheckerPtr(new timeStateValidityChecker(si)));
    si->setStateValidityCheckingResolution(0.03); // 3%

    // set motion validator
    // si->setMotionValidator(base::MotionValidatorPtr(new timeMotionValidator(si)));
    
    si->setup();

    // use RRTConnect for planning
    setup.setPlanner (base::PlannerPtr(new geometric::RRTConnect(setup.getSpaceInformation())));

    setup.setup();
    setup.print(std::cout);
    // attempt to solve the problem, and print it to screen if a solution is found
    if (setup.solve(60))
    {
        setup.simplifySolution();
        // setup.getSolutionPath().interpolate(); // plot the path
        setup.getSolutionPath().printAsMatrix(std::cout);
    }

    return 0;
}