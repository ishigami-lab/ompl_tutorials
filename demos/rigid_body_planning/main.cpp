#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <fstream>

using namespace std::placeholders;

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
  // cast the abstract state type to the type we expect
  const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

  // extract the first component of the state and cast it to what we expect
  const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

  // extract the second component of the state and cast it to what we expect
  const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

  // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
  return (const void*)rot != (const void*)pos;
}

void planWithoutSimpleSetup()
{
  // construct the state space we are planning in
  // a state in SE(3): position = (x, y, z), quaternion = (x, y, z, w)
  auto space(std::make_shared<ob::SE3StateSpace>());

  // set the bounds for the R^3 part of SE(3)
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-1);
  bounds.setHigh(1);

  space->setBounds(bounds);

  // construct an instance of  space information from this state space
  auto si(std::make_shared<ob::SpaceInformation>(space));

  // set state validity checking for this space
  si->setStateValidityChecker(isStateValid);

  // create a random start state
  ob::ScopedState<> start(space);
  start.random();

  // create a random goal state
  ob::ScopedState<> goal(space);
  goal.random();

  // create a problem instance
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));

  // set the start and goal states
  pdef->setStartAndGoalStates(start, goal);

  // create a planner for the defined space
  auto planner(std::make_shared<og::RRTConnect>(si));

  // set the problem we are trying to solve for the planner
  planner->setProblemDefinition(pdef);

  // perform setup steps for the planner
  planner->setup();


  // print the settings for this space
  si->printSettings(std::cout);

  // print the problem settings
  pdef->print(std::cout);

  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

  if (solved)
  {
    ob::PathPtr path = pdef->getSolutionPath();
    std::cout << "Found solution:" << std::endl;

    // print the path to screen
    path->print(std::cout);
  }
  else
    std::cout << "No solution found" << std::endl;
}

void planWithSimpleSetup()
{
  // construct the state space we are planning in
  // a state in SE(3): position = (x, y, z), quaternion = (x, y, z, w)
  auto space(std::make_shared<ob::SE3StateSpace>());

  // set the bounds for the R^3 part of SE(3)
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-1);
  bounds.setHigh(1);

  space->setBounds(bounds);

  // define a simple setup class
  og::SimpleSetup ss(space);

  // set state validity checking for this space
  ss.setStateValidityChecker(std::bind(&isStateValid, _1));

  // create a random start state
  ob::ScopedState<> start(space);
  start.random();
  std::cout << "Start point: "; start.print(std::cout);

  // create a random goal state
  ob::ScopedState<> goal(space);
  goal.random();
  std::cout << "Goal point: "; goal.print(std::cout);

  // set the start and goal states
  ss.setStartAndGoalStates(start, goal);

  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = ss.solve(1.0);

  if (solved)
  {
    std::cout << "Found solution:" << std::endl;

    std::ofstream ofs("../output/path.txt");
    ss.getSolutionPath().printAsMatrix(ofs);

    std::ofstream ofs_simplified("../output/path_simplified.txt");
    ss.simplifySolution();
    ss.getSolutionPath().printAsMatrix(ofs_simplified);
  }
  else
    std::cout << "No solution found" << std::endl;
}

int main(int argc, char** argv)
{
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  // select the planning method by using SimpleSetup class or not.
  // planWithoutSimpleSetup();
  planWithSimpleSetup();

  return 0;
}