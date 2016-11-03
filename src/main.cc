//
// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel
//
// This file is part of hpp-quadcopter
// hpp-quadcopter is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-quadcopter is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-quadcopter  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/quadcopter/steering-method.hh>
#include <hpp/quadcopter/configuration-shooter.hh>

#include <hpp/core/problem-solver.hh>
#include <hpp/corbaserver/server.hh>

// main function of the corba server
int main (int argc, const char* argv[])
{
  // create a ProblemSolver instance.
  // This class is a container that does the interface between hpp-core library
  // and component to be running in a middleware like CORBA or ROS.
  hpp::core::ProblemSolverPtr_t problemSolver =
    hpp::core::ProblemSolver::create ();

  // Add new steering method in factory
  problemSolver->add <hpp::core::SteeringMethodBuilder_t>
    ("Quadcopter",	hpp::quadcopter::SteeringMethod::create);
  // Add new steering method in factory
  problemSolver->add <hpp::core::ConfigurationShooterBuilder_t>
    ("Quadcopter",	hpp::quadcopter::ConfigurationShooter::create);

  // Create the CORBA server.
  hpp::corbaServer::Server server (problemSolver, argc, argv, true);
  // Start the CORBA server.
  server.startCorbaServer ();
  // Wait for CORBA requests.
  server.processRequest (true);
}
