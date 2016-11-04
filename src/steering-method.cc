// Copyright (c) 2016, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-quadcopter.
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
// hpp-quadcopter. If not, see <http://www.gnu.org/licenses/>.

# include <hpp/quadcopter/steering-method.hh>

# include <hpp/core/distance.hh>

# include <mav_planning_utils/polynomial_optimization_nonlinear.h>
# include <mav_planning_utils/polynomial_optimization.h>
# include <mav_planning_utils/polynomial_trajectory.h>

const int dimension = 3; // 3D path.
const size_t derivative_to_optimize = mav_planning_utils::derivative_order::SNAP;

namespace hpp {
  namespace quadcopter {
    namespace {
      struct QuadCopterCS {
        enum {
          HAS_VEL = true,
          HAS_ACC = true && HAS_VEL,
          HAS_JER = true && HAS_ACC
        };
        enum {
          POS_IDX = 0,
          QUA_IDX = 3,
          VEL_IDX = QUA_IDX + 4,
          ACC_IDX = VEL_IDX + (HAS_VEL ? 3 : 0),
          JER_IDX = ACC_IDX + (HAS_ACC ? 3 : 0),
          DIM     = JER_IDX + (HAS_JER ? 3 : 0)
        };
      };

      template <bool linear, int N, typename Derived>
      bool computeSegments (
          const Eigen::MatrixBase<Derived>& q1,
          const Eigen::MatrixBase<Derived>& q2,
          const value_type& distance,
          typename mav_planning_utils::Segment<N>::Vector& segments,
          const mav_planning_utils::NonlinearOptimizationParameters& nlOptParam)
      {
        using namespace mav_planning_utils;
        Vertex::Vector vertices;
        Vertex start(3), middle(3), end(3);

        start.addConstraint (derivative_order::POSITION,     q1.template head   <3>(  ));
        if (QuadCopterCS::HAS_VEL) start.addConstraint (derivative_order::VELOCITY,     q1.template segment<3>(QuadCopterCS::VEL_IDX));
        if (QuadCopterCS::HAS_ACC) start.addConstraint (derivative_order::ACCELERATION, q1.template segment<3>(QuadCopterCS::VEL_IDX));
        if (QuadCopterCS::HAS_JER) start.addConstraint (derivative_order::JERK,         q1.template segment<3>(QuadCopterCS::VEL_IDX));
        vertices.push_back(start);

        // Without this line, I get an error saying I need strictly more than 2 vertex.
        vertices.push_back(middle);

        end.addConstraint (derivative_order::POSITION,     q2.template head   <3>( ));
        if (QuadCopterCS::HAS_VEL) end.addConstraint (derivative_order::VELOCITY,     q2.template segment<3>(QuadCopterCS::VEL_IDX));
        if (QuadCopterCS::HAS_ACC) end.addConstraint (derivative_order::ACCELERATION, q2.template segment<3>(QuadCopterCS::VEL_IDX));
        if (QuadCopterCS::HAS_JER) end.addConstraint (derivative_order::JERK,         q2.template segment<3>(QuadCopterCS::VEL_IDX));
        vertices.push_back(end);

        std::vector<value_type> segment_times(2);
        // The formula was taken from mav_planning_utils::estimateSegmentTimes
        segment_times[1] = distance;
        // if (segment_times[1] < 1e-5) return PathPtr_t();
        // This value is not important.
        segment_times[0] = distance / 2;

        if (linear) {
          PolynomialOptimization<N> opt(dimension);
          opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
          opt.solveLinear();

          opt.getSegments(&segments);
        } else {
          PolynomialOptimizationNonLinear<N> nlopt(dimension, nlOptParam, true);
          nlopt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
          // nlopt.addMaximumMagnitudeConstraint(derivative_order::VELOCITY, 2);
          // nlopt.addMaximumMagnitudeConstraint(derivative_order::ACCELERATION, 2);
          nlopt.optimize();

          OptimizationInfo info = nlopt.getOptimizationInfo();

          switch (info.stopping_reason) {
            case nlopt::SUCCESS:
            case nlopt::FTOL_REACHED:
            case nlopt::XTOL_REACHED:
              break;
            case nlopt::FAILURE:
            case nlopt::INVALID_ARGS:
            case nlopt::OUT_OF_MEMORY:
            case nlopt::ROUNDOFF_LIMITED:
            case nlopt::FORCED_STOP:
            case nlopt::STOPVAL_REACHED:
            case nlopt::MAXEVAL_REACHED:
            case nlopt::MAXTIME_REACHED:
            default:
              std::stringstream ss;
              info.print(ss);
              hppDout (warning, "NLopt Finished: \n" << ss.str());
              return false;
          }
          nlopt.getPolynomialOptimizationRef().getSegments(&segments);
        }
        return true;
      }

      template <typename Derived>
      void computeYawSegments (
          const Eigen::MatrixBase<Derived>& /*q1*/,
          const Eigen::MatrixBase<Derived>& /*q2*/,
          const value_type& distance,
          typename mav_planning_utils::Segment<2>::Vector& segments)
      {
        segments.push_back(mav_planning_utils::Segment<2>(1));
        Eigen::Vector2d coeffs (0,0);
        // Eigen::Vector2d coeffs (
        // yawFromQuaternion(Eigen::Quaternion<value_type>(q1[3],q1[4],q1[5],q1[6])),
        // yawFromQuaternion(Eigen::Quaternion<value_type>(q2[3],q2[4],q2[5],q2[6])));
        segments[0][0].setCoefficients(coeffs);
        segments[0].setTime(distance); // or only tmax ?
      }
    }

    SteeringMethod::~SteeringMethod ()
    {
      delete param_;
    }

    void SteeringMethod::init (SteeringMethodWkPtr_t weak)
    {
      core::SteeringMethod::init (weak);
      weak_ = weak;

      linear_ = problem_->getParameter<bool>("Quadcopter_linear_opt", false);

      param_ = new mav_planning_utils::NonlinearOptimizationParameters;
      param_->max_iterations = 1000;
      // param_-> = problem_->getParameter<int>("NLPolyOpt_", -1);
      param_->print_debug_info = problem_->getParameter<bool>("NLPolyOpt_print_debug_info", false);

      param_->f_abs = problem_->getParameter<double>("NLPolyOpt_f_abs", -1);
      param_->f_rel = problem_->getParameter<double>("NLPolyOpt_f_rel", 0.05);
      param_->x_abs = problem_->getParameter<double>("NLPolyOpt_x_abs", -1);
      param_->x_rel = problem_->getParameter<double>("NLPolyOpt_x_rel", 0.1);

      param_->max_iterations                  = problem_->getParameter<int>("NLPolyOpt_max_iterations", 1000);
      param_->time_penalty                    = problem_->getParameter<double>("NLPolyOpt_time_penalty", 500.0);
      param_->initial_stepsize_rel            = problem_->getParameter<double>("NLPolyOpt_initial_stepsize_rel", 0.1);
      param_->inequality_constraint_tolerance = problem_->getParameter<double>("NLPolyOpt_inequality_constraint_tolerance", 0.1);

      // Define the algorithm.
      std::string algo = problem_->getParameter<std::string>("NLPolyOpt_algo", "LN_COBYLA");
      if (algo == "GN_ORIG_DIRECT")
        param_->algorithm = nlopt::GN_ORIG_DIRECT;
      else if (algo == "GN_ORIG_DIRECT_L")
        param_->algorithm = nlopt::GN_ORIG_DIRECT_L;
      else if (algo == "GN_ISRES")
        param_->algorithm = nlopt::GN_ISRES;
      else if (algo == "LN_COBYLA")
        param_->algorithm = nlopt::LN_COBYLA;
      else
        throw std::invalid_argument ("Unknow algorithm");
    }

    PathPtr_t SteeringMethod::impl_compute (ConfigurationIn_t q1,
        ConfigurationIn_t q2) const
    {
      value_type distance = problem_->distance()->operator() (q1, q2);

      mav_planning_utils::Segment<FlatPath::N>::Vector segments;
      if (!computeSegments <false, FlatPath::N> (q1, q2, distance, segments, *param_))
        return PathPtr_t ();

      // Sample the trajectory
      FlatPath::TrajectoryPtr_t trajectory (new FlatPath::Trajectory_t(dimension, segments));

      double tmin = trajectory->getMinTime();
      double tmax = trajectory->getMaxTime();
      // std::cout << tmax - tmin << ", " << distance << std::endl;

      // Yaw is kept constant.
      // TODO yaw velocity is not continuous here. Must use polynome of order 2.
      mav_planning_utils::Segment<2>::Vector yaw_segments;
      computeYawSegments (q1, q2, tmax - tmin, yaw_segments);
      FlatPath::YawTrajectoryPtr_t yaw_trajectory (new FlatPath::YawTrajectory_t(1, yaw_segments));

      PathPtr_t path = FlatPath::create (device_.lock (), trajectory, yaw_trajectory);
      // std::cout << (path->initial() - q1).transpose() << std::endl;
      return path;
    }
  } // namespace quadcopter
} // namespace hpp
