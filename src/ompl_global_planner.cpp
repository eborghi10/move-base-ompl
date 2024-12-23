/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 * Modificator: Windel Bouwman
 *********************************************************************/

/*

    Greatly copied from:

    http://ompl.kavrakilab.org/RigidBodyPlanningWithControls_8cpp_source.html

*/

#include <ompl_global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ompl_global_planner::OmplGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace ompl_global_planner
{


OmplGlobalPlanner::OmplGlobalPlanner()
    : _costmap_ros(NULL)
    , _initialized(false)
    , _allow_unknown(true)
    , _se2_space(new ob::DubinsStateSpace(2.4, false))
    , _so2_space(new ob::SO2StateSpace())
    , _velocity_space(new ob::RealVectorStateSpace(1))
    , _costmap_model(NULL)
{
    _space = std::make_shared<ob::CompoundStateSpace>();
    _space->addSubspace(_se2_space, 1.0);
    _space->addSubspace(_so2_space, 0.5);
    _space->addSubspace(_velocity_space, 0.0);
}

void OmplGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!_initialized)
    {
        ros::NodeHandle private_nh("~/" + name);
        _costmap_ros = costmap_ros;
        _frame_id = "map";
        _costmap_model = new base_local_planner::CostmapModel(*_costmap_ros->getCostmap());

        _plan_pub = private_nh.advertise<nav_msgs::Path>("plan", 1);
        private_nh.param("allow_unknown", _allow_unknown, true);

        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);

        _initialized = true;
        ROS_INFO("Ompl global planner initialized!");
    }
    else
    {
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    }

}

void OmplGlobalPlanner::fromOMPL(const ob::State *s, double& x, double& y, double& theta, double& v, double& psi)
{
    const auto compound_state = s->as<ob::CompoundStateSpace::StateType>();
    const auto se2state = compound_state->as<ob::DubinsStateSpace::StateType>(0);
    const auto so2state = compound_state->as<ob::SO2StateSpace::StateType>(1);
    const auto v_state = compound_state->as<ob::RealVectorStateSpace::StateType>(2);

    // Get the values:
    x = se2state->getX();
    y = se2state->getY();
    theta = se2state->getYaw();
    v = (*v_state)[0];
    psi = so2state->value;
}

// Store x,y and theta into state:
void OmplGlobalPlanner::toOMPL(ob::State* rs, const double& x, const double& y, const double& theta, const double& v, const double& psi)
{
    auto compound_state = rs->as<ob::CompoundStateSpace::StateType>();
    auto se2state = compound_state->as<ob::DubinsStateSpace::StateType>(0);
    auto so2_state = compound_state->as<ob::SO2StateSpace::StateType>(1);
    auto v_state = compound_state->as<ob::RealVectorStateSpace::StateType>(2);

    // Set values:
    se2state->setX(x);
    se2state->setY(y);
    se2state->setYaw(theta);
    (*v_state)[0] = v;
    so2_state->value = psi;

    // Make sure angle is (-pi,pi]:
    const auto yaw_ss = _se2_space->as<ob::DubinsStateSpace>()->as<ob::SO2StateSpace>(1);
    auto yaw_st = se2state->as<ob::SO2StateSpace::StateType>(1);
    yaw_ss->enforceBounds(yaw_st);

    const auto psi_ss = _so2_space->as<ob::SO2StateSpace>();
    psi_ss->enforceBounds(so2_state);
}

double OmplGlobalPlanner::calc_cost(const ob::State *state)
{
    double x, y, theta, velocity, psi;
    fromOMPL(state, x, y, theta, velocity, psi);

    // Get the cost of the footprint at the current location:
    double cost = _costmap_model->footprintCost(x, y, theta, _costmap_ros->getRobotFootprint());

    if (cost < 0)
    {
        // Unknown cell, assume zero cost here!
        cost = 0;
    }

    return cost;
}

// Calculate the cost of a motion:
double motion_cost(const ob::State* s1, const ob::State* s2)
{
    // int nd = validSegmentCount(s1, s2);
    // TODO: interpolate?
    double cst = 0;

    // cst =

    return cst;
}

// Check the current state:
bool OmplGlobalPlanner::isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    if (!si->satisfiesBounds(state))
    {
        return false;
    }

    // Get the cost of the footprint at the current location:
    double cost = calc_cost(state);

    // Too high cost:
    if (cost > 90)
    {
        return false;
    }

    // Error? Unknown space?
    if (cost < 0)
    {
    }

    // Extracting psi to handle jackknifing
    const double& psi = state->as<ob::CompoundStateSpace::StateType>()->as<ompl::base::SO2StateSpace::StateType>(1)->value;

    return !jackknifing(psi);
}

bool OmplGlobalPlanner::jackknifing(const double& psi) const
{
    return fabs(psi) > JACKKNIFING_ANGLE;
}

// Calculate vehicle dynamics, assume a velocity state, but steering to be instantly possible.
void OmplGlobalPlanner::propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State* result)
{
    // Implement vehicle dynamics:
    double x, y, theta, velocity, psi;
    fromOMPL(start, x, y, theta, velocity, psi);

    // extracting control inputs u = [v w], with v = driving vel. and w = steering vel.
    const auto& ctrl_state = control->as<ompl::control::CompoundControlSpace::ControlType>()[0];
    const double v = ctrl_state.as<ompl::control::DiscreteControlSpace::ControlType>(0)->value;
    const double w = ctrl_state.as<ompl::control::DiscreteControlSpace::ControlType>(1)->value;

    const auto x_n = x + v * cos(theta);
    const auto y_n = y + v * sin(theta);
    const auto theta_n = theta + w;
    const auto psi_n = psi - v * sin(psi) / lt_ + w * (-1 - lr_ / lt_ * cos(psi));

    // Store new state in result:
    toOMPL(result, x_n, y_n, theta_n, v, psi_n);
}

double calcYaw(const geometry_msgs::Pose pose)
{
    double yaw, pitch, roll;
    tf::Pose pose_tf;
    tf::poseMsgToTF(pose, pose_tf);
    pose_tf.getBasis().getEulerYPR(yaw, pitch, roll);
    return yaw;
}

bool OmplGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan)
{
    boost::mutex::scoped_lock lock(_mutex);
    if (!_initialized)
    {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = _frame_id;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;
    }

    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);

    ROS_INFO("Thinking about OMPL path..");
    // Create OMPL problem:
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-100);
    bounds.setHigh(100);
    _se2_space->as<ob::DubinsStateSpace>()->setBounds(bounds);

    ob::RealVectorBounds velocity_bounds(1);
    velocity_bounds.setHigh(2.0);
    velocity_bounds.setLow(0.0);
    _velocity_space->as<ob::RealVectorStateSpace>()->setBounds(velocity_bounds);

    /* discretizing the control space to model Dubins motion primitives on our own.
     * the control space is now compounded of two discretized sub-spaces;
     * in particular:
     * - the driving velocity input space (Dvel) is limited to [v_min, v_max]
     * - the steering velocity input space (Svel) is limited to [-w, +w]
     * 		so w takes value among {-w, -w+1, ..., -1, 0, 1, ..., +w-1, +w}
    */
    const auto Dvel(std::make_shared<oc::DiscreteControlSpace>(_space, 0.0, 2.0));
    const auto Svel(std::make_shared<oc::DiscreteControlSpace>(_space, -1.0, 1.0));

    auto ccs = new oc::CompoundControlSpace(_space);
    ccs->addSubspace(oc::ControlSpacePtr(Dvel));
    ccs->addSubspace(oc::ControlSpacePtr(Svel));
    const oc::ControlSpacePtr cspace(ccs);

    // Create space information:
    _si = std::make_shared<oc::SpaceInformation>(_space, cspace);
    _si->setStatePropagator(boost::bind(&OmplGlobalPlanner::propagate, this, _1, _2, _3,_4));
    _si->setStateValidityChecker(boost::bind(&OmplGlobalPlanner::isStateValid, this, _si.get(), _1));

    // Define problem:
    ob::ScopedState<> ss_start(_space);
    const auto start_se2_state = ss_start->as<ob::CompoundStateSpace::StateType>()->as<ob::DubinsStateSpace::StateType>(0);
    start_se2_state->setX(start.pose.position.x);
    start_se2_state->setY(start.pose.position.y);
    start_se2_state->setYaw(tf::getYaw(start.pose.orientation));
    // Do not really know without a sensor
    const auto start_so2_state = ss_start->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1);
    start_so2_state->value = 0.0;
    // Speed
    ss_start->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = 0.0;

    ob::ScopedState<> ss_goal(_space);
    const auto goal_se2_state = ss_goal->as<ob::CompoundStateSpace::StateType>()->as<ob::DubinsStateSpace::StateType>(0);
    goal_se2_state->setX(goal.pose.position.x);
    goal_se2_state->setY(goal.pose.position.y);
    goal_se2_state->setYaw(tf::getYaw(goal.pose.orientation));
    // Do not really know without a sensor
    const auto goal_so2_state = ss_goal->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1);
    goal_so2_state->value = 0.0;
    // Speed
    ss_goal->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = 0.0;

    // Optimize criteria:
    ob::OptimizationObjectivePtr cost_objective(new CostMapObjective(*this, _si));
    ob::OptimizationObjectivePtr length_objective(new ob::PathLengthOptimizationObjective(_si));
    ob::OptimizationObjectivePtr min_arrival_time_objective(new ob::MinimizeArrivalTime(_si));
    //ob::OptimizationObjectivePtr objective(new CostMapWorkObjective(*this, _si));

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(_si));
    pdef->setStartAndGoalStates(ss_start, ss_goal, 0.1);
    pdef->setOptimizationObjective(cost_objective + length_objective + min_arrival_time_objective);

    ROS_INFO("Problem defined, running planner");
    ob::PlannerPtr planner(new og::RRTstar(_si));
    planner->setProblemDefinition(pdef);
    planner->setup();
    ob::PlannerStatus solved = planner->solve(3.0);


    // Convert path into ROS messages:
    if (solved)
    {
        ROS_INFO("Ompl done!");
        const auto& solution = pdef->getSolutionPath();

        // Cast path into geometric path:
        auto& path = static_cast<og::PathGeometric&>(*solution);
        path.interpolate(path.getStateCount() * 10);

        // Create path:
        plan.push_back(start);

        // Conversion loop from states to messages:
        for (const auto& it : path.getStates())
        {
            // Get the data from the state:
            double x, y, theta, velocity, psi;
            fromOMPL(it, x, y, theta, velocity, psi);

            // Place data into the pose:
            geometry_msgs::PoseStamped ps = goal;
            ps.header.stamp = ros::Time::now();
            ps.pose.position.x = x;
            ps.pose.position.y = y;
            plan.push_back(ps);
        }

        plan.push_back(goal);
    }
    else
    {
        ROS_ERROR("Failed to determine plan");
    }


    //publish the plan for visualization purposes
    publishPlan(plan);
    return !plan.empty();
}

void OmplGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    if (!_initialized) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if (!path.empty()) {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    _plan_pub.publish(gui_path);
}


} //end namespace global_planner

