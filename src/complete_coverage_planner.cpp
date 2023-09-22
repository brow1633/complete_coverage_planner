/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
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
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include "fields2cover.h"

#include "nav2_complete_coverage_planner/complete_coverage_planner.hpp"

namespace nav2_complete_coverage_planner
{

    void CompleteCoverage::configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        // Parameter initialization
        nav2_util::declare_parameter_if_not_declared(
                node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
                    0.1));
        node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
    }

    void CompleteCoverage::cleanup()
    {
        RCLCPP_INFO(
                node_->get_logger(), "CleaningUp plugin %s of type CompleteCoverage",
                name_.c_str());
    }

    void CompleteCoverage::activate()
    {
        RCLCPP_INFO(
                node_->get_logger(), "Activating plugin %s of type CompleteCoverage",
                name_.c_str());
    }

    void CompleteCoverage::deactivate()
    {
        RCLCPP_INFO(
                node_->get_logger(), "Deactivating plugin %s of type CompleteCoverage",
                name_.c_str());
    }

    nav_msgs::msg::Path CompleteCoverage::createPlan(
            const geometry_msgs::msg::PoseStamped & start,
            const geometry_msgs::msg::PoseStamped & goal)
    {
        nav_msgs::msg::Path global_path;

        // Checking if the goal and start state is in the global frame
        if (start.header.frame_id != global_frame_) {
            RCLCPP_ERROR(
                    node_->get_logger(), "Planner will only except start position from %s frame",
                    global_frame_.c_str());
            return global_path;
        }

        if (goal.header.frame_id != global_frame_) {
            RCLCPP_INFO(
                    node_->get_logger(), "Planner will only except goal position from %s frame",
                    global_frame_.c_str());
            return global_path;
        }

        F2CPoint startpt (start.pose.position.x, start.pose.position.y);
        F2CPoint endpt (goal.pose.position.x, goal.pose.position.y);
        F2CPoint p3 (start.pose.position.x, goal.pose.position.y);
        F2CPoint p4 (goal.pose.position.x, start.pose.position.y);

        F2CLinearRing ring{startpt, p3, endpt, p4, startpt};

        F2CCell cell(ring);

        F2CCells cells(cell);

        F2CRobot robot (0.3, 0.3);
        robot.setMinRadius(0.15);
        robot.linear_curv_change = 50;

        f2c::obj::NSwath n_swath_obj;
        f2c::sg::BruteForce bf_sw_gen_nswath;

        F2CSwaths swaths_bf_nswath = bf_sw_gen_nswath.generateBestSwaths(n_swath_obj, 1.0, cells.getGeometry(0));

        f2c::rp::BoustrophedonOrder boustrophedon_sorter;
        F2CSwaths boustrophedon_swaths = boustrophedon_sorter.genSortedSwaths(swaths_bf_nswath);

        f2c::pp::PathPlanning path_planner;

        f2c::pp::DubinsCurvesCC dubins_cc;
        F2CPath path_dubins_cc = path_planner.searchBestPath(robot, boustrophedon_swaths, dubins_cc);

        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;

        f2c::types::PathSectionType prevType = path_dubins_cc.states[0].type;


        for(const auto& state : path_dubins_cc.states) {
            F2CPoint point = state.point;
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = point.getX();
            pose.pose.position.y = point.getY();
            pose.pose.position.z = 0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;

            if(global_path.poses.size() > 0) {
                if(prevType == f2c::types::PathSectionType::SWATH && state.type == f2c::types::PathSectionType::SWATH) {
                    std::vector<geometry_msgs::msg::PoseStamped> vec = straightLine(global_path.poses.back(), pose);
                    global_path.poses.insert(global_path.poses.end(), vec.begin(), vec.end());
                }
            }

            global_path.poses.push_back(pose);
            prevType = state.type;
        }
        RCLCPP_WARN(node_->get_logger(), "Number of Poses: %ld", global_path.poses.size());

        /*
        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;
        // calculating the number of loops for current value of interpolation_resolution_
        int total_number_of_loop = std::hypot(
                goal.pose.position.x - start.pose.position.x,
                goal.pose.position.y - start.pose.position.y) /
            interpolation_resolution_;
        double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
        double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

        for (int i = 0; i < total_number_of_loop; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = start.pose.position.x + x_increment * i;
            pose.pose.position.y = start.pose.position.y + y_increment * i;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;
            global_path.poses.push_back(pose);
        }

        geometry_msgs::msg::PoseStamped goal_pose = goal;
        goal_pose.header.stamp = node_->now();
        goal_pose.header.frame_id = global_frame_;
        global_path.poses.push_back(goal_pose);*/

        return global_path;
    }

    std::vector<geometry_msgs::msg::PoseStamped> CompleteCoverage::straightLine( const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal) {
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        // calculating the number of loops for current value of interpolation_resolution_
        int total_number_of_loop = std::hypot(
                goal.pose.position.x - start.pose.position.x,
                goal.pose.position.y - start.pose.position.y) /
            interpolation_resolution_;
        double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
        double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

        for (int i = 0; i < total_number_of_loop; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = start.pose.position.x + x_increment * i;
            pose.pose.position.y = start.pose.position.y + y_increment * i;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;
            poses.push_back(pose);
        }

        geometry_msgs::msg::PoseStamped goal_pose = goal;
        poses.push_back(goal_pose);
        return poses;
    }

}  // namespace nav2_complete_coverage_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_complete_coverage_planner::CompleteCoverage, nav2_core::GlobalPlanner)
