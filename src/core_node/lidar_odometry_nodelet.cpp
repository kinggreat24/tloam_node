/*
 * @Author: your name
 * @Date: 2022-03-06 20:18:28
 * @LastEditTime: 2022-03-08 14:55:25
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /tloam/src/core_node/lidar_odometry_nodelet.cpp
 */
/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2021/2/1 下午9:14
 * @FileName: lidar_odometry_nodelet.cpp.c
 * @Description: LiDAR Odometry Nodelet
 * @License: See LICENSE for the license information
 */

#include <iostream>
#include <string>

// for ros
#include <ros/ros.h>
#include <ecl/threads/thread.hpp>
// #include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "tloam/front_end/front_end.hpp"

namespace tloam
{
  class LidarOdometryNode
  {
  public:
    LidarOdometryNode(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
        : shutdown_requested_(false), nh_(nh), private_nh_(nh_private)
    {
    }

    ~LidarOdometryNode()
    {
      // NODELET_DEBUG_STREAM("waiting for lidar_odometry_nodelet update thread to finish.");
      shutdown_requested_ = true;
      // odometry_update_thread_.join();
    }

  public:
    void onInit()
    {
      // nh_ = getNodeHandle();
      // private_nh_ = getPrivateNodeHandle();

      // Analyze whether the currently initialized nodelet is complete
      // std::string namespace_ = private_nh_.getNamespace();
      // namespace_ = namespace_.substr(namespace_.find_last_of('/') + 1);
      // NODELET_INFO_STREAM("Initialising nodelet... [" << namespace_ << "]");
      init_params();
      front_ent_ptr_ = std::make_shared<FrontEnd>(nh_, odom_topic_);
      front_ent_ptr_->initWithConfig();

      // if (front_ent_ptr_->initWithConfig()){
      // NODELET_INFO_STREAM("Nodelet initialised. Spinning up update thread ... [" << namespace_ << "]");
      // odometry_update_thread_.start(&LidarOdometryNode::update, *this);
      // NODELET_INFO_STREAM("Nodelet initialised. [" << namespace_ << "]");
      // } else {
      //   NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart. [" << namespace_ << "]");
      // }

      ros::Rate loop_rate(10);
      while (ros::ok() && !shutdown_requested_)
      {
        front_ent_ptr_->spinOnce();
        ros::spinOnce();
      }
    }

  private:
    void update()
    {
      ros::Rate loop_rate(10);
      while (ros::ok() && !shutdown_requested_)
      {
        front_ent_ptr_->spinOnce();
        ros::spinOnce();
      }
    }

    void init_params()
    {
      auto &ph_ = private_nh_;
      cloud_topic_ = ph_.param<std::string>("cloud_topic", "/velodyne_points");
    }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    std::string cloud_topic_;
    std::string ground_topic_;
    std::string edge_topic_;
    std::string general_topic_;
    std::string odom_topic_;

    std::shared_ptr<FrontEnd> front_ent_ptr_;

    ecl::Thread odometry_update_thread_;

    bool shutdown_requested_;
  };

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tloam_segmentation_node");
  ros::NodeHandle nh, nh_private("~");
  tloam::LidarOdometryNode lidarOdometry_node(nh,nh_private);
  lidarOdometry_node.onInit();
  
  return 0;
}


// PLUGINLIB_EXPORT_CLASS(tloam::LidarOdometryNode, nodelet::Nodelet);