/*
 * @Author: your name
 * @Date: 2022-03-06 20:18:28
 * @LastEditTime: 2022-03-09 15:21:51
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /tloam/src/core_node/kitti_reader_nodelet.cpp
 */
/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/27 下午1:52
 * @FileName: kitti_reader_nodelet.cpp
 * @Description: Load KITTI Odometry Dataset
 * @License: See LICENSE for the license information
 */

#include <iostream>
#include <string>
#include <glog/logging.h>
#include <boost/filesystem.hpp>

// for nodelet
#include <ros/ros.h>
#include <ecl/threads/thread.hpp>
// #include <nodelet/nodelet.h>
// #include <pluginlib/class_list_macros.hpp>

#include "tloam/models/io/kitti_reader.hpp"
#include "tloam/models/utils/utils.hpp"
#include "tloam/models/utils/work_space_path.h"

namespace tloam
{

  class KittiNode
  {
  public:
    KittiNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
      : shutdown_requested_(false), nh_(nh), private_nh_(nh_private)
    {
    }

    ~KittiNode()
    {
      // NODELET_DEBUG_STREAM("waiting for kitti_reader_nodelet update thread to finish.");
      shutdown_requested_ = true;
      // kitti_update_thread_.join();
    }

  public:
    void onInit()
    {
      // nh_ = getNodeHandle();
      // private_nh_ = getPrivateNodeHandle();
      // Analyze whether the currently initialized nodelet is complete
      // std::string namespace_ = private_nh_.getNamespace();
      // namespace_ = namespace_.substr(namespace_.find_last_of('/') + 1);
      ROS_INFO("Initialising nodelet... ");

      init_params();
      kitti_reader_ptr_ = std::make_unique<KittiReader>(nh_);
      kitti_reader_ptr_->initScanFilenames();
      
      start_idx_ = 0;
      private_nh_.param("start_frame", start_idx_, start_idx_);
      kitti_reader_ptr_->setStartFrame(start_idx_);

      ROS_INFO("Tloam topic circle");

      static int file_index = start_idx_;
      ros::Rate loop_rate(5);

      ros::Duration(5.0).sleep();

      while (ros::ok() && !shutdown_requested_)
      {
        ros::spinOnce();
        kitti_reader_ptr_->spinOnce();
        ROS_INFO("Reading %d scan.", file_index);
        file_index++;
        loop_rate.sleep();
      }
    }

  private:
    void update()
    {
      static int file_index = 0;
      ros::Rate loop_rate(10);
      while (ros::ok() && !shutdown_requested_)
      {
        ros::spinOnce();
        kitti_reader_ptr_->spinOnce();
        ROS_INFO("Reading %d scan.", file_index);
        file_index++;
        loop_rate.sleep();
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
    ros::Publisher pub_raw_cloud_;
    std::string cloud_topic_;
    std::unique_ptr<KittiReader> kitti_reader_ptr_;
    ecl::Thread kitti_update_thread_;

    int start_idx_;

    bool shutdown_requested_;
  };

}

// PLUGINLIB_EXPORT_CLASS(tloam::KittiNode, nodelet::Nodelet);


int main(int argc, char** argv)
{
  ros::init(argc, argv,"tloam_kitti_reader");
  ros::NodeHandle nh, nh_private("~");

  tloam::KittiNode kitti_node(nh,nh_private);
  kitti_node.onInit();
  return 0;

}