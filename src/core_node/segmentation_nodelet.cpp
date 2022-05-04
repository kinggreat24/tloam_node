/*
 * @Author: your name
 * @Date: 2022-03-06 20:18:28
 * @LastEditTime: 2022-03-08 15:24:36
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /tloam/src/core_node/segmentation_nodelet.cpp
 */
/**
 * @Copyright 2021, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2021/2/1 上午9:55
 * @FileName: segmentation_nodelet.cpp
 * @Description: Complete the segmentation of point cloud
 * @License: See LICENSE for the license information
 */

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ecl/threads/thread.hpp>
// #include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include "tloam/models/segmentation/segmentation.hpp"
#include "tloam/models/utils/utils.hpp"

namespace tloam
{
  class SegmentationNode
  {
  public:
    SegmentationNode(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
        : shutdown_requested_(false), nh_(nh), private_nh_(nh_private)
    {
    }

    ~SegmentationNode()
    {
      // NODELET_DEBUG_STREAM("Waiting for Segmentation Node to finish.");
      shutdown_requested_ = true;
      // update_thread_.join();
    }

  public:
    void onInit()
    {
      init_params();
      segmentation_ptr_ = std::make_shared<Segmentation>(nh_, cloud_topic);
      segmentation_ptr_->initWithConfig();
      ros::Rate loop_rate(10);

      while (!shutdown_requested_ && ros::ok())
      {
        ros::spinOnce();
        segmentation_ptr_->spinOnce();
        ros::Duration(0.001).sleep();
      }
    }

  private:
    void update(void)
    {
      ros::Rate loop_rate(10);

      while (!shutdown_requested_ && ros::ok())
      {
        ros::spinOnce();
        segmentation_ptr_->spinOnce();
        ros::Duration(0.001).sleep();
      }
    }

    bool init_params(void)
    {
      auto &ph = private_nh_;
      cloud_topic = ph.param<std::string>("cloud_topic", "/velodyne_points");

      return true;
    }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    std::shared_ptr<Segmentation> segmentation_ptr_;
    std::string cloud_topic;

    ecl::Thread update_thread_;
    bool shutdown_requested_;
  };
}

// PLUGINLIB_EXPORT_CLASS(tloam::SegmentationNode, nodelet::Nodelet);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tloam_segmentation_node");
  ros::NodeHandle nh, nh_private("~");
  tloam::SegmentationNode segmentation_node(nh,nh_private);
  segmentation_node.onInit();
  
  return 0;
}