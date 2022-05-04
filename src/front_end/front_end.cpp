/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2021/2/13 上午9:11
 * @FileName: front_end.cpp
 * @Description: LiDAR Odometry node by TLS
 * @License: See LICENSE for the license information
 */

#include "tloam/front_end/front_end.hpp"

namespace tloam
{

  FrontEnd::FrontEnd(ros::NodeHandle &nh, std::string &odom_topic) : local_registration_ptr_(nullptr)
  {
#ifdef _OPENMP
    num_threads_ = omp_get_max_threads() / 2;
#else
    num_threads_ = 1;
#endif
    // subscriber
    pose_save_sub_ = nh.subscribe<std_msgs::String>("save_path", 1, &FrontEnd::onSaveLidarOdometryPoseCb, this);
    cloud_sub_ground_ptr_ = std::make_shared<CloudSubscriber>(nh, "/ground_points", 100000);
    cloud_sub_general_ptr_ = std::make_shared<CloudSubscriber>(nh, "/general_points", 100000);
    cloud_sub_edge_ptr_ = std::make_shared<CloudSubscriber>(nh, "/edge_points", 100000);
    cloud_sub_raw_ptr_ = std::make_shared<CloudSubscriber>(nh, "/velodyne_points", 100000);
    ground_truth_tf_ptr_ = std::make_unique<TFListener>(nh, "/map", "/velo_link");

    // publisher
    lidar_path_pub_ = nh.advertise<nav_msgs::Path>("lidar_odom_path", 1);
    raw_publish_ptr_ = std::make_shared<CloudPublisher>(nh, "/raw_cloud", "/map", 10000);
    lidar_odom_ptr_ = std::make_shared<OdometryPublisher>(nh, "/lidar_odom", "/map", "/base_link", 10000);
    ground_truth_ptr_ = std::make_shared<OdometryPublisher>(nh, "/ground_truth", "/map", "/velo_link", 100000);
    lidar_odom_tf_ = std::make_shared<TFBroadCaster>("/map", "/base_link");
    glocal_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "glocal_map", "/map", 10000);

    feature_extract_ptr_ = std::make_shared<featureExtract>();
  }

  FrontEnd::~FrontEnd()
  {
    std::vector<size_t>().swap(planar_scan_index);
    std::vector<size_t>().swap(planar_submap_index);
    std::vector<size_t>().swap(sphere_scan_index);
    std::vector<size_t>().swap(sphere_submap_index);
  }

  bool FrontEnd::initWithConfig()
  {
    std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/lidar_odometry.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    ground_down_sample = config_node["ground_down_sample"].as<double>();
    edge_down_sample = config_node["edge_down_sample"].as<double>();
    ground_down_sample_submap = config_node["ground_down_sample_submap"].as<double>();
    edge_down_sample_submap = config_node["edge_down_sample_submap"].as<double>();
    sphere_frame_size = config_node["sphere_frame_size"].as<int>();
    planar_frame_size = config_node["planar_frame_size"].as<int>();

    edge_crop_box_length = config_node["edge_crop_box_length"].as<double>();
    ground_crop_box_length = config_node["ground_crop_box_length"].as<double>();
    mapping_flag = config_node["mapping_flag"].as<bool>();

    initRegistraton(local_registration_ptr_, config_node);

    //read camera lidar ex
    int ni = 0;
    for (YAML::const_iterator it = config_node["extrinsicMatrix"].begin(); it != config_node["extrinsicMatrix"].end(); ++it)
    {
      int row = ni / 4;
      int col = ni % 4;
      T_velodyne2cam(row, col) = (*it).as<double>();
      ni++;
    }
    std::cout << "T_velodyne2cam: " << std::endl
              << T_velodyne2cam.matrix() << std::endl;

    return true;
  }

  void FrontEnd::spinOnce()
  {
    if (!readData())
      return;

    while (hasData())
    {
      if (!validData())
        continue;

      if (updateLidarOdometry())
      {

        static int odom_idx = 0;
        std::cout << "odom_idx: " << odom_idx++ << std::endl;

        lidar_pose_vec.push_back(lidar_odom_pose);
        //std::cout << "successful update lidar odometry" << std::endl;
        //current_time = ros::Time::now();
        if (lidar_odom_ptr_->HasSubscribers())
        {
          lidar_odom_ptr_->Publish(lidar_odom_pose, current_time.toSec());
        }

        if (ground_truth_ptr_->HasSubscribers())
        {
          ground_truth_ptr_->Publish(ground_truth, current_time.toSec());
        }

        lidar_odom_tf_->SendTransform(lidar_odom_pose, current_time.toSec());
        std::shared_ptr<open3d::geometry::PointCloud2> curr_map = std::make_shared<open3d::geometry::PointCloud2>();
        *curr_map = current_raw_data.cloud_ptr->Transform(lidar_odom_pose.matrix());
        std::cout << "Publish current cloud begin" << std::endl;
        raw_publish_ptr_->Publisher(curr_map, current_time.toSec());
        std::cout << "Publish current cloud over" << std::endl;

        if (mapping_flag)
        {
          if (glocal_map_pub_ptr_->HasSubscribers())
          {
            std::cout << "Publish global map cloud begin" << std::endl;
            glocal_map_pub_ptr_->Publisher(global_map_data.cloud_ptr, current_time.toSec());
            std::cout << "Publish global map cloud over" << std::endl;
          }
        }
      }
    }
  }

  bool FrontEnd::readData()
  {
    cloud_sub_ground_ptr_->ParseData(ground_data_buff_);
    cloud_sub_general_ptr_->ParseData(general_data_buff_);
    cloud_sub_edge_ptr_->ParseData(edge_data_buff_);
    cloud_sub_raw_ptr_->ParseData(raw_data_buff_);
    ground_truth_tf_ptr_->LookupData(ground_truth);

    return true;
  }

  bool FrontEnd::hasData()
  {
    if (ground_data_buff_.empty())
      return false;
    if (edge_data_buff_.empty())
      return false;
    if (general_data_buff_.empty())
      return false;
    if (raw_data_buff_.empty())
      return false;

    return true;
  }

  bool FrontEnd::validData()
  {
    current_ground_data = ground_data_buff_.front();
    current_edge_data = edge_data_buff_.front();
    current_general_data = general_data_buff_.front();
    current_raw_data = raw_data_buff_.front();

    // check all message have been synchronized
    if (current_general_data.time - current_ground_data.time < -0.5 * sensorPeriod || current_general_data.time - current_edge_data.time < -0.5 * sensorPeriod)
    {
      general_data_buff_.pop_front();
      return false;
    }
    if (current_general_data.time - current_ground_data.time > 0.5 * sensorPeriod)
    {
      ground_data_buff_.pop_front();
      return false;
    }
    if (current_general_data.time - current_edge_data.time > 0.5 * sensorPeriod)
    {
      edge_data_buff_.pop_front();
      return false;
    }

    current_time = ros::Time(current_general_data.time);

    ground_data_buff_.pop_front();
    edge_data_buff_.pop_front();
    general_data_buff_.pop_front();
    raw_data_buff_.pop_front();

    return true;
  }

  bool FrontEnd::setInitPose(const Eigen::Isometry3d &init_pose_)
  {
    this->init_pose = init_pose_;
    return true;
  }

  bool FrontEnd::initRegistraton(std::shared_ptr<RegistrationInterface> &registration_ptr_,
                                 const YAML::Node &config_node)
  {
    std::string registration_method = config_node["local_registration_method"].as<std::string>();

    if (registration_method == "TLS")
    {
      registration_ptr_ = std::make_shared<LocalRegistration>(config_node["TLS"]);
    }
    else
    {
      ROS_ERROR("Other methods are not yet supported.");
      return false;
    }
    return true;
  }

  void FrontEnd::savePose(std::ofstream &ofs, const Eigen::Matrix4d &pose)
  {
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        ofs << pose(i, j);
        if (i == 2 && j == 3)
          ofs << std::endl;
        else
          ofs << " ";
      }
    }
  }

  void FrontEnd::processCloud()
  {
    // ground feature
    current_scan.ground_feature = current_ground_data.cloud_ptr->VoxelDownSample(ground_down_sample);

    // edge features
    current_scan.edge_feature = current_edge_data.cloud_ptr->VoxelDownSample(edge_down_sample);

    planar_scan_index.clear();
    planar_submap_index.clear();
    sphere_scan_index.clear();
    sphere_submap_index.clear();

    // extract vertical planar features and sphere features
    feature_extract_ptr_->extractPlanarSphere(current_general_data, planar_scan_index, planar_submap_index, sphere_scan_index, sphere_submap_index);

    // planar features
    current_scan.planar_feature = current_general_data.cloud_ptr->SelectByIndex(planar_scan_index);
    current_scan.sphere_feature = current_general_data.cloud_ptr->SelectByIndex(sphere_scan_index);
  }

  void FrontEnd::updateSubmap()
  {
    FrameBuffer sphere_frame{};
    sphere_frame.pose = lidar_odom_pose.matrix();
    sphere_frame.cloud_data_ptr = current_general_data.cloud_ptr->SelectByIndex(sphere_submap_index);
    submap_sphere_buffer.emplace_back(sphere_frame);

    FrameBuffer planar_frame{};
    planar_frame.pose = lidar_odom_pose.matrix();
    planar_frame.cloud_data_ptr = current_general_data.cloud_ptr->SelectByIndex(planar_submap_index);
    submap_planar_buffer.emplace_back(planar_frame);

    while (submap_sphere_buffer.size() > static_cast<size_t>(sphere_frame_size))
    {
      submap_sphere_buffer.pop_front();
    }

    while (submap_planar_buffer.size() > static_cast<size_t>(planar_frame_size))
    {
      submap_planar_buffer.pop_front();
    }

    submap.sphere_feature.reset(new open3d::geometry::PointCloud2);
    for (auto &sphere_frame : submap_planar_buffer)
    {
      std::shared_ptr<open3d::geometry::PointCloud2> cloud_sphere = std::make_shared<open3d::geometry::PointCloud2>();
      size_t totalSize = sphere_frame.cloud_data_ptr->points_.size();
      cloud_sphere->points_.resize(totalSize);
      for (size_t i = 0; i < totalSize; ++i)
      {
        cloud_sphere->points_[i] = sphere_frame.cloud_data_ptr->points_[i];
      }

      *submap.sphere_feature += cloud_sphere->Transform(sphere_frame.pose);
    }

    submap.planar_feature.reset(new open3d::geometry::PointCloud2);
    for (auto &planar_frame : submap_planar_buffer)
    {
      std::shared_ptr<open3d::geometry::PointCloud2> cloud_planar = std::make_shared<open3d::geometry::PointCloud2>();
      size_t totalSize = planar_frame.cloud_data_ptr->points_.size();
      cloud_planar->points_.resize(totalSize);
      for (size_t i = 0; i < totalSize; ++i)
      {
        cloud_planar->points_[i] = planar_frame.cloud_data_ptr->points_[i];
      }

      *submap.planar_feature += cloud_planar->Transform(planar_frame.pose);
    }

    // crop edge and ground features
    *submap.edge_feature += current_scan.edge_feature->Transform(lidar_odom_pose.matrix());
    *submap.ground_feature += current_scan.ground_feature->Transform(lidar_odom_pose.matrix());

    // create box
    open3d::geometry::AxisAlignedBoundingBox crop_box{};
    Eigen::Vector3d curr_pos = lidar_odom_pose.translation();
    Eigen::Vector3d curr_pos_front = curr_pos + Eigen::Vector3d(edge_crop_box_length, edge_crop_box_length, edge_crop_box_length);
    Eigen::Vector3d curr_pos_back = curr_pos - Eigen::Vector3d(edge_crop_box_length, edge_crop_box_length, edge_crop_box_length);
    crop_box.min_bound_ = curr_pos_back;
    crop_box.max_bound_ = curr_pos_front;

    // perform crop
    submap.edge_feature = submap.edge_feature->Crop(crop_box)->VoxelDownSample(edge_down_sample_submap);

    curr_pos_front = curr_pos + Eigen::Vector3d(ground_crop_box_length, ground_crop_box_length, ground_crop_box_length);
    curr_pos_back = curr_pos - Eigen::Vector3d(ground_crop_box_length, ground_crop_box_length, ground_crop_box_length);
    crop_box.min_bound_ = curr_pos_back;
    crop_box.max_bound_ = curr_pos_front;

    submap.ground_feature = submap.ground_feature->Crop(crop_box)->VoxelDownSample(ground_down_sample_submap);

    // update feature submap to registration module.
    local_registration_ptr_->setInputTarget(submap);

    if (mapping_flag)
    {
      std::shared_ptr<open3d::geometry::PointCloud2> curr_map = std::make_shared<open3d::geometry::PointCloud2>();
      *curr_map = current_raw_data.cloud_ptr->Transform(lidar_odom_pose.matrix());
      *global_map_data.cloud_ptr += *curr_map->VoxelDownSample(1.0);
    }
  }

  bool FrontEnd::updateLidarOdometry()
  {
    static bool odometry_inited = false;
    //static int key_frame_index = 0;
    static Eigen::Isometry3d last_pose = init_pose;
    static Eigen::Isometry3d predicate_pose = init_pose;

    // check whether it is the first lidar frame
    if (!odometry_inited)
    {
      *submap.edge_feature += *current_edge_data.cloud_ptr;
      *submap.ground_feature += *current_ground_data.cloud_ptr->VoxelDownSample(ground_down_sample);

      feature_extract_ptr_->extractPlanarSphere(current_general_data, planar_scan_index, planar_submap_index, sphere_scan_index, sphere_submap_index);

      *submap.planar_feature += *current_general_data.cloud_ptr->SelectByIndex(planar_submap_index);
      *submap.sphere_feature += *current_general_data.cloud_ptr->SelectByIndex(sphere_submap_index);

      local_registration_ptr_->setInputTarget(submap);

      // free memory
      std::vector<size_t>().swap(planar_scan_index);
      std::vector<size_t>().swap(planar_submap_index);
      std::vector<size_t>().swap(sphere_scan_index);
      std::vector<size_t>().swap(sphere_submap_index);

      odometry_inited = true;

      return true;
    }
    //
    // extract corresponding features
    //
    processCloud();

    //
    // set source cloud
    //
    local_registration_ptr_->setInputSource(current_scan);

    //
    // perform scan-matching
    //
    Frame result_frame{};
    Timer timer;
    ROS_INFO("Begin scanMatching optimization");
    local_registration_ptr_->scanMatching(result_frame, predicate_pose, lidar_odom_pose);
    ROS_INFO("Total optimization time: %ld ms.", timer.elapsed());

    //std::cout << "current robot pose: \n" << lidar_odom_pose.translation() << std::endl;

    //
    // update init pose for next step
    //
    step_pose = last_pose.inverse() * lidar_odom_pose;
    predicate_pose = lidar_odom_pose * step_pose;

    last_pose = lidar_odom_pose;

    updateSubmap();

    return true;
  }

  void FrontEnd::onSaveLidarOdometryPoseCb(const std_msgs::String::ConstPtr &trajectory_file_name)
  {
    char file_name[128] = {0};
    sprintf(file_name, "/home/kinggreat24/tloam_%s_pred.txt", trajectory_file_name->data.c_str());
    std::ofstream f_map;
    f_map.open(file_name);
    f_map << std::fixed;

    for (size_t i = 0; i < lidar_pose_vec.size(); i++)
    {
      Eigen::Matrix4d twc_lidar = lidar_pose_vec[i].matrix();

      static Eigen::Matrix4d twc_origin = twc_lidar;

      //转到以第一帧为初始坐标系中
      twc_lidar = twc_origin.inverse() * twc_lidar;

      // Eigen::Matrix4d twc_cam   =  twc_lidar * T_velodyne2cam.inverse();

      //转换到相机坐标系中
      Eigen::Matrix4d twc_cam = T_velodyne2cam * twc_lidar;

      //位置信息
      Eigen::Vector3d t_cam = twc_lidar.topRightCorner(3, 1);

      //姿态信息
      Eigen::Matrix3d R_cam = twc_lidar.topLeftCorner(3, 3);

      f_map << std::setprecision(9)
            << R_cam(0, 0) << " " << R_cam(0, 1) << " " << R_cam(0, 2) << " " << t_cam(0) << " "
            << R_cam(1, 0) << " " << R_cam(1, 1) << " " << R_cam(1, 2) << " " << t_cam(1) << " "
            << R_cam(2, 0) << " " << R_cam(2, 1) << " " << R_cam(2, 2) << " " << t_cam(2) << std::endl;
    }
    f_map.close();
    std::cout << std::endl
              << "*******       trajectory saved!" << std::endl;
  }
}
