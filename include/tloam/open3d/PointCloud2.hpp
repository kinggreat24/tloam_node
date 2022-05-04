/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/08/19 下午9:24
 * @FileName: PointCloud2.hpp
 * @Description: Increase the intensity information and other functions of the point cloud
 * @License: See LICENSE for the license information
 */
#pragma once

#include <Eigen/Core>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>
#include <queue>

#include <open3d/geometry/Geometry3D.h>
#include <open3d/geometry/TetraMesh.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/KDTreeSearchParam.h>
#include <open3d/camera/PinholeCameraIntrinsic.h>
#include <open3d/geometry/Image.h>
#include <open3d/geometry/RGBDImage.h>
#include <open3d/geometry/VoxelGrid.h>
#include <open3d/utility/Console.h>


namespace open3d {

namespace camera {
    class PinholeCameraIntrinsic;
}

namespace geometry{

    class Image;
    class RGBDImage;
    class TriangleMesh;
    class VoxelGrid;

/// \class PointCloud2
///
/// \brief A point cloud consists of point coordinates, and optionally point
/// colors, point normals and point intensity.
class PointCloud2 : public Geometry3D {
public:
    /// \brief Default Constructor.
    PointCloud2() : Geometry3D(Geometry::GeometryType::PointCloud) {}
    /// \brief Parameterized Constructor.
    ///
    /// \param points Points coordinates.
    explicit PointCloud2(std::vector<Eigen::Vector3d> points)
            : Geometry3D(Geometry::GeometryType::PointCloud), points_(std::move(points)) {}
    ~PointCloud2() override = default;

public:
    PointCloud2 &Clear() override;
    bool IsEmpty() const override;
    Eigen::Vector3d GetMinBound() const override;
    Eigen::Vector3d GetMaxBound() const override;
    Eigen::Vector3d GetCenter() const override;
    AxisAlignedBoundingBox GetAxisAlignedBoundingBox() const override;
    OrientedBoundingBox GetOrientedBoundingBox() const override;
    PointCloud2 &Transform(const Eigen::Matrix4d &transformation) override;
    PointCloud2 &Translate(const Eigen::Vector3d &translation,
                           bool relative = true) override;
    PointCloud2 &Scale(const double scale,
                       const Eigen::Vector3d &center) override;
    PointCloud2 &Rotate(const Eigen::Matrix3d &R,
                        const Eigen::Vector3d &center) override;

    PointCloud2& operator+=(const PointCloud2& cloud);
    PointCloud2& operator+(const PointCloud2& cloud) const;

    /// Returns 'true' if the point cloud contains points.
    bool HasPoints() const { return !points_.empty(); }

    /// Returns `true` if the point cloud contains point normals.
    bool HasNormals() const {
        return !points_.empty() && normals_.size() == points_.size();
    }

    /// Returns `true` if the point cloud contains point colors.
    bool HasColors() const {
        return !points_.empty() && colors_.size() == points_.size();
    }

    /// Returns `true` if the point cloud contains point intensity.
    bool HasIntensity() const {
        return !intensity_.empty() && intensity_.size() == points_.size();
    }

    /// Normalize point normals to length 1.
    PointCloud2 &NormalizeNormals() {
        for (auto & normal : normals_) {
            normal.normalize();
        }
        return *this;
    }

    /// Assigns each point in the PointCloud the same color.
    ///
    /// \param color  RGB colors of points.
    PointCloud2 &PaintUniformColor(const Eigen::Vector3d &color) {
        ResizeAndPaintUniformColor(colors_, points_.size(), color);
        return *this;
    }

    /// \brief Remove all points from the point cloud that have a nan entry, or
    /// infinite entries.
    ///
    /// Also removes the corresponding normals and color entries.
    ///
    /// \param remove_nan Remove NaN values from the PointCloud.
    /// \param remove_infinite Remove infinite values from the PointCloud.
    PointCloud2 &RemoveNonFinitePoints(bool remove_nan = true,
                                       bool remove_infinite = true);

    /// \brief Function to select points from \p input pointcloud2 into
    /// \p output pointcloud2.
    ///
    /// Points with indices in \p indices are selected.
    ///
    /// \param indices Indices of points to be selected.
    /// \param invert Set to `True` to invert the selection of indices.
    std::shared_ptr<PointCloud2> SelectByIndex(
            const std::vector<size_t> &indices, bool invert = false) const;

    /// \brief Function to downsample input pointcloud2 into output pointcloud2
    /// with a voxel.
    ///
    /// Normals and colors are averaged if they exist.
    ///
    /// \param voxel_size Defines the resolution of the voxel grid,
    /// smaller value leads to denser output point cloud.
    std::shared_ptr<PointCloud2> VoxelDownSample(double voxel_size) const;

    /// \brief Function to downsample using geometry.PointCloud2.VoxelDownSample
    ///
    /// Also records point cloud2 index before downsampling.
    ///
    /// \param voxel_size Voxel size to downsample into.
    /// \param min_bound Minimum coordinate of voxel boundaries
    /// \param max_bound Maximum coordinate of voxel boundaries
    /// \param approximate_class Whether to approximate.
    std::tuple<std::shared_ptr<PointCloud2>,
            Eigen::MatrixXi,
            std::vector<std::vector<int>>>
    VoxelDownSampleAndTrace(double voxel_size,
                            const Eigen::Vector3d &min_bound,
                            const Eigen::Vector3d &max_bound,
                            bool approximate_class = false) const;

    /// \brief Function to downsample input pointcloud2 into output pointcloud2
    /// uniformly.
    ///
    /// The sample is performed in the order of the points with the 0-th point
    /// always chosen, not at random.
    ///
    /// \param every_k_points Sample rate, the selected point indices are [0, k,
    /// 2k, …].
    std::shared_ptr<PointCloud2> UniformDownSample(size_t every_k_points) const;

    /// \brief Function to downsample input pointcloud2 into output pointcloud2
    /// randomly.
    ///
    /// The sample is performed by randomly selecting the index of the points
    /// in the pointcloud.
    ///
    /// \param sampling_ratio Sampling ratio, the ratio of sample to total
    /// number of points in the pointcloud.
    std::shared_ptr<PointCloud2> RandomDownSample(double sampling_ratio) const;

    /// \brief RandomDownSample applies a random sampling with uniform probability through random library by C++11.
    /// Based off Algorithm A from the paper "Faster Methods for Random Sampling"
    /// by Jeffrey Scott Vitter. The algorithm runs in O(N) and results as return
    ///
    /// References: http://www.ittc.ku.edu/~jsv/Papers/Vit84.sampling.pdf
    /// https://github.com/PointCloudLibrary/pcl/blob/master/filters/include/pcl/filters/impl/random_sample.hpp
    /// \author Pengwei Zhou
    /// \param sample_num, the number of sample to total number of points in the pointcloud
    /// \param dummy, it is a dummy parameter and only used to distinguish function overloading.
    std::shared_ptr<PointCloud2> RandomDownSample(size_t sample_num, bool dummy) const;

    /// \brief Function to crop pointcloud2 into output pointcloud2
    ///
    /// All points with coordinates outside the bounding box \p bbox are
    /// clipped.
    ///
    /// \param bbox AxisAlignedBoundingBox to crop points.
    std::shared_ptr<PointCloud2> Crop(const AxisAlignedBoundingBox &bbox) const;

    /// \brief Function to crop pointcloud2 into output pointcloud2
    ///
    /// All points with coordinates outside the bounding box \p bbox are
    /// clipped.
    ///
    /// \param bbox OrientedBoundingBox to crop points.
    std::shared_ptr<PointCloud2> Crop(const OrientedBoundingBox &bbox) const;

    /// \brief Function to remove points that have less than \p nb_points in a
    /// sphere of a given radius.
    ///
    /// \param nb_points Number of points within the radius.
    /// \param search_radius Radius of the sphere.
    std::tuple<std::shared_ptr<PointCloud2>, std::vector<size_t>>
    RemoveRadiusOutliers(size_t nb_points, double search_radius) const;

    /// \brief Function to remove points that are further away from their
    /// \p nb_neighbor neighbors in average.
    ///
    /// \param nb_neighbors Number of neighbors around the target point.
    /// \param std_ratio Standard deviation ratio.
    std::tuple<std::shared_ptr<PointCloud2>, std::vector<size_t>>
    RemoveStatisticalOutliers(size_t nb_neighbors, double std_ratio) const;

    /// \brief Function to compute the normals of a point cloud2.
    ///
    /// Normals are oriented with respect to the input point cloud if normals
    /// exist.
    ///
    /// \param search_param The KDTree search parameters for neighborhood
    /// search. \param fast_normal_computation If true, the normal estiamtion
    /// uses a non-iterative method to extract the eigenvector from the
    /// covariance matrix. This is faster, but is not as numerical stable.
    void EstimateNormals(
            const KDTreeSearchParam &search_param = KDTreeSearchParamKNN(),
            bool fast_normal_computation = true);


    /// \brief Function to orient the normals of a point cloud2.
    ///
    /// \param orientation_reference Normals are oriented with respect to
    /// orientation_reference.
    void OrientNormalsToAlignWithDirection(
            const Eigen::Vector3d &orientation_reference =
            Eigen::Vector3d(0.0, 0.0, 1.0));

    /// \brief Function to orient the normals of a point cloud2.
    ///
    /// \param camera_location Normals are oriented with towards the
    /// camera_location.
    void OrientNormalsTowardsCameraLocation(
            const Eigen::Vector3d &camera_location = Eigen::Vector3d::Zero());

    /// \brief Function to consistently orient estimated normals based on
    /// consistent tangent planes as described in Hoppe et al., "Surface
    /// Reconstruction from Unorganized Points", 1992.
    ///
    /// \param k k nearest neighbour for graph reconstruction for normal
    /// propagation.
    void OrientNormalsConsistentTangentPlane(size_t k);

    /// \brief Function to compute the point to point distances between point
    /// clouds.
    ///
    /// For each point in the \p source point cloud, compute the distance to the
    /// \p target point cloud.
    ///
    /// \param target The target point cloud.
    std::vector<double> ComputePointCloudDistance(const PointCloud2 &target);

    /// Function to compute the mean and covariance matrix
    /// of a point cloud2.
    std::tuple<Eigen::Vector3d, Eigen::Matrix3d> ComputeMeanAndCovariance() const;

    /// \brief Function to compute the Mahalanobis distance for points
    /// in an input point cloud.
    ///
    /// See: https://en.wikipedia.org/wiki/Mahalanobis_distance
    std::vector<double> ComputeMahalanobisDistance() const;

    /// Function to compute the distance from a point to its nearest neighbor in
    /// the input point cloud
    std::vector<double> ComputeNearestNeighborDistance() const;

    /// Function that computes the convex hull of the point cloud using qhull
    std::tuple<std::shared_ptr<TriangleMesh>, std::vector<size_t>>
    ComputeConvexHull() const;

    /// \brief This is an implementation of the Hidden Point Removal operator
    /// described in Katz et. al. 'Direct Visibility of Point Sets', 2007.
    ///
    /// Additional information about the choice of radius
    /// for noisy point clouds can be found in Mehra et. al. 'Visibility of
    /// Noisy Point Cloud Data', 2010.
    ///
    /// \param camera_location All points not visible from that location will be
    /// removed. \param radius The radius of the spherical projection.
    std::tuple<std::shared_ptr<TriangleMesh>, std::vector<size_t>>
    HiddenPointRemoval(const Eigen::Vector3d &camera_location,
                       double radius) const;

    /// \brief Cluster PointCloud2 using the DBSCAN algorithm
    /// Ester et al., "A Density-Based Algorithm for Discovering Clusters
    /// in Large Spatial Databases with Noise", 1996
    ///
    /// Returns a list of point labels, -1 indicates noise according to
    /// the algorithm.
    ///
    /// \param eps Density parameter that is used to find neighbouring points.
    /// \param min_points Minimum number of points to form a cluster.
    /// \param print_progress If `true` the progress is visualized in the
    /// console.
    std::vector<int> ClusterDBSCAN(double eps,
                                   size_t min_points,
                                   bool print_progress = false) const;

    /// \brief Segment PointCloud plane using the RANSAC algorithm.
    ///
    /// \param distance_threshold Max distance a point can be from the plane
    /// model, and still be considered an inlier.
    /// \param ransac_n Number of initial points to be considered inliers in
    /// each iteration.
    /// \param num_iterations Number of iterations.
    /// \return Returns the plane model ax + by + cz + d = 0 and the indices of
    /// the plane inliers.
    std::tuple<Eigen::Vector4d, std::vector<size_t>> SegmentPlane(
            const double distance_threshold = 0.01,
            const int ransac_n = 3,
            const int num_iterations = 100) const;

    /// \brief Factory function to create a pointcloud2 from a depth image and a
    /// camera model.
    ///
    /// Given depth value d at (u, v) image coordinate, the corresponding 3d
    /// point is: z = d / depth_scale\n x = (u - cx) * z / fx\n y = (v - cy) * z
    /// / fy\n
    ///
    /// \param depth The input depth image can be either a float image, or a
    /// uint16_t image. \param intrinsic Intrinsic parameters of the camera.
    /// \param extrinsic Extrinsic parameters of the camera.
    /// \param depth_scale The depth is scaled by 1 / \p depth_scale.
    /// \param depth_trunc Truncated at \p depth_trunc distance.
    /// \param stride Sampling factor to support coarse point cloud extraction.
    ///
    /// \return An empty pointcloud if the conversion fails.
    /// If \param project_valid_depth_only is true, return point cloud, which
    /// doesn't
    /// have nan point. If the value is false, return point cloud, which has
    /// a point for each pixel, whereas invalid depth results in NaN points.
    static std::shared_ptr<PointCloud2> CreateFromDepthImage(
            const Image &depth,
            const camera::PinholeCameraIntrinsic &intrinsic,
            const Eigen::Matrix4d &extrinsic = Eigen::Matrix4d::Identity(),
            double depth_scale = 1000.0,
            double depth_trunc = 1000.0,
            int stride = 1,
            bool project_valid_depth_only = true);

    /// \brief Factory function to create a pointcloud from an RGB-D image and a
    /// camera model.
    ///
    /// Given depth value d at (u, v) image coordinate, the corresponding 3d
    /// point is: z = d / depth_scale\n x = (u - cx) * z / fx\n y = (v - cy) * z
    /// / fy\n
    ///
    /// \param image The input image.
    /// \param intrinsic Intrinsic parameters of the camera.
    /// \param extrinsic Extrinsic parameters of the camera.
    ///
    /// \return An empty pointcloud if the conversion fails.
    /// If \param project_valid_depth_only is true, return point cloud, which
    /// doesn't
    /// have nan point. If the value is false, return point cloud, which has
    /// a point for each pixel, whereas invalid depth results in NaN points.
    static std::shared_ptr<PointCloud2> CreateFromRGBDImage(
            const RGBDImage &image,
            const camera::PinholeCameraIntrinsic &intrinsic,
            const Eigen::Matrix4d &extrinsic = Eigen::Matrix4d::Identity(),
            bool project_valid_depth_only = true);

    /// \brief Function to create a PointCloud from a VoxelGrid.
    ///
    /// It transforms the voxel centers to 3D points using the original point
    /// cloud coordinate (with respect to the center of the voxel grid).
    ///
    /// \param voxel_grid The input VoxelGrid.
    std::shared_ptr<PointCloud2> CreateFromVoxelGrid(
            const VoxelGrid &voxel_grid);
public:
    /// Points coordinates.
    std::vector<Eigen::Vector3d> points_;
    /// Points normals.
    std::vector<Eigen::Vector3d> normals_;
    /// RGB colors of points.
    std::vector<Eigen::Vector3d> colors_;
    /// Points intensity
    std::vector<double> intensity_;
};

} // namespace geometry
} // namespace open3d



