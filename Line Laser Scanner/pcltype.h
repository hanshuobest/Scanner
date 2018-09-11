#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io//ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree.h>

typedef pcl::PointXYZ PointXYZ ;
typedef pcl::PointNormal PointNormal ;
typedef pcl::PointCloud<PointXYZ> Cloud ;
typedef pcl::PointCloud<pcl::PointXYZRGB> CloudColor ;
typedef pcl::PointCloud<PointNormal> CloudNormal ;
typedef pcl::PointCloud<PointXYZ>::Ptr CloudPtr ;
typedef CloudColor::Ptr CloudColorPtr ;
typedef pcl::PointCloud<PointNormal>::Ptr CloudNormalPtr ;
typedef pcl::visualization::PCLVisualizer PCLVisualizer ;
typedef pcl::search::KdTree<PointXYZ>::Ptr KdTreePtr ;
