#pragma once
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/filter.h>
namespace filterfunction
{
    /************************************************************************/
    /* 下采样                                                                     */
    /************************************************************************/
    template<typename T>
    typename pcl::PointCloud<T>::Ptr downSample(typename pcl::PointCloud<T>::Ptr cloud , float leafSize)
    {
        pcl::VoxelGrid<T> sor ;
        sor.setInputCloud(cloud) ;
        sor.setLeafSize(leafSize , leafSize , leafSize) ;
        pcl::PointCloud<T>::Ptr filterCloud(new pcl::PointCloud<T>) ;
        sor.filter(*filterCloud) ;
        return filterCloud ;
    }

    /************************************************************************/
    /* ApproximateVoxelGrid 下采样                                                                     */
    /************************************************************************/
    template<typename T>
    typename pcl::PointCloud<T>::Ptr ApproximateVoxeldownSample(typename pcl::PointCloud<T>::Ptr cloud , float leafSize)
    {
        pcl::ApproximateVoxelGrid<T> sor ;
        sor.setInputCloud(cloud) ;
        sor.setLeafSize(leafSize , leafSize , leafSize) ;
        sor.setDownsampleAllData(true) ;
        pcl::PointCloud<T>::Ptr filterCloud(new pcl::PointCloud<T>) ;
        sor.filter(*filterCloud) ;
        return filterCloud ;
    }
/************************************************************************/
/* 直通滤波                                                                     */
/************************************************************************/
    template<typename T>
    typename pcl::PointCloud<T>::Ptr passThrough(typename pcl::PointCloud<T>::Ptr cloud , char* fieldName , float lowerLimit , float upperLimit )
    {
        pcl::PointCloud<T>::Ptr filterCloud(new pcl::PointCloud<T>) ;
        pcl::PassThrough<T> pass ;
        pass.setInputCloud(cloud) ;
        pass.setFilterFieldName(fieldName) ;
        pass.setFilterLimits(lowerLimit , upperLimit) ;
        pass.filter(*filterCloud) ;
        return filterCloud ;
     }
/************************************************************************/
/* 统计分析滤波器                                                                     */
/************************************************************************/
    template<typename T>
    typename pcl::PointCloud<T>::Ptr statisticalOutlierRemoval(typename pcl::PointCloud<T>::Ptr cloud , int meanK , double stddevMulThresh)
    {
        pcl::PointCloud<T>::Ptr filterCloud(new pcl::PointCloud<T>) ;
        pcl::StatisticalOutlierRemoval<T> sor ;
        sor.setInputCloud(cloud) ;
        sor.setMeanK(meanK) ;//考虑查询点邻近点数
        sor.setStddevMulThresh(stddevMulThresh) ;//设置离群点的阈值
        sor.filter(*filterCloud) ;
        return filterCloud;
    }
    /************************************************************************/
    /* RadiusOutlierRemoval 滤波器                                                                     */
    /************************************************************************/
    template<typename T>
    typename pcl::PointCloud<T>::Ptr radiusRemoval(typename pcl::PointCloud<T>::Ptr cloud , double radius , int neighbors)
    {
        pcl::PointCloud<T>::Ptr filterCloud(new pcl::PointCloud<T>) ;
        pcl::RadiusOutlierRemoval<T> outrem ;
        outrem.setInputCloud(cloud) ;
        outrem.setRadiusSearch(radius) ;//设置半径搜索邻近点
        outrem.setMinNeighborsInRadius(neighbors) ;//小于邻近点数的点删除
        outrem.filter(*filterCloud) ;
        return filterCloud ;
    }
    /************************************************************************/
    /* 对点云的均匀概率随机抽样                                                                     */
    /************************************************************************/
    template<typename T>
    typename pcl::PointCloud<T>::Ptr randomSample(typename pcl::PointCloud<T>::Ptr cloud, int sam_num  )
    {
        pcl::PointCloud<T>::Ptr filterCloud(new pcl::PointCloud<T>) ;
        pcl::RandomSample<T> randsam ;
		randsam.setInputCloud(cloud) ;
        randsam.setSample(sam_num) ;
        randsam.setSeed(100) ;
        randsam.filter(*filterCloud) ;
        return filterCloud ;
    }
}
