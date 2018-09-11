#pragma 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/feature.h>
//#include <pcl/features/rsd.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/vfh.h>

namespace featurefunction
{
    /************************************************************************/
    /* 计算法向量                                                                     */
    /************************************************************************/
    template<typename T>
    typename pcl::PointCloud<pcl::Normal>::Ptr estimationSurfaceNormals(typename pcl::PointCloud<T>::Ptr in_cloud , float radius = 0.0025)
    {
        pcl::NormalEstimationOMP<T , pcl::Normal> normal_estimation ;
        pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>) ;
        normal_estimation.setSearchMethod(tree) ;
        normal_estimation.setRadiusSearch(radius) ;
//        normal_estimation.setKSearch(10) ;
        normal_estimation.setInputCloud(in_cloud) ;
        normal_estimation.setNumberOfThreads(10) ;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>) ;
        normal_estimation.compute(*normals) ;
        return normals ;
    }
    /************************************************************************/
    /* 计算带法线的点云                                                                     */
    /************************************************************************/
    pcl::PointCloud<pcl::PointNormal>::Ptr estimationCloudWithNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud , float radius = 0.0025)
    {
        pcl::NormalEstimationOMP<pcl::PointXYZ , pcl::PointNormal> ne ;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>) ;
        ne.setSearchMethod(tree) ;
        ne.setInputCloud(in_cloud) ;
        ne.setRadiusSearch(radius) ;
        ne.setNumberOfThreads(10) ;
        pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>) ;
        ne.compute(*normals) ;
        pcl::copyPointCloud<pcl::PointXYZ , pcl::PointNormal>(*in_cloud , *normals) ;
        return normals ;
    }
    /************************************************************************/
    /* 快速点特征直方图                                                                     */
    /************************************************************************/
    template <typename T>
    typename pcl::PointCloud<pcl::FPFHSignature33>::Ptr estimationFPFH(typename pcl::PointCloud<T>::Ptr in_cloud , pcl::PointCloud<pcl::Normal>::Ptr normal , float radius = 0.005)
    {
        pcl::FPFHEstimationOMP<T , pcl::Normal , pcl::FPFHSignature33> fpfh ;
        pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>) ;
        fpfh.setSearchMethod(tree) ;
        fpfh.setInputNormals(normal) ;
        fpfh.setNumberOfThreads(4) ;
        fpfh.setInputCloud(in_cloud) ;
        fpfh.setRadiusSearch(radius) ;
//        fpfh.setKSearch(12) ;
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr result(new pcl::PointCloud<pcl::FPFHSignature33>) ;
        fpfh.compute(*result) ;
        return result ;
    }
    /************************************************************************/
    /* 旋转图像描述子算法                                                                     */
    /************************************************************************/
    template<typename T>
    typename pcl::PointCloud<pcl::Histogram<153>>::Ptr spinImageEstimation(typename pcl::PointCloud<T>::Ptr in_cloud , pcl::PointCloud<pcl::Normal>::Ptr normal , double radius )
    {
        pcl::PointCloud<pcl::Histogram<153>>::Ptr result(new pcl::PointCloud<pcl::Histogram<153>>) ;
        pcl::SpinImageEstimation<T , pcl::Normal , pcl::Histogram<153>> spinImage ;
        spinImage.setInputCloud(in_cloud) ;
        spinImage.setInputNormals(normal) ;
        pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>) ;
        spinImage.setSearchMethod(tree) ;
        spinImage.setRadiusSearch(radius) ;
        spinImage.compute(*result) ;
        return result ;
    }
    /************************************************************************/
    /* 实现RSD描述子的估计                                                                     */
    /************************************************************************/
//     template<typename T>
//     typename pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr rsdEstimation(typename pcl::PointCloud<T>::Ptr in_cloud , pcl::PointCloud<pcl::Normal>::Ptr normal , int ndivision , double radius )
//     {
//         pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr result(new pcl::PointCloud<pcl::PrincipalRadiiRSD>) ;
//         pcl::RSDEstimation<T , pcl::Normal , pcl::PrincipalRadiiRSD> rsd ;
//         rsd.setNrSubdivisions(ndivision) ;
//         rsd.setPlaneRadius(radius) ;
//         rsd.compute(*result) ;
//         return result ;
//     }
    /************************************************************************/
    /* 估计VFH特征值                                                                     */
    /************************************************************************/
    template<typename T>
    typename pcl::PointCloud<pcl::VFHSignature308>::Ptr estimationVFH(typename pcl::PointCloud<T>::Ptr in_cloud , pcl::PointCloud<pcl::Normal>::Ptr normal)
    {
        pcl::PointCloud<pcl::VFHSignature308>::Ptr result(new pcl::PointCloud<pcl::VFHSignature308>) ;
//         pcl::VFHEstimation<pcl::PointXYZ , pcl::Normal , pcl::VFHSignature308> vfh ;
//         vfh.setInputCloud(in_cloud) ;
//         vfh.setInputNormals(normal) ;
//         pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>) ;
//         vfh.setSearchMethod(tree) ;
//         vfh.compute(*result) ;
        
        return result ;
    }
}
