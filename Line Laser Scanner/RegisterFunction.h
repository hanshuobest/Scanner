
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/joint_icp.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/ndt.h>
#include <boost/thread/thread.hpp>
#include "FeatureFunction.h"
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>

struct AlignResult
{
	Eigen::Matrix4f finalmatrix ; //保存最终的变换矩阵
	float score ; //保存欧式适度评分
};
namespace registerFunction
{
    void
        sampleRandomTransform (Eigen::Affine3f &trans, float max_angle, float max_trans) ;
    template<typename T>
    Eigen::Matrix4f estimatin_SVD(pcl::PointCloud<pcl::PointXYZ>::Ptr src , pcl::PointCloud<pcl::PointXYZ>::Ptr tgt , pcl::Correspondences &correspondence) ;
    template<typename T>
    typename pcl::PointCloud<T>::Ptr ICP(typename pcl::PointCloud<T>::Ptr source_cloud , typename pcl::PointCloud<T>::Ptr target_cloud , Eigen::Matrix4f &final_transformation) ;
    AlignResult* NDT(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud , float step = 0.05 , float resolution = 0.01) ;
    Eigen::Matrix4f estimationSACInitAlignment(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr src , const pcl::PointCloud<pcl::PointXYZ>::ConstPtr tgt) ;
     Eigen::Matrix4f testSampleConsensusInitialAlignment(pcl::PointCloud<pcl::PointXYZ>::Ptr src , pcl::PointCloud<pcl::PointXYZ>::Ptr tgt , pcl::PointCloud<pcl::PointXYZ>::Ptr result) ;
    /************************************************************************/
    /*  利用SVD方法求解变换矩阵                                                                    */
    /************************************************************************/
    Eigen::Matrix4f estimatin_SVD(pcl::PointCloud<pcl::PointXYZ>::Ptr src , pcl::PointCloud<pcl::PointXYZ>::Ptr tgt , pcl::Correspondences &correspondence)
    {
        Eigen::Matrix4f final_transformation = Eigen::Matrix4f::Identity() ;
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ , pcl::PointXYZ> tran_svd ;
        tran_svd.estimateRigidTransformation(*src , *tgt , correspondence , final_transformation) ;
        return final_transformation ;
    }
    /************************************************************************/
    /* 剔除错误对应点 
    * id表示选择剔除错误点的方法
    * transform 保存随机采样一致获取的变换矩阵*/
    /************************************************************************/
    pcl::CorrespondencesPtr rejectBadCorrespondences( pcl::CorrespondencesPtr &all_correspondences , const pcl::PointCloud<pcl::PointXYZ>::Ptr src , pcl::PointCloud<pcl::PointXYZ>::Ptr tgt , int id , Eigen::Matrix4f &transform)
    {
        transform = Eigen::Matrix4f::Identity() ;
        pcl::CorrespondencesPtr remaining_correspondence(new pcl::Correspondences) ;
        if (id == 1) //基于对应关系之间的距离阈值实现了一个简单的错误对应关系去除算法
        {
            pcl::registration::CorrespondenceRejectorDistance rej ;
            rej.setInputSource<pcl::PointXYZ>(src) ;
            rej.setInputTarget<pcl::PointXYZ>(tgt) ;
            rej.setMaximumDistance(0.05) ;
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>) ;
            rej.setSearchMethodTarget(tree) ;
            rej.setInputCorrespondences(all_correspondences) ;
            rej.getCorrespondences(*remaining_correspondence) ;
            return remaining_correspondence ;
        }
        if (id == 2)//基于随机采样一致性实现错误对应关系去除
        {
            pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>  rac ;
            rac.setInputCloud(src) ;
            rac.setInputTarget(tgt) ;
            rac.setInlierThreshold(0.05) ;
            rac.setMaxIterations(25) ;
            rac.setInputCorrespondences(all_correspondences) ;
            rac.getCorrespondences(*remaining_correspondence) ;
            transform = rac.getBestTransformation() ;
            return remaining_correspondence ;
        }
        if (id == 3)//两个对应点之间的平均距离
        {
            pcl::registration::CorrespondenceRejectorMedianDistance rej ;
            rej.setMedianFactor(8.7921104) ;
            rej.setInputCorrespondences(all_correspondences) ;
            rej.getCorrespondences(*remaining_correspondence) ;
            return remaining_correspondence ;
        }
    }

    /************************************************************************/
    /* 随机采样变换                                                                     */
    /************************************************************************/
    void
        sampleRandomTransform (Eigen::Affine3f &trans, float max_angle, float max_trans)
    {
        srand(0);
        // Sample random transform
        Eigen::Vector3f axis((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX);
        axis.normalize();
        float angle = (float)rand() / RAND_MAX * max_angle;
        Eigen::Vector3f translation((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX);
        translation *= max_trans;
        Eigen::Affine3f rotation(Eigen::AngleAxis<float>(angle, axis));
        trans = Eigen::Translation3f(translation) * rotation;
    }
    /************************************************************************/
    /* 经典ICP                                                                     */
    /************************************************************************/
    template<typename T>
    AlignResult* ICP(typename pcl::PointCloud<T>::Ptr source_cloud , typename pcl::PointCloud<T>::Ptr target_cloud )
    {  
        Eigen::Matrix4f final_transformtion = Eigen::Matrix4f::Identity() ;
        pcl::IterativeClosestPoint<pcl::PointXYZ , pcl::PointXYZ> icp ;
        pcl::PointCloud<T>::Ptr output_cloud(new pcl::PointCloud<T>) ;

        icp.setInputSource(source_cloud) ;
        icp.setInputTarget(target_cloud) ;

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>) ;
        tree->setInputCloud(source_cloud) ;
        icp.setSearchMethodSource(tree) ;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>) ;
        tree1->setInputCloud(target_cloud) ;
        icp.setSearchMethodTarget(tree1) ;

        icp.setMaximumIterations(50) ;
        icp.setTransformationEpsilon(1e-8) ;
        icp.setMaxCorrespondenceDistance(0.05) ;
        final_transformtion = registerFunction::testSampleConsensusInitialAlignment(source_cloud , target_cloud , output_cloud) ;

        icp.align(*output_cloud , final_transformtion) ;

		AlignResult *ar = new AlignResult ;
		ar->finalmatrix = final_transformtion ;
		ar->score = icp.getFitnessScore() ;
//         final_transformtion = icp.getFinalTransformation()  ;
//         std::cout<<"欧式距离评分： "<<icp.getFitnessScore()<<"\n" ;
//        return final_transformtion ;
		return ar ;
    }
    /************************************************************************/
    /* 经典NDT                                                                     */
    /************************************************************************/
    AlignResult* NDT(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud , float step , float resolution)
    {  
        Eigen::Matrix4f final_transformation = Eigen::Matrix4f::Identity() ;
        pcl::NormalDistributionsTransform<pcl::PointXYZ , pcl::PointXYZ> ndt ;
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>) ;

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>) ;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>) ;
        tree1->setInputCloud(source_cloud) ;
        tree2->setInputCloud(target_cloud) ;

        ndt.setStepSize (step);//步长0.05效果较好 More-Thuente线搜索
        ndt.setResolution (resolution);//设置NDT网格结构的分辨率
        ndt.setInputSource (source_cloud);
        ndt.setInputTarget (target_cloud);

        ndt.setSearchMethodSource(tree1) ;
        ndt.setSearchMethodTarget(tree2) ;

        ndt.setMaximumIterations (30);
        ndt.setTransformationEpsilon (1e-8);

        final_transformation = registerFunction::testSampleConsensusInitialAlignment(source_cloud , target_cloud , output_cloud) ;
        ndt.align(*output_cloud , final_transformation) ;

		AlignResult *ar = new AlignResult ;
		ar->finalmatrix = ndt.getFinalTransformation() ;
		ar->score = ndt.getFitnessScore() ;
        return ar ;
    }
    /************************************************************************/
    /* SAC-初始配准                                                                     */
    /************************************************************************/
    Eigen::Matrix4f estimationSACInitAlignment(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr src , const pcl::PointCloud<pcl::PointXYZ>::ConstPtr tgt)
    {
        Eigen::Matrix4f final_transformation = Eigen::Matrix4f::Identity() ;
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud = src->makeShared() ;
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud = tgt->makeShared() ;

        pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>) ;
        source_normals = featurefunction::estimationSurfaceNormals<pcl::PointXYZ>(source_cloud , 0.005) ;
        pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>) ;
        target_normals = featurefunction::estimationSurfaceNormals<pcl::PointXYZ>(target_cloud , 0.005) ;

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_FPFH(new pcl::PointCloud<pcl::FPFHSignature33>) ;
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_PFFH(new pcl::PointCloud<pcl::FPFHSignature33>) ;
        source_FPFH = featurefunction::estimationFPFH<pcl::PointXYZ>(source_cloud , source_normals , 0.05) ;
        target_PFFH = featurefunction::estimationFPFH<pcl::PointXYZ>(target_cloud , target_normals , 0.05) ;

        pcl::SampleConsensusInitialAlignment<pcl::PointXYZ , pcl::PointXYZ , pcl::FPFHSignature33> sac ;
        sac.setMinSampleDistance(0.005) ;
        sac.setMaxCorrespondenceDistance(0.1) ;
        sac.setMaximumIterations(30) ;

        sac.setInputSource(source_cloud) ;
        sac.setInputTarget(target_cloud) ;
        sac.setSourceFeatures(source_FPFH->makeShared()) ;
        sac.setTargetFeatures(target_PFFH->makeShared()) ;

        pcl::PointCloud<pcl::PointXYZ> result ;
        sac.align(result) ;
        final_transformation = sac.getFinalTransformation() ;
        return final_transformation ;
    }
    Eigen::Matrix4f testSampleConsensusInitialAlignment(pcl::PointCloud<pcl::PointXYZ>::Ptr src , pcl::PointCloud<pcl::PointXYZ>::Ptr tgt , pcl::PointCloud<pcl::PointXYZ>::Ptr result)
    {
        Eigen::Matrix4f final_transformation = Eigen::Matrix4f::Identity() ;
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud = src->makeShared() ;
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud = tgt->makeShared() ;

        //    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>) ;
        pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>) ;
        source_normals = featurefunction::estimationSurfaceNormals<pcl::PointXYZ>(source_cloud , 0.005) ;
        pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>) ;
        target_normals = featurefunction::estimationSurfaceNormals<pcl::PointXYZ>(target_cloud , 0.005) ;

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_FPFH(new pcl::PointCloud<pcl::FPFHSignature33>) ;
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_PFFH(new pcl::PointCloud<pcl::FPFHSignature33>) ;
        source_FPFH = featurefunction::estimationFPFH<pcl::PointXYZ>(src->makeShared() , source_normals , 0.05) ;
        target_PFFH = featurefunction::estimationFPFH<pcl::PointXYZ>(tgt->makeShared() , target_normals , 0.05) ;

        pcl::SampleConsensusInitialAlignment<pcl::PointXYZ , pcl::PointXYZ , pcl::FPFHSignature33> sac ;
        sac.setMinSampleDistance(0.005) ;
        sac.setMaxCorrespondenceDistance(0.1) ;
        sac.setMaximumIterations(30) ;

        sac.setInputSource(source_cloud) ;
        sac.setInputTarget(target_cloud) ;
        sac.setSourceFeatures(source_FPFH) ;
        sac.setTargetFeatures(target_PFFH) ;
        sac.align(*result) ;
        final_transformation = sac.getFinalTransformation() ;
        return final_transformation ;
    }
    /************************************************************************/
    /* id = 1 确定目标和查询点集（或特征）之间的对应关系 
    * id = 2 利用法线求对应关系*/
    /************************************************************************/
    pcl::Correspondences findCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr src , pcl::PointCloud<pcl::PointXYZ>::Ptr tgt , int id)
    {
        if (id == 1)
        {
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>) ;
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>) ;
            tree1->setInputCloud(src) ;
            tree2->setInputCloud(tgt) ;

            pcl::registration::CorrespondenceEstimation<pcl::PointXYZ , pcl::PointXYZ> ce ;
            ce.setInputSource(src) ;
            ce.setInputTarget(tgt) ;
            pcl::Correspondences correspondences ;
            ce.setSearchMethodSource(tree1) ;
            ce.setSearchMethodTarget(tree2) ;
            ce.determineReciprocalCorrespondences(correspondences) ;

            return correspondences ;
        }
        if (id == 2)
        {
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>) ;
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>) ;
            tree1->setInputCloud(src) ;
            tree2->setInputCloud(tgt) ;

            pcl::PointCloud<pcl::Normal>::Ptr src_normal(new pcl::PointCloud<pcl::Normal>) ;
            src_normal = featurefunction::estimationSurfaceNormals<pcl::PointXYZ>(src , 0.0025) ;
            pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointXYZ , pcl::PointXYZ , pcl::Normal> ces ;
            ces.setInputSource(src) ;
            ces.setSourceNormals(src_normal) ;
            ces.setInputTarget(tgt) ;
            pcl::Correspondences correspondences ;
            ces.determineCorrespondences(correspondences) ;
//            std::cout<<"对应点数为："<<correspondences.size()<<"\n" ;
            return correspondences ;
        }
    }
    /************************************************************************/
    /* 利用FPFH求对应关系                                                                     */
    /************************************************************************/
    pcl::Correspondences findCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_fpfh , pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgt_fpfh)
    {
        pcl::search::KdTree<pcl::FPFHSignature33>::Ptr tree1(new pcl::search::KdTree<pcl::FPFHSignature33>) ;
        pcl::search::KdTree<pcl::FPFHSignature33>::Ptr tree2(new pcl::search::KdTree<pcl::FPFHSignature33>) ;
        tree1->setInputCloud(src_fpfh) ;
        tree2->setInputCloud(tgt_fpfh) ;

        pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33 , pcl::FPFHSignature33> ces ;
        ces.setInputSource(src_fpfh) ;
        ces.setInputTarget(tgt_fpfh) ;
        ces.setSearchMethodSource(tree1) ;
        ces.setSearchMethodTarget(tree2) ;

        pcl::Correspondences correspondences ;
        ces.determineCorrespondences(correspondences) ;
        std::cout<<"对应点数为："<<correspondences.size()<<"\n" ;
        return correspondences ;
    }
}

