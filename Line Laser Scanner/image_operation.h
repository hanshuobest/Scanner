#ifndef IMAGE_OPERATION_H
#define IMAGE_OPERATION_H
#include <vector>
#include <opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "Matrix.h"
typedef unsigned char uchar;
using std::vector;

//视频信息参数
struct VideoParams
{
	VideoParams():flag(false)
	{

	}
	std::string videoname ;//视频名称
	int nframes ;          //视频总帧数
	int width   ;          //视频宽度
	int height  ;          //视频高度

	int roi_start_row ;   //视频中兴趣区起始行
	int roi_start_col ;   //视频中兴趣区起始列
	int roi_end_row ;     //视频中兴趣区结束行
	int roi_end_col ;     //视频中兴趣区结束列
	bool flag ;           //是否视频已加载
};

class ImageOperation
{
public:

	ImageOperation() ;
	vector<int> pointerToVector(uchar *, size_t);//将指针的值赋给容器
	vector<int> houghTransform(vector<int>, size_t, size_t);//hough变换，获取精确的角点
	vector<double> iCorToPCor(vector<int>);//将角点的图像坐标系转换到像平面坐标系
	vector<double> iCorToPCor(vector<double>);//将角点的图像坐标系转换到像平面坐标系
	vector<double> iCorToPCor(double, double);//将角点的图像坐标系转换到像平面坐标系
	vector<double> constructN(vector<double>, vector<double>);//构造系数矩阵N
	vector<double>constructL(vector<double>);//构造自由项l
	vector<double>getPoint(vector<double>);//求两直线的交点
	vector<double> getPoint(const vector<double> &v_vec , const vector<double> &h_vec) ;
	/************************************************************************/
	/* 函数说明        计算指定行像素的最大值坐标   
	* 参数 @Input     src 输入图像
	* 参数 @Input     refrow 参考行
	* 返回值          成功返回1 ，失败-1*/
	/************************************************************************/
	int findMaxPoint(cv::Mat &src , const int refrow , cv::Point2f &p2d) ;
	/************************************************************************/
	/* 函数说明        射线平面相交，求解点的三维坐标                                                             * 参数 @Input     plane_coeff 为平面方程的系数，如a,b,c,d
	* 参数 @Input     p3d 像点在摄像机坐标系下的坐标，注：z为焦距的负数
	* 参数 @Output    objectPoint 求得的空间点在摄像机坐标系下的坐标
	* 返回值          求解成功返回1*/
	/************************************************************************/
	int CalObjectPoints(const std::vector<double> &plane_coeff , const std::vector<double> &p3d , std::vector<double> &objectPoint) ;

	/************************************************************************/
	/* 函数说明        空间线性内插求出激光条纹右边界的位置
	* 参数 @Input     inputsrc 输入指定帧图像
	* 参数 @Input     input_refrow 设定参考行
	* 参数 @Input     rboundary_thresh 设定激光条纹边界阈值
	* 参数 @Output    output_pos 激光条纹右边界与参考行的交点
	* 返回值          成功返回1 ，否则返回-1*/
	/************************************************************************/
	int  interpolationPos(cv::Mat &inputsrc, const int input_refrow , const int rboundary_thresh , cv::Point2f &output_pos) ;

	/************************************************************************/
	/* 函数说明        空间线性内插求出激光条纹右边界的位置
	* 参数 @Input     inputsrc 输入指定帧图像
	* 参数 @Input     input_refrow 设定参考行
	* 参数 @Input     rboundary_thresh 设定激光条纹边界阈值
	* 参数 @Output    output_pos 激光条纹右边界与参考行的交点*/
	/************************************************************************/
	int interpolationPos(cv::Mat &inputsrc, const int input_refrow , const int rboundary_thresh , cv::vector<double> &output_pos) ;
	
	/************************************************************************/
	/* 函数说明         设置相机内参数
	* 参数 @Input      u，v摄像机的像主点坐标
	* 参数 @Input      vx,vy像元x，y方向的尺寸*/
	/************************************************************************/
	void setInParams(double u , double v , double vx , double vy) ;
	
	/************************************************************************/
	/* 函数说明：                获取视频信息
	* 参数 @Input      strvideo  视频文件路径
	* 参数 @Input      start_row 视频中感兴趣区域起始行
	* 参数 @Input      start_col 视频中感兴趣区域起始列
	* 参数 @Input      end_row   视频中感兴趣区域结束行
	* 参数 @Input      end_col   视频中感兴趣区域结束列
	* 参数 @Output     params    视频信息
	* 参数 @Output     red_ivec  保存红色通道信息
	* 参数 @Input      flag      是否减去背景，0不减
	* 返回             失败返回-1*/
	/************************************************************************/
// 	int getVideoInfo(const std::string strvideo , int start_row , int start_col , int end_row , int end_col , VideoParams &params , std::vector<cv::Mat> &red_ivec , int flag = 0) 
// 	{
// 		cv::VideoCapture caputre(strvideo) ;
// 		if (!caputre.isOpened())
// 		{
// 			std::cout<<"视频打开失败！\n" ;
// 			return -1 ;
// 		}
// 		params.flag = true ;
// 		params.videoname = strvideo ;  
// 		params.nframes = caputre.get(CV_CAP_PROP_FRAME_COUNT) ;//获取视频帧 ;
// 
// 		cv::Mat frame ;
// 		caputre.read(frame) ;
// 
// 		int w = frame.cols ;
// 		int h = frame.rows ;
// 
// 		params.width = w ;
// 		params.height = h ;
// 		params.roi_start_row = start_row ;
// 		params.roi_start_col = start_col ;
// 		params.roi_end_row = end_row ;
// 		params.roi_end_col = end_col ;
// 
// 		std::vector<cv::Mat> channels ;
// 		cv::split(frame , channels) ;
// 		red_ivec.push_back(channels.at(2)) ;
// 		std::vector<cv::Mat>().swap(channels) ;
// 
// 		cv::Mat filterframe ;
// 		cv::Mat backname ;
// 		backname = cv::imread(robotname , 1) ;
// 		while (1)
// 		{
// 			if (!caputre.read(frame))
// 			{
// 				break; 
// 			}              
// 			if (flag)
// 			{
// 				frame = frame - backname ;
// 			}
// 			cv::GaussianBlur(frame , filterframe , cv::Size(7 , 7) , 1.5 , 1.5) ;
// 			cv::split(frame , channels) ;
// 			red_ivec.push_back(channels.at(2)) ;
// 			std::vector<cv::Mat >().swap(channels) ;  
// 			//        std::cout<<"." ;
// 		}
// 		return 1 ;
// 	}

	/************************************************************************/
	/* 函数说明         灰度极值法获取激光条与参考行的交点
	* 参数 @Input      refrows 参考行的集合，默认为4个参考行，H、V平面上各两个
	* 参数 @Input      img 指定帧图像
	* 参数 @Output     v_points 输出参考行与激光条的交点
	* 返回值           成功返回1 ，失败返回-1*/
	/************************************************************************/
	int getReferRowPoints(const std::vector<int> &refrows , cv::Mat &img , std::vector<double> &v_points) ;

	/************************************************************************/
	/* 函数说明         求解两个向量的叉积 c = a x b
	* 参数 @Input      Input_a 
	* 参数 @Input      Input_b
	* 参数 @Output     Output_c叉积结果
	* 返回值           成功返回1 ，失败返回-1*/
	/************************************************************************/
	int corssProduct(const std::vector<double> &Input_a , const std::vector<double> &Input_b , std::vector<double> &Output_c) ;

	/************************************************************************/
	/* 函数说明               计算空间时间内插获得的点位置                                              
	* 参数 @Input             src_k第k帧图像
	* 参数 @Input             src_k1第k+1帧图像
	* 参数 @Input             k 第k帧
	* 参数 @Input             v_refrows 参考行集合
	* 参数 @Input             thresh阈值
	* 参数 @Input             t激光条纹过像元时间*/
	/************************************************************************/
	int CalculatePoints(cv::Mat &src_k , cv::Mat &src_k1 , const int &k , std::vector<int> &v_refrows , double &thresh , double &t , std::vector<double> &vpoints) ;

	void saveCloudPly(std::string strname , std::vector<double> &x_vec , std::vector<double> &y_vec , std::vector<double> &z_vec , std::vector<int> &r_vec , std::vector<int> &g_vec , std::vector<int> &b_vec , bool flag = true) ;

	void saveCloudPcd(std::string strname , std::vector<double> &x_vec , std::vector<double> &y_vec , std::vector<double> &z_vec , std::vector<int> &r_vec , std::vector<int> &g_vec , std::vector<int> &b_vec , bool flag = true) ;
	int readMatrix(const std::string strpath , std::vector<double> &R_matrix) ;
private:
	double m_u , m_v ;//标定的像主点坐标
	double m_vx , m_vy ;//x,y方向的像元尺寸
	matrix m_matrix ;
};

#endif //IMAGE_OPERATION_H
