#ifndef IMAGE_OPERATION_H
#define IMAGE_OPERATION_H
#include <vector>
#include <opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "Matrix.h"
typedef unsigned char uchar;
using std::vector;

//��Ƶ��Ϣ����
struct VideoParams
{
	VideoParams():flag(false)
	{

	}
	std::string videoname ;//��Ƶ����
	int nframes ;          //��Ƶ��֡��
	int width   ;          //��Ƶ���
	int height  ;          //��Ƶ�߶�

	int roi_start_row ;   //��Ƶ����Ȥ����ʼ��
	int roi_start_col ;   //��Ƶ����Ȥ����ʼ��
	int roi_end_row ;     //��Ƶ����Ȥ��������
	int roi_end_col ;     //��Ƶ����Ȥ��������
	bool flag ;           //�Ƿ���Ƶ�Ѽ���
};

class ImageOperation
{
public:

	ImageOperation() ;
	vector<int> pointerToVector(uchar *, size_t);//��ָ���ֵ��������
	vector<int> houghTransform(vector<int>, size_t, size_t);//hough�任����ȡ��ȷ�Ľǵ�
	vector<double> iCorToPCor(vector<int>);//���ǵ��ͼ������ϵת������ƽ������ϵ
	vector<double> iCorToPCor(vector<double>);//���ǵ��ͼ������ϵת������ƽ������ϵ
	vector<double> iCorToPCor(double, double);//���ǵ��ͼ������ϵת������ƽ������ϵ
	vector<double> constructN(vector<double>, vector<double>);//����ϵ������N
	vector<double>constructL(vector<double>);//����������l
	vector<double>getPoint(vector<double>);//����ֱ�ߵĽ���
	vector<double> getPoint(const vector<double> &v_vec , const vector<double> &h_vec) ;
	/************************************************************************/
	/* ����˵��        ����ָ�������ص����ֵ����   
	* ���� @Input     src ����ͼ��
	* ���� @Input     refrow �ο���
	* ����ֵ          �ɹ�����1 ��ʧ��-1*/
	/************************************************************************/
	int findMaxPoint(cv::Mat &src , const int refrow , cv::Point2f &p2d) ;
	/************************************************************************/
	/* ����˵��        ����ƽ���ཻ���������ά����                                                             * ���� @Input     plane_coeff Ϊƽ�淽�̵�ϵ������a,b,c,d
	* ���� @Input     p3d ��������������ϵ�µ����꣬ע��zΪ����ĸ���
	* ���� @Output    objectPoint ��õĿռ�������������ϵ�µ�����
	* ����ֵ          ���ɹ�����1*/
	/************************************************************************/
	int CalObjectPoints(const std::vector<double> &plane_coeff , const std::vector<double> &p3d , std::vector<double> &objectPoint) ;

	/************************************************************************/
	/* ����˵��        �ռ������ڲ�������������ұ߽��λ��
	* ���� @Input     inputsrc ����ָ��֡ͼ��
	* ���� @Input     input_refrow �趨�ο���
	* ���� @Input     rboundary_thresh �趨�������Ʊ߽���ֵ
	* ���� @Output    output_pos ���������ұ߽���ο��еĽ���
	* ����ֵ          �ɹ�����1 �����򷵻�-1*/
	/************************************************************************/
	int  interpolationPos(cv::Mat &inputsrc, const int input_refrow , const int rboundary_thresh , cv::Point2f &output_pos) ;

	/************************************************************************/
	/* ����˵��        �ռ������ڲ�������������ұ߽��λ��
	* ���� @Input     inputsrc ����ָ��֡ͼ��
	* ���� @Input     input_refrow �趨�ο���
	* ���� @Input     rboundary_thresh �趨�������Ʊ߽���ֵ
	* ���� @Output    output_pos ���������ұ߽���ο��еĽ���*/
	/************************************************************************/
	int interpolationPos(cv::Mat &inputsrc, const int input_refrow , const int rboundary_thresh , cv::vector<double> &output_pos) ;
	
	/************************************************************************/
	/* ����˵��         ��������ڲ���
	* ���� @Input      u��v�����������������
	* ���� @Input      vx,vy��Ԫx��y����ĳߴ�*/
	/************************************************************************/
	void setInParams(double u , double v , double vx , double vy) ;
	
	/************************************************************************/
	/* ����˵����                ��ȡ��Ƶ��Ϣ
	* ���� @Input      strvideo  ��Ƶ�ļ�·��
	* ���� @Input      start_row ��Ƶ�и���Ȥ������ʼ��
	* ���� @Input      start_col ��Ƶ�и���Ȥ������ʼ��
	* ���� @Input      end_row   ��Ƶ�и���Ȥ���������
	* ���� @Input      end_col   ��Ƶ�и���Ȥ���������
	* ���� @Output     params    ��Ƶ��Ϣ
	* ���� @Output     red_ivec  �����ɫͨ����Ϣ
	* ���� @Input      flag      �Ƿ��ȥ������0����
	* ����             ʧ�ܷ���-1*/
	/************************************************************************/
// 	int getVideoInfo(const std::string strvideo , int start_row , int start_col , int end_row , int end_col , VideoParams &params , std::vector<cv::Mat> &red_ivec , int flag = 0) 
// 	{
// 		cv::VideoCapture caputre(strvideo) ;
// 		if (!caputre.isOpened())
// 		{
// 			std::cout<<"��Ƶ��ʧ�ܣ�\n" ;
// 			return -1 ;
// 		}
// 		params.flag = true ;
// 		params.videoname = strvideo ;  
// 		params.nframes = caputre.get(CV_CAP_PROP_FRAME_COUNT) ;//��ȡ��Ƶ֡ ;
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
	/* ����˵��         �Ҷȼ�ֵ����ȡ��������ο��еĽ���
	* ���� @Input      refrows �ο��еļ��ϣ�Ĭ��Ϊ4���ο��У�H��Vƽ���ϸ�����
	* ���� @Input      img ָ��֡ͼ��
	* ���� @Output     v_points ����ο����뼤�����Ľ���
	* ����ֵ           �ɹ�����1 ��ʧ�ܷ���-1*/
	/************************************************************************/
	int getReferRowPoints(const std::vector<int> &refrows , cv::Mat &img , std::vector<double> &v_points) ;

	/************************************************************************/
	/* ����˵��         ������������Ĳ�� c = a x b
	* ���� @Input      Input_a 
	* ���� @Input      Input_b
	* ���� @Output     Output_c������
	* ����ֵ           �ɹ�����1 ��ʧ�ܷ���-1*/
	/************************************************************************/
	int corssProduct(const std::vector<double> &Input_a , const std::vector<double> &Input_b , std::vector<double> &Output_c) ;

	/************************************************************************/
	/* ����˵��               ����ռ�ʱ���ڲ��õĵ�λ��                                              
	* ���� @Input             src_k��k֡ͼ��
	* ���� @Input             src_k1��k+1֡ͼ��
	* ���� @Input             k ��k֡
	* ���� @Input             v_refrows �ο��м���
	* ���� @Input             thresh��ֵ
	* ���� @Input             t�������ƹ���Ԫʱ��*/
	/************************************************************************/
	int CalculatePoints(cv::Mat &src_k , cv::Mat &src_k1 , const int &k , std::vector<int> &v_refrows , double &thresh , double &t , std::vector<double> &vpoints) ;

	void saveCloudPly(std::string strname , std::vector<double> &x_vec , std::vector<double> &y_vec , std::vector<double> &z_vec , std::vector<int> &r_vec , std::vector<int> &g_vec , std::vector<int> &b_vec , bool flag = true) ;

	void saveCloudPcd(std::string strname , std::vector<double> &x_vec , std::vector<double> &y_vec , std::vector<double> &z_vec , std::vector<int> &r_vec , std::vector<int> &g_vec , std::vector<int> &b_vec , bool flag = true) ;
	int readMatrix(const std::string strpath , std::vector<double> &R_matrix) ;
private:
	double m_u , m_v ;//�궨������������
	double m_vx , m_vy ;//x,y�������Ԫ�ߴ�
	matrix m_matrix ;
};

#endif //IMAGE_OPERATION_H
