/************************************************************************/
/* 相机标定类  
 * 2016.1.29
 * 韩硕*/
/************************************************************************/
#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H
#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

/************************************************************************/
/* 单目相机参数                                                                     */
/************************************************************************/
struct CameraParams
{
    CameraParams():flag(false)
    {

    }
    cv::Size imageSize ;//图像分辨率
    cv::Mat  cameraMatrix ;//相机内参数阵
    cv::Mat  distortionCoefficients ;//相机畸变系数
    std::vector<cv::Mat> rotations ;//棋盘格图像的旋转向量，需要做罗里格斯变换求旋转矩阵
    std::vector<cv::Mat> translations ;//棋盘格图像的平移量
    std::vector<std::vector<cv::Point3f>> p3d ;//角点的世界坐标
    std::vector<std::vector<cv::Point2f>> p2d ;//角点的像素坐标

    bool flag ; //判断是否已标定，为true表示已标定，否则为假
    CameraParams &operator =(const CameraParams &params)
    {
        if (this == &params)
        {
            return *this ;
        }
        flag = params.flag ;
        imageSize = params.imageSize ;
        cameraMatrix = params.cameraMatrix ;
        distortionCoefficients = params.distortionCoefficients ;
        rotations = params.rotations ;
        translations = params.translations ;
        p3d = params.p3d ;
        p2d = params.p2d ;
        return *this ;
    }
};

//立体相机参数
struct StereoParams
{
    StereoParams():flag(false)
    {

    }
    bool flag ; //是否已获取相机信息
    cv::Size imageSize ;//图像分辨率
    CameraParams cameraParams1 ;//左摄像机标定参数
    CameraParams cameraParams2 ;//右摄像机标定参数
    cv::Mat rotation ;//旋转矩阵
    cv::Mat translation ;
    cv::Mat essential ;//本质矩阵
    cv::Mat foundational ;//基础矩阵

    StereoParams &operator =(const StereoParams &params)
    {
        if (this == &params)
        {
            return *this ;
        }
        imageSize = params.imageSize ;
        cameraParams1 = params.cameraParams1 ;
        cameraParams2 = params.cameraParams2 ;
        rotation = params.rotation ;
        translation = params.translation ;
        essential = params.essential ;
        foundational = params.foundational ;
    }
};
class CameraCalibrator 
{
private:
    // 输入点位于世界坐标的点
    std::vector<std::vector<cv::Point3f>> objectPoints;
    //像素坐标的点
    std::vector<std::vector<cv::Point2f>> imagePoints;
    //保存旋转量和平移量
    std::vector<cv::Mat> rvecs, tvecs;
    // 输出矩阵，内参数阵和畸变系数真
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat cameraRotation ;//保存外参数矩阵
    cv::Mat cameraTranslation ;//保存平移量
    // 标定的方式
    int flag;
    // 用于图像去畸变
    cv::Mat map1,map2; 
    bool mustInitUndistort;

    CameraParams m_cameraParams ;//保存相机参数
    cv::Size m_boardsize ;//棋盘格尺寸
    int m_squareWidth ;
public:

    CameraCalibrator() ;
    // 打开棋盘图像并提取角点
    int addChessboardPoints(const std::vector<std::string>& filelist, cv::Size & boardSize);
    //标定函数
    double calibrate(cv::Size &imageSize);

    void setCalibrationFlag(bool radial8CoeffEnabled=false, bool tangentialParamEnabled=false);

    cv::Mat remap(const cv::Mat &image);

	int prejecttoPlane(const CameraParams &cameraParams , const int pos , std::vector<cv::Point2f> &v_p2d) ;

    //获取指定棋盘格的外方位矩阵
    void CalExtrinx(int i) ;  

    //获得相机的内参数矩阵
    cv::Mat getCameraExtrinsic(){return cameraRotation ;}
    void saveRotation(std::string str) ;

    //获取相机的内参数矩阵
    cv::Mat getCameraMatrix() { return cameraMatrix; }

    //保存相机的内参数矩阵
    void saveCameraMatrix(std::string str) ;

    //获得指定棋盘格的平移量
    cv::Mat getTranslations(){ return cameraTranslation ;} 

    //保存平移量
    void saveTranslations(std::string str) ;

    //获取相机的畸变系数
    cv::Mat getDistCoeffs()   { return distCoeffs; }

    void printMatrix(const cv::Mat matrix) ;

    //读取棋盘格列表
    void readImagelist(const char* path , std::vector<std::string> &filelist) ;

    //获取棋盘格角点物理坐标
    std::vector<std::vector<cv::Point3f>> getObjectPoints() const {return objectPoints ;}

    //获取棋盘格角点像点坐标
    std::vector<std::vector<cv::Point2f>> getImagepoints() const {return imagePoints ;}

    //获得相机参数信息
    CameraParams getCameraParams(){return m_cameraParams ;}

    //设置棋盘格尺寸
    void setboardSize(cv::Size boardsize) ;

    //设置棋盘格大小
    void setboardWidth(int squarewidth) ;

	/************************************************************************/
	/* 函数说明         画指定图片的棋盘格图像
	 * @Param src       原图像
	 * 返回值           画棋成功返回trure*/
	/************************************************************************/
	bool drawchessboardCorners(cv::Mat &src , cv::Mat &dst) 
	{
		if (src.empty())
		{
			return false ;
		}
		dst = src.clone() ;
		cv::Mat temp ;
		cv::cvtColor(src , temp , CV_RGB2GRAY) ;
		std::vector<cv::Point2f> myimagepoints ;
		bool found = false ;
		found = cv::findChessboardCorners(temp , m_boardsize , myimagepoints , 1) ;
		if (found)
		{
			cv::cornerSubPix(temp , myimagepoints , cv::Size(5 , 5) , cv::Size(-1 , -1) , cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS , 40 , 0.001)) ;
			if (myimagepoints.size() == m_boardsize.area())
			{
				cv::drawChessboardCorners(dst , m_boardsize , myimagepoints , found) ;
				return true ;
			}		
		}
		return false ;
	}

protected:
    void addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners);
};

#endif // CAMERACALIBRATOR_H
