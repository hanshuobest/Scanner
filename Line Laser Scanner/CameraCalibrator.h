/************************************************************************/
/* ����궨��  
 * 2016.1.29
 * ��˶*/
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
/* ��Ŀ�������                                                                     */
/************************************************************************/
struct CameraParams
{
    CameraParams():flag(false)
    {

    }
    cv::Size imageSize ;//ͼ��ֱ���
    cv::Mat  cameraMatrix ;//����ڲ�����
    cv::Mat  distortionCoefficients ;//�������ϵ��
    std::vector<cv::Mat> rotations ;//���̸�ͼ�����ת��������Ҫ�������˹�任����ת����
    std::vector<cv::Mat> translations ;//���̸�ͼ���ƽ����
    std::vector<std::vector<cv::Point3f>> p3d ;//�ǵ����������
    std::vector<std::vector<cv::Point2f>> p2d ;//�ǵ����������

    bool flag ; //�ж��Ƿ��ѱ궨��Ϊtrue��ʾ�ѱ궨������Ϊ��
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

//�����������
struct StereoParams
{
    StereoParams():flag(false)
    {

    }
    bool flag ; //�Ƿ��ѻ�ȡ�����Ϣ
    cv::Size imageSize ;//ͼ��ֱ���
    CameraParams cameraParams1 ;//��������궨����
    CameraParams cameraParams2 ;//��������궨����
    cv::Mat rotation ;//��ת����
    cv::Mat translation ;
    cv::Mat essential ;//���ʾ���
    cv::Mat foundational ;//��������

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
    // �����λ����������ĵ�
    std::vector<std::vector<cv::Point3f>> objectPoints;
    //��������ĵ�
    std::vector<std::vector<cv::Point2f>> imagePoints;
    //������ת����ƽ����
    std::vector<cv::Mat> rvecs, tvecs;
    // ��������ڲ�����ͻ���ϵ����
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat cameraRotation ;//�������������
    cv::Mat cameraTranslation ;//����ƽ����
    // �궨�ķ�ʽ
    int flag;
    // ����ͼ��ȥ����
    cv::Mat map1,map2; 
    bool mustInitUndistort;

    CameraParams m_cameraParams ;//�����������
    cv::Size m_boardsize ;//���̸�ߴ�
    int m_squareWidth ;
public:

    CameraCalibrator() ;
    // ������ͼ����ȡ�ǵ�
    int addChessboardPoints(const std::vector<std::string>& filelist, cv::Size & boardSize);
    //�궨����
    double calibrate(cv::Size &imageSize);

    void setCalibrationFlag(bool radial8CoeffEnabled=false, bool tangentialParamEnabled=false);

    cv::Mat remap(const cv::Mat &image);

	int prejecttoPlane(const CameraParams &cameraParams , const int pos , std::vector<cv::Point2f> &v_p2d) ;

    //��ȡָ�����̸���ⷽλ����
    void CalExtrinx(int i) ;  

    //���������ڲ�������
    cv::Mat getCameraExtrinsic(){return cameraRotation ;}
    void saveRotation(std::string str) ;

    //��ȡ������ڲ�������
    cv::Mat getCameraMatrix() { return cameraMatrix; }

    //����������ڲ�������
    void saveCameraMatrix(std::string str) ;

    //���ָ�����̸��ƽ����
    cv::Mat getTranslations(){ return cameraTranslation ;} 

    //����ƽ����
    void saveTranslations(std::string str) ;

    //��ȡ����Ļ���ϵ��
    cv::Mat getDistCoeffs()   { return distCoeffs; }

    void printMatrix(const cv::Mat matrix) ;

    //��ȡ���̸��б�
    void readImagelist(const char* path , std::vector<std::string> &filelist) ;

    //��ȡ���̸�ǵ���������
    std::vector<std::vector<cv::Point3f>> getObjectPoints() const {return objectPoints ;}

    //��ȡ���̸�ǵ��������
    std::vector<std::vector<cv::Point2f>> getImagepoints() const {return imagePoints ;}

    //������������Ϣ
    CameraParams getCameraParams(){return m_cameraParams ;}

    //�������̸�ߴ�
    void setboardSize(cv::Size boardsize) ;

    //�������̸��С
    void setboardWidth(int squarewidth) ;

	/************************************************************************/
	/* ����˵��         ��ָ��ͼƬ�����̸�ͼ��
	 * @Param src       ԭͼ��
	 * ����ֵ           ����ɹ�����trure*/
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
