#include "CameraCalibrator.h"
#include <fstream>
#include <algorithm>


CameraCalibrator::CameraCalibrator()
    : flag(0), mustInitUndistort(true),m_squareWidth(25)
{
 
}
/************************************************************************/
/* ����˵��            ������ͼ������ȡ�ǵ�
 * @Input              filelistͼ���б�
 * @Input              boardSize���̳ߴ�
 * ����ֵ              ���سɹ�������̸�ͼ�������*/
/************************************************************************/
int CameraCalibrator::addChessboardPoints(
    const std::vector<std::string>& filelist, 
    cv::Size & boardSize) 
{
    // �����ϵ����������
    std::vector<cv::Point2f> imageCorners;
    std::vector<cv::Point3f> objectCorners;

    // 3D �����еõ�
    // ����������ϵ�г�ʼ�����̽ǵ�
    // ��Щ��λ��(X,Y,Z)=(i,j,0)
    for (int i=0; i<boardSize.height; i++) 
    {
        for (int j=0; j<boardSize.width; j++) 
        {
            objectCorners.push_back(cv::Point3f(j * m_squareWidth , i * m_squareWidth , 0.0f));
        }
    }

    cv::Mat image; // Ϊ�˱�������ͼ��
    int successes = 0;
    // ������ͼ
    for (int i = 0; i< filelist.size(); i++)
    {
        // ��ͼ��
        image = cv::imread(filelist[i] , 0);
        // �õ��ǵ�
        bool found = cv::findChessboardCorners(image, boardSize, imageCorners , 1);

        // ��ȡ�����ؼ�����
        if (found)
        {
            cv::cornerSubPix(image, imageCorners, 
                cv::Size(5,5), 
                cv::Size(-1,-1), 
                cv::TermCriteria( 
                CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 
                40, //maxCount=40
                0.001 ));     // ��С����

            // ����ǵ���������Ҫ����ô�����������ݣ�ȫ���ǵ������
            if (imageCorners.size() == boardSize.area()) 
            {
                // ���һ���ӽ��е�ͼ��㼰������
                addPoints(imageCorners, objectCorners);
                successes++;
            }
        }
    }
    m_cameraParams.p3d = objectPoints ;
    m_cameraParams.p2d = imagePoints ;
    return successes;
}


/************************************************************************/
/* ����˵����       ��ӳ��������Ӧ��ͼ���   
 * @Input           imageCornersͼ���
 * @Input           objectCorners������*/
/************************************************************************/
void CameraCalibrator::addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners) 
{
    // 2D ͼ���
    imagePoints.push_back(imageCorners);          
    // ��Ӧ3D�����еõ�
    objectPoints.push_back(objectCorners);
}

/************************************************************************/
/* ����˵����     ���б궨��������ͶӰ���    
 * @Input         imageSizeͼƬ�ߴ�
 * ����ֵ         �ɹ�����1*/
/************************************************************************/
double CameraCalibrator::calibrate(cv::Size &imageSize)
{
    //�������ȥ����
    mustInitUndistort= true;

    //�����ת��ƽ��
    /*    std::vector<cv::Mat> rvecs, tvecs;*/

    calibrateCamera(objectPoints, // 3D��
        imagePoints,  // ͼ���
        imageSize,   // ͼ��ߴ�
        cameraMatrix, // ������������
        distCoeffs,   // ����Ļ������
        rvecs, tvecs, // ��ת��ƽ��
        flag);        // ����ѡ��

    m_cameraParams.flag = true ;//��ע�ѱ궨
    m_cameraParams.cameraMatrix = cameraMatrix ;
    m_cameraParams.distortionCoefficients = distCoeffs ;
    m_cameraParams.rotations = rvecs ;
    m_cameraParams.translations = tvecs ;
    m_cameraParams.imageSize = imageSize ;
    //					,CV_CALIB_USE_INTRINSIC_GUESS);
    return 1;
}


/************************************************************************/
/* ����˵����            ���ñ궨���ȥ��ͼ��Ļ��� 
 * ������@Input          image�����������ͼ��
 * ���أ�@Return         ȥ������ͼ��
 */
/************************************************************************/
cv::Mat CameraCalibrator::remap(const cv::Mat &image) 
{
    cv::Mat undistorted;

    if (mustInitUndistort)
    { //ÿ�α궨ֻ���ʼ��һ��   
        cv::initUndistortRectifyMap(
            cameraMatrix,  // ����ڲξ���
            distCoeffs,    // ����õ��Ļ������
            cv::Mat(),     // optional rectification (none) 
            cv::Mat(),     // camera matrix to generate undistorted
            //            cv::Size(640,480),
            image.size(),  // �޻���ͼ��ĳߴ�
            CV_32FC1,      // ���ӳ��ͼ�������
            map1, map2);   // x�����y����ӳ�亯��

        mustInitUndistort= false;
    }

    // Ӧ��ӳ�亯��
    cv::remap(image, undistorted, map1, map2, 
        cv::INTER_LINEAR); // interpolation type

    return undistorted;
}


void CameraCalibrator::setCalibrationFlag(bool radial8CoeffEnabled, bool tangentialParamEnabled) {

    // Set the flag used in cv::calibrateCamera()
    flag = 0;
    if (!tangentialParamEnabled) 
        flag += CV_CALIB_ZERO_TANGENT_DIST;
    if (radial8CoeffEnabled)
        flag += CV_CALIB_RATIONAL_MODEL;
}

/************************************************************************/
/* ����˵����   ��ӡ����                                                                     */
/************************************************************************/
void CameraCalibrator::printMatrix(const cv::Mat matrix)
{
    int w = matrix.cols ;
    int h = matrix.rows ;

    for (int i = 0 ; i < h ; ++i)
    {
        for (int j = 0 ; j < w ; ++j)
        {
            std::cout<< matrix.at<double>(i , j)<<"\t" ;
        }
        std::cout<<"\n" ;
    }
}

/************************************************************************/
/* ����˵����      ����������ڲ�������   
 * ������@Input    str ����·��*/
/************************************************************************/
void CameraCalibrator::saveCameraMatrix(std::string str)
{
    if (cameraMatrix.empty())
    {
        std::cout<<"����ڲ�������Ϊ��!\n" ;
        return ;
    }
    int w = cameraMatrix.cols ;
    int h = cameraMatrix.rows ;
    std::ofstream out(str , std::ios::out) ;
    if (!out)
    {
        std::cout<<"��ʧ�ܣ�\n" ;
        return ;
    }
    for (int i = 0 ; i < h ; ++i)
    {
        for (int j = 0 ; j < w ; ++j)
        {
            out<<cameraMatrix.at<double>(i , j)<<"\t" ;
        }
        out<<"\n" ;
    }
    out.close() ;
}
/************************************************************************/
/* ����˵����    ��ȡͼ���б�
 * ����@Input    path ����ͼ���б�·��
 * ����@Output   filelist��������ͼƬ·��*/
/************************************************************************/
void CameraCalibrator::readImagelist(const char* path , std::vector<std::string> &filelist)
{
    std::ifstream in(path , std::ios::in) ;
    if (!in)
    {
        return ;
    }
    std::string str ;
    int i = 0 ;
    while (!in.eof())
    {
        in>>str ;
        filelist.push_back(str) ;  
        ++i ;
    }
    in.close() ;
}
/************************************************************************/
/*����˵����            �����ⷽλ����
 *���� @Input           str ����·��*/
/************************************************************************/
void CameraCalibrator::saveRotation(std::string str)
{
    if (cameraRotation.empty())
    {
        std::cout<<"�ⷽλ����Ϊ��!\n" ;
        return ;
    }
    //��ȡ���������������
    int w = cameraRotation.cols ;
    int h = cameraRotation.rows ;
    std::ofstream out(str , std::ios::out) ;
    if (!out)
    {
        std::cout<<"�����ļ�ʧ�ܣ�\n" ;
        return ;
    }
    else
    {
        for (int i = 0 ; i < h ; ++i)
        {
            for (int j = 0 ; j < w ; ++j)
            {
                out<<cameraRotation.at<double>(i , j)<<"\t" ;
            }
            out<<"\n" ;
        }
    }
}

/************************************************************************/
/* ��ȡָ�����̸���ⷽλԪ��
 * Input i ��ʾ��i��ͼ�� */
/************************************************************************/
void CameraCalibrator::CalExtrinx(int i)
{
    Rodrigues(rvecs[i] , cameraRotation) ;
    cameraTranslation = tvecs[i] ;
}

/************************************************************************/
/* ����˵��      ���ռ�㷴ͶӰ����ƽ����
* ���� @Input   cameraParams�Ѿ��궨����������ṹ��
* ���� @Input   pos �趨ͶӰ����pos����ƽ����
* ���� @Ouput   v_p2d����ռ��ͷӦ����ƽ���ϵ�����
* ����          ʧ�ܷ���-1 */
/************************************************************************/
int CameraCalibrator::prejecttoPlane(const CameraParams &cameraParams , const int pos , std::vector<cv::Point2f> &v_p2d)
{
	if (!cameraParams.flag)
	{
		std::cout<<"���û�о����궨���޷����з�ͶӰ���㣡\n" ;
		return -1 ;
	}
	cv::projectPoints(cameraParams.p3d[pos] , cameraParams.rotations[pos] , cameraParams.translations[pos] , cameraParams.cameraMatrix , cameraParams.distortionCoefficients , v_p2d) ;

	return 1 ;
}

/************************************************************************/
/* ����˵����    ����ƽ����
 * @Input        str �����ļ�����*/
/************************************************************************/
void CameraCalibrator::saveTranslations(std::string str)
{
    if (cameraTranslation.empty())
    {
        std::cout<<"����������ƽ������Ϊ�գ�\n" ;
        return ;
    }
    int w = cameraTranslation.cols ;
    int h = cameraTranslation.rows ;
    std::ofstream out(str , std::ios::out) ;
    if (!out)
    {
        std::cout<<"д���ļ�ʧ�ܣ�\n" ;
        return ;
    }
    for (int i = 0 ; i < h ; ++i)
    {
        for (int j = 0 ; j < w ; ++j)
        {
            out<<cameraTranslation.at<double>(i , j)<<"\t" ;
        }
        out<<"\n" ;
    }

    out.close() ;
}

/************************************************************************/
/* ��������               �������̸�ߴ�
 * ���� @Input            boardsize ���̸�ߴ�
 * */
/************************************************************************/
void CameraCalibrator::setboardSize(cv::Size boardsize)
{
    m_boardsize = boardsize ;
}

void CameraCalibrator::setboardWidth(int squarewidth)
{
    m_squareWidth = squarewidth ;
}
