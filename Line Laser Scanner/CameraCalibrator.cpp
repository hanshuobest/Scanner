#include "CameraCalibrator.h"
#include <fstream>
#include <algorithm>


CameraCalibrator::CameraCalibrator()
    : flag(0), mustInitUndistort(true),m_squareWidth(25)
{
 
}
/************************************************************************/
/* 函数说明            打开棋盘图像并且提取角点
 * @Input              filelist图像列表
 * @Input              boardSize棋盘尺寸
 * 返回值              返回成功检测棋盘格图像的数量*/
/************************************************************************/
int CameraCalibrator::addChessboardPoints(
    const std::vector<std::string>& filelist, 
    cv::Size & boardSize) 
{
    // 棋盘上点的两种坐标
    std::vector<cv::Point2f> imageCorners;
    std::vector<cv::Point3f> objectCorners;

    // 3D 场景中得点
    // 在棋盘坐标系中初始化棋盘角点
    // 这些点位于(X,Y,Z)=(i,j,0)
    for (int i=0; i<boardSize.height; i++) 
    {
        for (int j=0; j<boardSize.width; j++) 
        {
            objectCorners.push_back(cv::Point3f(j * m_squareWidth , i * m_squareWidth , 0.0f));
        }
    }

    cv::Mat image; // 为了保存棋盘图像
    int successes = 0;
    // 所有视图
    for (int i = 0; i< filelist.size(); i++)
    {
        // 打开图像
        image = cv::imread(filelist[i] , 0);
        // 得到角点
        bool found = cv::findChessboardCorners(image, boardSize, imageCorners , 1);

        // 获取亚像素级精度
        if (found)
        {
            cv::cornerSubPix(image, imageCorners, 
                cv::Size(5,5), 
                cv::Size(-1,-1), 
                cv::TermCriteria( 
                CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 
                40, //maxCount=40
                0.001 ));     // 最小精度

            // 如果角点数据满足要求，那么将它加入数据（全部角点检测出）
            if (imageCorners.size() == boardSize.area()) 
            {
                // 添加一个视角中得图像点及场景点
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
/* 函数说明：       添加场景点与对应的图像点   
 * @Input           imageCorners图像点
 * @Input           objectCorners场景点*/
/************************************************************************/
void CameraCalibrator::addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners) 
{
    // 2D 图像点
    imagePoints.push_back(imageCorners);          
    // 对应3D场景中得点
    objectPoints.push_back(objectCorners);
}

/************************************************************************/
/* 函数说明：     进行标定，返回重投影误差    
 * @Input         imageSize图片尺寸
 * 返回值         成功返回1*/
/************************************************************************/
double CameraCalibrator::calibrate(cv::Size &imageSize)
{
    //必须进行去畸变
    mustInitUndistort= true;

    //输出旋转和平移
    /*    std::vector<cv::Mat> rvecs, tvecs;*/

    calibrateCamera(objectPoints, // 3D点
        imagePoints,  // 图像点
        imageSize,   // 图像尺寸
        cameraMatrix, // 输出的相机矩阵
        distCoeffs,   // 输出的畸变矩阵
        rvecs, tvecs, // 旋转和平移
        flag);        // 额外选项

    m_cameraParams.flag = true ;//标注已标定
    m_cameraParams.cameraMatrix = cameraMatrix ;
    m_cameraParams.distortionCoefficients = distCoeffs ;
    m_cameraParams.rotations = rvecs ;
    m_cameraParams.translations = tvecs ;
    m_cameraParams.imageSize = imageSize ;
    //					,CV_CALIB_USE_INTRINSIC_GUESS);
    return 1;
}


/************************************************************************/
/* 函数说明：            利用标定结果去除图像的畸变 
 * 参数：@Input          image输入待矫正的图像
 * 返回：@Return         去畸变后的图像
 */
/************************************************************************/
cv::Mat CameraCalibrator::remap(const cv::Mat &image) 
{
    cv::Mat undistorted;

    if (mustInitUndistort)
    { //每次标定只需初始化一次   
        cv::initUndistortRectifyMap(
            cameraMatrix,  // 相机内参矩阵
            distCoeffs,    // 计算得到的畸变矩阵
            cv::Mat(),     // optional rectification (none) 
            cv::Mat(),     // camera matrix to generate undistorted
            //            cv::Size(640,480),
            image.size(),  // 无畸变图像的尺寸
            CV_32FC1,      // 输出映射图像的类型
            map1, map2);   // x坐标和y坐标映射函数

        mustInitUndistort= false;
    }

    // 应用映射函数
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
/* 函数说明：   打印矩阵                                                                     */
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
/* 函数说明：      保存相机的内参数矩阵   
 * 参数：@Input    str 保存路径*/
/************************************************************************/
void CameraCalibrator::saveCameraMatrix(std::string str)
{
    if (cameraMatrix.empty())
    {
        std::cout<<"相机内参数矩阵为空!\n" ;
        return ;
    }
    int w = cameraMatrix.cols ;
    int h = cameraMatrix.rows ;
    std::ofstream out(str , std::ios::out) ;
    if (!out)
    {
        std::cout<<"打开失败！\n" ;
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
/* 函数说明：    读取图像列表
 * 参数@Input    path 保存图像列表路径
 * 参数@Output   filelist保存所有图片路径*/
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
/*函数说明：            保存外方位矩阵
 *参数 @Input           str 保存路径*/
/************************************************************************/
void CameraCalibrator::saveRotation(std::string str)
{
    if (cameraRotation.empty())
    {
        std::cout<<"外方位矩阵为空!\n" ;
        return ;
    }
    //获取矩阵的行数和列数
    int w = cameraRotation.cols ;
    int h = cameraRotation.rows ;
    std::ofstream out(str , std::ios::out) ;
    if (!out)
    {
        std::cout<<"保存文件失败！\n" ;
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
/* 获取指定棋盘格的外方位元素
 * Input i 表示第i张图像 */
/************************************************************************/
void CameraCalibrator::CalExtrinx(int i)
{
    Rodrigues(rvecs[i] , cameraRotation) ;
    cameraTranslation = tvecs[i] ;
}

/************************************************************************/
/* 函数说明      将空间点反投影到像平面上
* 参数 @Input   cameraParams已经标定的相机参数结构体
* 参数 @Input   pos 设定投影到第pos个像平面上
* 参数 @Ouput   v_p2d保存空间点头应道像平面上的坐标
* 返回          失败返回-1 */
/************************************************************************/
int CameraCalibrator::prejecttoPlane(const CameraParams &cameraParams , const int pos , std::vector<cv::Point2f> &v_p2d)
{
	if (!cameraParams.flag)
	{
		std::cout<<"相机没有经过标定，无法进行反投影计算！\n" ;
		return -1 ;
	}
	cv::projectPoints(cameraParams.p3d[pos] , cameraParams.rotations[pos] , cameraParams.translations[pos] , cameraParams.cameraMatrix , cameraParams.distortionCoefficients , v_p2d) ;

	return 1 ;
}

/************************************************************************/
/* 函数说明：    保持平移量
 * @Input        str 保存文件名称*/
/************************************************************************/
void CameraCalibrator::saveTranslations(std::string str)
{
    if (cameraTranslation.empty())
    {
        std::cout<<"相机外参数的平移量量为空！\n" ;
        return ;
    }
    int w = cameraTranslation.cols ;
    int h = cameraTranslation.rows ;
    std::ofstream out(str , std::ios::out) ;
    if (!out)
    {
        std::cout<<"写入文件失败！\n" ;
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
/* 函数功能               设置棋盘格尺寸
 * 参数 @Input            boardsize 棋盘格尺寸
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
