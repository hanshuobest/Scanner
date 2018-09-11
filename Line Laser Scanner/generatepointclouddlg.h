/************************************************************************/
/* ���ɵ���                                                                     */
/************************************************************************/
#ifndef GENERATEPOINTCLOUDDLG_H
#define GENERATEPOINTCLOUDDLG_H

#include <QDialog>
#include "ui_generatepointclouddlg.h"
#include <opencv.hpp>
#include <vector>
#include "image_operation.h"
#include "pointcloudtypedlg.h"

struct PointCloudInfo
{
	std::vector<double> x_ivec ;
	std::vector<double> y_ivec ;
	std::vector<double> z_ivec ;
	std::vector<int> r_ivec ;
	std::vector<int> g_ivec ;
	std::vector<int> b_ivec ;
};
Q_DECLARE_METATYPE(PointCloudInfo) ;
struct Point2d
{
	Point2d(){} ;
	Point2d(const Point2d &p2d)
	{
		x = p2d.x ;
		y = p2d.y ;
	}
	Point2d &operator = (const Point2d &p2d)
	{
		if (this == &p2d)
		{
			return *this ;
		}
		x = p2d.x ;
		y = p2d.y ;
		return *this ;
	}
	void setXY(int x_ , int y_)
	{
		x = x_ ;
		y = y_ ;
	}
	int x ;
	int y ;
};
struct Point3d
{
	Point3d(){} ;
	Point3d(double x_ , double y_ , double z_):x(x_),y(y_),z(z_){} ;
	Point3d(const Point3d &p3d)
	{
		x = p3d.x ;
		y = p3d.y ;
		z = p3d.z ;
	}
	Point3d& operator =(const Point3d &p3d)
	{
		if (this == &p3d)
		{
			return *this ;
		}
		x = p3d.x ;
		y = p3d.y ;
		z = p3d.z ;
		return *this ;
	}
	void setXYZ(double x_ , double y_ , double z_)
	{
		x = x_ ;
		y = y_ ;
		z = z_ ;
	}

	double x ;
	double y ;
	double z ;
};
//ÿ������-��ά��Ľṹ��
struct Per_Pix
{
	Per_Pix():flag(0),R(0),G(0),B(0){} ;
	Per_Pix(const Per_Pix &pp)
	{
		p2d = pp.p2d ;
		p3d = pp.p3d ;
		flag = pp.flag ;
		R = pp.R ;
		G = pp.G ;
		B = pp.B ;
	}
	Per_Pix &operator = (const Per_Pix &pp)
	{
		if (this == &pp)
		{
			return *this ;
		}
		p2d = pp.p2d ;
		p3d = pp.p3d ;
		flag = pp.flag ;
		R = pp.R ;
		G = pp.G ;
		B = pp.B ;
	}
	void setXYZ(Point2d &p2d_ , Point3d &p3d_ , int *color)
	{
		p2d = p2d_ ;
		p3d = p3d_ ;
		R = color[0] ;
		G = color[1] ;
		B = color[2] ;
	}
	void setXYZ(Point2d &p2d_ , Point3d &p3d_ , int _R , int _G , int _B)
	{
		p2d = p2d_ ;
		p3d = p3d_ ;
		R = _R ;
		G = _G ;
		B = _B ;
	}
	Point2d p2d ;//����ÿ�������������
	Point3d p3d ;//ÿ�����Ӧ����ά����
	int flag ;
	int R ;
	int G ;
	int B ;
};

class VideoThread ;
class QGraphicsScene ;
class PointcloudTypeDlg ;
class QMetaType ;
class QVariant ;
class GeneratePointCloudDlg : public QDialog
{
	Q_OBJECT

public:   
	GeneratePointCloudDlg(QWidget *parent = 0);
	~GeneratePointCloudDlg();
	void createactions() ;
	void init() ;
	/************************************************************************/
	/* ��ȡ����ڲ������󣬱��浽intrinsic                                                                     */
	/************************************************************************/
	void readIntrinsic(std::string intrinsicfile) ;
	void readExtrinsc(std::string extrinsicfile) ;
	void readTranslation(std::string translationfile) ;
public slots:
	void slot_selectVideo() ;
	void slot_selectIntrinsic() ;
	void slot_selectExtrinsic() ;
	void slot_selectTranslation() ;
	void slot_selectDistort() ;
	void slot_addRow() ;
	void slot_delRow() ;
	void slot_start() ;
	void slot_radio_click() ;
	void slot_receiveframe(QVariant var) ;
	void slot_closeThread() ;
	void slot_colorback() ;
	void slot_recicetype(int type) ;
	void slot_savepath(QString path) ;
signals:
	void sendMessage(const QString &message) ;
	void sendPointcloudInfo(QVariant &var) ;
private:
	void print(QString str) ;
	/************************************************************************/
	/* ����˵����                 ������Ƶ��Ϣ
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
	int getVideoInfo(const std::string strvideo , int start_row , int start_col , int end_row , int end_col , VideoParams &params , std::vector<cv::Mat> &red_ivec , int flag = 0) ;
private:
	Ui::GeneratePointCloudDlg ui;
	QString m_video ;
	QString m_intrinsic ;
	QString m_extrinsic ;
	QString m_distort ;
	cv::Mat m_backImage ;//���汳��ͼƬ
	QString m_translation ;
	//������ʼ����
	int m_startrow ;
	int m_startcol ;
	int m_endrow ;
	int m_endcol ;

	std::vector<int> m_vref ; //���洹ֱ��Ĳο���
	std::vector<int> m_href ; //����ˮƽ��Ĳο���

	VideoThread *m_videothread ;
	QGraphicsScene *m_scene ;

	std::vector<double> m_intrinsicmatrix ;//�ڲ�������
	std::vector<double> m_extrinsicmatrix ;//���������
	std::vector<double> m_translationmatrix ;//ƽ����

	VideoParams m_vparam ;//��Ƶ������Ϣ 
	int m_useback ;//�Ƿ�ʹ�ñ���ͼƬ
	std::vector<Per_Pix> m_perpix ;
	PointcloudTypeDlg *m_typedlg ;
	int m_type ;//��������
	QString m_path ;
};

#endif // GENERATEPOINTCLOUDDLG_H
