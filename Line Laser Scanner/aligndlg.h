#ifndef ALIGNDLG_H
#define ALIGNDLG_H

#include <QDialog>
#include "ui_aligndlg.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

class AlignDlg : public QDialog
{
	Q_OBJECT

public:
	AlignDlg(QWidget *parent = 0);
	~AlignDlg();
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPlyFile(std::string filename) ;
	pcl::PointCloud<pcl::PointXYZ>::Ptr loadPlyFile(std::string filename) ;
public slots:
	void slot_selectsource() ;
	void slot_selecttarget() ;
	QString getsourcepath() const ;
	QString gettargetpath() const ;

private:
	Ui::AlignDlg ui;
	QString m_sourcepath ;
	QString m_targetpath ;
};

#endif // ALIGNDLG_H
