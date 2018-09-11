#ifndef POINTCLOUDTYPEDLG_H
#define POINTCLOUDTYPEDLG_H

#include <QDialog>
#include "ui_pointcloudtypedlg.h"

enum PointType
{
	ply,
	pcd
};
class PointcloudTypeDlg : public QDialog
{
	Q_OBJECT

public:
	PointcloudTypeDlg(QWidget *parent = 0);
	~PointcloudTypeDlg();
public slots:
	void slot_radio() ;
	void slot_save() ;
	QString getpath() const ;
signals:
	void sendtype(int type) ;
	void sendPath(QString path) ;
private:
	Ui::PointcloudTypeDlg ui;
	int m_type ;//�����������
	QString m_path ;
};

#endif // POINTCLOUDTYPEDLG_H
