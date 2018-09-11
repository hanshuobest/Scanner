#ifndef LOADSTEREOIMGDLG_H
#define LOADSTEREOIMGDLG_H

#include <QDialog>
#include "ui_loadstereoimgdlg.h"
#include <QMetaType>
#include "struct_type.h"
class QVariant ;


class LoadStereoImgDlg : public QDialog
{
	Q_OBJECT

public:
	LoadStereoImgDlg(QWidget *parent = 0);
	~LoadStereoImgDlg();
	void createActions() ;
	int getX() const ;
	int getY() const ;
	int getSize() const ;
protected:
	void setX(int x) ;
	void setY(int y) ;
	void setSize(int s) ;

public slots:
	void slot_select1() ;
	void slot_select2() ;
	void slot_Ok() ;
	void slot_Cancel() ;
	void slot_selectComb(int index) ;
signals:
	void sendlist1(QStringList lst) ;
	void sendlist2(QStringList lst) ;
	void sendPattenparam(QVariant var) ;
private:
	Ui::LoadStereoImgDlg ui;
	QString m_str1 ; //保存目录1
	QString m_str2 ; //保存目录2
	QStringList m_strlist1 ;
	QStringList m_strlist2 ;
	int m_X ;
	int m_Y ;
	int m_Size ;
	Param m_param ;
};

#endif // LOADSTEREOIMGDLG_H
