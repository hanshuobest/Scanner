#ifndef VIDEOTOFRAME_H
#define VIDEOTOFRAME_H

#include <QDialog>
#include "ui_videotoframe.h"
#include <QTimer>

class VideotoFrame : public QDialog
{
	Q_OBJECT

public:
	VideotoFrame(QWidget *parent = 0);
	~VideotoFrame();
public slots:
	void slot_selectvideo() ;
	void slot_saveas() ;
	void slot_videoToframe() ;
private:
	Ui::VideotoFrame ui;
	std::string m_video ;
	QTimer m_timer ;
	QString m_dir ;
};

#endif // VIDEOTOFRAME_H
