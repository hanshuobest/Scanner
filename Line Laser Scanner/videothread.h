#ifndef VIDEOTHREAD_H
#define VIDEOTHREAD_H

#include <QThread>
#include <opencv.hpp>
#include <QVariant>
#include <QMetaType>

class QTimer ;
class QVariant ;

Q_DECLARE_METATYPE(cv::Mat) ;

class VideoThread : public QThread
{
    Q_OBJECT

public:
    VideoThread(QObject *parent , QString videostr) ;
    ~VideoThread();

protected:
    void run() ;
public slots:
    void slot_readFrame() ;

signals:
    void captureImage(QVariant var) ;
private:
    QString m_videofile ;
    QTimer *m_timer ;
    cv::VideoCapture m_capture ;
};

#endif // VIDEOTHREAD_H
