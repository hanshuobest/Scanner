/************************************************************************/
/* 单目立体标定                                                                     */
/************************************************************************/
#include "singlecalibrator.h"
#include <QAction>
#include <QMenu>
#include <QMessageBox>
#include <QFileInfo>
#include <QFileDialog>
#include <QPixmap>
#include <QGraphicsScene>
#include "showcameraparamdlg.h"


SingleCalibrator::SingleCalibrator(QWidget *parent)
	: QMainWindow(parent),m_strName(""),m_w(0),m_h(0),m_scale(0.0),m_flag(false),m_psdlg(NULL),m_size(0)
{
	ui.setupUi(this);
	this->setAttribute(Qt::WA_DeleteOnClose) ;
	this->setAttribute(Qt::WA_ShowModal , true) ;
	ui.listWidget_3->hide() ;
	m_scene = new QGraphicsScene ;
	createActions() ;
	createMenus() ;
	createTools() ;
}

SingleCalibrator::~SingleCalibrator()
{

}

void SingleCalibrator::createActions()
{
	m_addImageAct = new QAction(tr("Add Image") , this) ;
	m_addImageAct->setStatusTip(QStringLiteral("增加图片")) ;
	m_addImageAct->setIcon(QIcon("Resources\\image_add.png")) ;
	connect(m_addImageAct , SIGNAL(triggered()) , this , SLOT(slot_addImage())) ;

	m_startCalibAct = new QAction(tr("Start Calib") , this) ;
	m_startCalibAct->setEnabled(false) ;
	m_startCalibAct->setStatusTip(QStringLiteral("开始标定")) ;
	m_startCalibAct->setIcon(QIcon("Resources\\start.png")) ;
	connect(m_startCalibAct , SIGNAL(triggered()) , this , SLOT(slot_startCalib())) ;

}

void SingleCalibrator::createMenus()
{

}

void SingleCalibrator::slot_addImage()
{
	if (!m_list.isEmpty())
	{
		m_list.clear() ;
		int ncount = m_list.size() ;
		for (int i = 0 ; i < ncount ; i++)
		{
			QListWidgetItem *item = ui.listWidget_3->takeItem(0) ;
			delete item ;
		}
	}

	m_list = QFileDialog::getOpenFileNames(this , QStringLiteral("选择图片列表") , "" , tr("Image File(*.bmp *.BMP *.jpg)")) ;
	if (m_list.isEmpty())
	{
		return ;
	}
	ui.listWidget_3->show() ;
	createThumbnail(m_list) ;
	
    m_time = QTime::currentTime() ;
	int h = m_time.hour() ;
	int minus = m_time.minute() ;
	int second = m_time.second() ;
	QString str = QString("[ %1: %2: %3]:has load %4 pictures").arg(QString::number(h)).arg(QString::number(minus)).arg(QString::number(second)).arg(QString::number(m_list.size())) ;
	m_startCalibAct->setEnabled(true) ;
	emit sendMessage(str) ;

	m_psdlg = new PattenSizeDlg ;
	connect(m_psdlg , SIGNAL(sendParam(QVariant)) , this , SLOT(slot_recieveParam(QVariant))) ;
	m_psdlg->show() ;
}

void SingleCalibrator::slot_startCalib()
{
	ui.statusBar->showMessage(QStringLiteral("标定中...")) ;
	std::vector<std::string> piclist ;//图片列表
	int num = m_list.size() ;
	for (int i = 0 ; i < num ; i++)
	{
		std::string str = m_list[i].toStdString() ;
		piclist.push_back(str) ;
	}
	m_Calib.setboardWidth(m_size) ;
	m_Calib.addChessboardPoints(piclist , m_boardsize) ;
	cv::Mat image = cv::imread(piclist.at(1)) ;
	m_Calib.calibrate(image.size()) ;
	m_cameraparam = m_Calib.getCameraParams() ;
	m_Calib.CalExtrinx(0) ;


	m_time = QTime::currentTime() ;
	int h = m_time.hour() ;
	int minus = m_time.minute() ;
	int second = m_time.second() ;
	QString str = QString("[ %1: %2: %3]:calib success!").arg(QString::number(h)).arg(QString::number(minus)).arg(QString::number(second)).arg(QString::number(m_list.size())) ;
	emit sendMessage(str) ;

	if (QMessageBox::question(this , QStringLiteral("提示") , QStringLiteral("是否保存？") , QMessageBox::Yes|QMessageBox::Cancel) != QMessageBox::Cancel)
	{
		cv::Mat rmatrix = m_Calib.getCameraExtrinsic() ;
		cv::Mat tmatrix = m_Calib.getTranslations() ;
		cv::Mat dmatrix = m_Calib.getDistCoeffs() ;
		m_showCamParam = new ShowCameraParamDlg(this , m_cameraparam.cameraMatrix , rmatrix , tmatrix , dmatrix) ;
		m_showCamParam->show() ;

	}
}

void SingleCalibrator::createTools()
{
	ui.mainToolBar->addAction(m_addImageAct) ;
	ui.mainToolBar->addAction(m_startCalibAct) ;
}

void SingleCalibrator::createThumbnail(const QStringList strlist)
{
	if (strlist.isEmpty())
	{
		return ;
	}
	ui.listWidget_3->setViewMode(QListView::IconMode) ;
	ui.listWidget_3->setResizeMode(QListView::Fixed) ;
	QImage image(strlist.at(0)) ;
	m_w = image.width() ;
	m_h = image.height() ;
	m_scale = m_w/m_h ;
	ui.listWidget_3->setIconSize(QSize(150 , 150/m_scale)) ;

	QFileInfo fi ;
	int ncount = strlist.size() ;
	for (int i = 0 ; i < ncount ; i++)
	{
		QPixmap pixmap(strlist.at(i)) ;
		fi = QFileInfo(strlist.at(i)) ;

		QListWidgetItem *item = new QListWidgetItem(QIcon(pixmap.scaled(150 , 150/m_scale)) , fi.baseName()) ;
		ui.listWidget_3->insertItem(i , item) ;
	}
	connect(ui.listWidget_3 , SIGNAL(itemClicked(QListWidgetItem *)) , this , SLOT(slot_itemclick(QListWidgetItem *))) ;

}

void SingleCalibrator::slot_itemclick(QListWidgetItem *item)
{
	if (!item)
	{
		return ;
	}
	m_flag = true ;
	int nRowindex = ui.listWidget_3->row(item) ;
	m_strName = m_list.at(nRowindex) ;
	update() ;
}

void SingleCalibrator::slot_recieveParam(QVariant var)
{
	m_param = var.value<Param>() ;
	m_boardsize = cv::Size(m_param.x , m_param.y) ;
	m_size = m_param.size ;
}


void SingleCalibrator::paintEvent(QPaintEvent *event)
{
	if (m_flag)
	{
		setBackImage(m_strName) ;
	}
}

void SingleCalibrator::setBackImage(const QString strname)
{
	if (ui.graphicsView->items().count() > 0)
	{
		m_scene->removeItem(ui.graphicsView->items().first()) ;
	}
	cv::Mat src = cv::imread(strname.toStdString()) ;
	cv::Mat dst ;
	drawchessboardCorners(src , dst) ;
	QImage image = QImage((const uchar*)(dst.data) , dst.cols , dst.rows , dst.step , QImage::Format_RGB888).rgbSwapped() ;

	if(m_w > ui.graphicsView->width())
	{
		m_scene->addPixmap(QPixmap::fromImage(image).scaled(ui.graphicsView->width() , 500)) ;
	}
	else
	{
		m_scene->addPixmap(QPixmap::fromImage(image)) ;
	}
	ui.graphicsView->setScene(m_scene) ;
	ui.graphicsView->show() ;
}

bool SingleCalibrator::drawchessboardCorners(cv::Mat &src , cv::Mat &dst)
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