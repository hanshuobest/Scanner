#pragma once
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/Node>
#include <osg/MatrixTransform>
class UseEventHandler :
	public osgGA::GUIEventHandler
{
public:
	enum 
	{
		ZOOMIN = 0 ,
		ZOOMOUT
	};
	UseEventHandler(void);
	virtual ~UseEventHandler(void);
	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		osgViewer::Viewer *viewer = dynamic_cast<osgViewer::Viewer *>(&aa) ;
		if (!viewer)
		{
			return false ;
		}
		osg::MatrixTransform *mt = dynamic_cast<osg::MatrixTransform *>(viewer->getSceneData()) ;
		if (mt)
		{
			static double i = 0.0 ;
			switch (ea.getEventType())
			{
			case osgGA::GUIEventAdapter::PUSH:
				if (ea.getButtonMask() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
				{
					
				}
				break;
			case osgGA::GUIEventAdapter::DRAG:
				if (ea.getButtonMask() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
				{

				}
				break;
			case osgGA::GUIEventAdapter::SCROLL:
				switch (ea.getScrollingMotion())
				{
				case osgGA::GUIEventAdapter::SCROLL_UP:
					break;
				case osgGA::GUIEventAdapter::SCROLL_DOWN:
					break;
				default:
					break;
				}
			case osgGA::GUIEventAdapter::KEYDOWN:
				if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Left)
				{
					mt->setMatrix(osg::Matrix::rotate(i , osg::Vec3(0.0 , 0.0 , 1.0))) ;
					i += 0.1 ;
				}
				else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
				{
					mt->setMatrix(osg::Matrix::rotate(i , osg::Vec3(0.0 , 0.0 , 1.0))) ;
					i -= 0.1 ;
				}
				break;
			default:
				break;
			}
		}
		return false ;
	}

protected:
	void zoom(int sens , int inverse) ;
};

