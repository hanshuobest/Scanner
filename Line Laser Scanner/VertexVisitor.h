#pragma once
#include <osgViewer/Viewer>

#include <osg/Node>
#include <osg/Geode>
#include <osg/Group>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osgUtil/Optimizer>

#include <fstream>
#include <iostream>

//顶点访问器，继承自osg::NodeVisitor类
class VertexVisitor : public osg::NodeVisitor
{
public:

	//保存顶点数据
	osg::ref_ptr<osg::Vec3Array> extracted_verts;

	//构造函数，初始化一下并设置为遍历所有子节点
	VertexVisitor():osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
	{
		extracted_verts = new osg::Vec3Array();
	}

	//重载apply方法
	void apply( osg::Geode& geode )
	{
		//得到每一个drawable
		for( unsigned int i=0; i < geode.getNumDrawables(); ++i )
		{
			//得到几何体
			osg::Geometry* geom = dynamic_cast<osg::Geometry*>( geode.getDrawable(i) );
			if( !geom )
			{
				continue ;
			}

			//得到顶点数组
			osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>( geom->getVertexArray() );
			if( !verts )
			{
				continue;
			}

			//添加到extracted_verts
			extracted_verts->insert( extracted_verts->end(), verts->begin(), verts->end() );
		}
	}
};

