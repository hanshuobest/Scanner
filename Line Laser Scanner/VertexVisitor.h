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

//������������̳���osg::NodeVisitor��
class VertexVisitor : public osg::NodeVisitor
{
public:

	//���涥������
	osg::ref_ptr<osg::Vec3Array> extracted_verts;

	//���캯������ʼ��һ�²�����Ϊ���������ӽڵ�
	VertexVisitor():osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
	{
		extracted_verts = new osg::Vec3Array();
	}

	//����apply����
	void apply( osg::Geode& geode )
	{
		//�õ�ÿһ��drawable
		for( unsigned int i=0; i < geode.getNumDrawables(); ++i )
		{
			//�õ�������
			osg::Geometry* geom = dynamic_cast<osg::Geometry*>( geode.getDrawable(i) );
			if( !geom )
			{
				continue ;
			}

			//�õ���������
			osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>( geom->getVertexArray() );
			if( !verts )
			{
				continue;
			}

			//��ӵ�extracted_verts
			extracted_verts->insert( extracted_verts->end(), verts->begin(), verts->end() );
		}
	}
};

