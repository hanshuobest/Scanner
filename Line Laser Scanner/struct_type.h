#ifndef STRUCT_TYPE_H
#define STRUCT_TYPE_H
//模板参数结构体
typedef struct PattenParam
{
	PattenParam()
	{

	}
	PattenParam(const PattenParam &param)
	{
		size = param.size ;
		x = param.x ;
		y = param.y ;
	}
	PattenParam &operator=(const PattenParam &param)
	{
		if (this == &param)
		{
			return *this ;
		}
		size = param.size ;
		x = param.x ;
		y = param.y ;
		return *this ;
	}
	int size ;
	int x ;
	int y ;
}Param ;
Q_DECLARE_METATYPE(Param) ;


#endif // !STRUCT_TYPE_H
