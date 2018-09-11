#ifndef MATRIX_OPERATION_H
#define MATRIX_OPERATION_H

#include <vector>
using std::vector;
class matrix
{
public:
	vector<double> addition(vector<double>,vector<double>);
	vector<double> addition(vector<double>,double);
	vector<double> addition(double,vector<double>);//��ɾ���֮���Լ������볣���ļӷ�
	vector<double> subtraction(vector<double>,vector<double>);
	vector<double> subtraction(vector<double>,double);
	vector<double> subtraction(double,vector<double>);//��ɾ���֮���Լ������볣���ļ���
	vector<double> multiplication(const vector<double>&,size_t,size_t,const vector<double>&,size_t);
	vector<double> multiplication(vector<double>,double);
	vector<double> multiplication(double,vector<double>);//��ɾ���֮���Լ������볣���ĳ˷��������������ˣ�
	vector<double> inverse(vector<double>,size_t);//��ɾ��������
	vector<double> transposition(vector<double>,size_t,size_t);//��ɾ����ת��

    vector<double>least_line(const vector<double> x_vec , const vector<double> y_vec) ; //��С�������ֱ��ϵ��
    vector<double> least_plane(const vector<double> x_vec , const vector<double> y_vec , const vector<double> z_vec) ;//��С�������ƽ��
};
#endif