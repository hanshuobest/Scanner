#ifndef MATRIX_OPERATION_H
#define MATRIX_OPERATION_H

#include <vector>
using std::vector;
class matrix
{
public:
	vector<double> addition(vector<double>,vector<double>);
	vector<double> addition(vector<double>,double);
	vector<double> addition(double,vector<double>);//完成矩阵之间以及矩阵与常数的加法
	vector<double> subtraction(vector<double>,vector<double>);
	vector<double> subtraction(vector<double>,double);
	vector<double> subtraction(double,vector<double>);//完成矩阵之间以及矩阵与常数的减法
	vector<double> multiplication(const vector<double>&,size_t,size_t,const vector<double>&,size_t);
	vector<double> multiplication(vector<double>,double);
	vector<double> multiplication(double,vector<double>);//完成矩阵之间以及矩阵与常数的乘法（包括数乘与叉乘）
	vector<double> inverse(vector<double>,size_t);//完成矩阵的求逆
	vector<double> transposition(vector<double>,size_t,size_t);//完成矩阵的转置

    vector<double>least_line(const vector<double> x_vec , const vector<double> y_vec) ; //最小二乘拟合直线系数
    vector<double> least_plane(const vector<double> x_vec , const vector<double> y_vec , const vector<double> z_vec) ;//最小二乘拟合平面
};
#endif