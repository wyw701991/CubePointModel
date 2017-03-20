#ifndef CUBEPOINTMODEL_H
#define CUBEPOINTMODEL_H
#include <iostream>
#include<vector>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
using namespace std;
typedef struct point{       //存储读取的三维点
	double x;
	double y;
	double z;
	point(){};
	point(double a, double b, double c):
	     x(a),y(b),z(c){
	}
};
vector<point> readData(std::string filename);//从数据集中读取数据
class CubePointModel{              //立方体类
public:
	CubePointModel();
	CubePointModel(vector<point> &V); //初始化最小立方体
	~CubePointModel();
	vector<point> GetBound();
	vector<point> GetBound(vector<point> &V); //得到边界点
	vector<point> GetNearBy(point v_0, int X);//得到近邻点
	vector<point> GetContained(vector<point> &b);//得到范围内的点
public:
	vector<point> CubeBoundPoint;          //立方体边界点
	vector<point> V;              //点云数据
	point v_min;
	point v_max;
};
#endif  //BUNDLERTHREAD_H