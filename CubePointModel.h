#ifndef CUBEPOINTMODEL_H
#define CUBEPOINTMODEL_H
#include <iostream>
#include<vector>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
using namespace std;
typedef struct point{       //�洢��ȡ����ά��
	double x;
	double y;
	double z;
	point(){};
	point(double a, double b, double c):
	     x(a),y(b),z(c){
	}
};
vector<point> readData(std::string filename);//�����ݼ��ж�ȡ����
class CubePointModel{              //��������
public:
	CubePointModel();
	CubePointModel(vector<point> &V); //��ʼ����С������
	~CubePointModel();
	vector<point> GetBound();
	vector<point> GetBound(vector<point> &V); //�õ��߽��
	vector<point> GetNearBy(point v_0, int X);//�õ����ڵ�
	vector<point> GetContained(vector<point> &b);//�õ���Χ�ڵĵ�
public:
	vector<point> CubeBoundPoint;          //������߽��
	vector<point> V;              //��������
	point v_min;
	point v_max;
};
#endif  //BUNDLERTHREAD_H