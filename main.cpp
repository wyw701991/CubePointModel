#include<vector>
#include<iostream>
#include"CubePointModel.h"
#include<iomanip>
using namespace std;
int main(){
	vector<point> V = readData("../src/data.txt");
	CubePointModel cubePointModel(V);  //��ʼ��
	//�����С������ı߽��
	std::cout << "��С������ı߽��Ϊ:" << endl;
	for (int i = 0; i < cubePointModel.CubeBoundPoint.size(); i++){
		std::cout <<fixed<<setprecision(6)<< cubePointModel.CubeBoundPoint[i].x << " " << cubePointModel.CubeBoundPoint[i].y << " " << cubePointModel.CubeBoundPoint[i].z << endl;
	}
	vector<point> boundPoint = cubePointModel.GetBound();
	//����õ��ı߽��
	std::cout << "���Ƶı߽��Ϊ:" << endl;
	std::cout << setprecision(6) << "(v_min,v_max)=((" << boundPoint[0].x << " " << boundPoint[0].y << " " << boundPoint[0].z << "),(" << boundPoint[1].x << " " << boundPoint[1].y << " " << boundPoint[1].z << "))" << endl;
	point v_0;
	int X;
	std::cout << "��������ά��v_0���Լ����ڵ�ĸ���X:";
	cin >> v_0.x >> v_0.y >> v_0.z>>X;
	vector<point> nearVPoint = cubePointModel.GetNearBy(v_0, X);
	//���v_0�Ľ��ڵ�
	std::cout << "v_0�Ľ��ڵ�Ϊ:" << endl;
	for (int j = 0; j < nearVPoint.size(); j++){
		std::cout << setprecision(6) << nearVPoint[j].x << " " << nearVPoint[j].y << " " << nearVPoint[j].z << endl;
	}
	std::cout << "������������С��Χ�������Χ��:" << endl;
	vector<point> b;
	b.resize(2);
	std::cin >> b[0].x >> b[0].y >> b[0].z;
	std::cin >> b[1].x >> b[1].y >> b[1].z;
	vector<point> containPoint = cubePointModel.GetContained(b);
	std::cout << "������b��Χ�ڵĵ�ĸ���Ϊ:" << containPoint.size() << endl;
	std::cout << "������b��Χ�ڵĵ�:" << endl;
	//���������b��Χ�ڵĵ�
	for (int j = 0; j < containPoint.size(); j++){
		std::cout << setprecision(6) << containPoint[j].x << " " << containPoint[j].y << " " << containPoint[j].z << endl;
	}

	return 0;
}