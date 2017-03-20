#include<vector>
#include<iostream>
#include"CubePointModel.h"
#include<iomanip>
using namespace std;
int main(){
	vector<point> V = readData("./data.txt");
	CubePointModel cubePointModel(V);  //初始化
	//输出最小立方体的边界点
	std::cout << "最小立方体的边界点为:" << endl;
	for (int i = 0; i < cubePointModel.CubeBoundPoint.size(); i++){
		std::cout <<fixed<<setprecision(6)<< cubePointModel.CubeBoundPoint[i].x << " " << cubePointModel.CubeBoundPoint[i].y << " " << cubePointModel.CubeBoundPoint[i].z << endl;
	}
	vector<point> boundPoint = cubePointModel.GetBound();
	//输出得到的边界点
	std::cout << "点云的边界点为:" << endl;
	std::cout << setprecision(6) << "(v_min,v_max)=((" << boundPoint[0].x << " " << boundPoint[0].y << " " << boundPoint[0].z << "),(" << boundPoint[1].x << " " << boundPoint[1].y << " " << boundPoint[1].z << "))" << endl;
	point v_0;
	int X;
	std::cout << "请输入三维点v_0，以及近邻点的个数X:";
	cin >> v_0.x >> v_0.y >> v_0.z>>X;
	vector<point> nearVPoint = cubePointModel.GetNearBy(v_0, X);
	//输出v_0的近邻点
	std::cout << "v_0的近邻点为:" << endl;
	for (int j = 0; j < nearVPoint.size(); j++){
		std::cout << setprecision(6) << nearVPoint[j].x << " " << nearVPoint[j].y << " " << nearVPoint[j].z << endl;
	}
	std::cout << "请依次输入最小范围点与最大范围点:" << endl;
	vector<point> b;
	b.resize(2);
	std::cin >> b[0].x >> b[0].y >> b[0].z;
	std::cin >> b[1].x >> b[1].y >> b[1].z;
	vector<point> containPoint = cubePointModel.GetContained(b);
	std::cout << "包含在b范围内的点的个数为:" << containPoint.size() << endl;
	std::cout << "包含在b范围内的点:" << endl;
	//输出包含在b范围内的点
	for (int j = 0; j < containPoint.size(); j++){
		std::cout << setprecision(6) << containPoint[j].x << " " << containPoint[j].y << " " << containPoint[j].z << endl;
	}

	return 0;
}
