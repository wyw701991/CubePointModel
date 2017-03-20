// CubePointModel.cpp : 定义控制台应用程序的入口点。
//

#include"CubePointModel.h"
#include<fstream>
#include<sstream>
#include<algorithm>
vector<point> readData(std::string filename){
	vector<point> points;
	ifstream readData(filename);
	if (!readData)
	{
		std::cout << "[readImage] error:, Cannot open the dir!\n" << endl;
		exit(0);
	}
	std::cout << "Loading " << filename.c_str() << " ... " << endl;
	stringstream str;
	char line[1024];
	while (readData.getline(line, sizeof(line))){
		double a, b, c;
		str.str(line);
		str.clear();
		str >> a;
		str >> b;
		str >> c;
		point p(a, b, c);
		points.push_back(p);
	}
	readData.close();
	std::cout <<"Finish read Data"<< endl;
	return points;
}
CubePointModel::CubePointModel(){

}
CubePointModel::CubePointModel(vector<point> &V){
	if (V.size() != 0){
		this->V = V;
		vector<double> X;
		vector<double> Y;
		vector<double> Z;
		for (int i = 0; i < V.size(); i++){
			X.push_back(V[i].x);
			Y.push_back(V[i].y);
			Z.push_back(V[i].z);
		}
		auto v_minx = min_element(X.begin(), X.end());
		auto v_maxx = max_element(X.begin(), X.end());
		auto v_miny = min_element(Y.begin(), Y.end());
		auto v_maxy = max_element(Y.begin(), Y.end());
		auto v_minz = min_element(Z.begin(), Z.end());
		auto v_maxz = max_element(Z.begin(), Z.end());
		v_min.x = *v_minx;
		v_min.y = *v_miny;
		v_min.z = *v_minz;
		v_max.x = *v_maxx;
		v_max.y = *v_maxy;
		v_max.z = *v_maxz;
		CubeBoundPoint.push_back(point(v_min.x, v_min.y, v_min.z));
		CubeBoundPoint.push_back(point(v_min.x, v_min.y, v_max.z));
		CubeBoundPoint.push_back(point(v_min.x, v_max.y, v_min.z));
		CubeBoundPoint.push_back(point(v_min.x, v_max.y, v_max.z));
		CubeBoundPoint.push_back(point(v_max.x, v_min.y, v_min.z));
		CubeBoundPoint.push_back(point(v_max.x, v_min.y, v_max.z));
		CubeBoundPoint.push_back(point(v_max.x, v_max.y, v_min.z));
		CubeBoundPoint.push_back(point(v_max.x, v_max.y, v_max.z));
	}
}
vector<point> CubePointModel::GetBound(){
	return this->CubeBoundPoint;
}
vector<point> CubePointModel::GetBound(vector<point> &V){
	return this->CubeBoundPoint;
}
vector<point> CubePointModel::GetContained(vector<point> &b){
	vector<point> containPoint;
	vector<point> X, Y;
	for (int i = 0; i < V.size(); i++){
		if (V[i].x >= b[0].x&&V[i].x <= b[1].x)
			X.push_back(V[i]);
	}
	for (int j = 0; j < X.size(); j++){
		if (X[j].y >= b[0].y&&X[j].y <= b[1].y)
			Y.push_back(X[j]);
	}
	for (int k = 0; k < Y.size(); k++){
		if (Y[k].z >= b[0].z&&Y[k].z <= b[1].z)
			containPoint.push_back(Y[k]);
	}
	X.clear();
	Y.clear();
	return containPoint;
}
vector<point> CubePointModel::GetNearBy(point v_0, int X){
	vector<point> NearByPoint;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = V.size();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (int i = 0; i < V.size(); i++){
		cloud->points[i].x = V[i].x;
		cloud->points[i].y = V[i].y;
		cloud->points[i].z = V[i].z;
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	pcl::PointXYZ searchPoint;
	searchPoint.x = v_0.x;
	searchPoint.y = v_0.y;
	searchPoint.z = v_0.z;
	std::vector<int> pointIdxNKNSearch(X);
	std::vector<float> pointNKNSquaredDistance(X);
	std::cout << "K nearest neighbor search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with K=" << X << std::endl;
	if (kdtree.nearestKSearch(searchPoint, X, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			NearByPoint.push_back({ cloud->points[pointIdxNKNSearch[i]].x, cloud->points[pointIdxNKNSearch[i]].y, cloud->points[pointIdxNKNSearch[i]].z });
	}
	return NearByPoint;
}
CubePointModel::~CubePointModel(){
	CubeBoundPoint.clear();
	V.clear();
}
