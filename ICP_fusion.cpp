#include <iostream>
#include <fstream>
#include <time.h>
#include <math.h>
#include "Eigen/Eigen"
#include "ICP.hpp"
#include "ICP_fusion.hpp"

using namespace std;
using namespace Eigen;

ICP_fusion::ICP_fusion(int x, int y, int z, int cx, int cy, int cz, float scale, MatrixXf pc_origin){
	BoxSize[0] = x;
	BoxSize[1] = y;
	BoxSize[2] = z;
	Center[0] = cx;
	Center[1] = cy;
	Center[2] = cz;
	ViewScale = scale;
	Rmtx << 1, 0, 0,
			0, 1, 0,
			0, 0, 1;
			
	Tvec << 0, 0, 0;
	
	PointCloud = pc_origin;
	ViewBoxCount = 0;
	
	//initail View Box
	vector < bool > temp1;
	temp1.assign(x,0);
	vector < vector< bool > > temp2;
	temp2.assign(y,temp1);
	ViewBox.assign(z,temp2);
	
	combine_view_box(pc_origin);
	cout << "Initialization Success !!" << endl;
}

int ICP_fusion::get_view_box(int x, int y, int z){
	if(x>0 && x<BoxSize[0] && y>0 && y<BoxSize[1] && z>0 && z<BoxSize[2])
		return ViewBox[z][y][x];
	else{
		cout << "Out of bound !!" << endl;
		return 0;
	}
}

void ICP_fusion::set_view_box(int x, int y, int z, bool value){
	if(x>0 && x<BoxSize[0] && y>0 && y<BoxSize[1] && z>0 && z<BoxSize[2])
		ViewBox[z][y][x] = value;
	else 
		cout << "Out of bound !!" << endl;
}

Matrix3f ICP_fusion::get_Rmtx(){
	return Rmtx.transpose();
}

Vector3f ICP_fusion::get_Tvec(){
	return Tvec;
}

void ICP_fusion::update(MatrixXf pc_now, int iterate, float threshold, int sel){
	ICP icp_match(pc_now, PointCloud);
	icp_match.align(iterate, threshold);
	Rmtx = icp_match.get_Rmtx() * Rmtx;
	Tvec = icp_match.get_Rmtx() * Tvec + icp_match.get_Tvec();
	
	//MatrixXf pc_correct = rigid_transform(pc_now, Rmtx, Tvec, 2);
	//cout << ViewBoxPc.cols() << " " << pc_correct.cols() << endl;
	//ICP icp_correct(ViewBoxPc, pc_correct);
	//icp_correct.align(iterate, threshold);
	//Rmtx = Rmtx * (icp_correct.get_Rmtx()).transpose();
	//Tvec = Tvec - (icp_correct.get_Rmtx()).transpose()*icp_correct.get_Tvec();
	
	PointCloud = pc_now;
	
	if(sel == 1)
		combine_view_box(pc_now);
}

void ICP_fusion::combine_view_box(MatrixXf pc){
	int x,y,z;
	int i,j;
	
	for(i=0; i<pc.cols(); i++){
		pc.col(i) -= Tvec; 
	}
	pc = Rmtx.transpose() * pc;

	for(i=0; i<pc.cols(); i++){
		x = (int)(pc(0,i)/ViewScale) + Center[0];
		y = (int)(pc(1,i)/ViewScale) + Center[1];
		z = (int)(pc(2,i)/ViewScale) + Center[2];
		//cout << x << " " << y << " " << z << endl;
		if(x>0 && x<BoxSize[0] && y>0 && y<BoxSize[1] && z>0 && z<BoxSize[2]){
			if(ViewBox[z][y][x] == 0){
				++ViewBoxCount;
				ViewBoxPc.resize(3, ViewBoxCount);
				ViewBoxPc(0,ViewBoxCount-1) = x;
				ViewBoxPc(1,ViewBoxCount-1) = y;
				ViewBoxPc(2,ViewBoxCount-1) = z;
				ViewBox[z][y][x] = 1;
			}
		}
		else 
			cout << "Out of bound !!" << endl;
	}
}

MatrixXf ICP_fusion::rigid_transform(MatrixXf pc, Matrix3f R, Vector3f T, int sel){
	int total = pc.cols();
	MatrixXf pc_mid;
	int i;
	if(sel == 1){
		pc_mid = R*pc;
		for(i=1; i<total; i++){
			pc_mid.col(i) += T;
		}
	}
	else if(sel == 2){
		pc_mid = pc;
		for(i=1; i<total; i++){
			pc_mid.col(i) -= T;
		}
		pc_mid = R.transpose() * pc_mid;
	}
	return pc_mid;
}

void ICP_fusion::print_result(){
	fstream file;
		
	file.open("R.txt",ios::out);
	file << Rmtx;
	file.close();
	
	file.open("T.txt",ios::out);
	file << Tvec(0) << " " << Tvec(1) << " " << Tvec(2);
	file.close();
}

void ICP_fusion::print_cube(){
	fstream file;
	file.open("Cube.txt",ios::out);
	
	int i,j,k;
	for(k=0; k<BoxSize[2]; k++){
		for(j=0; j<BoxSize[1]; j++){
			for(i=0; i<BoxSize[0]; i++){
				if(ViewBox[k][j][i] == 1){
					file << i << " " << j << " " << k << endl;
					//cout << i << " " << j << " " << k << endl;
				}
			}
		}
	}
	
	file.close();
}
