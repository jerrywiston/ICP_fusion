#include <iostream>
#include <fstream>
#include <stdio.h>
#include "Eigen/Eigen"
#include <limits>
#include <cmath>
#include "ICP.hpp"
#include <time.h>
//#include <omp.h>

using namespace std;
using namespace Eigen;

//=====================================
//Constructor 
//=====================================
ICP::ICP(MatrixXf pco, MatrixXf pcm){
	set_pco(pco);
	set_pcm(pcm);
	tot_init();
}

//======================================
// get set function
//======================================
void ICP::set_pco(MatrixXf pco){
	pc_origin = pco;
}

void ICP::set_pco(int index, Vector3f pt){
	pc_origin.col(index) = pt;
}

MatrixXf ICP::get_pco(){
	return pc_origin;
}

Vector3f ICP::get_pco(int index){
	return pc_origin.col(index);
}

void ICP::set_pcm(MatrixXf pcm){
	pc_match = pcm;
	pc_nns = MatrixXf(3,pc_match.cols());
}

void ICP::set_pcm(int index, Vector3f pt){
	pc_match.col(index) = pt;
}

MatrixXf ICP::get_pcm(){
	return pc_match;
}

Vector3f ICP::get_pcm(int index){
	return pc_match.col(index);
}

Matrix3f ICP::get_Rmtx(){
	return Rmtx;
}

Vector3f ICP::get_Tvec(){
	return Tvec;
}

void ICP::print_result(){
	fstream file;
	
	file.open("pco.txt",ios::out);
	file << pc_origin;
	file.close();
	
	file.open("pcm.txt",ios::out);
	file << pc_match;
	file.close();
	
	file.open("R.txt",ios::out);
	file << Rmtx;
	file.close();
	
	file.open("T.txt",ios::out);
	file << Tvec(0) << " " << Tvec(1) << " " << Tvec(2);
	file.close();
}

//=====================================
//Initialize function
//=====================================
void ICP::rot_init(){
	Rmtx << 1,0,0,
			0,1,0,
			0,0,1;
}

void ICP::tra_init(){
	Tvec << 0,0,0;
}

void ICP::mch_init(){
	int len = pc_match.rows();
}

void ICP::tot_init(){
	rot_init();
	tra_init();
	mch_init();
}

//======================================
//Align
//======================================

float ICP::align(int iterate, float threshold){
	//initialize some info
	int sizeo = pc_origin.cols();
	int sizem = pc_match.cols();
	
	//initialize kd-tree
	struct kd_node* root;
	struct kd_node wp[sizeo];
	root = kdtree_build(wp);
	//cout << pc_origin << endl;
	//printf("%f,%f,%f\n",root->left->position[0], root->left->position[1],root->left->position[2]);
	
	/*
	int n;
	cout << pc_origin << endl << endl;
	for(n=0; n<sizem; n++)
		cout << wp[n].position[0] << " " << wp[n].position[1] << " " << wp[n].position[2] << endl;
	system("pause");
	*/
	
	Vector3f Xave;	//mean value of X
	Vector3f Pave;	//mean value of Y
	
	MatrixXf Xc;	//the center origin point 
	MatrixXf Pc;	//the center match point
	
	Matrix3f Rtot;	//total rotation
	Rtot << 1,0,0,
			0,1,0,
			0,0,1;
	Vector3f Ttot;	//total translate
	Ttot << 0,0,0;
	Matrix3f R;	//rotation matrix
	Vector3f T;	//translation vector
	float Err = 0;
	
	//iterative to align
	int i,j;
	for(i=0; i<iterate; i++){
		Pc = Rtot*pc_match;
		for(j=0; j<sizem; j++)
			Pc.col(j) += Ttot;

		//nns_test(Pc);
		nns_find(root, Pc);
		Xc = pc_nns;
		
		//calculate mwan value
		Xave << Xc.row(0).sum() / sizem,
			    Xc.row(1).sum() / sizem,
			    Xc.row(2).sum() / sizem;
		Pave << Pc.row(0).sum() / sizem,
				Pc.row(1).sum() / sizem,
				Pc.row(2).sum() / sizem;
		
		//calculate center distance
		for(j=0; j<sizem; j++){
			Xc.col(j) -= Xave;
			Pc.col(j) -= Pave;
		}
		
		//SVD decomposition
		MatrixXf W = Xc * Pc.transpose();
		JacobiSVD<MatrixXf> svd(W, ComputeFullU | ComputeFullV);
		R = svd.matrixU() * svd.matrixV().transpose();
		T = Xave - R*Pave;
		
		//transform
		Rtot = R * Rtot;
		Ttot = T + R*Ttot;
		
		//calculate error
		Err = 0;
		for(j=0; j<sizem; j++){
			Err += pow(Xc(0,j),2) + pow(Xc(1,j),2) + pow(Xc(2,j),2);
			Err += pow(Pc(0,j),2) + pow(Pc(1,j),2) + pow(Pc(2,j),2);
		}
		for(j=0; j<svd.singularValues().size(); j++)
			Err -= 2*(svd.singularValues())(j);
		
		Err = Err/sizem;
		
		if(Err < threshold)break;
		//cout << Err/sizem << endl;
	}
	//cout << endl << Rtot << endl << endl << Ttot << endl << endl;
	Rmtx = Rtot;
	Tvec = Ttot;
	return Err;
}
//
void ICP::nns_test(MatrixXf Pc){
	int sizeo = pc_origin.cols();
	int sizem = Pc.cols();
	
	float rec;
	float mid;
	int plabel;
	
	int i,j;
	for(j=0; j<sizem; j++){
		rec = 9999999;
		for(i=0; i<sizeo; i++){
			mid = pow(Pc(0,j) - pc_origin(0,i), 2);
			mid += pow(Pc(1,j) - pc_origin(1,i), 2);
			mid += pow(Pc(2,j) - pc_origin(2,i), 2);
			if(mid < rec){
				plabel = i;
				rec = mid;
			}
		}
		pc_nns.col(j) = pc_origin.col(plabel);
	}
}

//======================================
//NNS
//======================================
struct kd_node* ICP::kdtree_build(struct kd_node *wp){
	int i,j;
	for(i=0; i<pc_origin.cols(); i++){
		for(j=0; j<3; j++)
			wp[i].position[j] = pc_origin(j,i);
	}
	return build_tree(wp,pc_origin.cols(),0,3);
}

void ICP::nns_find(struct kd_node *root, MatrixXf Pc){
	struct kd_node *found;	
	struct kd_node target;
	float best_dist;
	
	int i,j;
	//#pragma omp parallel for num_threads(4)
	for(i=0; i<Pc.cols(); i++){
		for(j=0; j<3; j++){
			target.position[j] = Pc(j,i);
		}
		nns_init();
		NNS(root,&target,0,3,&found,&best_dist);
		for(j=0; j<3; j++){
			pc_nns(j,i) = found -> position[j];
		}
	}
}
