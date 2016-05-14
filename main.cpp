#include <iostream>
#include <string>
#include <time.h>
#include <math.h>
#include <fstream>
#include "Eigen/Eigen"
#include "ICP_fusion.hpp"

using namespace std;
using namespace Eigen;

float rand_scope(float lb, float ub,int len){
	float lbp = lb * pow(10,len);
	float ubp = ub * pow(10,len);
	float dist = ubp - lbp;
	
	int rr = rand();
	float r = lbp + fmod((float)rr , dist);
	return r/pow(10,len);
}

void print_pc(const char* fn, MatrixXf pc){
	fstream file;
	file.open(fn,ios::out);
	file << pc;
	file.close();
}

MatrixXf create_cube(int cx, int cy, int cz,int total){
	MatrixXf cube_origin(3,total);
	int i;
	for(i=0; i<total; i++){
        if(i<total/6){
            cube_origin(0,i) = 0;
			cube_origin(1,i) = rand_scope(0,cy,2);
			cube_origin(2,i) = rand_scope(0,cz,2);
        }
		else if(i>=total/6 && i<2*total/6){
			cube_origin(0,i) = cx;
			cube_origin(1,i) = rand_scope(0,cy,2);
			cube_origin(2,i) = rand_scope(0,cz,2);
		}
        else if(i>=2*total/6 && i<3*total/6){
			cube_origin(0,i) = rand_scope(0,cx,2);
			cube_origin(1,i) = 0;
			cube_origin(2,i) = rand_scope(0,cz,2);
		}
		else if(i>=3*total/6 && i<4*total/6){
			cube_origin(0,i) = rand_scope(0,cx,2);
			cube_origin(1,i) = cy;
			cube_origin(2,i) = rand_scope(0,cz,2);
		}
        else if(i>=4*total/6 && i<5*total/6){
			cube_origin(0,i) = rand_scope(0,cx,2);
			cube_origin(1,i) = rand_scope(0,cy,2);
			cube_origin(2,i) = 0;
		}				
        else{
			cube_origin(0,i) = rand_scope(0,cx,2);
			cube_origin(1,i) = rand_scope(0,cy,2);
			cube_origin(2,i) = cz;
		}
	}
	return cube_origin;
}

MatrixXf transform(float rx, float ry, float rz, float tx, float ty, float tz, MatrixXf pc){
	//rotation
	rx = 30*M_PI/180;
	ry = 60*M_PI/180;
	rz = 60*M_PI/180;
	
	Matrix3f rotz;
	rotz <<
	cosf(rz), -sinf(rz), 0,
	sinf(rz), cosf(rz), 0,
	0, 0, 1;
		
	Matrix3f rotx;
	rotx <<
	1, 0, 0,
	0, cosf(rx), -sinf(rx),
	0, sinf(rx), cosf(rx);
		
		
	Matrix3f roty;
	roty << 
	cosf(ry), 0, -sinf(ry),
	0, 1, 0,
	sinf(ry), 0, cosf(ry);
	
	Vector3f tran;
	tran << tx, ty, tz;
	
	pc = rotz * rotx * roty *pc;
	int i;
	for(i=0; i<pc.cols(); i++){
		pc.col(i) += tran;
	}
	
	return pc;
}

int main(){
	srand(time(0));
	
	//build a box 640*480
	MatrixXf pc = create_cube(16, 8, 4, 768);
	MatrixXf pcm = pc;
	ICP_fusion fusion_test(1000,1000,1000,500,500,500,0.5,pc);

	int iterate = 100;
	float threshold = 0.01;
	int draw_cube = 1;
	
	clock_t start,finish;
	double duration;
	start = clock();
	/*
	pcm = transform(20,10,10,13,11,17,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);  
	pcm = transform(10,30,10,4,23,11,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	pcm = transform(20,10,5,21,1,7,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	pcm = transform(40,10,50,50,20,13,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	pcm = transform(20,60,5,21,13,47,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
 
	pcm = transform(10,30,10,4,23,11,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	pcm = transform(20,10,5,21,1,7,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	pcm = transform(40,10,50,50,20,13,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	pcm = transform(20,60,5,21,13,47,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	pcm = transform(21,37,5,47,22,32,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	
	pcm = transform(10,30,10,4,23,11,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	pcm = transform(20,10,5,21,1,7,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	pcm = transform(40,10,50,50,20,13,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	pcm = transform(20,60,5,21,13,47,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	pcm = transform(21,37,5,47,22,32,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	
	pcm = transform(10,30,10,4,23,11,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	pcm = transform(20,10,5,21,1,7,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	pcm = transform(40,10,50,50,20,13,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	pcm = transform(20,60,5,21,13,47,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	pcm = transform(21,37,5,47,22,32,pcm);
	fusion_test.update(pcm,iterate,threshold,draw_cube);
	*/
	for(int l=0; l<100; l++){
		pcm = transform(2,3,4,4,9,10,pcm);
		fusion_test.update(pcm,iterate,threshold,draw_cube);
	}
	
	finish = clock();   
    duration = (double)(finish - start) / CLOCKS_PER_SEC;
	printf( "%f seconds\n", duration ); 
	cout << "print result ..." << endl;
	fusion_test.print_result();
	fusion_test.print_cube();
	print_pc("pco.txt",pc);
	print_pc("pcm.txt",pcm);
	/*
	//ICP align
	clock_t start,finish;
	double duration;
	
	start = clock();
	ICP icp_test(pco, pcm);
	cout << "Error : " << icp_test.align(100, 0.001) << endl << endl;
	finish = clock();   
    duration = (double)(finish - start) / CLOCKS_PER_SEC;
	printf( "%f seconds\n", duration ); 
	
	icp_test.print_result();
	*/
	return 0;
}