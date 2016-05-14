#include <iostream>
#include <time.h>
#include <math.h>
#include "Eigen/Eigen"
#include "ICP.hpp"
#include <stdbool.h>

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

int main(){
	srand(time(0));
	//build a box 640*480
	int pn = 3072; //20*20 3072
	int pn2 = 1000;
	MatrixXf pco = create_cube(16, 8, 4, pn);
	MatrixXf pcm = create_cube(16, 8, 4, pn2);

	//rotation
	float ax = 30*M_PI/180;
	float ay = 60*M_PI/180;
	float az = 60*M_PI/180;
	
	Matrix3f rotz;
	rotz <<
	cosf(az), -sinf(az), 0,
	sinf(az), cosf(az), 0,
	0, 0, 1;
		
	Matrix3f rotx;
	rotx <<
	1, 0, 0,
	0, cosf(ax), -sinf(ax),
	0, sinf(ax), cosf(ax);
		
		
	Matrix3f roty;
	roty << 
	cosf(ay), 0, -sinf(ay),
	0, 1, 0,
	sinf(ay), 0, cosf(ay);
	
	Vector3f tran;
	tran << 13,9,17;
	
	pcm = rotz * rotx * roty *pcm;
	int i;
	for(i=0; i<pn2; i++){
		pcm.col(i) += tran;
	}
	
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
	return 0;
}