#ifndef ICP_H
#define ICP_H
#include <iostream>
#include "Eigen/Eigen"
#include <limits>
#include <cmath>
#include "kd_tree.hpp"
//test
using namespace std;
using namespace Eigen;

class ICP{
	public:
		ICP(MatrixXf pco, MatrixXf pcm);
		float align(int iterate, float threshold);
		
		void set_pco(MatrixXf pco);
		void set_pco(int index, Vector3f pt);
		MatrixXf get_pco();
		Vector3f get_pco(int index);
		
		void set_pcm(MatrixXf pcm);
		void set_pcm(int index, Vector3f pt);
		MatrixXf get_pcm();
		Vector3f get_pcm(int index);
		
		Matrix3f get_Rmtx();
		Vector3f get_Tvec();
		
		void tot_init();
		
		void print_result();
		
	protected:
		void rot_init();
		void tra_init();
		void mch_init();
		
		struct kd_node* kdtree_build(struct kd_node *wp);
		void nns_find(struct kd_node *root, MatrixXf Pc);
		
		void nns_test(MatrixXf Pc);

	private:
		MatrixXf pc_origin;
		MatrixXf pc_match;
		MatrixXf pc_nns;
		
		Matrix3f Rmtx;
		Vector3f Tvec;
};

#endif	//ICP_H
