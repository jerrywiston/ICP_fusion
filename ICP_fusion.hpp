#include <iostream>
#include <time.h>
#include <math.h>
#include <vector>
#include "Eigen/Eigen"
#include "ICP.hpp"


using namespace Eigen;
using namespace std;

typedef vector< vector< vector< bool > > > Cube;

class ICP_fusion{
	public:
		ICP_fusion(int x, int y, int z, int cx, int cy, int cz, float scale, MatrixXf pc_origin);
		void update(MatrixXf pc_now,  int iterate, float threshold,int sel); //sel=1 combine view box
		void set_view_box(int x, int y, int z, bool value);
		int get_view_box(int x, int y, int z);
		
		Matrix3f get_Rmtx();
		Vector3f get_Tvec();
		
		void print_result();
		void print_cube();
		
	protected:
		void combine_view_box(MatrixXf pc_now);
		MatrixXf rigid_transform(MatrixXf pc, Matrix3f R, Vector3f T, int sel);
		
	private:
		Cube ViewBox;
		MatrixXf ViewBoxPc;
		int ViewBoxCount;
		
		int BoxSize[3]; //0:x 1:y 2:z
		int Center[3];
		float ViewScale;
		
		Matrix3f Rmtx;
		Vector3f Tvec;
		
		MatrixXf PointCloud;
};
