#include <iostream>
#include <vector>

using namespace std;

typedef struct vec_struct{
	float x;
	float y;
	float z;
}vec;

typedef vector < vec > pdata;

class kdtree(){
	public:
		kdtree(pdata pcloud);
		vec search();
		vec get_node(int index);
	
	protected:
		void sort();
		vec median(int dim);
	
	private:
		pdata node;
		int total;
}

kdtree::kdtree(pdata pcloud){
	
	
	
}