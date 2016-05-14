#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "kd_tree.hpp"

int visited = 0;

using namespace std;

float distance(struct kd_node *A,struct kd_node *B,int dim){
	int i=0;
	float d=0;
	for(i=0;i<dim;i++){
		d=d+(A->position[i]-B->position[i])*(A->position[i]-B->position[i]);
	}
	return d;
}

void swap(struct kd_node *A ,struct kd_node *B){

    float tmp[MAX_DIM];
    memcpy(tmp,  A->position, sizeof(tmp));
    memcpy(A->position, B->position, sizeof(tmp));
    memcpy(B->position, tmp,  sizeof(tmp));
}

struct kd_node* find_mid(struct kd_node *start , int len , int dim){
    struct kd_node *end=start+len-1;
    if(end < start){
        return NULL;
    }

    else if(end == start){
        return start;
    }

    else{
        struct kd_node *p, *store, *md = start+ ((end+1)-start)/2;
        float pivot;
        while (1) {
            pivot = start->position[dim];
            store=start;
            for (p = start+1; p <= end; p++) {
                if (p->position[dim] < pivot) {
                    store++;
                    swap(store,p);
                }
            }
            swap(start,store);
            if (store == md) return md;
            if (store > md) end = store-1;
            else  start = store+1;

    }

    }
}

struct kd_node* build_tree(struct kd_node* start,int len,int i,int dim){
    struct kd_node *n;
    if(!len){
        return 0;
    }
    if((n=find_mid(start,len,i))){
        i=(i+1)%dim;
        n->left=build_tree(start,n-start,i,dim);
        n->right=build_tree(n+1,start+len-(n+1),i,dim);
    };
    return n;
}

void NNS(struct kd_node *root,struct kd_node *target,int i,int dim,struct kd_node **best,float *best_dist){
	if(!root){
		return;
	}
	
	//visit
	//printf("+ ");
	float d=distance(root,target,dim);
	float dx=root->position[i]-target->position[i];
	float dx2 = dx * dx;
	
	if(*best!=NULL || d<*best_dist){
		//printf("- ");
		*best_dist=d;
		*best=root;
	}
	visited++;
	//printf("<%d>\n",visited);
	//system("pause");
	i++;
	if(*best_dist == 0){
		return; //root和target match
	}
	if(i>=dim){
		i=0;
	}
	if(dx>0){
		NNS(root->left,target,i,dim,best,best_dist);
	}
	else{
		NNS(root->right,target,i,dim,best,best_dist);
	}
	
	if (d >= *best_dist) return; 
	
	if(dx>0){
		NNS(root->right,target,i,dim,best,best_dist);
	}
	else{
		NNS(root->left,target,i,dim,best,best_dist);
	}
}

void dfs(kd_node* root){
	printf("%f,%f,%f\n",root->position[0], root->position[1],root->position[2]);
	
	if((root->left)!=NULL)
		dfs(root->left);
	
	if((root->right)!=NULL)
		dfs(root->right);
}

void nns_init(){
	visited = 0;
}

/*int readfile(){
	FILE *ptr;
	ptr=fopen("node.txt","r");
	int i=0;
	if (!ptr) {
        printf("檔案開啟失敗...\n");
        exit(1);
    }
	while(!feof(ptr)){
		fscanf(ptr,"%d %d",&kd_node[i].position[0],&kd_node[i].position[1]);
        i++;
	}
	return i-1;
}*/
/*
int main(){
	
	struct kd_node target = {{9, 2,2}};
    struct kd_node *found;
    float best_dist;
	
    struct kd_node wp[] = {
         {2, 3,1}, {21, 4,3}, {5,9, 6}, {8,4, 7}, {11,8, 1}, {19,7, 2}
    };
	

	
	struct kd_node *root;
	root=build_tree(wp,sizeof(wp) / sizeof(wp[1]),0,MAX_DIM);
	//printf("%f,%f,%f\n",root->position[0], root->position[1],root->position[2]);
	dfs(root);
	printf("===\n");
	
	NNS(root,&target,0,MAX_DIM,&found,&best_dist);
	
	printf("%f,%f,%f   %f   %d\n",found->position[0], found->position[1],found->position[2], sqrt(best_dist), visited);

	//printf("%f,%f\n",find_mid(wp,sizeof(wp) / sizeof(wp[1]),0)->position[0]);
	int i=0;

	return 0;
}
//*/
