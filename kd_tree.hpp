#ifndef KD_TREE_HPP
#define KD_TREE_HPP

#include <stdio.h>
#define MAX_DIM 3

using namespace std;

struct kd_node{
	float position[ MAX_DIM ];
	kd_node *left;
	kd_node *right;
};

float  distance(struct kd_node *A,struct kd_node *B,int dim);

void swap(struct kd_node *A ,struct kd_node *B);

struct kd_node* find_mid(struct kd_node *start, int len, int dim);

struct kd_node* build_tree(struct kd_node* start, int len, int i, int dim);

void NNS(struct kd_node *root,struct kd_node *target,int i,int dim,struct kd_node **best,float *best_dist);

void dfs(struct kd_node* root);

void nns_init();

#endif