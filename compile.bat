cls
g++ -O2 -c kd_tree.cpp
g++ -O2 -c ICP.cpp
g++ -O2 -c ICP_fusion.cpp
g++ -O2 -o main main.cpp kd_tree.o ICP.o ICP_fusion.o