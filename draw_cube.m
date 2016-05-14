clc;
clear;
clf;

pco = textread('Cube.txt');
plot3(pco(:,1),pco(:,2),pco(:,3),'bo');
axis([0 1000 0 1000 0 1000]);
%axis equal;