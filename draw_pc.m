clc;
clear;
clf;
pco = textread('pco.txt');
plot3(pco(1,:),pco(2,:),pco(3,:),'ko');
hold on;

pcm = textread('pcm.txt');
plot3(pcm(1,:),pcm(2,:),pcm(3,:),'bo');
hold on;

R = textread('R.txt');
T = textread('T.txt');

pca = R*pco;
size1 = size(pca);
len = size1(2);
for i=1:len;
    pca(:,i) = pca(:,i) + transpose(T);
end
plot3(pca(1,:),pca(2,:),pca(3,:),'ro');
hold on;

Ri = transpose(R);
size2 = size(pcm);
len2 = size2(2);
pcb = pcm;
for i=1:len2;
    pcb(:,i) = pcb(:,i) - transpose(T);
end
pcb = Ri*pcb;
plot3(pcb(1,:),pcb(2,:),pcb(3,:),'go');
hold on;

axis equal;
%disp(pco);
