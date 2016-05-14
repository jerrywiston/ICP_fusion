#include <iostream>
#include "Include\OpenNI.h"

using namespace std;

//=========================================
int main(){
	//initailize OpenNI
	openni::OpenNI::initialize();
	
	//open a device
	openni::Device Sensor;
	Sensor.open(openni::ANY_DEVICE);
	
	//create depth stream
	openni::VideoStream streamDepth;
	streamDepth.create(Sensor, openni::SENSOR_DEPTH);
	streamDepth.start();
	
	//main loop
	openni::VideoFrameRef frameDepth;
	int i;
	//for(i=0; i<100; ++i){
	for(;;){
		//get frameDepth
		streamDepth.readFrame(&frameDepth);
		
		//get data array
		const openni::DepthPixel* pDepth
			= (const openni::DepthPixel*)frameDepth.getData();
		
		//output the septh value of center pointer
		int idx = frameDepth.getWidth() * (frameDepth.getHeight() + 1)/2;
		
		//=============
		float fx,fy,fz;
		int x,y;
		for(y=0; y<frameDepth.getHeight()-1; ++y){
			for(x=0; x<frameDepth.getHeight()-1; ++x){
				openni::CoordinateConverter::convertDepthToWorld(streamDepth, x, y, pDepth[idx], &fx, &fy, &fz);
			}
		}
		//==============
	
		//cout << pDepth[idx] << endl;		
		cout << fx << " " << fy << " " << fz << " " << pDepth[idx] << endl;
	}
	
	//closestream
	streamDepth.destroy();
	Sensor.close();
	openni::OpenNI::shutdown();
	return 0;
}