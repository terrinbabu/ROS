/*************************************************************************************
Copyright (c) 2013, Carnegie Mellon University
All rights reserved.
Authors: Anurag Jakhotia<ajakhoti@andrew.cmu.edu>, Prasanna Velagapudi<pkv@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without modification, are 
permitted provided that the following conditions are met:

 -	Redistributions of source code must retain the above copyright notice, this list 
 	of conditions and the following disclaimer.
 -	Redistributions in binary form must reproduce the above copyright notice, this 
 	list of conditions and the following disclaimer in the documentation and/or other 
 	materials provided with the 	distribution.
 -	Neither the name of Carnegie Mellon University nor the names of its contributors 
 	may be used to endorse or promote products derived from this software without 
 	specific prior written 	permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY 
WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/
#include "k2_client/k2_client.h"

int imageSize = 8294400;
int streamSize = imageSize + sizeof(double);
std::string cameraName = "rgb";
std::string imageTopicSubName = "image_color";
std::string cameraFrame = "";

int main(int argC,char **argV)
{
	ros::init(argC,argV,"startRGB");
	ros::NodeHandle n(cameraName);
	image_transport::ImageTransport imT(n);
	std::string serverAddress;
	n.getParam("/serverNameOrIP",serverAddress);
	n.getParam(ros::this_node::getNamespace().substr(1,std::string::npos) +
	"/rgb_frame", cameraFrame);
	Socket mySocket(serverAddress.c_str(),"9000",streamSize);
	image_transport::CameraPublisher cameraPublisher = imT.advertiseCamera(
		imageTopicSubName, 1);
	camera_info_manager::CameraInfoManager camInfoMgr(n,cameraName);
	camInfoMgr.loadCameraInfo("");
	cv::Mat frame;
	cv_bridge::CvImage cvImage;
	sensor_msgs::Image rosImage;
	while(ros::ok())
	{
		printf("Got a frame.\n");
		
		mySocket.readData();
		printf("Creating mat.\n");
		frame = cv::Mat(cv::Size(1920,1080), CV_8UC3, mySocket.mBuffer);
		cv::flip(frame,frame,1);
		printf("Getting time.\n");
		double utcTime;
		memcpy(&utcTime,&mySocket.mBuffer[imageSize],sizeof(double));
		cvImage.header.frame_id = cameraFrame.c_str();
		cvImage.encoding = "bgr8";
		cvImage.image = frame;
		cvImage.toImageMsg(rosImage);
		sensor_msgs::CameraInfo camInfo = camInfoMgr.getCameraInfo();
		camInfo.header.frame_id = cvImage.header.frame_id;
		printf("Updating.\n");
		cameraPublisher.publish(rosImage, camInfo, ros::Time(utcTime));
		ros::spinOnce();
	}
	return 0;
}
