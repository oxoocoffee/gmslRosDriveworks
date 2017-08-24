#include "cv_connection.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>

#define TCP_WITH_CV	1

#if !TCP_SERVER	
  #include <cv_bridge/cv_bridge.h>
  #include <sensor_msgs/Image.h>
  #include <sensor_msgs/image_encodings.h>

  OpenCVConnector::OpenCVConnector(std::string topic_name) : it(nh), counter(0)	{
      pub = it.advertise(topic_name, 1);
  }
#endif



#if TCP_SERVER

struct message_t
{
    uint32_t size;
    uint8_t cameraID;
    uint16_t width;
    uint16_t height;
} __attribute__((packed));

OpenCVConnector::OpenCVConnector(int camID) : _camID(camID), counter(0) {
	std::cout << "CamearID[" << _camID << "]" << std::endl;
}

bool OpenCVConnector::WriteToOpenCV(TCPSocket& client, unsigned char* buffer, int width, int height)
{
	struct message_t message;

#if TCP_WITH_CV
	int bitsPerPixel = 3;

	cv::Mat converted;

	cv::Mat mat_img(cv::Size(width, height), CV_8UC4, buffer);

	cv::cvtColor(mat_img, converted, cv::COLOR_RGBA2RGB);

        uint8_t* data    = converted.data; 
#else
	int bitsPerPixel = 4;
        uint8_t* data    = buffer; 

#endif

	message.size     = 5 + width*height*bitsPerPixel; 
	message.cameraID = _camID;
	message.width    = width;
	message.height   = height;

	// Convert to char array
	const uint8_t* msg = reinterpret_cast<const uint8_t*>(&message); 

	if(client.send(msg, sizeof(message_t)) <= 0)
        {
		std::cerr << "Connection Terminated" << std::endl;
		return false;
	}


	if(client.send(data, width * height * bitsPerPixel) <= 0)
        {
		std::cerr << "Connection Terminated" << std::endl;
		return false;
	}

	return true;
}

#else

void OpenCVConnector::WriteToOpenCV(unsigned char* buffer, int width, int height) {
    // create a cv::Mat from a dwImageNvMedia rgbaImage
    cv::Mat mat_img(cv::Size(width, height), CV_8UC4, buffer);

    cv::Mat converted;//=new cv::Mat();

    cv::cvtColor(mat_img,converted,cv::COLOR_RGBA2RGB);   //=COLOR_BGRA2BGR

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent

    std_msgs::Header header; // empty header
    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, converted);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    pub.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
}

#endif


