#ifndef OPEN_CV_CONNECTOR
#define OPEN_CV_CONNECTOR

#include "buildType.h"

#if PRINT_DURATION
#include <chrono>
#include <ctime>

typedef std::chrono::time_point<std::chrono::system_clock> TChrono;
typedef std::chrono::duration<float> TDuration;

#endif // PRINT_DURATION

#if TCP_SERVER
	#include "tcpSocket.h"
#else
	#include <ros/ros.h>
	#include <image_transport/image_transport.h>
#endif

#include <string>

class OpenCVConnector {
	public:

#if TCP_SERVER
		OpenCVConnector(int camID);
		bool WriteToOpenCV(TCPSocket& client, unsigned char*, int, int);

		int _camID;
#else
		OpenCVConnector(std::string topic_name);

		void WriteToOpenCV(unsigned char*, int, int);

		ros::NodeHandle                   nh;
		image_transport::ImageTransport   it;
		image_transport::Publisher        pub;
		std::string                       topic_name;
#endif

        unsigned int                      counter;
};



#endif
