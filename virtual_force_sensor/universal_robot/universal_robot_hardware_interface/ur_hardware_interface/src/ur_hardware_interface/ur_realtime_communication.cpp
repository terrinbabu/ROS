/*
 * ur_realtime_communication.cpp
 *
 * Copyright 2015 Thomas Timm Andersen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ur_hardware_interface/ur_realtime_communication.h"

UrRealtimeCommunication::UrRealtimeCommunication(
		std::condition_variable& msg_cond, std::string host,
    unsigned int safety_count_max, const double& speedj_timeout) {
	robot_state_ = new RobotStateRT(msg_cond);
	bzero((char *) &serv_addr_, sizeof(serv_addr_));
	sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd_ < 0) {
		ROS_FATAL("ERROR opening socket");
	}
	server_ = gethostbyname(host.c_str());
	if (server_ == NULL) {
		ROS_FATAL("ERROR, no such host");
	}
	serv_addr_.sin_family = AF_INET;
	bcopy((char *) server_->h_addr, (char *)&serv_addr_.sin_addr.s_addr, server_->h_length);
	serv_addr_.sin_port = htons(30003);
	flag_ = 1;
	setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_, sizeof(int));
	setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag_, sizeof(int));
	setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *) &flag_, sizeof(int));
	fcntl(sockfd_, F_SETFL, O_NONBLOCK);
	connected_ = false;
	keepalive_ = false;
	safety_count_ = safety_count_max + 1;
	safety_count_max_ = safety_count_max;
    m_speedj_timeout=0.016;
}

bool UrRealtimeCommunication::start() {
	fd_set writefds;
	struct timeval timeout;

	keepalive_ = true;
	ROS_DEBUG("Realtime port: Connecting...");

	connect(sockfd_, (struct sockaddr *) &serv_addr_, sizeof(serv_addr_));
	FD_ZERO(&writefds);
	FD_SET(sockfd_, &writefds);
	timeout.tv_sec = 10;
	timeout.tv_usec = 0;
	select(sockfd_ + 1, NULL, &writefds, NULL, &timeout);
	unsigned int flag_len;
	getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &flag_, &flag_len);
	if (flag_ < 0) {
		ROS_FATAL("Error connecting to RT port 30003");
		return false;
	}
	sockaddr_in name;
	socklen_t namelen = sizeof(name);
	int err = getsockname(sockfd_, (sockaddr*) &name, &namelen);
	if (err < 0) {
		ROS_FATAL("Could not get local IP");
		close(sockfd_);
		return false;
	}
	char str[18];
	inet_ntop(AF_INET, &name.sin_addr, str, 18);
	local_ip_ = str;
	comThread_ = std::thread(&UrRealtimeCommunication::run, this);
	return true;
}

void UrRealtimeCommunication::halt() {
	keepalive_ = false;
	comThread_.join();
}

void UrRealtimeCommunication::addCommandToQueue(std::string inp) {
	int bytes_written;
	if (inp.back() != '\n') {
		inp.append("\n");
	}
	if (connected_)
  {
		bytes_written = write(sockfd_, inp.c_str(), inp.length());
    if (bytes_written<-1)
      ROS_WARN("bytes_written is -1");
  }
  else
		ROS_ERROR_STREAM("Could not send command \"" +inp + "\". The robot is not connected! Command is discarded" );
}

void UrRealtimeCommunication::setSpeed(double q0, double q1, double q2,	double q3, double q4, double q5, double acc) {
	char cmd[1024];
	if( robot_state_->getVersion() >= 3.3 ) {
		sprintf(cmd,
				"speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, %1.5f)\n",
				q0, q1, q2, q3, q4, q5, acc,m_speedj_timeout);
	}
	else if( robot_state_->getVersion() >= 3.1 ) {
		sprintf(cmd,
				"speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f)\n",
				q0, q1, q2, q3, q4, q5, acc);
	}
	else {
		sprintf(cmd,
				"speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, 0.02)\n",
				q0, q1, q2, q3, q4, q5, acc);		
	}
	addCommandToQueue((std::string) (cmd));
	if (q0 != 0. or q1 != 0. or q2 != 0. or q3 != 0. or q4 != 0. or q5 != 0.) {
		//If a joint speed is set, make sure we stop it again after some time if the user doesn't
		safety_count_ = 0;
	}
}

void UrRealtimeCommunication::stop(const double& acc)
{
  char cmd[1024];
  sprintf(cmd, "stopj(%1.5f)\n",acc);
  addCommandToQueue((std::string) (cmd));
  safety_count_ = 0;
  
}

void UrRealtimeCommunication::setPosition(double q0, double q1, double q2, double q3, double q4, double q5, double dt, double lookahead, double gain)
{
  char cmd[1024];
  //sprintf(cmd,  "servoj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], 0, 0, %f, %f, %f)\n", q0, q1, q2, q3, q4, q5, dt, lookahead, gain);
  sprintf(cmd,  "servoj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], t=%.4f, lookahead_time=%.4f, gain=%.0f)\n", q0, q1, q2, q3, q4, q5, dt, lookahead, gain);
  addCommandToQueue((std::string) (cmd));
  
}


void UrRealtimeCommunication::run() {
	uint8_t buf[2048];
	int bytes_read;
	bzero(buf, 2048);
	struct timeval timeout;
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(sockfd_, &readfds);
	ROS_DEBUG("Realtime port: Got connection");
	connected_ = true;
	while (keepalive_) {
		while (connected_ && keepalive_) {
			timeout.tv_sec = 0; //do this each loop as selects modifies timeout
			timeout.tv_usec = 500000; // timeout of 0.5 sec
			select(sockfd_ + 1, &readfds, NULL, NULL, &timeout);
			bytes_read = read(sockfd_, buf, 2048);
			if (bytes_read > 0) {
				setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag_, 
						sizeof(int));
				robot_state_->unpack(buf);
				if (safety_count_ == safety_count_max_) {
					setSpeed(0., 0., 0., 0., 0., 0.);
				}
				safety_count_ += 1;
			} else {
				connected_ = false;
				close(sockfd_);
			}
		}
		if (keepalive_) {
			//reconnect
			ROS_WARN("Realtime port: No connection. Is controller crashed? Will try to reconnect in 10 seconds...");
			sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
			if (sockfd_ < 0) {
				ROS_FATAL("ERROR opening socket");
			}
			flag_ = 1;
			setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_,
					sizeof(int));
			setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag_, 
					sizeof(int));
	
			setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *) &flag_,
					sizeof(int));
			fcntl(sockfd_, F_SETFL, O_NONBLOCK);
			while (keepalive_ && !connected_) {
				std::this_thread::sleep_for(std::chrono::seconds(10));
				fd_set writefds;

				connect(sockfd_, (struct sockaddr *) &serv_addr_,
						sizeof(serv_addr_));
				FD_ZERO(&writefds);
				FD_SET(sockfd_, &writefds);
				select(sockfd_ + 1, NULL, &writefds, NULL, NULL);
				unsigned int flag_len;
				getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &flag_, &flag_len);
				if (flag_ < 0) {
					ROS_ERROR("Error re-connecting to RT port 30003. Is controller started? Will try to reconnect in 10 seconds...");
				} else {
					connected_ = true;
					ROS_INFO("Realtime port: Reconnected");
				}
			}
		}
	}
	setSpeed(0., 0., 0., 0., 0., 0.);
	close(sockfd_);
}

void UrRealtimeCommunication::setSafetyCountMax(uint inp) {
	safety_count_max_ = inp;
}

std::string UrRealtimeCommunication::getLocalIp() {
	return local_ip_;
}
