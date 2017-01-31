/*
 * mjpeg_server.h
 *
 *  Created on: Jan 31, 2016
 *      Author: kevin
 */

#ifndef MJPEG_SERVER_H_
#define MJPEG_SERVER_H_

#include <unistd.h>
#include <iostream>    //cout
#include <stdio.h> //printf
#include <string.h>    //strlen
#include <string>  //string
#include <sys/socket.h>    //socket
#include <arpa/inet.h> //inet_addr
#include <netdb.h> //hostent
#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>

/**
    TCP Client class
*/
class mjpeg_server
{
private:

    int sockfd, newsockfd, portno, n;
    socklen_t clilen;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    std::vector<uchar> buf;
    std::string initResponse;
    std::string contentType;
    std::string boundary;
    bool imageReady;

public:
    mjpeg_server();
    int initMJPEGServer(int port);
    void host(void *args);
    bool setImageToHost(cv::Mat image);
    void error(char *msg);
};



#endif /* TCP_CLIENT_H_ */
