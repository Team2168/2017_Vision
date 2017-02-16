
#include "mjpeg_server.h"


#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

// I got tired of the warning so I just set a compiler argument to stop showing it.
// Im using a depricated way of convert string to const char * or something like that.
#pragma GCC diagnostic ignored "-Wwrite-strings"

int sockfd, newsockfd, portno, n;
socklen_t clilen;
char buffer[256];


struct sockaddr_in serv_addr, cli_addr;

std::vector<uchar> buf;

std::string initResponse;
std::string contentType;
std::string boundary;

bool imageReady;

mjpeg_server::mjpeg_server()
{
    sockfd = -1;
    newsockfd = -1;
    portno = 0;
    n=0;
    imageReady = false;


	initResponse =
		"HTTP/1.1 200 OK\n"
		"Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0\n"
		"Content-Type: multipart/x-mixed-replace;boundary=boundarydonotcross\n\n";


	contentType = "Content-Type: image/jpeg\n\n";

	boundary = "--boundarydonotcross";



}


int mjpeg_server::initMJPEGServer(int port) {
	sockfd = socket(AF_INET, SOCK_STREAM, 0);

	char *begin = (char *) &serv_addr;
	char *end = begin + sizeof(serv_addr);
	std::fill(begin, end, 0);

	//bzero((char *) &serv_addr, sizeof(serv_addr));

	portno = port;

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(portno);

	serv_addr.sin_addr.s_addr = INADDR_ANY;

	if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		error("Cannot bind socket");
		return 0;
	}
	return 1;
}

void mjpeg_server::host(void *args) {

	//continuously listen and open socket until success
	do
	{
	listen(sockfd, 5);
     	clilen = sizeof(cli_addr);
     	newsockfd = accept(sockfd,
                 	(struct sockaddr *) &cli_addr,
                 	&clilen);
     	if (newsockfd < 0)
          	error("ERROR on accept");
	}
     	while ((newsockfd < 0));

	char *begin = buffer;
	char *end = begin + sizeof(buffer);
	std::fill(begin, end, 0);

	//bzero(buffer,  256);
     	n = recv(newsockfd,buffer,sizeof(buffer), 0);

	std::cout << buffer << std::endl;
	std::cout << initResponse << std::endl;

	if (send(newsockfd, initResponse.c_str(), strlen(initResponse.c_str()), 0)<0)
	{
		std::cout<<"MJpeg Stream Lost Client during init, going back to listening"<<std::endl;
		imageReady = false;
		host(NULL);

	}

	imageReady = true;


}

bool mjpeg_server::setImageToHost(cv::Mat image)
{
	int statusByte1, statusByte2, statusByte3;
	if (imageReady) {
		cv::imencode(".jpg", image, buf, std::vector<int>() );

		//check for failure/broken pipe and then restart listening on socket
		//make sure signal(SIGPIPE, SIG_IGN) is set somewhere or Linux event handler will cause
		//program to crash upon writing to a broken pipe
		statusByte1 = send(newsockfd, contentType.c_str(), strlen(contentType.c_str()), 0);
		statusByte2 = send(newsockfd, (const char*)&buf[0], buf.size(), 0);
		statusByte3 = send(newsockfd, boundary.c_str(), strlen(boundary.c_str()), 0);

		if (statusByte1 <= 0 || statusByte2 <= 0 || statusByte3 <= 0)
		{
			imageReady = false;
			return false; //fail because send failed
		}
		return true; //success
	}
	return false; //fail because image isn't ready
}
void mjpeg_server::error(char *msg)
{
	perror(msg);
	//exit(1);
}
