/**
    C++ client example using sockets
*/
#include <unistd.h>
#include "tcp_client.h"


using namespace std;



tcp_client::tcp_client()
{
    sock = -1;
    port = 0;
    address = "";
}

/**
    Connect to a host on a certain port number
    Address can be host name or IP address
*/
bool tcp_client::conn(string address , int port)
{
    //create socket if it is not already created
    if(sock == -1)
    {
        //Create socket
        sock = socket(AF_INET , SOCK_STREAM , 0);
        if (sock == -1)
        {
            perror("Could not create socket");
        }

    }
    else    {   /* OK , nothing */  }


    //setup address structure
    if(inet_addr(address.c_str()) == -1)
    {
        struct hostent *he;
        struct in_addr **addr_list;

        //resolve the hostname, its not an ip address
        if ( (he = gethostbyname( address.c_str() ) ) == NULL)
        {
            //gethostbyname failed
            herror("gethostbyname");
            cout<<"Failed to resolve hostname, check server IP address\n";

            return false;
        }

        //Cast the h_addr_list to in_addr , since h_addr_list also has the ip address in long format only
        addr_list = (struct in_addr **) he->h_addr_list;

        for(int i = 0; addr_list[i] != NULL; i++)
        {
            //strcpy(ip , inet_ntoa(*addr_list[i]) );
            server.sin_addr = *addr_list[i];

            cout<<address<<" resolved to "<<inet_ntoa(*addr_list[i])<<endl;

            break;
        }
    }

    //plain ip address
    else
    {
        server.sin_addr.s_addr = inet_addr( address.c_str() );
    }


    server.sin_family = AF_INET;
    server.sin_port = htons( port );

    int count = 0;


    //Connect to remote server
   while (connect(sock, (struct sockaddr *)&server, sizeof(server)) == -1)
    {
    	count++;
        cout<<"Failed to Connect to Server, retrying "<<count<<endl;
        usleep(1000000); //retry to connect once a second
    }


    cout<<"Connected\n";
    return true;
}

/**
    Send data to the connected host
*/
bool tcp_client::send_data(string data)
{
    //Send some data
    if( send(sock , data.c_str() , strlen( data.c_str() ) , 0) < 0)
    {
        perror("Send failed : Client may have broken connection");
    	sock = -1;
        return false;
    }

    return true;
}

/**
    Receive data from the connected host
*/
string tcp_client::receive(int size=512)
{

    char current;
    char buffer[size];
    string reply;

    int i = 0;

    //keep reading each byte from the stream
    while(recv(sock , &current , 1 , 0))
    {
    		//append each byte read to the buffer
        	buffer[i] = current;
        	i++;


        	//if we reach an end of line character, we are done for now
        	//convert buffer to string and remove /n
        	if (current == '\n')
        	{
        		reply = string(buffer, i-1);
        		break;
        	}


    }


    return reply;


}
