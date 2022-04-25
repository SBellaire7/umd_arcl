#include "tcpClient.h"

//create a socket, connect to host, and return socket descriptor
int createSocket(const char* ipAddr, int port)
{
  int sock;
  struct sockaddr_in server;

  //create socket
  sock = socket(AF_INET, SOCK_STREAM, 0);
  if(sock == -1) return -1;

  //setup server
  server.sin_addr.s_addr = inet_addr(ipAddr);
  server.sin_family = AF_INET;
  server.sin_port = htons(port);

  //setup socket timeout to 5s for send and recv
  struct timeval tv;
  tv.tv_sec = 3;
  tv.tv_usec = 0;

  int status = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  if(status < 0) return -1;

  status = setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  if(status < 0) return -1;

  //connect to host
  status = connect(sock, (struct sockaddr*)&server, sizeof(server));
  if(status < 0) return -2;

  //return socket handle
  return sock;
}

//receive data on socket and return number of bytes read
int socketReceive(int sock, char* buf, int buflen, int readLen)
{
  //clear buffer first
  memset(buf, 0, buflen);

  //check if data is available on the socket
  int nb = 0;
  ioctl(sock, FIONREAD, &nb);
  if(nb == 0) return 0;

  //make sure there is room for a null terminator
  if(readLen >= buflen - 1) readLen = buflen - 1;

  //read specified number of data from socket
  nb = recv(sock, buf, readLen, 0);
  if(nb < 0) return -1;
  return nb;
}

//receive data on socket and return number of bytes. blocks if no data is available
int socketReceiveBlocking(int sock, char* buf, int buflen, int readLen)
{
  //clear buffer first
  memset(buf, 0, buflen);

  //make sure there is room for a null terminator
  if(readLen >= buflen - 1) readLen = buflen - 1;

  //read specified number of data from socket
  int nb = recv(sock, buf, readLen, 0);
  if(nb < 0) return -1;
  return nb;
}


//peek at data on socket and return number of bytes read
int socketPeek(int sock, char* buf, int buflen)
{
  //clear buffer first
  memset(buf, 0, buflen);

  //check if data is available on the socket
  int nb = 0;
  ioctl(sock, FIONREAD, &nb);
  if(nb == 0) return 0;

  //peek at socket contents, making sure the buf has room for a null terminator
  nb = recv(sock, buf, buflen - 1, MSG_PEEK);
  if(nb < 0) return -1;
  return nb;
}

//read data on socket until newline
int socketReceiveLine(int sock, char* buf, int buflen)
{
  //peek at data to make sure there is a full line
  int nb = socketPeek(sock, buf, buflen);
  if(nb < 0) return -1;
  char* c = strchr(buf, '\n');
  if(c = NULL) return 0;

  //if there is a full line, read up to (and including) the \n
  int idx = (int)(c - buf);
  nb = socketReceive(sock, buf, buflen, idx+1);
  if(nb < 0) return -1;
  return nb;
}

//read data on socket until keyword is found and return the line it is found
int socketReadUntil(int sock, char* buf, int buflen, const char* until)
{
  bool found = false;
  int nb = 0;
  do
  {
    //read a line
    nb = socketReceiveLine(sock, buf, buflen);
    if(nb < 0) return -1;
    else if(nb == 0) continue;

    //check if that line contains the keyword
    if(strstr(buf, until) != NULL) found = true;
  }while(!found);

  return 0;
}

//send data on socket and return number of bytes sent
int socketSend(int sock, const char* msg)
{
  int nb = send(sock, msg, strlen(msg), 0);
  if(nb < 0) return -1;
  return nb;
}
