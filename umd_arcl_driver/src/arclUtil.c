#include "arclUtil.h"

//login to arcl server and return socket descritor
int arclLogin(const char* ipAddr, int port, const char* pw)
{
  //check if ip or pw are nullptr
  if(ipAddr == NULL || pw == NULL) return -1;

  //connect to server
  int sock = createSocket(ipAddr, port);
  if(sock < 0) return -1;

  //wait for prompt to enter password
  char buf[65536];
  int nb = socketReadUntil(sock, buf, 65536, "password");
  if(nb < 0) return -2;

  //add newline to pw
  char pwnl[128];
  strcpy(pwnl, pw);
  strcat(pwnl, "\n");

  //send password
  nb = socketSend(sock, pwnl);
  if(nb < 0) return -3;

  //wait a few ms and check if login succeeded through blocking read
  usleep(5000);
  nb = socketReceiveBlocking(sock, buf, 65536, 65535);
  if(strstr(buf, "Welcome") == NULL) return -4;

  return sock;
}

int arclQuit(int sock)
{
  int nb = socketSend(sock, "quit\n");
  if(nb < 0) return -1;
  close(sock);
  return 0;
}

float* arclParseStatus(const char* status)
{
  //check if status is a nullptr
  if(status == NULL) return NULL;

  //find the location of useful info
  const char* soc = strstr(status, "Charge");
  const char* loc = strstr(status, "Location");
  const char* tmp = strstr(status, "Temperature");

  //check if info was found
  if(soc == NULL || loc == NULL || tmp == NULL) return NULL;

  //calculate length of substrings to extract
  int socLen = loc - soc - 8;
  int locLen = tmp - loc - 10;

  //create buffers for substrings
  char socBuf[socLen + 1];
  char locBuf[locLen + 1];

  //extract substrings with numbers
  memcpy(socBuf, soc + 8, socLen);
  socBuf[socLen] = '\0';
  memcpy(locBuf, loc + 10, locLen);
  locBuf[locLen] = '\0';

  float* f = calloc(4, sizeof(float));

  //get state of charge
  f[0] = atof(socBuf);

  //get pose
  char* tok = strtok(locBuf, " ");
  f[1] = atof(tok);
  tok = strtok(NULL, " ");
  if(tok != NULL) f[2] = atof(tok);
  tok = strtok(NULL, " ");
  if(tok != NULL) f[3] = atof(tok);

  //return status values
  return f;
}

void arclParseLidar(char* lidar, Point2DArr* pArr)
{
  //init size to 0
  pArr->size = 0;

  //check if lidar is a nullptr
  if(lidar == NULL) return;

  //skip first 2 tokens in string
  char* tok = strtok(lidar, " ");
  tok = strtok(NULL, " ");

  //get point values
  while(tok != NULL)
  {
    tok = strtok(NULL, " ");
    if(tok == NULL) break;
    pArr->points[pArr->size].x = (float)atoi(tok);

    tok = strtok(NULL, " ");
    if(tok == NULL) break;
    pArr->points[pArr->size].y = (float)atoi(tok);

    pArr->size = pArr->size + 1;
  }

  return;
}

