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

  //check if login succeeded through blocking read
  nb = socketReceiveBlocking(sock, buf, 65536, 1);
  if(nb <= 0) return -4;

  //skip command list
  nb = socketReadUntil(sock, buf, 65536, "End of commands");

  return sock;
}

int arclQuit(int sock)
{
  int nb = socketSend(sock, "quit\n");
  if(nb < 0) return -1;
  close(sock);
  return 0;
}

float* arclParseStatus(char* status)
{
  //check if status is a nullptr
  if(status == NULL) return NULL;

  //make sure info exists
  char* cptr = strstr(status, "Charge");
  char* lptr = strstr(status, "Location");
  if(cptr == NULL || lptr == NULL) return NULL;

  float* f = calloc(4, sizeof(float));

  //get pose
  char* tok = strtok(lptr, " ");
  for(int i = 0; i < 3; i++)
  {
    tok = strtok(NULL, " ");
    if(tok != NULL) f[i+1] = (float)atoi(tok);
  }

  //get state of charge
  tok = strtok(cptr, " ");
  tok = strtok(NULL, " ");
  if(tok != NULL) f[0] = atof(tok);

  //return status
  return f;
}


int* arclParseOdometer(char* odometer)
{
  //check if odometer is a nullptr
  if(odometer == NULL) return NULL;

  //make sure info exists
  const char* optr = strstr(odometer, "Odometer");
  if(optr == NULL) return NULL;

  int* o = calloc(3, sizeof(int));

  //get values
  char* tok = strtok(odometer, " ");
  for(int i = 0; i < 3; i++)
  {
    tok = strtok(NULL, " ");
    if(tok != NULL) o[i] = atoi(tok);
    tok = strtok(NULL, " "); 
  }

  //return odometer
  return o;
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

