#ifdef __cplusplus
extern "C" {
#endif

#ifndef ARCLUTIL_H
#define ARCLUTIL_H

#include "tcpClient.h"

typedef struct Point2D
{
  float x;
  float y;
} Point2D;

//points arr is statically defined for speed
typedef struct Point2DArr
{
  Point2D points[16384];
  int size;
} Point2DArr;

int arclLogin(const char* ipAddr, int port, const char* pw);
int arclQuit(int sock);
float* arclParseStatus(const char* status);
void arclParseLidar(char* lidar, Point2DArr* pArr);

#endif

#ifdef __cplusplus
}
#endif
