#ifdef __cplusplus
extern "C" {
#endif

#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

int createSocket(const char* ipAddr, int port);
int socketReceive(int sock, char* buf, int buflen, int readLen);
int socketReceiveBlocking(int sock, char* buf, int buflen, int readLen);
int socketPeek(int sock, char* buf, int buflen);
int socketReceiveLine(int sock, char* buf, int buflen);
int socketReadUntil(int sock, char* buf, int buflen, const char* until);
int socketSend(int sock, const char* msg);

#endif

#ifdef __cplusplus
}
#endif
