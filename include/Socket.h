#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

class Socket
{
public:
  void open(void);

  void close(void);

  int writeData(const char * msg, int size);

private:
  int serv_sock = -1;
  int clnt_sock = -1;
};
