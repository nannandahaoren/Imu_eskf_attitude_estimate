#ifndef SERIAL_H_
#define SERIAL_H_
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <iostream>
#include <string>
#include <cstring>

using namespace std;

class Serial
{
public:
    Serial();
    Serial(string _path);

    ~Serial();

    int open(void);

    int close(void);

    size_t read(uint8_t *check_head, size_t counter);

    int setConfig(void);

    void flush(void);

    string name = "default";

private:
    int fd = -1;
    string path = "/dev/ttyUSB0";
};

#endif