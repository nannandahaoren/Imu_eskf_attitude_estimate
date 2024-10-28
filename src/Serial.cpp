#include "../include/Serial.h"

Serial::Serial(){
    path = "/dev/ttyUSB0";
};

Serial::Serial(string _path= "/dev/ttyUSB0")
{
    path = _path;
}

Serial::~Serial(){
    close();
};

int Serial::open(void)
{
    fd = ::open(path.data(), O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        printf("(Serial), Fail to Open device\n");
        return -1;
    }
    printf("(Serial), serial Open succ.\n");
    return 0;
}

int Serial::close(void)
{
    if (fd == -1)
        return -1;

    return ::close(fd);
}

/**
 * @brief 读
 *
 * @param buff 串口读内容放置的容器
 * @param counter 读的字节数
 * @return size_t
 */
size_t Serial::read(uint8_t *buff, size_t counter)
{
    if(fd == -1) {
        printf("(Serial), not open device\r\n");
        return -1;
    }

    size_t readCount = ::read(fd, buff, counter);

    if (readCount == -1)
    {
        printf("(Serial), read fail...\r\n");
        std::cout<<readCount<<","<<counter<<std::endl;
    }

    return readCount;
};

void Serial::flush(void){
    tcflush(fd, TCIOFLUSH);
};


int Serial::setConfig(void){
    if(fd == -1) {
        printf("(Serial), not open device\r\n");
        return -1;
    }

    struct termios opt;

    // 设置串口输出波特率
    cfsetospeed(&opt, B921600);

    // 设置串口输入波特率
    cfsetispeed(&opt, B921600);

    // 设置数据位数 8位数据位
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;

    // 校验位 无校验位
    opt.c_cflag &= ~PARENB;
    opt.c_iflag &= ~INPCK;

    // 设置停止位  1位停止位
    opt.c_cflag &= ~CSTOPB;
    opt.c_cflag |= CLOCAL | CREAD;
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_oflag &= ~OPOST;
    opt.c_iflag &= ~(BRKINT | ICRNL | ISTRIP | IXON);

    // 设置停止time
    opt.c_cc[VTIME] = 20;
    opt.c_cc[VMIN] = 0;

    // 更新配置
    int ret = tcsetattr(fd, TCSANOW, &opt);
    std::cout <<"(Serial), "<<path<< " is set to 921600bps,8N1\n" << std::endl;

    tcflush(fd, TCIOFLUSH);
    return ret;
}