#include "../include/Socket.h"

void Socket::open(void)
{
  // AF_INET :   表示使用 IPv4 地址		可选参数
  // SOCK_STREAM 表示使用面向连接的数据传输方式，
  // IPPROTO_TCP 表示使用 TCP 协议
  serv_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

  // 将套接字和IP、端口绑定
  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));           // 每个字节都用0填充
  serv_addr.sin_family = AF_INET;                     // 使用IPv4地址
  serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1"); // 具体的IP地址
  serv_addr.sin_port = htons(1234);                   // 端口
  bind(serv_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr));

  listen(serv_sock, 512);
  std::cout << "等待用户发起请求..." << std::endl;

  // 等待接收客户端请求
  struct sockaddr_in clnt_addr;
  socklen_t clnt_addr_size = sizeof(clnt_addr);
  clnt_sock = accept(serv_sock, (struct sockaddr *)&clnt_addr, &clnt_addr_size);

  std::cout << "连接建立成功..." << std::endl;
}

void Socket::close(void)
{
  ::close(clnt_sock);
  ::close(serv_sock);
}

int Socket::writeData(const char *msg, int size)
{
  ::write(clnt_sock, msg, size);
  return 0;
}
