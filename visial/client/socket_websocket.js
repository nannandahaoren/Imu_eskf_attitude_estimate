const net = require('net');
const WebSocketServer = require('ws').Server;

const SocketState = {
  "WAITING": "WAITING",
  "CONNECTED": "CONNECTED",
}
class Socket {
  constructor(websocket) {
    this.websocket = websocket;

    this.readeDataBuf = "";
    this.state = SocketState.WAITING;

    console.log("等待 Socket serve...");
    setInterval(() => {
      if (this.state === SocketState.CONNECTED) return;

      this.init();
    }, 1000);
  }

  init() {
    const socket = new net.Socket();

    const port = 1234;

    const hostname = '127.0.0.1';

    socket.setEncoding = 'UTF-8';

    socket.connect(port, hostname, () => {
      console.log("Socket 已经连接...");
      this.state = SocketState.CONNECTED;
    });

    socket.on('data', data => {
      if (!data) return;

      this.readeDataBuf = data.toString();
      this.websocket.sendData(this.readeDataBuf);
    });

    socket.on('error', error => {
      // console.log('Socket error', error);
    });

    socket.on('close', () => {
      if (this.state === SocketState.CONNECTED) {
        this.state = SocketState.WAITING;
        console.log('Socket close');
      }
    });
  }
}

class WebSocket {
  constructor() {
    this.ws = null;
    this.init();
  }

  sendData(data) {
    if (!this.ws) {
      return;
    }

    this.ws.send(data);
  }

  init() {
    // 创建websocket服务器
    const wss = new WebSocketServer({ port: 8888 });
    // 注册连接开启事件，同时其他事件的初始化
    wss.on('connection', ws => {
      console.log('WebSocket 已经连接..., running at ws://127.0.0.1:8888/');
      this.ws = ws;
    });
  }
}

function main() {
  const webSocket = new WebSocket();
  new Socket(webSocket);
}

main();
