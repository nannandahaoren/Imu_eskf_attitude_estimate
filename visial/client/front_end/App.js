import Stage from './Stage.js'

window.THREE = THREE

class App {
  constructor() {
    window.lm = this
    this.stage = new Stage("#app")
    this.stage.run()
    this.addBox(10)
    this.initSocket();
  }

  initSocket() {
    var ws = new WebSocket("ws://localhost:8888/websocket");

    ws.onopen = function () {
      console.log("收到服务器Websocket握手包.", 1);
      console.log("Websocket连接已建立，正在等待数据...")
    }
    ws.onmessage = a=> {
      console.log(a.data, 0)
      // "&10&20&30&"
      const data = a.data.split("&");
      this.axisAngle = {
        x: +data[1],
        y: +data[2],
        z: +data[3],
      };
      console.log(this.axisAngle);
      this.cube.rotation.x = this.axisAngle.x * Math.PI / 180;
      this.cube.rotation.y = this.axisAngle.y * Math.PI / 180;
      this.cube.rotation.z = this.axisAngle.z * Math.PI / 180;
    }
    ws.onclose = function () {
      console.log("和服务器断开连接！", 0)
    }

    ws.onerror = function () {
      console.log("发生错误", 0)
    }
  }

  addBox() {
    var geometry = new THREE.BoxGeometry(10, 2, 5);
    var material = new THREE.MeshNormalMaterial({
      color: 0x63e42a,
      emissive: 0x072534,
      side: THREE.DoubleSide,
    })
    var cube = new THREE.Mesh(geometry, material);
    cube.name = "test_cube"
    this.cube = cube;
    this.stage.scene.add(cube)
    // return cube
  }
}

window.onload = () => {
  let app = new App()
}
