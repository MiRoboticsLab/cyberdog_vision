# 简介
运行于小米CyberDog，实现人脸录入功能。

# 编译
## 编译
```console
colcon build --merge-install --packages-up-to cyberdog_vision
```

## 运行

启动camera模块
```console
ros2 run camera_test camera_server
```
启动AI模块：
```console
ros2 run cyberdog_vision vision_manager
```

开始录入命令：
```console
$ ros2 service call /facemanager protocol/srv/FaceManager "{command: 0,args: 'id=usename;host=true'}"
//id=用户名
//host=true or false 是否管理员
```
取消录入命令
```console
$ ros2 service call /facemanager protocol/srv/FaceManager "{command: 1,args: ''}"
```

确认录入命令
```console
$ ros2 service call /facemanager protocol/srv/FaceManager "{command: 2,args: 'id=usename;host=true'}"
```

修改用户名命令
```console
$ ros2 service call /facemanager protocol/srv/FaceManager "{command: 3,args: 'oriname:newname'}"
```

删除用户名命令
```console
$ ros2 service call /facemanager protocol/srv/FaceManager "{command: 4,args: 'id=usename'}"
```

获取已经录入的人脸命令
```console
$ ros2 service call /facemanager protocol/srv/FaceManager "{command: 5,args: ''}"
```
