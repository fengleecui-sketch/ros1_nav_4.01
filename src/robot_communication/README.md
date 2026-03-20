## 注意事项v2.0

此功能包只放置自定义通讯文件，不编写任何代码
相对应的action,msg和srv放进相对应的文件夹内
自定义的通讯文件要有规范命名
例如串口发给视觉的消息则命名为
usart2vision.msg(2即代表to)
其他的按照这个例子来，发布者放前面，执行者或者接受者放后面
两部分负责人一定要沟通好通讯内容再编写通讯文件
需要通讯的时候其他部分的人要积极配合

## 若编译遇到问题

先编译robot_communication
```
catkin_make  --pkg robot_communication
```


## ros跨package使用自定义消息配置方法
+ 添加编译依赖和运行依赖  
```c++
<!-- 20.04或者更高版本使用 -->
<build_depend>robot_communication</build_depend>  
<exec_depend>message_generation</exec_depend>
<!-- 18.04及更低版本可以使用下面的 -->
<run_depend>robot_communication</run_depend>
```

+ 修改CMakeLists.txt
find_package中加入robot_communication(注意采用action 通讯时需要再添加 actionlib actionlib_msgs)

