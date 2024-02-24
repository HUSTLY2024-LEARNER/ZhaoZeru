# 第 3 章 ROS通信机制进阶

http://www.autolabor.com.cn/book/ROSTutorials/di-3-zhang-ros-tong-xin-ji-zhi-jin-jie.html

上一章内容，主要介绍了ROS通信的实现，内容偏向于粗粒度的通信框架的讲解，没有详细介绍涉及的API，也没有封装代码，鉴于此，本章主要内容如下:

- ROS常用API介绍；
- ROS中自定义头文件与源文件的使用。

预期达成的学习目标:

- 熟练掌握ROS常用API；
- 掌握ROS中自定义头文件与源文件的配置。

> 什么是`API`？
>
>  `API` = Application Programming Interface 程序之间的接口
>
> API 是允许两个软件组件使用一组定义和协议相互通信的机制
>
> ### API和函数的关系是什么
>
> 那么你要说了，上面的代码不就是一个函数吗？
>
> 的确，很多情况下，如果你在直接使用语言提供的API的话，API的形式可能就是一个函数。
>
> 但其实不然，因为还有很多其它的情况，API并不一定是一个函数，有可能是一个类，一个HTTP网络请求（最主要的API，请见下文）等
>
> ## API接口如何定义
>
> 目前最通用、使用最广泛的API标准叫作REST API。具体如何设计REST API请参考文章: [REST API设计指南](https://link.zhihu.com/?target=https%3A//kalasearch.cn/blog/rest-api-best-practices/)
>
> ## 哪里有API的一些例子
>
> 如上文所说，国内的互联网公司开放API的进程还比较慢，如果你可以阅读英文，我们推荐你参考以下优秀的API
>
> - [GitHub API](https://link.zhihu.com/?target=https%3A//docs.github.com/en/rest)
> - [Stripe API](https://link.zhihu.com/?target=https%3A//stripe.com/docs/api)
> - [Twitter API](https://link.zhihu.com/?target=https%3A//developer.twitter.com/en/docs)
>
> 国内的一些API例子有
>
> - [卡拉搜索API(硬广来了)](https://link.zhihu.com/?target=https%3A//kalasearch.cn/docs)
> - [Leancloud API](https://link.zhihu.com/?target=https%3A//leancloud.cn/docs/rest_api.html)
> - [Ping++ API](https://link.zhihu.com/?target=https%3A//www.pingxx.com/api/API%20%E6%96%87%E6%A1%A3%E7%AE%80%E4%BB%8B.html)

## 3.1 常用API

首先，建议参考官方API文档或参考源码:

- ROS节点的初始化相关API;
- NodeHandle 的基本使用相关API;
- 话题的发布方，订阅方对象相关API;
- 服务的服务端，客户端对象相关API;
- 时间相关API;
- 日志输出相关API。

参数服务器相关API在第二章已经有详细介绍和应用，在此不再赘述。

> **另请参考:**
>
> - http://wiki.ros.org/APIs
> - https://docs.ros.org/en/api/roscpp/html/

### 3.1.1 初始化 `ros::init()`

------

#### C++

##### 初始化

```cpp
/** @brief ROS初始化函数。
 *
 * 该函数可以解析并使用节点启动时传入的参数(通过参数设置节点名称、命名空间...) 
 *
 * 该函数有多个重载版本，如果使用NodeHandle建议调用该版本。 
 *
 * \param argc 参数个数
 * \param argv 参数列表
 * \param name 节点名称，需要保证其唯一性，不允许包含命名空间
 * \param options 节点启动选项，被封装进了ros::init_options
 *
 */
void init(int &argc, char **argv, const std::string& name, uint32_t options = 0);
```

------

### 3.1.2 话题与服务相关对象

------

#### C++

在 roscpp 中，话题和服务的相关对象一般由 `NodeHandle` 创建。

`NodeHandle`有一个重要作用是可以用于设置命名空间，这是后期的重点，但是本章暂不介绍。

##### 1.发布对象

###### 对象获取:

```cpp
/**
* \brief 根据话题生成发布对象
*
* 在 ROS master 注册并返回一个发布者对象，该对象可以发布消息
*
* 使用示例如下:
*
*   ros::Publisher pub = handle.advertise<std_msgs::Empty>("my_topic", 1);
*
* \param topic 发布消息使用的话题
*
* \param queue_size 等待发送给订阅者的最大消息数量
*
* \param latch (optional) 如果为 true,该话题发布的最后一条消息将被保存，并且后期当有订阅者连接时会将该消息发送给订阅者
*
* \return 调用成功时，会返回一个发布对象
*
*
*/
template <class M> //范型编程 函数模板
Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false)
```

###### 消息发布函数:

```cpp
/**
* 发布消息          
*/
template <typename M>
void publish(const M& message) const
```

##### 2.订阅对象

###### 对象获取:

```cpp
/**
   * \brief 生成某个话题的订阅对象
   *
   * 该函数将根据给定的话题在ROS master 注册，并自动连接相同主题的发布方，每接收到一条消息，都会调用回调
   * 函数，并且传入该消息的共享指针，该消息不能被修改，因为可能其他订阅对象也会使用该消息。
   * 
   * 使用示例如下:

void callback(const std_msgs::Empty::ConstPtr& message)
{
}

ros::Subscriber sub = handle.subscribe("my_topic", 1, callback);

   *
* \param M [template] M 是指消息类型
* \param topic 订阅的话题
* \param queue_size 消息队列长度，超出长度时，头部的消息将被弃用
* \param fp 当订阅到一条消息时，需要执行的回调函数
* \return 调用成功时，返回一个订阅者对象，失败时，返回空对象
* 

void callback(const std_msgs::Empty::ConstPtr& message){...}
ros::NodeHandle nodeHandle;
ros::Subscriber sub = nodeHandle.subscribe("my_topic", 1, callback);
if (sub) // Enter if subscriber is valid
{
...
}

*/
template<class M>
Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr<M const>&), const TransportHints& transport_hints = TransportHints())
```

##### 3.服务对象

###### 对象获取:

```cpp
/**
* \brief 生成服务端对象
*
* 该函数可以连接到 ROS master，并提供一个具有给定名称的服务对象。
*
* 使用示例如下:
\verbatim
bool callback(std_srvs::Empty& request, std_srvs::Empty& response)
{
return true;
}

ros::ServiceServer service = handle.advertiseService("my_service", callback);
\endverbatim
*
* \param service 服务的主题名称
* \param srv_func 接收到请求时，需要处理请求的回调函数
* \return 请求成功时返回服务对象，否则返回空对象:
\verbatim
bool Foo::callback(std_srvs::Empty& request, std_srvs::Empty& response)
{
return true;
}
ros::NodeHandle nodeHandle;
Foo foo_object;
ros::ServiceServer service = nodeHandle.advertiseService("my_service", callback);
if (service) // Enter if advertised service is valid
{
...
}
\endverbatim

*/
template<class MReq, class MRes>
ServiceServer advertiseService(const std::string& service, bool(*srv_func)(MReq&, MRes&))
```

##### 4.客户端对象

###### 对象获取:

```cpp
/** 
  * @brief 创建一个服务客户端对象
  *
  * 当清除最后一个连接的引用句柄时，连接将被关闭。
  *
  * @param service_name 服务主题名称
  */
 template<class Service>
 ServiceClient serviceClient(const std::string& service_name, bool persistent = false, 
                             const M_string& header_values = M_string())
```

###### 请求发送函数:

```cpp
/**
   * @brief 发送请求
   * 返回值为 bool 类型，true，请求处理成功，false，处理失败。
   */
  template<class Service>
  bool call(Service& service)
```

###### 等待服务函数1:

```cpp
/**
 * ros::service::waitForService("addInts");
 * \brief 等待服务可用，否则一致处于阻塞状态
 * \param service_name 被"等待"的服务的话题名称
 * \param timeout 等待最大时常，默认为 -1，可以永久等待直至节点关闭
 * \return 成功返回 true，否则返回 false。
 */
ROSCPP_DECL bool waitForService(const std::string& service_name, ros::Duration timeout = ros::Duration(-1));
```

* 等待服务函数2:

  ```cpp
  /**
  * client.waitForExistence();
  * \brief 等待服务可用，否则一致处于阻塞状态
  * \param timeout 等待最大时常，默认为 -1，可以永久等待直至节点关闭
  * \return 成功返回 true，否则返回 false。
  */
  bool waitForExistence(ros::Duration timeout = ros::Duration(-1));
  ```


###  3.1.3 回旋函数

#### C++

在ROS程序中，频繁的使用了 ros::spin() 和 ros::spinOnce() 两个回旋函数，可以用于处理回调函数。

> 回调函数是一种特殊的函数，它作为参数传递给另一个函数，并在被调用函数执行完毕后被调用。回调函数通常用于事件处理、异步编程和处理各种操作系统和框架的API。
>
> 基本概念：
>
> 1. 回调：指被传入到另一个函数的函数。
> 2. 异步编程：指在代码执行时不会阻塞程序运行的方式。
> 3. 事件驱动：指程序的执行是由外部事件触发而不是顺序执行的方式。
>
> ### 回调函数的作用和使用场景
>
> 回调函数是一种常见的编程技术，它可以在异步操作完成后调用一个预定义的函数来处理结果。回调函数通常用于处理事件、执行异步操作或响应用户输入等场景。
>
> 回调函数的作用是将代码逻辑分离出来，使得代码更加模块化和可维护。使用回调函数可以避免阻塞程序的运行，提高程序的性能和效率。另外，回调函数还可以实现代码的复用，因为它们可以被多个地方调用。
>
> 回调函数的使用场景包括：
>
> 1. 事件处理：回调函数可以用于处理各种事件，例如鼠标点击、键盘输入、网络请求等。
> 2. 异步操作：回调函数可以用于异步操作，例如读取文件、发送邮件、下载文件等。
> 3. 数据处理：回调函数可以用于处理数据，例如对数组进行排序、过滤、映射等。
> 4. 插件开发：回调函数可以用于开发插件，例如 WordPress 插件、jQuery 插件等。
>
> 回调函数是一种非常灵活和强大的编程技术，可以让我们更好地处理各种异步操作和事件。
>
> ## 回调函数的优缺点
>
> 优点：
>
> 1. 提高代码的复用性和灵活性：回调函数可以将一个函数作为参数传递给另一个函数，从而实现模块化编程，提高代码的复用性和灵活性。
> 2. 解耦合：回调函数可以将不同模块之间的关系解耦，使得代码更易于维护和扩展。
> 3. 可以异步执行：回调函数可以在异步操作完成后被执行，这样避免了阻塞线程，提高应用程序的效率。
>
> 缺点：
>
> 1. 回调函数嵌套过多会导致代码难以维护：如果回调函数嵌套层数过多，代码会变得非常复杂，难以维护。
> 2. 回调函数容易造成竞态条件：如果回调函数中有共享资源访问，容易出现竞态条件，导致程序出错。
> 3. 代码可读性差：回调函数的使用可能会破坏代码的结构和可读性，尤其是在处理大量数据时。
>
> 小结：代码灵活、易于扩展，但是不易于阅读、容易出错。

##### 1.spinOnce()

```cpp
/**
 * \brief 处理一轮回调
 *
 * 一般应用场景:
 *     在循环体内，处理所有可用的回调函数
 * 
 */
ROSCPP_DECL void spinOnce();
```

##### 2.spin()

```cpp
/** 
 * \brief 进入循环处理回调 
 */
ROSCPP_DECL void spin();
```

##### 3.二者比较

**相同点:**二者都用于处理回调函数；

**不同点:**ros::spin() 是进入了循环执行回调函数，而 ros::spinOnce() 只会执行一次回调函数(没有循环)，在 ros::spin() 后的语句不会执行到，而 ros::spinOnce() 后的语句可以执行。

### 3.1.4 时间

ROS中时间相关的API是极其常用，比如:获取当前时刻、持续时间的设置、执行频率、休眠、定时器...都与时间相关。

------

#### C++

##### 1.时刻

获取时刻，或是设置指定时刻:

```cpp
ros::init(argc,argv,"hello_time");
ros::NodeHandle nh;//必须创建句柄，否则时间没有初始化，导致后续API调用失败
ros::Time right_now = ros::Time::now();//将当前时刻封装成对象
ROS_INFO("当前时刻:%.2f",right_now.toSec());//获取距离 1970年01月01日 00:00:00 的秒数
ROS_INFO("当前时刻:%d",right_now.sec);//获取距离 1970年01月01日 00:00:00 的秒数

ros::Time someTime(100,100000000);// 参数1:秒数  参数2:纳秒
ROS_INFO("时刻:%.2f",someTime.toSec()); //100.10
ros::Time someTime2(100.3);//直接传入 double 类型的秒数
ROS_INFO("时刻:%.2f",someTime2.toSec()); //100.30
```

##### 2.持续时间

设置一个时间区间(间隔):

```cpp
ROS_INFO("当前时刻:%.2f",ros::Time::now().toSec());
ros::Duration du(10);//持续10秒钟,参数是double类型的，以秒为单位
du.sleep();//按照指定的持续时间休眠
ROS_INFO("持续时间:%.2f",du.toSec());//将持续时间换算成秒
ROS_INFO("当前时刻:%.2f",ros::Time::now().toSec());
```

##### 3.持续时间与时刻运算

为了方便使用，ROS中提供了时间与时刻的运算:

```cpp
ROS_INFO("时间运算");
ros::Time now = ros::Time::now();
ros::Duration du1(10);
ros::Duration du2(20);
ROS_INFO("当前时刻:%.2f",now.toSec());
//1.time 与 duration 运算
ros::Time after_now = now + du1;
ros::Time before_now = now - du1;
ROS_INFO("当前时刻之后:%.2f",after_now.toSec());
ROS_INFO("当前时刻之前:%.2f",before_now.toSec());

//2.duration 之间相互运算
ros::Duration du3 = du1 + du2;
ros::Duration du4 = du1 - du2;
ROS_INFO("du3 = %.2f",du3.toSec());
ROS_INFO("du4 = %.2f",du4.toSec());
//PS: time 与 time 不可以运算
// ros::Time nn = now + before_now;//异常
```

##### 4.设置运行频率

```cpp
ros::Rate rate(1);//指定频率
while (true)
{
    ROS_INFO("-----------code----------");
    rate.sleep();//休眠，休眠时间 = 1 / 频率。
}
```

##### 5.定时器

ROS 中内置了专门的定时器，可以实现与 ros::Rate 类似的效果:

```cpp
ros::NodeHandle nh;//必须创建句柄，否则时间没有初始化，导致后续API调用失败

 // ROS 定时器
 /**
* \brief 创建一个定时器，按照指定频率调用回调函数。
*
* \param period 时间间隔
* \param callback 回调函数
* \param oneshot 如果设置为 true,只执行一次回调函数，设置为 false,就循环执行。
* \param autostart 如果为true，返回已经启动的定时器,设置为 false，需要手动启动。
*/
 //Timer createTimer(Duration period, const TimerCallback& callback, bool oneshot = false,
 //                bool autostart = true) const;

 // ros::Timer timer = nh.createTimer(ros::Duration(0.5),doSomeThing);
 ros::Timer timer = nh.createTimer(ros::Duration(0.5),doSomeThing,true);//只执行一次

 // ros::Timer timer = nh.createTimer(ros::Duration(0.5),doSomeThing,false,false);//需要手动启动
 // timer.start();
 ros::spin(); //必须 spin
```

定时器的回调函数:

```cpp
void doSomeThing(const ros::TimerEvent &event){
    ROS_INFO("-------------");
    ROS_INFO("event:%s",std::to_string(event.current_real.toSec()).c_str());
}

```

### 3.1.5 其他函数

在发布实现时，一般会循环发布消息，循环的判断条件一般由节点状态来控制，C++中可以通过 ros::ok() 来判断节点状态是否正常，而 python 中则通过 rospy.is_shutdown() 来实现判断，导致节点退出的原因主要有如下几种:

- 节点接收到了关闭信息，比如常用的 ctrl + c 快捷键就是关闭节点的信号；
- 同名节点启动，导致现有节点退出；
- 程序中的其他部分调用了节点关闭相关的API(C++中是ros::shutdown()，python中是rospy.signal_shutdown())

另外，日志相关的函数也是极其常用的，在ROS中日志被划分成如下级别:

- DEBUG(调试):只在调试时使用，此类消息不会输出到控制台；
- INFO(信息):标准消息，一般用于说明系统内正在执行的操作；
- WARN(警告):提醒一些异常情况，但程序仍然可以执行；
- ERROR(错误):提示错误信息，此类错误会影响程序运行；
- FATAL(严重错误):此类错误将阻止节点继续运行。

------

#### C++

1.节点状态判断

```cpp
/** \brief 检查节点是否已经退出
 *
 *  ros::shutdown() 被调用且执行完毕后，该函数将会返回 false
 *
 * \return true 如果节点还健在, false 如果节点已经火化了。
 */
bool ok();
```

2.节点关闭函数

```cpp
/*
*   关闭节点
*/
void shutdown();
```

3.日志函数

使用示例

```cpp
ROS_DEBUG("hello,DEBUG"); //不会输出
ROS_INFO("hello,INFO"); //默认白色字体
ROS_WARN("Hello,WARN"); //默认黄色字体
ROS_ERROR("hello,ERROR");//默认红色字体
ROS_FATAL("hello,FATAL");//默认红色字体
```



## 3.2 ROS中的头文件与源文件

本节主要介绍ROS的C++实现中，如何使用头文件与源文件的方式封装代码，具体内容如下:

1. 设置头文件，可执行文件作为源文件；
2. 分别设置头文件，源文件与可执行文件。

在ROS中关于头文件的使用，核心内容在于CMakeLists.txt文件的配置，不同的封装方式，配置上也有差异。

### 3.2.1 自定义头文件调用

**需求:**设计头文件，可执行文件本身作为源文件。

**流程:**

1. 编写头文件；
2. 编写可执行文件(同时也是源文件)；
3. 编辑配置文件并执行。

------

#### 1.头文件

在功能包下的 include/功能包名 目录下新建头文件: hello.h，示例内容如下:

```cpp
#ifndef _HELLO_H
#define _HELLO_H

namespace hello_ns{

class HelloPub {

public:
    void run();
};

}

#endif
```

**注意:**

在 VScode 中，为了后续包含头文件时不抛出异常，请配置 .vscode 下 c_cpp_properties.json 的 includepath属性

```
"/home/用户/工作空间/src/功能包/include/**"
```

#### 2.可执行文件

在 src 目录下新建文件:hello.cpp，示例内容如下:

```cpp
#include "ros/ros.h"
#include "test_head/hello.h"

namespace hello_ns {

void HelloPub::run(){
    ROS_INFO("自定义头文件的使用....");
}

}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"test_head_node");
    hello_ns::HelloPub helloPub;
    helloPub.run();
    return 0;
}
```

#### 3.配置文件

配置CMakeLists.txt文件，头文件相关配置如下:

```makefile
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
```

可执行配置文件配置方式与之前一致:

```makefile
add_executable(hello src/hello.cpp)

add_dependencies(hello ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hello
  ${catkin_LIBRARIES}
)
```

最后，编译并执行，控制台可以输出自定义的文本信息。

### 3.2.2 自定义源文件调用

**需求:**设计头文件与源文件，在可执行文件中包含头文件。

**流程:**

1. 编写头文件；
2. 编写源文件；
3. 编写可执行文件；
4. 编辑配置文件并执行。

------

#### 1.头文件

头文件设置于 3.2.1 类似，在功能包下的 include/功能包名 目录下新建头文件: haha.h，示例内容如下:

```cpp
#ifndef _HAHA_H
#define _HAHA_H

namespace hello_ns {

class My {

public:
    void run();

};

}

#endif
```

**注意:**

在 VScode 中，为了后续包含头文件时不抛出异常，请配置 .vscode 下 c_cpp_properties.json 的 includepath属性

```
"/home/用户/工作空间/src/功能包/include/**"
```

#### 2.源文件

在 src 目录下新建文件:haha.cpp，示例内容如下:

```cpp
#include "test_head_src/haha.h"
#include "ros/ros.h"

namespace hello_ns{

void My::run(){
    ROS_INFO("hello,head and src ...");
}

}
```

#### 3.可执行文件

在 src 目录下新建文件: use_head.cpp，示例内容如下:

```cpp
#include "ros/ros.h"
#include "test_head_src/haha.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"hahah");
    hello_ns::My my;
    my.run();
    return 0;
}
```

#### 4.配置文件

头文件与源文件相关配置:

```makefile
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

## 声明C++库
add_library(head
  include/test_head_src/haha.h
  src/haha.cpp
)

add_dependencies(head ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(head
  ${catkin_LIBRARIES}
)
```

可执行文件配置:

```makefile
add_executable(use_head src/use_head.cpp)

add_dependencies(use_head ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#此处需要添加之前设置的 head 库
target_link_libraries(use_head
  head
  ${catkin_LIBRARIES}
)
```