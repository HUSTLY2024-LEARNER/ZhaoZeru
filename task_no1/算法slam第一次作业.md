# 算法/slam第一次作业

## 任务一

已安装linux双系统

Ubuntu20.04 

## 任务二 了解Linux文件系统

/ 根目录

/ 下树结构

## 任务三 Linux命令

最常用的命令位于 	/bin or /user/bin 目录下

![img](file:////home/zero/.config/QQ/nt_qq_28f162e074e3b6696b99d325c66fda61/nt_data/Pic/2023-10/Ori/01fec4e9e053eb0da907aa6fee183cc9.png)

mkdir    touch

## 任务四 CMake简单实践

![img](file:////home/zero/.config/QQ/nt_qq_28f162e074e3b6696b99d325c66fda61/nt_data/Pic/2023-10/Ori/7fd9877ab1639b73c0ae2a53902756c4.png)

![img](file:////home/zero/.config/QQ/nt_qq_28f162e074e3b6696b99d325c66fda61/nt_data/Pic/2023-10/Ori/b316f35bb697b7c8b946449200d0eb8c.png)

## 任务五 编译安装OpenCV

配置OpenCV 4.2.0

## 任务六 CMake实践

![img](file:////home/zero/.config/QQ/nt_qq_28f162e074e3b6696b99d325c66fda61/nt_data/Pic/2023-10/Ori/c380313e89d09d068b5343fb368ded2f.png)



# 报错及其处理

## 1.opencv4头文件问题

###### 【opencv4】——fatal error: opencv2/opencv.hpp: No such file or directory  #include ＜opencv2/opencv.hpp＞

经查看，文件真实路径为 

```
/usr/include/opencv4/opencv2/opencv.hpp
```

解决方案：将opencv2 cp 一份到 include文件夹中

![e6eaeb0d6f05535ecba3b8e9776299af](../../.config/QQ/nt_qq_28f162e074e3b6696b99d325c66fda61/nt_data/Pic/2023-10/Ori/e6eaeb0d6f05535ecba3b8e9776299af.png)![96222c68cd646d108ccb3cd7f885904b](../../.config/QQ/nt_qq_28f162e074e3b6696b99d325c66fda61/nt_data/Pic/2023-10/Ori/96222c68cd646d108ccb3cd7f885904b.png)



## 2.opencv版本冲突问题

因为本机装载了两个版本的opencv，文件相互链接，导致库函数调用出错，出现==疑似内存泄漏== 的问题

解决方案：重装opencv  现用版本4.2.0

![img](file:////home/zero/.config/QQ/nt_qq_28f162e074e3b6696b99d325c66fda61/nt_data/Pic/2023-10/Ori/e6eaeb0d6f05535ecba3b8e9776299af.png)



## 3.图片导入路径出错，无法载入图片

源文件路径与本机图片所在路径不同，导致图片无法载入，进而报错

解决方法：进入main.cpp中，修改读取图片的路径即可