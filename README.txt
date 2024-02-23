ubuntu20.04出现Unable to find either executable ‘empy’ or Python module ‘em’… try
问题

1 -- Could NOT find PY_em (missing: PY_EM) 
2 CMake Error at /opt/ros/noetic/share/catkin/cmake/empy.cmake:30 (message):
3   Unable to find either executable 'empy' or Python module 'em'...  try
4   installing the package 'python3-empy'

原因

这是因为catkin找的python版本为anaconda下面的版本，所以需要改为指定采用下面的命令
解决办法

catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
