#!/bin/sh  
  
exe="smartcar" #发布的程序名称   这是我的可执行程序的名字
des="/home/zsx/logistics_ws/src/ui" #你的路径   这是我的可执行文件的路径 对应的相应的更改
  
deplist=$(ldd $exe | awk  '{if (match($3,"/")){ printf("%s "),$3 } }')  
cp $deplist $des


