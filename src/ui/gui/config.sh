#!/bin/sh  
  
exe="smartcar" #发布的程序名称   这是我的可执行程序的名字
des="/home/logistics_ws/src/ui/gui/lib/" #这个是用来存放依赖库的位置
  
deplist=$(ldd $exe | awk  '{if (match($3,"/")){ printf("%s "),$3 } }')  
cp $deplist $des
