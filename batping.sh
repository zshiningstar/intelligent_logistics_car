#!/bin/bash
#用法：sh filename 保存ip的文件名通过第一个参数传入
#文件中每行就是一条ip地址

while read dstip
do
    if ping -c1 $dstip > /dev/null; then
        echo "$dstip ping通"
    else
        echo "$dstip ping不通"
    fi
done < $1

