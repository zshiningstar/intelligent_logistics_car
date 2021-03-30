#! /usr/bin/env python
#coding=utf-8

"""
虚拟串口:
用于测试程序发出的串口指令，而无需使用外部设备的串口调试助手
同时可以向测试程序发送串口指令
使用方法：
1. 运行本python脚本将会输出receiver和sender对应的虚拟串口号
2. 使用本机串口调试助手打开sender串口号
3. 修改待测试程序所用的串口号为receiver串口号并启动待测试程序
然后就可以通过本机串口调试助手进行程序调试
"""
#coding=utf-8
from __future__ import print_function
import pty
import os
import sys
import select
from time import sleep
import binascii


def str_to_hex(s):
	#return ' '.join([hex(ord(c)).replace('0x', '') for c in s])
	return ' '.join(['%02X' %(ord(c)) for c in s])

def mkpty():
	master, slave = pty.openpty()
	slave_name = os.ttyname(slave)
	return master, slave_name

def main(argv):
	if(len(argv)>2 and argv[1] == "hex"):
		is_hex = True
	else:
		is_hex = False

	master1,slave_name1 = mkpty()
	print("receiver: %s" %slave_name1)

	master2,slave_name2 = mkpty()
	print("sender: %s" %slave_name2)


	while True:
		readables, writables, errors = select.select([master1,master2], [], [])

		for master in readables:
			data = os.read(master, 128)
			if(master == master1):
#				if(is_hex):
#					print(str_to_hex(data))
#				else:
#					print(data,end='')
#					
#				sys.stdout.flush()
				os.write(master2, data)
			else:
				os.write(master1, data)

if __name__ == "__main__":
	try:
		main(sys.argv)
	except KeyboardInterrupt:
		print("")
		pass
