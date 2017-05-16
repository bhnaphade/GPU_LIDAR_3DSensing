#!/usr/bin/python

import subprocess
import os
import sys
import shutil

filename = os.path.abspath(__file__)
dir = os.path.dirname(filename)
build = dir + "/build"
print build
if os.path.exists(build):
  shutil.rmtree(build)
os.mkdir(build)

dirContent = os.listdir(dir)
for files in dirContent:
    if files.endswith(".o"):
        os.remove(dir + "/"+ files)
    elif files.endswith(".a"):
        os.remove(dir + "/"+ files)

#Image = sys.argv[1]
BUILD4 = "make"

GPUCMD1 = "nvcc -I/usr/local/cuda/include -I. -L/usr/local/cuda/lib64 -c -lineinfo -g -o temp.o  cudaImageTransform.cu"
GPUCMD2 = "nvcc -dlink -o cudaImageTransform.o temp.o -lcuda -lcudart"
GPUCMD3 = "ar cru libcudaImageTransform.a cudaImageTransform.o temp.o"
GPUCMD4 = "ranlib libcudaImageTransform.a"

LIDARLIBCMD1 = "gcc -c lib_ldr_data.c -o lib_ldr_data.o"
LIDARLIBCMD2 = "ar rcs libLDRData.a lib_ldr_data.o"

subprocess.call(GPUCMD1,shell=True)
subprocess.call(GPUCMD2,shell=True)
subprocess.call(GPUCMD3,shell=True)
subprocess.call(GPUCMD4,shell=True)

subprocess.call(LIDARLIBCMD1,shell=True)
subprocess.call(LIDARLIBCMD2,shell=True)

os.chdir(build)
subprocess.call("cmake ..",shell=True)
subprocess.call(BUILD4,shell=True)
