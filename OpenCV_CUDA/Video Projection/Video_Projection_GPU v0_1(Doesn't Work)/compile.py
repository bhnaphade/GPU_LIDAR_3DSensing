#!/usr/bin/python

import subprocess
import os
import sys
import shutil

filename = os.path.abspath(__file__)
#"/home/ubuntu/Desktop/Gest_Detect_GPU1/build"
dir = os.path.dirname(filename)
build = dir + "/build"
print build
if os.path.exists(build):
  shutil.rmtree(build)
os.mkdir(build)

#Image = sys.argv[1]
BUILD4 = "make"

GPUCMD1 = "nvcc -I/usr/local/cuda/include -I. -L/usr/local/cuda/lib64 -c -lineinfo -g -o temp.o  cudaImageTransformation.cu"
GPUCMD2 = "nvcc -dlink -o cudaImageTransformation.o temp.o -lcuda -lcudart"
GPUCMD3 = "ar cru libcudaImageTransformation.a cudaImageTransformation.o temp.o"
GPUCMD4 = "ranlib libcudaImageTransformation.a"

subprocess.call(GPUCMD1,shell=True)
subprocess.call(GPUCMD2,shell=True)
subprocess.call(GPUCMD3,shell=True)
subprocess.call(GPUCMD4,shell=True)

os.chdir(build)
subprocess.call("cmake ..",shell=True)
subprocess.call(BUILD4,shell=True)
