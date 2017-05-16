Compilation - Run compile.py with "python compile.py" command
	--> This will delete all old compiled files
	--> First compile cuda code and create cuda kernel library
		nvcc -I/usr/local/cuda/include -I. -L/usr/local/cuda/lib64 -c -lineinfo -g -o temp.o  cudaImageTransform.cu
		nvcc -dlink -o cudaImageTransform.o temp.o -lcuda -lcudart
		ar cru libcudaImageTransform.a cudaImageTransform.o temp.o
		ranlib libcudaImageTransform.a
	--> 