System Requirements - OpenCv, CUDA, Application runs only on Jetson development platforms

Compilation - Run compile.py with "python compile.py" command
	--> This will delete all old compiled files
	--> First compile cuda code and create cuda kernel library
		nvcc -I/usr/local/cuda/include -I. -L/usr/local/cuda/lib64 -c -lineinfo -g -o temp.o  cudaImageTransform.cu
		nvcc -dlink -o cudaImageTransform.o temp.o -lcuda -lcudart
		ar cru libcudaImageTransform.a cudaImageTransform.o temp.o
		ranlib libcudaImageTransform.a
	--> After CUDA compilation is completed, python script invokes OpenCV default compilation process using CMakeLists.txt file

Run the application -
	--> Compilation file creates the executable file in the build folder inside the source code folder
	--> To run the application change your current directory to soource folder and run with "./build/cv_Video_LIDAR_Projection_GPU"