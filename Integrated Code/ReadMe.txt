System Requirements - OpenCv, CUDA, Application runs only on Jetson development platforms
	Arduino Connected to Jetson TK1 on UART 1 (ttyTHS1)
	Pin Connection - J3A1 Pin 2 GND  - Level Shifer GND
			 J3A1 Pin 3 1.8V - Level Shifter Lo Voltage
			 J3A2 Pin 65 Rx  - Level Shifter one pin corresponding to Arduino Tx 
			 J3A2 Pin 65 Tx  - Level Shifter one pin corresponding to Arduino Rx

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
