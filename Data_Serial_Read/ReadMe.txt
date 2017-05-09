Purpose: Library to read data from serial port on Jetson TK1. Program will read 8 bytes after sending start to slave device(ardiuno) connected to Jetson on UART.
main.c is a test program for the library.

Below are steps to link library.
1. Compile library files
	gcc -c lib_ldr_data.c -o lib_ldr_data.o

2. Create static library. This step is to bundle multiple object files in one static library (see ar for details). The output of this step is static library.
	ar rcs lib_ldr_data.a lib_ldr_data.o

3. Compile the main program
	gcc -c main.c -o main.o

4. Link the compiled main program to the static library. Note that -L. is used to tell that the static library is in current folder (See this for details of -L and -l options).
	gcc -o main main.o -L. -L_ldr_data

5. Run the main program
./main