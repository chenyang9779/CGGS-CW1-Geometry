# Section 3

To compile the code:
```bash
cd code
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```
Alternatively, you can use the `-G` option in `cmake` and create a solution for your chosen C++ IDE (for instance, `-G Xcode`).

All the point cloud files used in the report are located in data-section3. 

Grading is for testing the function, it is the same as grading for seciton 1. 

To use the code (reproducing results in the report), in the build folder:
```bash
./Section3/Section3 128 0.0001 1 0.025 ../data-section3/armadillo
./Section3/Section3 128 0.0001 1 0.025 ../data-section3/quad_163032
./Section3/Section3 128 0.0001 1 0.025 ../data-section3/igea
```

Parameters used for section 3 analysis:
* GridRes: 128 
* Epsilon: 0.00001 
* N: 1 
* h: 0.025