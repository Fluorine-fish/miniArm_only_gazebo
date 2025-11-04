#include "alg_calc/matrix.h"
#include <cstdint>
#include <iostream>
#include <string>

template <int rows, int cols>
void PrintMatrix (Matrixf<rows, cols> m, const std::string &name){
    std::cout << "Matrix " << name << " :\n";
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            float tmp = m[i][j];
            if (fabs(tmp) < 0.0001) tmp = 0.0f; 
            std::cout << tmp << " ";
        }
        std::cout << "\n";
    }
}
