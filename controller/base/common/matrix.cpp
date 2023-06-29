/**
 ******************************************************************************
 * @file    matrix.cpp/h
 * @brief   Matrix/vector calculation. 矩阵/向量运算
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "matrix.h"

// Pay attention to the size of storage space
arm_status MatrixResize(Matrix* mat, uint16_t nRows, uint16_t nCols) {
  mat->numRows = nRows;
  mat->numCols = nCols;
  return ARM_MATH_SUCCESS;
}

// Matrix inverse calculation (Gaussian elimination)
arm_status MatrixInv(const Matrix* pSrc, Matrix* pDst) {
  // Check matrix size
  if (pSrc->numRows != pDst->numRows || pSrc->numCols != pDst->numCols ||
      pSrc->numRows != pSrc->numCols) {
    return ARM_MATH_SIZE_MISMATCH;
  }
  int n = pSrc->numRows;

  // extended matrix [A|I]
  Matrix ext_mat;
  float* ext_mat_data = new float[2 * n * n];
  MatrixInit(&ext_mat, n, 2 * n, ext_mat_data);
  VectorFill(0, ext_mat_data, 2 * n * n);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      ext_mat_data[2 * n * i + j] = *(pSrc->pData + i * n + j);
    }
    ext_mat_data[2 * n * i + n + i] = 1;
  }

  // elimination
  for (int i = 0; i < n; i++) {
    // find maximum absolute value in the first column in lower right block
    float abs_max = fabs(ext_mat_data[2 * n * i + i]);
    int abs_max_row = i;
    for (int row = i; row < n; row++) {
      if (abs_max < fabs(ext_mat_data[2 * n * row + i])) {
        abs_max = fabs(ext_mat_data[2 * n * row + i]);
        abs_max_row = row;
      }
    }
    if (abs_max < 1e-12f) {  // singular
      delete[] ext_mat_data;
      return ARM_MATH_SINGULAR;
    }
    if (abs_max_row != i) {  // row exchange
      float tmp;
      for (int j = i; j < 2 * n; j++) {
        tmp = ext_mat_data[2 * n * i + j];
        ext_mat_data[2 * n * i + j] = ext_mat_data[2 * n * abs_max_row + j];
        ext_mat_data[2 * n * abs_max_row + j] = tmp;
      }
    }
    float k = 1.f / ext_mat_data[2 * n * i + i];
    for (int col = i; col < 2 * n; col++) {
      ext_mat_data[2 * n * i + col] *= k;
    }
    for (int row = 0; row < n; row++) {
      if (row == i) {
        continue;
      }
      k = ext_mat_data[2 * n * row + i];
      for (int j = i; j < 2 * n; j++) {
        ext_mat_data[2 * n * row + j] -= k * ext_mat_data[2 * n * i + j];
      }
    }
  }

  // inv = ext_mat(:,n+1:2n)
  for (int row = 0; row < n; row++) {
    memcpy((float*)pDst->pData + n * row, &ext_mat_data[2 * n * row + n],
           n * sizeof(float));
  }

  delete[] ext_mat_data;
  return ARM_MATH_SUCCESS;
}

// hat of vector
Matrixf<3, 3> vector3f::hat(Matrixf<3, 1> vec) {
  float hat[9] = {0,          -vec[2][0], vec[1][0], vec[2][0], 0,
                  -vec[0][0], -vec[1][0], vec[0][0], 0};
  return Matrixf<3, 3>(hat);
}

// cross product
Matrixf<3, 1> vector3f::cross(Matrixf<3, 1> vec1, Matrixf<3, 1> vec2) {
  return vector3f::hat(vec1) * vec2;
}
