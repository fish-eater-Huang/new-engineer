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

#ifndef MATRIX_H
#define MATRIX_H

#include "lib/arm_math/arm_math.h"

// Matrix and vector based on arm_math
typedef arm_matrix_instance_f32 Matrix;

// Matrix initialization
#define MatrixInit(pMat, nRows, nCols, pData) \
  arm_mat_init_f32(pMat, nRows, nCols, pData)
// Pay attention to the size of storage space when resize matrix
arm_status MatrixResize(Matrix* mat, uint16_t nRows, uint16_t nCols);

// Matrix calculation
// return: ARM_MATH_SIZE_MISMATCH or ARM_MATH_SUCCESS
#define MatrixAdd(pSrcA, pSrcB, pDst) arm_mat_add_f32(pSrcA, pSrcB, pDst)
#define MatrixSub(pSrcA, pSrcB, pDst) arm_mat_sub_f32(pSrcA, pSrcB, pDst)
#define MatrixScale(pSrc, scale, pDst) arm_mat_scale_f32(pSrc, scale, pDst)
#define MatrixMult(pSrcA, pSrcB, pDst) arm_mat_mult_f32(pSrcA, pSrcB, pDst)
#define MatrixTrans(pSrc, pDst) arm_mat_trans_f32(pSrc, pDst)
// #define MatrixInv(pSrc, pDst) arm_mat_inverse_f32(pSrc, pDst)
// Matrix inverse calculation
// arm_mat_inverse_f32() in arm_math is incorrect!
arm_status MatrixInv(const Matrix* pSrc, Matrix* pDst);

// Vector calculation
#define VectorAdd(pSrcA, pSrcB, pDst, blockSize) \
  arm_add_f32(pSrcA, pSrcB, pDst, blockSize)
#define VectorSub(pSrcA, pSrcB, pDst, blockSize) \
  arm_sub_f32(pSrcA, pSrcB, pDst, blockSize)
#define VectorScale(pSrc, scale, pDst, blockSize) \
  arm_scale_f32(pSrc, scale, pDst, blockSize)
#define VectorAbs(pSrc, pDst, blockSize) arm_abs_f32(pSrc, pDst, blockSize)
#define VectorDot(pSrcA, pSrcB, blockSize, pRes) \
  arm_dot_prod_f32(pSrcA, pSrcB, blockSize, pRes)
#define VectorMult(pSrcA, pSrcB, pDst, blockSize) \
  arm_mult_f32(pSrcA, pSrcB, pDst, blockSize)
#define VectorOffset(pSrc, offset, pDst, blockSize) \
  arm_offset_f32(pSrc, offset, pDst, blockSize)
#define VectorNegate(pSrc, pDst, blockSize) \
  arm_negate_f32(pSrc, pDst, blockSize)
#define VectorCopy(pSrc, pDst, blockSize) arm_copy_f32(pSrc, pDst, blockSize)
#define VectorFill(value, pDst, blockSize) arm_fill_f32(value, pDst, blockSize)
#define VectorPower(pSrc, blockSize, pRes) arm_power_f32(pSrc, blockSize, pRes)

// Statistics
#define VectorMean(pSrc, blockSize, pRes) arm_mean_f32(pSrc, blockSize, pRes)
#define VectorVar(pSrc, blockSize, pRes) arm_var_f32(pSrc, blockSize, pRes)
#define VectorRms(pSrc, blockSize, pRes) arm_rms_f32(pSrc, blockSize, pRes)
#define VectorStd(pSrc, blockSize, pRes) arm_std_f32(pSrc, blockSize, pRes)
#define VectorMin(pSrc, blockSize, pRes, pIndex) \
  arm_min_f32(pSrc, blockSize, pRes, pIndex)
#define VectorMax(pSrc, blockSize, pRes, pIndex) \
  arm_max_f32(pSrc, blockSize, pRes, pIndex)

// Matrix class
template <int _rows, int _cols>
class Matrixf {
 public:
  // Constructor without input data
  Matrixf(void) : rows_(_rows), cols_(_cols) {
    arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_);
  }
  // Constructor with input data
  Matrixf(float data[_rows * _cols]) : Matrixf() {
    memcpy(this->data_, data, _rows * _cols * sizeof(float));
  }
  // Copy constructor
  Matrixf(const Matrixf<_rows, _cols>& mat) : Matrixf() {
    memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
  }
  // Destructor
  ~Matrixf(void) {}

  // Row size
  int rows(void) { return _rows; }
  // Column size
  int cols(void) { return _cols; }

  // Element
  float* operator[](const int& row) { return &this->data_[row * _cols]; }

  // Operators
  Matrixf<_rows, _cols>& operator=(const Matrixf<_rows, _cols> mat) {
    memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
    return *this;
  }
  Matrixf<_rows, _cols>& operator+=(const Matrixf<_rows, _cols> mat) {
    arm_status s;
    s = arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
    return *this;
  }
  Matrixf<_rows, _cols>& operator-=(const Matrixf<_rows, _cols> mat) {
    arm_status s;
    s = arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
    return *this;
  }
  Matrixf<_rows, _cols>& operator*=(const float& val) {
    arm_status s;
    s = arm_mat_scale_f32(&this->arm_mat_, val, &this->arm_mat_);
    return *this;
  }
  Matrixf<_rows, _cols>& operator/=(const float& val) {
    arm_status s;
    s = arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &this->arm_mat_);
    return *this;
  }
  Matrixf<_rows, _cols> operator+(const Matrixf<_rows, _cols>& mat) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
    return res;
  }
  Matrixf<_rows, _cols> operator-(const Matrixf<_rows, _cols>& mat) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
    return res;
  }
  Matrixf<_rows, _cols> operator*(const float& val) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_scale_f32(&this->arm_mat_, val, &res.arm_mat_);
    return res;
  }
  friend Matrixf<_rows, _cols> operator*(const float& val,
                                         const Matrixf<_rows, _cols>& mat) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_scale_f32(&mat.arm_mat_, val, &res.arm_mat_);
    return res;
  }
  Matrixf<_rows, _cols> operator/(const float& val) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &res.arm_mat_);
    return res;
  }
  // Matrix multiplication
  template <int cols>
  friend Matrixf<_rows, cols> operator*(const Matrixf<_rows, _cols>& mat1,
                                        const Matrixf<_cols, cols>& mat2) {
    arm_status s;
    Matrixf<_rows, cols> res;
    s = arm_mat_mult_f32(&mat1.arm_mat_, &mat2.arm_mat_, &res.arm_mat_);
    return res;
  }

  // Submatrix
  template <int rows, int cols>
  Matrixf<rows, cols> block(const int& start_row, const int& start_col) {
    Matrixf<rows, cols> res;
    for (int row = start_row; row < start_row + rows; row++) {
      memcpy((float*)res[0] + (row - start_row) * cols,
             (float*)this->data_ + row * _cols + start_col,
             cols * sizeof(float));
    }
    return res;
  }
  // Specific row
  Matrixf<1, _cols> row(const int& row) { return block<1, _cols>(row, 0); }
  // Specific column
  Matrixf<_rows, 1> col(const int& col) { return block<_rows, 1>(0, col); }

  // Transpose
  Matrixf<_cols, _rows> trans(void) {
    Matrixf<_cols, _rows> res;
    arm_mat_trans_f32(&arm_mat_, &res.arm_mat_);
    return res;
  }
  // Trace
  float trace(void) {
    float res = 0;
    for (int i = 0; i < fmin(_rows, _cols); i++) {
      res += *this[i][i];
    }
    return res;
  }
  // Norm
  float norm(void) { return sqrtf((this->trans() * *this)[0][0]); }

 public:
  // arm matrix instance
  arm_matrix_instance_f32 arm_mat_;

 protected:
  // size
  int rows_, cols_;
  // data
  float data_[_rows * _cols];
};

// Matrix funtions
namespace matrixf {

// Special Matrices
// Zero matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> zeros(void) {
  float data[_rows * _cols] = {0};
  return Matrixf<_rows, _cols>(data);
}
// Ones matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> ones(void) {
  float data[_rows * _cols] = {0};
  for (int i = 0; i < _rows * _cols; i++) {
    data[i] = 1;
  }
  return Matrixf<_rows, _cols>(data);
}
// Identity matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> eye(void) {
  float data[_rows * _cols] = {0};
  for (int i = 0; i < fmin(_rows, _cols); i++) {
    data[i * _cols + i] = 1;
  }
  return Matrixf<_rows, _cols>(data);
}
// Diagonal matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> diag(Matrixf<_rows, 1> vec) {
  Matrixf<_rows, _cols> res = matrixf::zeros<_rows, _cols>();
  for (int i = 0; i < fmin(_rows, _cols); i++) {
    res[i][i] = vec[i][0];
  }
  return res;
}

// Inverse
template <int _dim>
Matrixf<_dim, _dim> inv(Matrixf<_dim, _dim> mat) {
  arm_status s;
  // extended matrix [A|I]
  Matrixf<_dim, 2 * _dim> ext_mat = matrixf::zeros<_dim, 2 * _dim>();
  for (int i = 0; i < _dim; i++) {
    memcpy(ext_mat[i], mat[i], _dim * sizeof(float));
    ext_mat[i][_dim + i] = 1;
  }
  // elimination
  for (int i = 0; i < _dim; i++) {
    // find maximum absolute value in the first column in lower right block
    float abs_max = fabs(ext_mat[i][i]);
    int abs_max_row = i;
    for (int row = i; row < _dim; row++) {
      if (abs_max < fabs(ext_mat[row][i])) {
        abs_max = fabs(ext_mat[row][i]);
        abs_max_row = row;
      }
    }
    if (abs_max < 1e-12f) {  // singular
      return matrixf::zeros<_dim, _dim>();
      s = ARM_MATH_SINGULAR;
    }
    if (abs_max_row != i) {  // row exchange
      float tmp;
      Matrixf<1, 2 * _dim> row_i = ext_mat.row(i);
      Matrixf<1, 2 * _dim> row_abs_max = ext_mat.row(abs_max_row);
      memcpy(ext_mat[i], row_abs_max[0], 2 * _dim * sizeof(float));
      memcpy(ext_mat[abs_max_row], row_i[0], 2 * _dim * sizeof(float));
    }
    float k = 1.f / ext_mat[i][i];
    for (int col = i; col < 2 * _dim; col++) {
      ext_mat[i][col] *= k;
    }
    for (int row = 0; row < _dim; row++) {
      if (row == i) {
        continue;
      }
      k = ext_mat[row][i];
      for (int j = i; j < 2 * _dim; j++) {
        ext_mat[row][j] -= k * ext_mat[i][j];
      }
    }
  }
  // inv = ext_mat(:,n+1:2n)
  s = ARM_MATH_SUCCESS;
  Matrixf<_dim, _dim> res;
  for (int i = 0; i < _dim; i++) {
    memcpy(res[i], &ext_mat[i][_dim], _dim * sizeof(float));
  }
  return res;
}

}  // namespace matrixf

namespace vector3f {

// hat of vector
Matrixf<3, 3> hat(Matrixf<3, 1> vec);

// cross product
Matrixf<3, 1> cross(Matrixf<3, 1> vec1, Matrixf<3, 1> vec2);

}  // namespace vector3f

#endif  // MATRIX_H
