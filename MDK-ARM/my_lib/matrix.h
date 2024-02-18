
/**
 ******************************************************************************
 * @file    matrix.cpp/h
 * @brief   Matrix/vector calculation. 鐭╅樀/鍚戦噺杩愮畻
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef MATRIX_H
#define MATRIX_H

#include "arm_math.h"

// Matrix class
template <int _rows, int _cols>//模板是行,列,在创建对象的时候必须给定
class Matrixf {
 public://构造函数
  // Constructor without input data构建一个没有数据的矩阵
  Matrixf(void) : rows_(_rows), cols_(_cols) {
    //调用arm_math库,构建矩阵,传入数据
    arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_);
  }
  // Constructor with input data
  //         把一个数组的数据存进矩阵   这里调用了上面那个构造函数
  Matrixf(float data[_rows * _cols]) : Matrixf() {
    //把一个长的数组存进去
    memcpy(this->data_, data, _rows * _cols * sizeof(float));
  }
  // Copy constructor
  //用一个矩阵给这个矩阵赋值           取对象的地址
  Matrixf(const Matrixf<_rows, _cols>& mat) : Matrixf() {
    memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
  }
  // Destructor虚函数,在对象消失的时候被调用
  ~Matrixf(void) {}

  // Row size
  int rows(void) { return _rows; }
  // Column size
  int cols(void) { return _cols; }

  // Element矩阵的单个元素
  //重载了[]操作符,转换成返回这一行的起始位置
  float* operator[](const int& row) { return &this->data_[row * _cols]; }

  // Operators重载
  //重载=运算,复制矩阵
  Matrixf<_rows, _cols>& operator=(const Matrixf<_rows, _cols> mat) {
    memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
    return *this;
  }
  //重载+=运算,矩阵加法
  Matrixf<_rows, _cols>& operator+=(const Matrixf<_rows, _cols> mat) {
    arm_status s;
    //                          这个矩阵    加的矩阵         和存储在这个矩阵
    s = arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
    return *this;
  }
  //同理,减法
  Matrixf<_rows, _cols>& operator-=(const Matrixf<_rows, _cols> mat) {
    arm_status s;
    s = arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
    return *this;
  }
  //乘一个数
  Matrixf<_rows, _cols>& operator*=(const float& val) {
    arm_status s;
    s = arm_mat_scale_f32(&this->arm_mat_, val, &this->arm_mat_);
    return *this;
  }
  //除以一个数
  Matrixf<_rows, _cols>& operator/=(const float& val) {
    arm_status s;
    s = arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &this->arm_mat_);
    return *this;
  }
  //加法
  Matrixf<_rows, _cols> operator+(const Matrixf<_rows, _cols>& mat) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
    return res;
  }
  //减
  Matrixf<_rows, _cols> operator-(const Matrixf<_rows, _cols>& mat) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
    return res;
  }
  //数乘
  Matrixf<_rows, _cols> operator*(const float& val) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_scale_f32(&this->arm_mat_, val, &res.arm_mat_);
    return res;
  }
  //数乘满足交换律
  friend Matrixf<_rows, _cols> operator*(const float& val,
                                         const Matrixf<_rows, _cols>& mat) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_scale_f32(&mat.arm_mat_, val, &res.arm_mat_);
    return res;
  }
  //除法
  Matrixf<_rows, _cols> operator/(const float& val) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &res.arm_mat_);
    return res;
  }
  // Matrix multiplication矩阵乘法,这里friend跟矩阵乘一个数是freind
  template <int cols>//这里指定列数
  friend Matrixf<_rows, cols> operator*(const Matrixf<_rows, _cols>& mat1,
                                        const Matrixf<_cols, cols>& mat2) {
    arm_status s;
    Matrixf<_rows, cols> res;
    s = arm_mat_mult_f32(&mat1.arm_mat_, &mat2.arm_mat_, &res.arm_mat_);
    return res;
  }

  // Submatrix构造子矩阵,即把一个矩阵分割出来一小块
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
  // Specific row拿出特定行
  Matrixf<1, _cols> row(const int& row) { return block<1, _cols>(row, 0); }
  // Specific column特定列
  Matrixf<_rows, 1> col(const int& col) { return block<_rows, 1>(0, col); }

  // Transpose转置
  //     这里交换了行与列
  Matrixf<_cols, _rows> trans(void) {
    Matrixf<_cols, _rows> res;
    //                   对象        存储对象
    arm_mat_trans_f32(&arm_mat_, &res.arm_mat_);
    return res;
  }
  // Trace迹   对角线上元素的和
  float trace(void) {
    float res = 0;
    for (int i = 0; i < fmin(_rows, _cols); i++) {
      res += (*this)[i][i];
    }
    return res;
  }
  // Norm  2-范数  用来优化整体稳定性的
  //用转置乘以原矩阵得平方数,然后求和 ,再开方    第一个*是乘 第二个*是指针,指向自己
  float norm(void) { return sqrtf((this->trans() * *this)[0][0]); }

 public:
  // arm matrix instance实例化矩阵
  arm_matrix_instance_f32 arm_mat_;

 protected:
  // size
  int rows_, cols_;
  // data
  float data_[_rows * _cols];
};

// Matrix funtions矩阵运算的函数
namespace matrixf {

// Special Matrices特殊的矩阵
// Zero matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> zeros(void) {
  float data[_rows * _cols] = {0};
  return Matrixf<_rows, _cols>(data);
}
// Ones matrix全是1
template <int _rows, int _cols>
Matrixf<_rows, _cols> ones(void) {
  float data[_rows * _cols] = {0};
  for (int i = 0; i < _rows * _cols; i++) {
    data[i] = 1;
  }
  return Matrixf<_rows, _cols>(data);
}
// Identity matrix单位矩阵
template <int _rows, int _cols>
Matrixf<_rows, _cols> eye(void) {
  float data[_rows * _cols] = {0};
  for (int i = 0; i < fmin(_rows, _cols); i++) {
    data[i * _cols + i] = 1;
  }
  return Matrixf<_rows, _cols>(data);
}
// Diagonal matrix
//转换成对角矩阵
template <int _rows, int _cols>
//                                这是一个列向量,然后把他转换成行*行的对角矩阵
Matrixf<_rows, _cols> diag(Matrixf<_rows, 1> vec) {
  Matrixf<_rows, _cols> res = matrixf::zeros<_rows, _cols>();
  for (int i = 0; i < fmin(_rows, _cols); i++) {
    res[i][i] = vec[i][0];
  }
  return res;
}

// Inverse求逆,好吊啊
template <int _dim>
Matrixf<_dim, _dim> inv(Matrixf<_dim, _dim> mat) {
  arm_status s;
  // extended matrix [A|I]向下延长两倍矩阵
  Matrixf<_dim, 2 * _dim> ext_mat = matrixf::zeros<_dim, 2 * _dim>();
  for (int i = 0; i < _dim; i++) {
    memcpy(ext_mat[i], mat[i], _dim * sizeof(float));//复制内容,并且将对角线的内容全部赋值1
    ext_mat[i][_dim + i] = 1;
  }

  // elimination高斯消元法
  for (int i = 0; i < _dim; i++) {
    // find maximum absolute value in the first column in lower right block
    float abs_max = fabs(ext_mat[i][i]);//绝对值
    int abs_max_row = i;
    for (int row = i; row < _dim; row++) {
      if (abs_max < fabs(ext_mat[row][i])) {
        abs_max = fabs(ext_mat[row][i]);
        abs_max_row = row;
      }
    }
    if (abs_max < 1e-12f) {  // singular奇异状态给赋值0矩阵
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
  Matrixf<_dim, _dim> res;//最终赋值输出
  for (int i = 0; i < _dim; i++) {
    memcpy(res[i], &ext_mat[i][_dim], _dim * sizeof(float));
  }
  return res;
}

}  // namespace matrixf
//向量运算
namespace vector3f {

// hat of vector
Matrixf<3, 3> hat(Matrixf<3, 1> vec);

// cross product向量叉积
Matrixf<3, 1> cross(Matrixf<3, 1> vec1, Matrixf<3, 1> vec2);

}  // namespace vector3f

#endif  // MATRIX_H

