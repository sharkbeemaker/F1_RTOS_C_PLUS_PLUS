
/**
 ******************************************************************************
 * @file    robotics.cpp/h
 * @brief   Robotic toolbox on STM32. STM32������ѧ��
 * @author  Spoon Guan
 * @ref     [1] SJTU ME385-2, Robotics, Y.Ding
 *          [2] Bruno Siciliano, et al., Robotics: Modelling, Planning and
 *              Control, Springer, 2010.
 *          [3] R.Murry, Z.X.Li, and S.Sastry, A Mathematical Introduction
 *              to Robotic Manipulation, CRC Press, 1994.
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "robotics.h"
//����ת����ת����ŷ���Ǳ�ʾ
Matrixf<3, 1> robotics::r2rpy(Matrixf<3, 3> R) {
  float rpy[3] = {
      atan2f(R[1][0], R[0][0]),                                        // yaw
      atan2f(-R[2][0], sqrtf(R[2][1] * R[2][1] + R[2][2] * R[2][2])),  // pitch
      atan2f(R[2][1], R[2][2])                                         // roll
  };
  return Matrixf<3, 1>(rpy);
}
//��ŷ����ת������ת����
Matrixf<3, 3> robotics::rpy2r(Matrixf<3, 1> rpy) {
  float c[3] = {cosf(rpy[0][0]), cosf(rpy[1][0]), cosf(rpy[2][0])};
  float s[3] = {sinf(rpy[0][0]), sinf(rpy[1][0]), sinf(rpy[2][0])};
  float R[9] = {
      c[0] * c[1],                       // R11
      c[0] * s[1] * s[2] - s[0] * c[2],  // R12
      c[0] * s[1] * c[2] + s[0] * s[2],  // R13
      s[0] * c[1],                       // R21
      s[0] * s[1] * s[2] + c[0] * c[2],  // R22
      s[0] * s[1] * c[2] - c[0] * s[2],  // R23
      -s[1],                             // R31
      c[1] * s[2],                       // R32
      c[1] * c[2]                        // R33
  };
  return Matrixf<3, 3>(R);
}
//������ת,����ת����ת����������ת�Ƚ�
Matrixf<4, 1> robotics::r2angvec(Matrixf<3, 3> R) {
  float theta = acosf(math::limit(0.5f * (R.trace() - 1), -1, 1));
  if (theta == 0 || theta == PI) {
    float angvec[4] = {//���ذ���ת�����������Լ�ת��
        (1 + R[0][0] - R[1][1] - R[2][2]) / 4.0f,  // rx=(1+R11-R22-R33)/4
        (1 - R[0][0] + R[1][1] - R[2][2]) / 4.0f,  // ry=(1-R11+R22-R33)/4
        (1 - R[0][0] - R[1][1] + R[2][2]) / 4.0f,  // rz=(1-R11-R22+R33)/4
        theta                                      // theta
    };
    return Matrixf<4, 1>(angvec);
  }
  float angvec[4] = {
      (R[2][1] - R[1][2]) / (2.0f * sinf(theta)),  // rx=(R32-R23)/2sin��
      (R[0][2] - R[2][0]) / (2.0f * sinf(theta)),  // ry=(R13-R31)/2sin��
      (R[1][0] - R[0][1]) / (2.0f * sinf(theta)),  // rz=(R21-R12)/2sin��
      theta                                        // theta
  };
  return Matrixf<4, 1>(angvec);
}
//������ת,��������ת�Ƚ�ת������ת����
Matrixf<3, 3> robotics::angvec2r(Matrixf<4, 1> angvec) {
  float theta = angvec[3][0];
  Matrixf<3, 1> r = angvec.block<3, 1>(0, 0);
  Matrixf<3, 3> R;
  // Rodrigues formula: R=I+��^sin��+��^^2(1-cos��)
  R = matrixf::eye<3, 3>() + vector3f::hat(r) * sinf(theta) +
      vector3f::hat(r) * vector3f::hat(r) * (1 - cosf(theta));
  return R;
}
//����ת����ת������Ԫ��
Matrixf<4, 1> robotics::r2quat(Matrixf<3, 3> R) {
  float q[4] = {
      0.5f * sqrtf(math::limit(R.trace(), -1, 1) + 1),  // q0=cos(��/2)ʵ��
      math::sign(R[2][1] - R[1][2]) * 0.5f *
          sqrtf(math::limit(R[0][0] - R[1][1] - R[2][2], -1, 1) +
                1),  // q1=rx*sin(��/2)    X�����
      math::sign(R[0][2] - R[2][0]) * 0.5f *
          sqrtf(math::limit(-R[0][0] + R[1][1] - R[2][2], -1, 1) +
                1),  // q2=ry*sin(��/2)    Y�����
      math::sign(R[1][0] - R[0][1]) * 0.5f *
          sqrtf(math::limit(-R[0][0] - R[1][1] + R[2][2], -1, 1) +
                1),  // q3=rz*sin(��/2)    Z�����
  };
  return (Matrixf<4, 1>(q) / Matrixf<4, 1>(q).norm());//��������ķ����Թ�һ����Ԫ��
}
//��Ԫ��ת������ת����
Matrixf<3, 3> robotics::quat2r(Matrixf<4, 1> q) {
  float R[9] = {
      1 - 2.0f * (q[2][0] * q[2][0] + q[3][0] * q[3][0]),  // R11
      2.0f * (q[1][0] * q[2][0] - q[0][0] * q[3][0]),      // R12
      2.0f * (q[1][0] * q[3][0] + q[0][0] * q[2][0]),      // R13
      2.0f * (q[1][0] * q[2][0] + q[0][0] * q[3][0]),      // R21
      1 - 2.0f * (q[1][0] * q[1][0] + q[3][0] * q[3][0]),  // R22
      2.0f * (q[2][0] * q[3][0] - q[0][0] * q[1][0]),      // R23
      2.0f * (q[1][0] * q[3][0] - q[0][0] * q[2][0]),      // R31
      2.0f * (q[2][0] * q[3][0] + q[0][0] * q[1][0]),      // R32
      1 - 2.0f * (q[1][0] * q[1][0] + q[2][0] * q[2][0])   // R33
  };
  return Matrixf<3, 3>(R);
}
//����Ԫ��ת����RPY�Ƕ�
Matrixf<3, 1> robotics::quat2rpy(Matrixf<4, 1> q) {
  float rpy[3] = {
      atan2f(2.0f * (q[1][0] * q[2][0] + q[0][0] * q[3][0]),
             1 - 2.0f * (q[2][0] * q[2][0] + q[3][0] * q[3][0])),  // yaw
      asinf(2.0f * (q[0][0] * q[2][0] - q[1][0] * q[3][0])),       // pitch
      atan2f(2.0f * (q[2][0] * q[3][0] + q[0][0] * q[1][0]),
             1 - 2.0f * (q[1][0] * q[1][0] + q[2][0] * q[2][0]))  // rol
  };
  return Matrixf<3, 1>(rpy);
}
//��RPY�Ƕ�ת������Ԫ��
Matrixf<4, 1> robotics::rpy2quat(Matrixf<3, 1> rpy) {
  float c[3] = {cosf(0.5f * rpy[0][0]), cosf(0.5f * rpy[1][0]),
                cosf(0.5f * rpy[2][0])};  // cos(*/2)
  float s[3] = {sinf(0.5f * rpy[0][0]), sinf(0.5f * rpy[1][0]),
                sinf(0.5f * rpy[2][0])};  // sin(*/2)
  float q[4] = {
      c[0] * c[1] * c[2] + s[0] * s[1] * s[2],  // q0=cos(��/2)
      c[0] * c[1] * s[2] - s[0] * s[1] * c[2],  // q1=rx*sin(��/2)
      c[0] * s[1] * c[2] + s[0] * c[1] * s[2],  // q2=ry*sin(��/2)
      s[0] * c[1] * c[2] - c[0] * s[1] * s[2]   // q3=rz*sin(��/2)
  };
  return (Matrixf<4, 1>(q) / Matrixf<4, 1>(q).norm());
}
//��Ԫ��ת�������
Matrixf<4, 1> robotics::quat2angvec(Matrixf<4, 1> q) {
  float cosq0;
  float theta = 2.0f * acosf(math::limit(q[0][0], -1, 1));
  if (theta == 0 || theta == PI) {
    float angvec[4] = {0, 0, 0, theta};  // ��=0||PI, return[0;��]
    return Matrixf<4, 1>(angvec);
  }
  Matrixf<3, 1> vec = q.block<3, 1>(1, 0);
  float angvec[4] = {
      vec[0][0] / vec.norm(),  // rx
      vec[1][0] / vec.norm(),  // ry
      vec[2][0] / vec.norm(),  // rz
      theta                    // theta
  };
  return Matrixf<4, 1>(angvec);
}
//���ת������Ԫ��
Matrixf<4, 1> robotics::angvec2quat(Matrixf<4, 1> angvec) {
  float c = cosf(0.5f * angvec[3][0]), s = sinf(0.5f * angvec[3][0]);
  float q[4] = {
      c,                 // q0=cos(��/2)
      s * angvec[0][0],  // q1=rx*sin(��/2)
      s * angvec[1][0],  // q2=ry*sin(��/2)
      s * angvec[2][0]   // q3=rz*sin(��/2)
  };
  return Matrixf<4, 1>(q) / Matrixf<4, 1>(q).norm();
}
//����תƽ�Ƶ�4*4����ת������ת����3*3
Matrixf<3, 3> robotics::t2r(Matrixf<4, 4> T) {
  return T.block<3, 3>(0, 0);  // R=T(1:3,1:3)
}
//��3*3����ת����ת����4*4����תƽ�ƾ���,��ʱ��ƽ������0
Matrixf<4, 4> robotics::r2t(Matrixf<3, 3> R) {
  // T=[R,0;0,1]
  float T[16] = {R[0][0], R[0][1], R[0][2], 0, R[1][0], R[1][1], R[1][2], 0,
                 R[2][0], R[2][1], R[2][2], 0, 0,       0,       0,       1};
  return Matrixf<4, 4>(T);
}
//ȡ��תƽ�ƾ����ƽ����
Matrixf<3, 1> robotics::t2p(Matrixf<4, 4> T) {
  return T.block<3, 1>(0, 3);  // p=T(1:3,4)
}
/*     ��ת      ƽ��
      1 0 0      P
      0 1 0      P
 T =  0 0 1      P
      0 0 0      1
*/
//��ƽ�ƾ��������4*4��ת�任����
Matrixf<4, 4> robotics::p2t(Matrixf<3, 1> p) {
  // T=[I,P;0,1]
  float T[16] = {1, 0, 0, p[0][0], 0, 1, 0, p[1][0],
                 0, 0, 1, p[2][0], 0, 0, 0, 1};
  return Matrixf<4, 4>(T);
}
//����ת�����ƽ�ƾ���ƴ�ӳ�һ��4*4����ת�任����
Matrixf<4, 4> robotics::rp2t(Matrixf<3, 3> R, Matrixf<3, 1> p) {
  // T=[R,P;0,1]
  float T[16] = {R[0][0], R[0][1], R[0][2], p[0][0], R[1][0], R[1][1],
                 R[1][2], p[1][0], R[2][0], R[2][1], R[2][2], p[2][0],
                 0,       0,       0,       1};
  return Matrixf<4, 4>(T);
}
//��תƽ�ƾ��� ��任
Matrixf<4, 4> robotics::invT(Matrixf<4, 4> T) {
  Matrixf<3, 3> RT = t2r(T).trans();//ת����ת����(��任)
  Matrixf<3, 1> p_ = -1.0f * RT * t2p(T);//ƽ������ȡ��,����ת�ú����ת�������,�����任��ƽ��
  float invT[16] = {RT[0][0], RT[0][1], RT[0][2], p_[0][0], RT[1][0], RT[1][1],
                    RT[1][2], p_[1][0], RT[2][0], RT[2][1], RT[2][2], p_[2][0],
                    0,        0,        0,        1};
  return Matrixf<4, 4>(invT);
}
//��תƽ�ƾ���ת����RPY
Matrixf<3, 1> robotics::t2rpy(Matrixf<4, 4> T) {
  return r2rpy(t2r(T));
}
//RPYת������תƽ��
Matrixf<4, 4> robotics::rpy2t(Matrixf<3, 1> rpy) {
  return r2t(rpy2r(rpy));
}
//��תƽ��ת�������
Matrixf<4, 1> robotics::t2angvec(Matrixf<4, 4> T) {
  return r2angvec(t2r(T));
}
//���ת������תƽ��
Matrixf<4, 4> robotics::angvec2t(Matrixf<4, 1> angvec) {
  return r2t(angvec2r(angvec));
}

Matrixf<4, 1> robotics::t2quat(Matrixf<4, 4> T) {
  return r2quat(t2r(T));
}

Matrixf<4, 4> robotics::quat2t(Matrixf<4, 1> quat) {
  return r2t(quat2r(quat));
}
//��ƽ�Ƹ�X�������ת����ƴ����,�γ�Ťתʸ��
Matrixf<6, 1> robotics::t2twist(Matrixf<4, 4> T) {
  Matrixf<3, 1> p = t2p(T);
  Matrixf<4, 1> angvec = t2angvec(T);
  Matrixf<3, 1> phi = angvec.block<3, 1>(0, 0) * angvec[3][0];
  float twist[6] = {p[0][0], p[1][0], p[2][0], phi[0][0], phi[1][0], phi[2][0]};
  return Matrixf<6, 1>(twist);
}

Matrixf<4, 4> robotics::twist2t(Matrixf<6, 1> twist) {
  Matrixf<3, 1> p = twist.block<3, 1>(0, 0);
  float theta = twist.block<3, 1>(3, 0).norm();
  float angvec[4] = {0, 0, 0, theta};
  if (theta != 0) {
    angvec[0] = twist[3][0] / theta;
    angvec[1] = twist[4][0] / theta;
    angvec[2] = twist[5][0] / theta;
  }
  return rp2t(angvec2r(angvec), p);
}
//�˶�ѧ����
Matrixf<4, 4> robotics::DH_t::fkine() {
  float ct = cosf(theta), st = sinf(theta);  // cos��, sin��
  float ca = cosf(alpha), sa = sinf(alpha);  // cos��, sin��

  // T =
  // | c��  -s��c��   s��s��   ac�� |
  // | s��   c��c��  -c��s��   as�� |
  // |  0     s��     c��     d |
  // |  0      0      0     1 |
  T[0][0] = ct;
  T[0][1] = -st * ca;
  T[0][2] = st * sa;
  T[0][3] = a * ct;

  T[1][0] = st;
  T[1][1] = ct * ca;
  T[1][2] = -ct * sa;
  T[1][3] = a * st;

  T[2][0] = 0;
  T[2][1] = sa;
  T[2][2] = ca;
  T[2][3] = d;

  T[3][0] = 0;
  T[3][1] = 0;
  T[3][2] = 0;
  T[3][3] = 1;

  return T;
}

/// @brief �ؽڶ���
/// @param theta ����н�
/// @param d ����ƽ����
/// @param a ���
/// @param alpha �ؽڼн�
/// @param type ����(��ת/ƽ��)
/// @param offset ����
/// @param qmin ��С����
/// @param qmax �������
/// @param m ����
/// @param rc ��������
/// @param I ���Ծ���
robotics::Link::Link(float theta,
                     float d,
                     float a,
                     float alpha,
                     robotics::Joint_Type_e type,
                     float offset,
                     float qmin,
                     float qmax,
                     float m,
                     Matrixf<3, 1> rc,
                     Matrixf<3, 3> I) {
  dh_.theta = theta;
  dh_.d = d;
  dh_.alpha = alpha;
  dh_.a = a;
  type_ = type;
  offset_ = offset;
  qmin_ = qmin;
  qmax_ = qmax;
  m_ = m;
  rc_ = rc;
  I_ = I;
}
//ֱ���ùؽ�����ֵ
robotics::Link::Link(const Link& link) {
  dh_.theta = link.dh_.theta;
  dh_.d = link.dh_.d;
  dh_.alpha = link.dh_.alpha;
  dh_.a = link.dh_.a;
  type_ = link.type_;
  offset_ = link.offset_;
  qmin_ = link.qmin_;
  qmax_ = link.qmax_;
  m_ = link.m_;
  rc_ = link.rc_;
  I_ = link.I_;
}
//����=�����
robotics::Link& robotics::Link::operator=(Link link) {
  dh_.theta = link.dh_.theta;
  dh_.d = link.dh_.d;
  dh_.alpha = link.dh_.alpha;
  dh_.a = link.dh_.a;
  type_ = link.type_;
  offset_ = link.offset_;
  qmin_ = link.qmin_;
  qmax_ = link.qmax_;
  m_ = link.m_;
  rc_ = link.rc_;
  I_ = link.I_;
  return *this;
}
/// @brief ��ת/ƽ��  q����/�Ƕ�
/// @param q
/// @return ��תƽ�ƾ���(�˶�ѧ����)
Matrixf<4, 4> robotics::Link::T(float q) {
  if (type_ == R)//��ת����
 {
    if (qmin_ >= qmax_)
      dh_.theta = q + offset_;
    else
      dh_.theta = math::limit(q + offset_, qmin_, qmax_);
  }
  else //ƽ��
  {
    if (qmin_ >= qmax_)
      dh_.d = q + offset_;
    else
      dh_.d = math::limit(q + offset_, qmin_, qmax_);
  }

  return dh_.fkine();
}

