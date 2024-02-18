/**
 ******************************************************************************
 * @file    robotics.cpp/h
 * @brief   Robotic toolbox on STM32. STM32机器人学�
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

#ifndef ROBOTICS_H
#define ROBOTICS_H

#include "utils.h"
#include "matrix.h"

namespace robotics
{
  // rotation matrix(R) -> RPY([yaw;pitch;roll])
  Matrixf<3, 1> r2rpy(Matrixf<3, 3> R);
  // RPY([yaw;pitch;roll]) -> rotation matrix(R)
  Matrixf<3, 3> rpy2r(Matrixf<3, 1> rpy);
  // rotation matrix(R) -> angle vector([r;θ])
  Matrixf<4, 1> r2angvec(Matrixf<3, 3> R);
  // angle vector([r;θ]) -> rotation matrix(R)
  Matrixf<3, 3> angvec2r(Matrixf<4, 1> angvec);
  // rotation matrix(R) -> quaternion, [q0;q1;q2;q3]=[cos(θ/2);rsin(θ/2)]
  Matrixf<4, 1> r2quat(Matrixf<3, 3> R);
  // quaternion, [q0;q1;q2;q3]=[cos(θ/2);rsin(θ/2)] -> rotation matrix(R)
  Matrixf<3, 3> quat2r(Matrixf<4, 1> quat);
  // quaternion, [q0;q1;q2;q3]=[cos(θ/2);rsin(θ/2)] -> RPY([yaw;pitch;roll])
  Matrixf<3, 1> quat2rpy(Matrixf<4, 1> q);
  // RPY([yaw;pitch;roll]) -> quaternion, [q0;q1;q2;q3]=[cos(θ/2);rsin(θ/2)]
  Matrixf<4, 1> rpy2quat(Matrixf<3, 1> rpy);
  // quaternion, [q0;q1;q2;q3]=[cos(θ/2);rsin(θ/2)] -> angle vector([r;θ])
  Matrixf<4, 1> quat2angvec(Matrixf<4, 1> q);
  // angle vector([r;θ]) -> quaternion, [q0;q1;q2;q3]=[cos(θ/2);rsin(θ/2)]
  Matrixf<4, 1> angvec2quat(Matrixf<4, 1> angvec);
  // homogeneous transformation matrix(T) -> rotation matrix(R)
  Matrixf<3, 3> t2r(Matrixf<4, 4> T);
  // rotation matrix(R) -> homogeneous transformation matrix(T)
  Matrixf<4, 4> r2t(Matrixf<3, 3> R);
  // homogeneous transformation matrix(T) -> translation vector(p)
  Matrixf<3, 1> t2p(Matrixf<4, 4> T);
  // translation vector(p) -> homogeneous transformation matrix(T)
  Matrixf<4, 4> p2t(Matrixf<3, 1> p);
  // rotation matrix(R) & translation vector(p) -> homogeneous transformation
  // matrix(T)
  Matrixf<4, 4> rp2t(Matrixf<3, 3> R, Matrixf<3, 1> p);
  // homogeneous transformation matrix(T) -> RPY([yaw;pitch;roll])
  Matrixf<3, 1> t2rpy(Matrixf<4, 4> T);
  // inverse of homogeneous transformation matrix(T^-1=[R',-R'P;0,1])
  Matrixf<4, 4> invT(Matrixf<4, 4> T);
  // RPY([yaw;pitch;roll]) -> homogeneous transformation matrix(T)
  Matrixf<4, 4> rpy2t(Matrixf<3, 1> rpy);
  // homogeneous transformation matrix(T) -> angle vector([r;θ])
  Matrixf<4, 1> t2angvec(Matrixf<4, 4> T);
  // angle vector([r;θ]) -> homogeneous transformation matrix(T)
  Matrixf<4, 4> angvec2t(Matrixf<4, 1> angvec);
  // homogeneous transformation matrix(T) -> quaternion,
  // [q0;q1;q2;q3]=[cos(θ/2);rsin(θ/2)]
  Matrixf<4, 1> t2quat(Matrixf<4, 4> T);
  // quaternion, [q0;q1;q2;q3]=[cos(θ/2);rsin(θ/2)] -> homogeneous transformation
  // matrix(T)
  Matrixf<4, 4> quat2t(Matrixf<4, 1> quat);
  // homogeneous transformation matrix(T) -> twist coordinates vector ([p;rθ])
  Matrixf<6, 1> t2twist(Matrixf<4, 4> T);
  // twist coordinates vector ([p;rθ]) -> homogeneous transformation matrix(T)
  Matrixf<4, 4> twist2t(Matrixf<6, 1> twist);

  // joint type: R-revolute joint, P-prismatic joint
  typedef enum joint_type
  {
    R = 0,
    P = 1,
  } Joint_Type_e;

  // Denavit–Hartenberg(DH) method   DH参数结构�
  struct DH_t
  {
    // forward kinematic
    Matrixf<4, 4> fkine();
    // DH parameter
    float theta;
    float d;
    float a;
    float alpha;
    Matrixf<4, 4> T;
  };

  class Link
  {
  public:
    Link(){}; // 构�?�对�
    Link(float theta, float d, float a, float alpha, Joint_Type_e type = R,
         float offset = 0, float qmin = 0, float qmax = 0, float m = 1,
         Matrixf<3, 1> rc = matrixf::zeros<3, 1>(),
         Matrixf<3, 3> I = matrixf::zeros<3, 3>());
    Link(const Link &link);

    Link &operator=(Link link);

    float qmin() { return qmin_; } // 直接返回数据
    float qmax() { return qmax_; }
    Joint_Type_e type() { return type_; }
    float m() { return m_; }
    Matrixf<3, 1> rc() { return rc_; }
    Matrixf<3, 3> I() { return I_; }

    Matrixf<4, 4> T(float q); // forward kinematic  单个关节正运动学旋转平移矩阵

  public:
    // kinematic parameter  DH参数
    DH_t dh_;
    float offset_;
    // limit(qmin,qmax), no limit if qmin<=qmax
    float qmin_;
    float qmax_;
    // joint type
    Joint_Type_e type_;
    // dynamic parameter
    float m_;          // mass质量
    Matrixf<3, 1> rc_; // centroid(link coordinate)质心
    Matrixf<3, 3> I_;  // inertia tensor(3*3)惯�?�矩�
  };

  template <uint16_t _n = 1>
  class Serial_Link
  {
  public:
    // 构�?�对�          关节�
    Serial_Link(Link links[_n])
    {
      for (int i = 0; i < _n; i++)
        links_[i] = links[i];
      gravity_ = matrixf::zeros<3, 1>();
      gravity_[2][0] = -9.81f; // 默认重力加�?�度
    }
    /// @brief 构�?�对�
    /// @param gravity 重力加�?�度
    Serial_Link(Link links[_n], Matrixf<3, 1> gravity)
    {
      for (int i = 0; i < _n; i++)
        links_[i] = links[i];
      gravity_ = gravity;
    }

    // forward kinematic: T_n^0  整个臂的正运动学,返回末端的旋转平移矩�
    // param[in] q: joint variable vector  每个关节的变化量
    // param[out] T_n^0
    Matrixf<4, 4> fkine(Matrixf<_n, 1> q)
    {
      T_ = matrixf::eye<4, 4>();                     // 单位矩阵
      for (int iminus1 = 0; iminus1 < _n; iminus1++) // 遍历每个关节
        T_ = T_ * links_[iminus1].T(q[iminus1][0]);  // 累乘每个关节的变换量
      return T_;
    }

    // forward kinematic: T_k^0  求输入q后第�?个到第某个关节的正运动学    返回第一个到第k个的旋转平移矩阵
    // param[in] q: joint variable vector
    // param[in] k: joint number
    // param[out] T_k^0
    Matrixf<4, 4> fkine(Matrixf<_n, 1> q, uint16_t k)
    {
      if (k > _n)
        k = _n;
      Matrixf<4, 4> T = matrixf::eye<4, 4>();
      for (int iminus1 = 0; iminus1 < k; iminus1++)
        T = T * links_[iminus1].T(q[iminus1][0]);
      return T;
    }

    // T_k^k-1: homogeneous transformation matrix of link k   求输入q后第k个关节的旋转平移矩阵
    // param[in] q: joint variable vector
    // param[in] kminus: joint number k, input k-1
    // param[out] T_k^k-1
    Matrixf<4, 4> T(Matrixf<_n, 1> q, uint16_t kminus1)
    {
      if (kminus1 >= _n)
        kminus1 = _n - 1;
      return links_[kminus1].T(q[kminus1][0]);
    }

    // jacobian matrix, J_i = [J_pi;j_oi]
    // param[in] q: joint variable vector
    // param[out] jacobian matix J_6*n          雅可比矩阵固� 6�    列数=关节�
    Matrixf<6, _n> jacob(Matrixf<_n, 1> q)
    {
      Matrixf<3, 1> p_e = t2p(fkine(q));              // p_e输入了q之后的整体平移量
      Matrixf<4, 4> T_iminus1 = matrixf::eye<4, 4>(); // T_i-1^0   4*4的旋转平移矩�
      Matrixf<3, 1> z_iminus1;                        // z_i-1^0
      Matrixf<3, 1> p_iminus1;                        // p_i-1^0
      Matrixf<3, 1> J_pi;
      Matrixf<3, 1> J_oi;
      for (int iminus1 = 0; iminus1 < _n; iminus1++)
      {
        // revolute joint: J_pi = z_i-1x(p_e-p_i-1), J_oi = z_i-1
        if (links_[iminus1].type() == R) // 旋转类型
        {
          z_iminus1 = T_iminus1.block<3, 1>(0, 2);                  // Z轴的旋转
          p_iminus1 = t2p(T_iminus1);                               // Z轴的平移�
          T_iminus1 = T_iminus1 * links_[iminus1].T(q[iminus1][0]); // 累乘每个关节输入q之后整体的旋转平移矩�
          J_pi = vector3f::cross(z_iminus1, p_e - p_iminus1);       // 用Z轴的旋转量叉� (整体-第iminus1个关节的Z轴的平移� )      矢量积法求雅可比
          J_oi = z_iminus1;                                         // Z轴旋转量
        }
        // prismatic joint: J_pi = z_i-1, J_oi = 0
        else // 平移类型
        {
          z_iminus1 = T_iminus1.block<3, 1>(0, 2);                  // Z轴平�
          T_iminus1 = T_iminus1 * links_[iminus1].T(q[iminus1][0]); // 累乘每个关节输入q之后整体的旋转平移矩�
          J_pi = z_iminus1;                                         // 整体没有旋转 �?以就是Z轴平�
          J_oi = matrixf::zeros<3, 1>();                            // 没有旋转  0矩阵
        }
        J_[0][iminus1] = J_pi[0][0];
        J_[1][iminus1] = J_pi[1][0];
        J_[2][iminus1] = J_pi[2][0];
        J_[3][iminus1] = J_oi[0][0];
        J_[4][iminus1] = J_oi[1][0];
        J_[5][iminus1] = J_oi[2][0];
      }
      return J_;
    }

    // inverse kinematic, numerical solution(Newton method)  牛顿法�?�运动学求解
    // param[in] T: homogeneous transformation matrix of end effector
    // param[in] q: initial joint variable vector(q0) for Newton method's
    //              iteration 初始关节变量向量
    // param[in] tol: tolerance of error (norm(error of twist vector))  收敛的误�
    // param[in] max_iter: maximum iterations, default 30       �?大迭代次�
    // param[out] q: joint variable vector  返回每个关节的角�
    Matrixf<_n, 1> ikine(Matrixf<4, 4> Td,
                         Matrixf<_n, 1> q = matrixf::zeros<_n, 1>(),
                         float tol = 1e-4f, uint16_t max_iter = 50)
    {
      Matrixf<4, 4> T;
      Matrixf<3, 1> pe, we;
      Matrixf<6, 1> err, new_err; // 雅可比的误差
      Matrixf<_n, 1> dq;          // 每次累积的输�
      float step = 1;             // 防止误差不断扩大的标志位
      for (int i = 0; i < max_iter; i++)
      {
        T = fkine(q);          // 计算当前角度的运动学 旋转平移矩阵
        pe = t2p(Td) - t2p(T); // 目标位置  -   当前位置  计算位置误差
        // angvec(Td*T^-1), transform angular vector(T->Td) in world coordinate
        we = t2twist(Td * invT(T)).block<3, 1>(3, 0); // 目标姿�?  与当前姿�  的误�
        for (int i = 0; i < 3; i++)                   // 赋�?�误�  前面是位� ,后面是姿�
        {
          err[i][0] = pe[i][0];
          err[i + 3][0] = we[i][0];
        }
        if (err.norm() < tol) // � 2-范数来判断误差是否符合允许范围内,如果符合就返回输出�?
          return q;
        // adjust iteration step
        Matrixf<6, _n> J = jacob(q);                                   // 求当前输出的雅可�   求当前角度与末端的�?�度之间的关�
        for (int j = 0; j < 5; j++)                                    // 循环5�
        {                                                              // n*n的矩�     *       n*1的矩�        =n*1的矩� ,用于反应每个关节的输出微�
          dq = matrixf::inv(J.trans() * J) * (J.trans() * err) * step; // 如果雅可比矩阵J的转置乘以雅可比矩阵J的�?�矩阵产生奇�
                                                                       // 则说明雅可比矩阵奇异，无法直接求�
                                                                       // step的作用是抵消误差放大作用,误差累积之后乘step之后可以减弱误差的放�
          if (dq[0][0] == INFINITY)                                    // 结果是无穷大则确�  �  J'*J singular 奇异
          {
            dq = matrixf::inv(J.trans() * J + 0.1f * matrixf::eye<_n, _n>()) * // 使用带有李雅普诺夫正则项的伪逆求�
                 J.trans() * err * step;
            // SVD<6, _n> JTJ_svd(J.trans() * J);
            // dq = JTJ_svd.solve(err) * step * 5e-2f;
            q += dq;                     // 更新输出,前馈作用
            for (int i = 0; i < _n; i++) // 对每个旋转关节进行限�
            {
              if (links_[i].type() == R)
                q[i][0] = math::loopLimit(q[i][0], -PI, PI);
            }
            break; // �?出循� ,进行下一次迭�
          }
          T = fkine(q + dq);                            // 求输出之后的旋转平移
          pe = t2p(Td) - t2p(T);                        // 位移误差
          we = t2twist(Td * invT(T)).block<3, 1>(3, 0); // 三轴姿�?�误�
          for (int i = 0; i < 3; i++)                   // 赋�?�误�
          {
            new_err[i][0] = pe[i][0];
            new_err[i + 3][0] = we[i][0];
          }
          if (new_err.norm() < err.norm()) // 更新之后的误差与原误差比� ,若误差更� ,则累积输� ,反之让标志位减半
          {
            q += dq;                     // 累加输出
            for (int i = 0; i < _n; i++) // 对每个关节进行限�
            {
              if (links_[i].type() == robotics::Joint_Type_e::R)
              {
                q[i][0] = math::loopLimit(q[i][0], -PI, PI); // 超过  -Π~Π就自动减2Π�?个周�
              }
            }
            break;
          }
          else // 误差反�?�变大了, 标志位减� ,这个标志位是防止误差持续随着迭代不断放大�
          {
            step /= 2.0f;
          }
        }
        if (step < 1e-3f) // 误差累积到一定程度了,返回,迭代结束,防止继续累积误差
          return q;
      }
      return q;
    }
    // 逆动力学解析�
    //  (Reserved function) inverse kinematic, analytic solution(geometric method)
    Matrixf<_n, 1> (*ikine_analytic)(Matrixf<4, 4> T);

    // inverse dynamic, Newton-Euler method    牛顿欧拉�   逆动力学计算力矩
    // param[in]  q: joint variable vector 每个关节的目标角� /位置
    // param[in]  qv: dq/dt          每个关节的目标�?�度/角�?�度
    // param[in]  qa: d^2q/dt^2   目标加�?�度
    // param[in]  he: load on end effector [f;μ], default 0 //加载末端执行� (?)
    Matrixf<_n, 1> rne(Matrixf<_n, 1> q,                            // 每个关节的目标角�
                       Matrixf<_n, 1> qv = matrixf::zeros<_n, 1>(), // 每个关节的目标�?�度
                       Matrixf<_n, 1> qa = matrixf::zeros<_n, 1>(), // 目标加�?�度
                       Matrixf<6, 1> he = matrixf::zeros<6, 1>())
    {
      //前馈
      // forward propagation
      // record each links' motion state in matrices
      // [ωi] angular velocity
      Matrixf<3, _n + 1> w = matrixf::zeros<3, _n + 1>(); // 角�?�度
      // [βi] angular acceleration
      Matrixf<3, _n + 1> b = matrixf::zeros<3, _n + 1>(); // 角加速度
      // [pi] position of joint
      Matrixf<3, _n + 1> p = matrixf::zeros<3, _n + 1>(); // 位置
      // [vi] velocity of joint
      Matrixf<3, _n + 1> v = matrixf::zeros<3, _n + 1>(); // 速度
      // [ai] acceleration of joint
      Matrixf<3, _n + 1> a = matrixf::zeros<3, _n + 1>(); // 加�?�度
      // [aci] acceleration of mass center
      Matrixf<3, _n + 1> ac = matrixf::zeros<3, _n + 1>(); // 质心加�?�度
      // temperary vectors
      Matrixf<3, 1> w_i, b_i, p_i, v_i, ai, ac_i;//未来状�?
      // i & i-1 coordinate convert to 0 coordinate
      Matrixf<4, 4> T_0i = matrixf::eye<4, 4>();
      Matrixf<4, 4> T_0iminus1 = matrixf::eye<4, 4>();
      Matrixf<3, 3> R_0i = matrixf::eye<3, 3>();
      Matrixf<3, 3> R_0iminus1 = matrixf::eye<3, 3>();
      // unit vector of z-axis  Z轴的单位向量[0 1 0]
      Matrixf<3, 1> ez = matrixf::zeros<3, 1>();
      ez[2][0] = 1;

      for (int i = 1; i <= _n; i++)
      {
        T_0i = T_0i * T(q, i - 1);    // T_i^0  每个关节   输入q之后的旋转平移矩�
        R_0i = t2r(T_0i);             // R_i^0    �  每个关节的三轴旋转角�
        R_0iminus1 = t2r(T_0iminus1); // R_i-1^0 后一个关节的旋转
        // ω_i = ω_i-1+qv_i*R_i-1^0*ez
        w_i = w.col(i - 1) + qv[i - 1][0] * R_0iminus1 * ez;//后一个关节的角�?�度=前一个关节的角�?�度+前一个关节的速度乘前�?个关节的Z轴旋�
        // β_i = β_i-1+ω_i-1x(qv_i*R_i-1^0*ez)+qa_i*R_i-1^0*ez
        b_i = b.col(i - 1) +
              vector3f::cross(w.col(i - 1), qv[i - 1][0] * R_0iminus1 * ez) +
              qa[i - 1][0] * R_0iminus1 * ez;
        p_i = t2p(T_0i); // p_i = T_i^0(1:3,4)
        // v_i = v_i-1+ω_ix(p_i-p_i-1)
        v_i = v.col(i - 1) + vector3f::cross(w_i, p_i - p.col(i - 1));
        // a_i = a_i-1+β_ix(p_i-p_i-1)+ω_ix(ω_ix(p_i-p_i-1))
        ai = a.col(i - 1) + vector3f::cross(b_i, p_i - p.col(i - 1)) +
             vector3f::cross(w_i, vector3f::cross(w_i, p_i - p.col(i - 1)));
        // ac_i = a_i+β_ix(R_0^i*rc_i^i)+ω_ix(ω_ix(R_0^i*rc_i^i))
        ac_i =
            ai + vector3f::cross(b_i, R_0i * links_[i - 1].rc()) +
            vector3f::cross(w_i, vector3f::cross(w_i, R_0i * links_[i - 1].rc()));
        for (int row = 0; row < 3; row++)//赋�?
        {
          w[row][i] = w_i[row][0];
          b[row][i] = b_i[row][0];
          p[row][i] = p_i[row][0];
          v[row][i] = v_i[row][0];
          a[row][i] = ai[row][0];
          ac[row][i] = ac_i[row][0];
        }
        T_0iminus1 = T_0i; // T_i-1^0
      }
      //向后迭代
      // backward propagation
      // record each links' force
      Matrixf<3, _n + 1> f = matrixf::zeros<3, _n + 1>();  // joint force�
      Matrixf<3, _n + 1> mu = matrixf::zeros<3, _n + 1>(); // joint moment力矩
      // temperary vector
      Matrixf<3, 1> f_iminus1, mu_iminus1;
      // {T,R',P}_i^i-1
      Matrixf<4, 4> T_iminus1i;
      Matrixf<3, 3> RT_iminus1i;
      Matrixf<3, 1> P_iminus1i;
      // I_i-1(in 0 coordinate)
      Matrixf<3, 3> I_i;
      // joint torque
      Matrixf<_n, 1> torq;

      // load on end effector
      for (int row = 0; row < 3; row++)
      {
        f[row][_n] = he.block<3, 1>(0, 0)[row][0];
        mu[row][_n] = he.block<3, 1>(3, 0)[row][0];
      }
      for (int i = _n; i > 0; i--)
      {
        T_iminus1i = T(q, i - 1);              // T_i^i-1
        P_iminus1i = t2p(T_iminus1i);          // P_i^i-1
        RT_iminus1i = t2r(T_iminus1i).trans(); // R_i^i-1'
        R_0iminus1 = R_0i * RT_iminus1i;       // R_i-1^0
        // I_i^0 = R_i^0*I_i^i*(R_i^0)'
        I_i = R_0i * links_[i - 1].I() * R_0i.trans();
        // f_i-1 = f_i+m_i*ac_i-m_i*g
        f_iminus1 = f.col(i) + links_[i - 1].m() * ac.col(i) -
                    links_[i - 1].m() * gravity_;
        // μ_i-1 = μ_i+f_ixrc_i-f_i-1xrc_i-1->ci+I_i*b_i+ω_ix(I_i*ω_i)
        mu_iminus1 = mu.col(i) +
                     vector3f::cross(f.col(i), R_0i * links_[i - 1].rc()) -
                     vector3f::cross(f_iminus1, R_0i * (RT_iminus1i * P_iminus1i +
                                                        links_[i - 1].rc())) +
                     I_i * b.col(i) + vector3f::cross(w.col(i), I_i * w.col(i));
        // τ_i = μ_i-1'*(R_i-1^0*ez)
        torq[i - 1][0] = (mu_iminus1.trans() * R_0iminus1 * ez)[0][0];
        for (int row = 0; row < 3; row++)
        {
          f[row][i - 1] = f_iminus1[row][0];
          mu[row][i - 1] = mu_iminus1[row][0];
        }
        R_0i = R_0iminus1;
      }

      return torq;
    }

    private:
    Link links_[_n];        // 关节�
    Matrixf<3, 1> gravity_; // 重力加�?�度

    Matrixf<4, 4> T_;  // 旋转平移矩阵
    Matrixf<6, _n> J_; // 雅可比矩�
  };
}; // namespace robotics

#endif // ROBOTICS_H

