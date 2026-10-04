---
title: CMU 最优控制笔记 5：旋转矩阵、四元数与刚体姿态
author: Daliang
published: 2025-08-07
updated: 2026-10-04
description: 整理三维旋转的坐标变换、单位四元数、姿态运动学与数值积分，并比较旋转矩阵和四元数的约束。
toc: true
toc-depth: 4
toc-title: Contents
tags:
  - 最优控制
  - 四元数
  - 旋转矩阵
  - Julia
category: "CMU Optimal Control 16-745"
licenseName: "CC BY 4.0"
---

学习三维姿态表示时，我把旋转矩阵和单位四元数放在一起比较。下面先明确坐标系约定，再整理角速度、姿态微分方程以及数值积分后的约束检查。

**先修知识：**线性代数、叉乘、刚体动力学与常微分方程数值积分。

**阅读路线：**

1. 从世界坐标系与机体坐标系之间的变换理解旋转矩阵。
2. 用四元数乘法、共轭与单位长度约束表示旋转。
3. 对照两种姿态动力学实现，检查积分后的正交性和四元数模长。


## Lecture 14 旋转

欧拉角直观，但在特定姿态存在参数化奇异性。旋转矩阵和单位四元数可以避免欧拉角的这类奇异性，代价是使用冗余参数并满足相应约束；此外，$q$ 与 $-q$ 表示同一旋转，四元数表示也不是唯一的。

本文约定 $Q$ 将机体系向量变换到世界系，四元数按“实部在前”排列，姿态运动学中的角速度用机体系表示。换用其他约定时，乘法顺序和符号都要重新核对。

### 旋转矩阵

三维空间坐标点在以$[\mathbf{e}_{1},\mathbf{e}_{2},\mathbf{e}_{3}]$单位正交基组成的世界坐标系$\mathcal{N}$中的描述和在体坐标系$\mathcal{B}$（基底$[\mathbf{e}_{1}',\mathbf{e}_{2}',\mathbf{e}_{3}']$）是一致的，即有

$$
[\mathbf{e}_{1},\mathbf{e}_{2},\mathbf{e}_{3}]\left[\begin{array}{c}
    {^N}x_1 \\
    {^N}x_2 \\
    {^N}x_3 \\
\end{array} \right] =[\mathbf{e}_{1}',\mathbf{e}_{2}',\mathbf{e}_{3}']\left[ \begin{array}{c}
    {^B}x_1 \\
    {^B}x_2 \\
    {^B}x_3 \\
\end{array} \right]
$$

由于基底的正交性，可以得出在世界坐标系N和体坐标系下坐标旋转变换关系

$$
\left[ \begin{array}{c}
    {^N}x_1 \\
    {^N}x_2 \\
    {^N}x_3 \\
\end{array} \right] = Q \left[ \begin{array}{c}
    {^B}x_1 \\
    {^B}x_2 \\
    {^B}x_3 \\
\end{array} \right]
$$

$Q$是旋转矩阵，是一个行列式为1的正交矩阵，它的逆就是它的转置，并且旋转矩阵构成一个$SO(3)$李群（特殊正交群）

- $Q^\mathrm{T}Q=I$
- $det(Q)=1$
- $Q\in SO(3)$ special orthogonal in 3D
  拓展：特殊欧式群（special Euclidean Group）SE(3)

$$
SE(3)=\{T=\left[\begin{array}{cc}
R & t\\ \mathbf{0}_{1\times3}&1
\end{array}\right] \in \mathbb{R}^{4\times4}|R\in SO(3),t\in \mathbb{R}^{3} \}
$$

#### 描述陀螺仪的旋转

以陀螺仪举例，考虑固定在本身的体坐标系B和地面坐标系的坐标描述

$$
{^N}\mathbf{x} =Q(t){^B}\mathbf{x}
$$

对时间t求导

$$
\begin{align}{^N}\dot {\mathbf{x} }&=\dot Q(t){^B}\mathbf{x}+Q(t) {^B}\dot {\mathbf{x}}\\
&=\dot Q(t){^B}\mathbf{x}
\end{align}
$$

当物体以$\omega$的角速度旋转，那么${^N}\mathbf{x}$的导数

$$
{^N}\dot{x}={^N}\omega\times{^N}x=Q({^B}\omega\times{^B}x)
$$

那么

$$
\dot Q(t){^B}\mathbf{x}=Q({^B}\omega\times{^B}x)\quad\Longrightarrow\quad\dot{Q}=Q\widehat{{^B}\omega}
$$

$$
Q_{k+1} = Q_{k}+\dot{Q}_{k}\Delta t
$$

### 四元数：一个实部与三个虚部

$$
\mathbf{q}=w+x\mathbf{i}+y\mathbf{j}+z\mathbf{k}
$$

本文讨论的都是单位四元数

$$
||\mathbf{q}||=\sqrt{w^2+x^2+y^2+z^2}=1
$$

也可以通过轴角描述(给定一个旋转轴$\mathbf{u}$和旋转角$\theta$)

$$
\mathbf{q} = [ cos\frac{\theta}{2},\mathbf{u}sin\frac{\theta}{2}]
$$

四元数的乘法

$$
\begin{align}
\mathbf{q}_1\otimes \mathbf{q}_2&=\left[ \begin{array}{c}
	{w}_1\\
	\mathbf{v}_1\\
\end{array} \right]\otimes\left[ \begin{array}{c}
	{w}_2\\
	\mathbf{v}_2\\
\end{array} \right]=\left[ \begin{array}{c}
	{w}_1{w}_2-\mathbf{v}_1\cdot \mathbf{v}_2\\
	{w}_1\mathbf{v}_2+{w}_2\mathbf{v}_1+\mathbf{v}_1\times \mathbf{v}_2\\
\end{array} \right]\\
&=\begin{bmatrix} w_1 &-\mathbf{v}_1^\mathrm{T}\\ 
\mathbf{v}_1& w_1I+\hat{\mathbf{v}} _1
\end{bmatrix}\begin{bmatrix} w_2\\\mathbf{v}_2\end{bmatrix}=
L(\mathbf{q}_1)\begin{bmatrix} w_2\\\mathbf{v}_2\end{bmatrix}\\
&=\begin{bmatrix} w_2 &-\mathbf{v}_2^\mathrm{T}\\ 
\mathbf{v}_2& w_2I-\hat{\mathbf{v}} _2
\end{bmatrix}\begin{bmatrix} w_1\\\mathbf{v}_1\end{bmatrix}=
R(\mathbf{q}_2)\begin{bmatrix} w_1\\\mathbf{v}_1\end{bmatrix}\\
\end{align}
$$

旋转一个向量(使用纯虚四元数表示姿态向量$H\mathbf{v}=\begin{bmatrix}0 \\ \mathbf{v}\end{bmatrix}$)

$$
\begin{align}H \mathbf{v}^{\prime}
&=\mathbf{q}\otimes \begin{bmatrix}0 \\ \mathbf{v}\end{bmatrix}\otimes \mathbf{q}^{-1}\\
&= L(\mathbf{q})R(\mathbf{q})^TH\mathbf{v}\\
& = R(\mathbf{q})^TL(\mathbf{q})H\mathbf{v}
\end{align}
$$

其中$R(\mathbf{q})$和$L(\mathbf{q})$四元数的左乘和右乘矩阵,从这里可以得到四元数和旋转矩阵的关系

$$
Q(\mathbf{q})=H^{\mathrm{T}}R(\mathbf{q})^TL(\mathbf{q})H
$$

四元数的逆与共轭四元数的关系

$$
\mathbf{q}^{-1}=\frac{\mathbf{q}^*}{||\mathbf{q}||^2}
$$

单位四元数有$\mathbf{q}^{-1}=\mathbf{q}^*$

用四元数表示刚体的姿态运动学quaternion Kinematics

$$
\dot{\mathbf{q}}=\frac{1}{2}L(\mathbf{q})H\boldsymbol{\omega}
$$

那么完整的位置和姿态运动学方程和速度和角速度动力学方程如下

$$
\dot{\mathbf{x}} = \begin{bmatrix} \dot{\mathbf{r}} \\ \dot{\mathbf{q}} \\ \dot{\mathbf{v}} \\ \dot{\boldsymbol{\omega}} \end{bmatrix} = \begin{bmatrix} \mathbf{v} \\ \frac{1}{2}L(\mathbf{q})H\boldsymbol{\omega} \\ \frac{1}{m} {}^W\mathbf{F}(\mathbf{x}, \mathbf{u}) \\ \mathbf{J}^{-1} \left( {}^B\mathbf{\tau}(\mathbf{x}, \mathbf{u}) - \boldsymbol{\omega} \times \mathbf{J} \boldsymbol{\omega} \right) \end{bmatrix}
$$

其中 $H\omega=[0;\omega]$ 为纯虚四元数，不能与 $3\times3$ 叉乘矩阵 $\hat\omega$ 混用。$^W\mathbf F$ 为世界系的总外力（有重力时应包含它），$^B\tau$ 为机体系的总外力矩，$J$ 为机体系中恒定的惯性矩阵。

### 刚体姿态动力学分析

使用**旋转矩阵表示法**（9参数）和**四元数表示法**（4参数）实现一个刚体姿态动力学仿真系统

1. 核心库

   ```julia
   using LinearAlgebra   # 线性代数运算
   using ForwardDiff     # 自动微分
   ```
2. 关键函数定义

   <details>
    <summary>点击展开代码</summary>

   ```julia
   # 向量的反对称矩阵
   function hat(v)
      [0 -v[3] v[2];
       v[3] 0 -v[1];
       -v[2] v[1] 0]
   end
   function L(q)  # 四元数左乘矩阵
      s = q[1]
      v = q[2:4]
      [s    -v';
       v  s*I+hat(v)]
   end

   function R(q)  # 四元数右乘矩阵
      s = q[1]
      v = q[2:4]
      [s    -v';
       v  s*I-hat(v)]
   end
   ```

   </details>

3. 初始条件

   ```julia
   # 旋转矩阵表示
   Q0 = I(3)         # 初始姿态(单位矩阵)
   ω0 = randn(3)     # 随机初始角速度
   x0 = [vec(Q0); ω0]

   # 四元数表示
   q0 = [1; 0; 0; 0] # 单位四元数(无旋转)
   x0q = [q0; ω0]
   ```
4. 动力学模型

   ```julia
   # 旋转矩阵表示
   function dynamics(x)
       Q = reshape(x[1:9],3,3)
       ω = x[10:12]

       Q̇ = Q*hat(ω)        # 姿态微分方程
       ω̇ = -J\(hat(ω)*J*ω) # 欧拉动力学方程

       [vec(Q̇); ω̇]
   end
   # 四元数
   function qdynamics(x)
       q = x[1:4]
       ω = x[5:7]

       q̇ = 0.5*L(q)*H*ω    # 四元数微分方程
       ω̇ = -J\(hat(ω)*J*ω) # 欧拉动力学方程

       [q̇; ω̇]
   end
   ```
5. 数值积分方法RK4

   ```julia
   function rkstep(x)
       f1 = dynamics(x)
       f2 = dynamics(x + 0.5*h*f1)
       f3 = dynamics(x + 0.5*h*f2)
       f4 = dynamics(x + h*f3)
       xn = x + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)
       # 四元数表示，额外进行归一化
       #xn[1:4] .= xn[1:4]./norm(xn[1:4])  # 保持单位四元数
   end
   ```
6. 仿真结果验证

   - 旋转矩阵验证

     ```powershell
     Qk'*Qk  # 应保持正交性≈ I(3) 
     3×3 Matrix{Float64}:
       0.962748    -0.00123354  -0.00149965
      -0.00123354   0.977413     0.0178415
      -0.00149965   0.0178415    0.984187

     ```
   - 四元数验证

     ```julia
     norm(qk)  # 应保持单位长度≈ 1.0 
     0.9999999999999999
     Q(qk)'*Q(qk)   # 转换矩阵应正交≈ I(3)
     3×3 Matrix{Float64}:
      1.0          1.73472e-17  2.77556e-17
      1.73472e-17  1.0          0.0
      2.77556e-17  0.0          1.0

     ```

## 复现与数值误差

这里的 Julia 片段仍依赖未在本文完整列出的惯性矩阵 `J`、四元数嵌入矩阵 `H`、步长 `h` 和仿真循环；当前不应视为可以独立运行的完整程序。`rkstep` 默认调用旋转矩阵模型，切换到四元数时必须同时更换动力学函数与状态布局，不能只取消归一化注释。

原日志中的 $Q^\top Q$ 已明显偏离单位矩阵，说明直接积分没有严格保持正交性。这是需要检查的误差，而不是正交性验证通过。对四元数归一化有助于维持单位长度，但不能代替积分精度和动力学结果的验证。

## 整理与核查说明

本笔记原有许可为 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)。引用的课程材料、代码和图片仍须遵守其各自的许可。

**核查状态：待复现（2026-10-04）。** 已核对四元数顺序与坐标系说明；原旋转积分实验未重新运行。

2026-10-04 整理时修正了已定位的公式和实现问题。文中的图片、动画和输出保留自学习时的实验记录，不代表修订后的代码已经完整重跑。作业片段依赖原项目环境，不能直接作为完整可运行教程；具体核查范围与尚未复现事项见正文。

## 参考资料

- [Quaternions and Rotations∗](https://graphics.stanford.edu/courses/cs348a-17-winter/Papers/quaternion.pdf)
- Planning With Attitude
