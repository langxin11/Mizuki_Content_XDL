---
title: CMU Optimal control and reinforcement learning 16-745 2025学习简要记录5
author: Daliang
published: 2025-08-07
toc: true
toc-depth: 4
toc-title: Contents
tags:
  - 优化控制
  - 四元数/旋转矩阵
category: "CMU Optimal Control 16-745"
licenseName: "CC BY 4.0"
---
# CMU Optimal control and reinforcement learning 16-745 2025学习简要记录5

## Lecture 14 旋转

主要介绍了如何描述物体旋转的一些参数化表示方法，欧拉角（俯仰角、偏航角和滚转角）的描述十分直观，但仅靠三个参数会出现奇异性（即是某些姿态参数化表示不唯一或者无法表示）。引入旋转矩阵和单位四元数克服旋转描述的非奇异性。

### **旋转矩阵**

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
- $Q\in SO(3)$ special orthogoual in 3D
  拓展：特殊欧式群（special Euclidean Group）SE(3)

$$
SE(3)=\{T=\left[\begin{array}{c}
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
{^N}\dot{x}={^N}\omega\times{^N}x={_{N}^B}Q({^B}\omega\times{^B}x)
$$

那么

$$
\dot Q(t){^B}\mathbf{x}={_{N}^B}Q({^B}\omega\times{^B}x\rightarrow\dot{Q}=Q\hat{\omega}
$$

$$
Q_{k+1} = Q_{k}+\dot{Q}_{k}\Delta t
$$

### **四元数**(一个实部、三个虚部)

$$
\mathbf{q}=\omega+x\mathbf{i}+y\mathbf{j}+z\mathbf{k}
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
\dot{\mathbf{q}}=\frac{1}{2}L(\mathbf{q})Hw
$$

那么完整的位置和姿态运动学方程和速度和角速度动力学方程如下

$$
\dot{\mathbf{x}} = \begin{bmatrix} \dot{\mathbf{r}} \\ \dot{\mathbf{q}} \\ \dot{\mathbf{v}} \\ \dot{\boldsymbol{\omega}} \end{bmatrix} = \begin{bmatrix} \mathbf{v} \\ \frac{1}{2} \mathbf{q} \otimes \hat{\boldsymbol{\omega}} = \frac{1}{2} \mathbf{L}(\mathbf{q}) \mathbf{H} \boldsymbol{\omega} \\ \frac{1}{m} {}^W\mathbf{F}(\mathbf{x}, \mathbf{u}) \\ \mathbf{J}^{-1} \left( {}^B\mathbf{\tau}(\mathbf{x}, \mathbf{u}) - \boldsymbol{\omega} \times \mathbf{J} \boldsymbol{\omega} \right) \end{bmatrix}
$$

其中$^W\mathbf{F}(\mathbf{x}, \mathbf{u})$是世界坐标系下的外力表示，$^B\mathbf{\tau}(\mathbf{x}, \mathbf{u})$是机体坐标系的外力矩

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

   ```Julia
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

## 参考资料

- [Quaternions and Rotations∗](https://graphics.stanford.edu/courses/cs348a-17-winter/Papers/quaternion.pdf)
- Planning With Attitude
