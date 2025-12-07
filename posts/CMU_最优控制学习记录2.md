---
title: CMU Optimal control and reinforcement learning 16-745 2025学习简要记录2
author: Daliang
published: 2025-07-28
toc: true
toc-depth: 4
toc-title: Contents
tags:
  - 优化控制
  - LQR/MPC/DDP
category: "CMU Optimal Control 16-745"
licenseName: "CC BY 4.0"
---
# CMU Optimal control and reinforcement learning 16-745 2025学习简要记录2

主要是配合Homework来的（在[课程官网答案](https://optimalcontrol.ri.cmu.edu/homeworks/)有完整的代码）

较早的更完整的课程记录可以参考[向阳的笔记](https://github.com/Zhihaibi/Optimal_control_16-745/blob/main/CMU16_745_Optimial%20control%20Lecture_Notes_zhihai%20Bi.pdf)和知乎[我爱科研](https://www.zhihu.com/column/c_1635315526615388160) 的整理

## Lecture 9 Linear Quadratic Regulator

 线性二次型最优控制问题

$$
\begin{align}
\min_{x_{1:N},{u}_{1:N-1}}& \quad \sum_{k=1}^{N-1}(\frac{1}{2}{x_k}^TQx_k + \frac{1}{2}{u_k}^TRu_k)+\frac{1}{2}{x_N}^TQ_Nx_N \\
\text{s.t.}&\quad x_{k+1}=A_kx_k+B_ku_k
\end{align}
$$

其中，$Q\succeq 0,R\succ 0$。

- 根据$A,B,Q,R$是否随时间变化可以分为时不变和时变LQR，time-invariant LQRs 经常用于在稳定平衡点，TVLQR 用于轨迹跟踪
- 可以通过线性化求解非线性优化问题，在实际问题中得到广泛运用

### 将LQR转换为一个标准的QP问题求解

定义优化变量$z$

$$
z=\begin{bmatrix}
  u_1 \\
  x_2 \\
  u_2 \\
  . \\
  . \\
  x_N
  \end{bmatrix}
$$

  $J=\frac{1}{2}z^THz$

$$
H=
  \begin{bmatrix}
  R_1 & 0 & ... & 0 \\
  0 & Q_2 & ... & 0 \\
   & & . \\
  0 & 0 & ... & Q_N
  \end{bmatrix}
$$

  $Cz=d$

$$
\begin{aligned}
   & C=
  \begin{bmatrix}
  B_1 & (-I) & ... & ... & ... & 0 \\
  0 & A & B & (-I) & ... & 0 \\
   & & . \\
  0 & 0 & ... & A_{N-1} & B_{N-1} & (-I)
  \end{bmatrix} \\
   & d=
  \begin{bmatrix}
  -A_1x_1 \\
  0 \\
  . \\
  0
  \end{bmatrix}
  \end{aligned}
$$

  这样就形式上转换成了一个标准的QP问题

$$
\begin{aligned}
   & \min_z\frac{1}{2}z^THz \\
   & s.t.\quad Cz=d
  \end{aligned}
$$

  通过引入拉格然日乘子给出拉格朗日函数，得到KKT方程，就可以用Newton方法迭代求解

$$
\begin{bmatrix}
  H & C^T \\
  C & 0
  \end{bmatrix}
  \begin{bmatrix}
  z \\
  \lambda
  \end{bmatrix}=
  \begin{bmatrix}
  0 \\
  d
  \end{bmatrix}
$$

### Riccati 方程求解（利用KKT中的稀疏性）

$$
\begin{bmatrix}
  R & & & & & & . & B^T &  \\
   & Q & & & & & . & -I & A^T  \\
   & & R & & & & . & & B^T  \\
   & & & Q & & & . & & -I & A^T \\
   & & & & R & & . & & & B^T \\
   & & & & & Q_N & . & & & -I \\
  . & . & . & . & . & . & . & . & . & . \\
  B & -I & & & & & . & 0 & 0 & 0 \\
   & A & B & -I & & & . & 0 & 0 & 0 \\
   & & & A & B & -I & . & 0 & 0 & 0
  \end{bmatrix}
  \begin{bmatrix}
  u_1 \\
  x_2 \\
  u_2 \\
  x_3 \\
  u_3 \\
  x_4 \\
  \lambda_2 \\
  \lambda_3 \\
  \lambda_4
  \end{bmatrix}=
  \begin{bmatrix}
  0 \\
  0 \\
  0 \\
  0 \\
  0 \\
  0 \\
  -Ax_1 \\
  0 \\
  0
  \end{bmatrix}
$$

  从末态$x_4$开始

$$
Q_Nx_4-\lambda_4 \Longrightarrow\lambda_4  =Q_Nx_4
$$

  考虑$u_3$(分别代入$\lambda_4  =Q_Nx_4$和$x_4=Ax_3+Bu_3$)

$$
\begin{aligned}
  &Ru_3+B^T\lambda_4=Ru_3+B^TQ_Nx_4=Ru_3+B^TQ_N(Ax_3+Bu_3)=0\\
  &\Longrightarrow u_3  =-(R+B^TQ_NB)^{-1}B^TQ_NAx_3
  \end{aligned}
$$

  记成$u_3=-K_3x_3$

  到$x_3$

$$
\begin{align}
  &Qx_3-\lambda_3+A^T\lambda_4=0\\
  \Longrightarrow& Qx_3-\lambda_3+A^TQ_Nx_4=0\\
  \Longrightarrow& Qx_3-\lambda_3+A^TQ_N(Ax_3+Bu_3)=0\\
  \Longrightarrow& Qx_3-\lambda_3+A^TQ_N(A-BK_3)x_3=0\\
  \Longrightarrow& \lambda_3= [Q+A^TQ_N(A-BK_3)]x_3\\
  \end{align}
$$

  记为$\lambda_3=P_3x_3$

  这样依次递推出$K_n$和$P_n$，便可求出控制序列$u_{1:N-1}$

$$
\begin{align}
  P_N& = Q_N\\
  K_n& = (R+B^TP_{n+1}B)^{-1}B^TP_{n+1}A\\
  P_n& = Q+A^TP_{n+1}(A-BK_n)
  \end{align}
$$

### 例子-HW2 Q1

#### Part A 离散化动力学模型

考虑一个二阶积分系统，状态和控制变量如下

$$
\begin{align} x &= [p_1, p_2, v_1, v_2] \\ u &= [a_1, a_2] \end{align}
$$

状态空间方程$\dot{x}=Ax+Bu$

$$
\begin{align} \dot{x} =  \begin{bmatrix} 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \end{bmatrix} x + \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ 1 & 0 \\ 0 & 1 \end{bmatrix} u\end{align}
$$

离散状态空间方程$x_{k+1}=A_dx_k+B_du_k$

其中$A_d=e^{Adt},B_d=Idt+A\frac{dt^2}{2}(\text{注意这里的}A^2=0)$

#### Part B: Finite Horizon LQR via Convex Optimization

定义性能指标和约束方程，使用 `Convex.jl`求解得到 `Xcvx,Ucvx = convex_trajopt(A,B,Q,R,Qf,N,x_ic)`

$$
\begin{align} \min_{x_{1:N},u_{1:N-1}} \quad & \sum_{i=1}^{N-1} \bigg[ \frac{1}{2} x_i^TQx_i + \frac{1}{2} u_i^TRu_i \bigg] + \frac{1}{2}x_N^TQ_fx_N\\ 
 \text{st} \quad & x_1 = x_{\text{IC}} \\ 
 & x_{i+1} = A x_i + Bu_i \quad \text{for } i = 1,2,\ldots,N-1 
 \end{align}
$$

初态$x_{ic} = [5,7,2,-1.4]$

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q1_2.svg" style="zoom:67%;" />

**验证LQR问题的Bellman's Principle of Optimality**，即在1：N时间的中间任何时刻$L(1<L<N)$到达平衡点都是最优的，也即是原始LQR问题和新的LQR问题得到的轨迹是重合的。

$$
\begin{align} \min_{x_{L:N},u_{L:N-1}} \quad & \sum_{i=L}^{N-1} \bigg[ \frac{1}{2} x_i^TQx_i + \frac{1}{2} u_i^TRu_i \bigg] + \frac{1}{2}x_N^TQ_fx_N\\ 
 \text{st     } \quad & x_L = x^*_L \\ 
 & x_{i+1} = A x_i + Bu_i \quad \text{for } i = L,L + 1,\ldots,N-1 
 \end{align}
$$

<img src="../image/HW2_Q1_3.svg" style="zoom:67%;" />

#### Part C：Finite-Horizon LQR via Riccati

使用Riccati 方程递推求解离散LQR问题的解析解并与convex.jl的求解结果进行对比，结果是一致的

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q1_4.svg" style="zoom:67%;" />

多次随机初始状态结果也是一致的

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q1_5.svg" style="zoom:67%;" />

#### Part D: Why LQR is so great LQR的优异性

求出控制序列后，给实际的动力学系统加入噪声

$$
x_{k+1} = Ax_k + Bu_k + \text{noise}
$$

```julia
noise = [.005*randn(2);.1*randn(2)]
```

可以看出与求解关于优化变量$z=[x1,x2,..,x_N,u_1,...,u_{N-1}]^\mathrm{T}$的QP问题相比，由Ricatti递推求解得到的控制律$u=-Kx$鲁棒性更强。

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q1_6.svg" style="zoom:67%;" />

设定一个不在平衡点的目标xgoal = [-3.5,-3.5,0,0]，LQR求的控制律$u=-K(x-x_{goal})$依旧能实现

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q1_8.svg" style="zoom:67%;" />

![](https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q1_7.svg)

#### Part E: Infinite -horizon LQR 无限时间二次型调节问题

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q1_9.svg" style="zoom:67%;" />

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q1_10.svg" style="zoom:67%;" />

矩阵$P K$各元素的变化如图所示，可以看出对于一个有限时间的LQR系统，矩阵$P(n_x\times n_x)$和$K(n_u\times n_x)$在Ricatti反向迭代的过程中，经过一段时间就很快收敛。

那么对于一个无限时间LQR系统，求出的$K$会是一个常数矩阵。

### 例子-HW Q2 LQR for nonlinear systems

#### Part 0 预备知识

**非线性系统线性化**

给定参考状态轨迹$\bar{x}_{1:N}$和参考控制轨迹$\bar{u}_{1:N-1}$，定义增量坐标

$$
x_k=\bar{x}_k+\Delta x_k,u_k=\bar{u}_k+\Delta u_k.
$$

对离散非线性动力学系统$x_{k+1}=f(x_k,u_k)$进行线性化（一阶泰勒展开）

$$
x_{k+1}\approx f(\bar{x}_k,\bar{u}_k)+\underbrace{\frac{\partial f}{\partial x}|_{\bar{x}_k,\bar{u}_k}}_{A_k}\Delta x_k+\underbrace{\frac{\partial f}{\partial u}|_{\bar{x}_k,\bar{u}_k}}_{B_k}\Delta u_k
$$

其中，$A_k$是状态雅可比矩阵$(n_x\times n_x)$，$B_k$是控制雅可比矩阵$(n_x\times n_u)$

如果参考轨迹是动态可行的（即满足$\bar{x}_{k+1}=f(\bar{x}_k,\bar{u}_k)$），则泰勒展开式可简化为

$$
\bar{x}_{k+1}+\Delta x_{k+1}\approx f(\bar{x}_k,\bar{u}_k)+A_k \Delta x_k+B_k \Delta u_k
$$

得到增量动力学方程

$$
\Delta x_{k+1}\approx A_k \Delta x_k+B_k \Delta u_k
$$

**线性时不变系统离散化方法**

$$
\dot{x}(t)=Ax(t)+Bu(t)
$$

连续系统的解为

$$
x(t)=e^{A(t-t_0)}x(t_0)+\int_{t_0}^{t}e^{A(t-\tau)}Bu(\tau)d\tau
$$

使用零阶保持器(ZOH)控制$u(t)=u_k$在$t\in[t_k,t_{k+1}]$，则

$$
x_{k+1}=e^{A\Delta t}x_k+(\int_{0}^{\Delta t}e^{A\tau}d\tau)Bu_k
$$

构造增广系统

$$
\frac{d}{dt}
\begin{bmatrix}
x(t) \\
u(t)
\end{bmatrix}=
\begin{bmatrix}
A & B \\
0 & 0
\end{bmatrix}
\begin{bmatrix}
x(t) \\
u(t)
\end{bmatrix}
$$

在ZOH假设下$\dot{u}(t)=0$，计算该系统的状态转移矩阵

$$
\exp\left(
\begin{bmatrix}
A & B \\
0 & 0
\end{bmatrix}\Delta t\right)=
\begin{bmatrix}
e^{A\Delta t} & \int_{0}^{\Delta t}e^{A\tau}d\tau B \\
0 & I
\end{bmatrix}
$$

通过计算增广矩阵的指数

$$
\exp\left(\begin{bmatrix}A&B\\0&0\end{bmatrix}\Delta t\right)=\begin{bmatrix}A_k&B_k\\0&I\end{bmatrix}
$$

- $A_k$为左上$n\times n$块
- $B_k$为右上$n\times m$块

#### Part A: Infinite Horizon LQR about an equilibrium

小车倒立摆（CartPole）的动力学方程

$$
H(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)=Bu
$$

- 广义坐标$q=[p,\theta]$
- 质量惯性矩阵$H=
  \begin{bmatrix}
  m_c+m_p & m_pl\cos\theta \\
  m_pl\cos\theta & m_pl^2
  \end{bmatrix}$
- 科里奥利矩阵$C=
  \begin{bmatrix}
  0 & -m_pl\dot{\theta}\sin\theta \\
  0 & 0
  \end{bmatrix}$
- 重力向量$G=\begin{bmatrix}0\\m_pglsin\theta\end{bmatrix}$
- 输入映射矩阵$B=\begin{bmatrix}1\\0\end{bmatrix}$

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/cartpole.png" style="zoom:33%;" />

使用Infinite Horizon LQR将倒立摆小车稳定到平衡位置

```Julia
xgoal = [0, pi, 0, 0]
x0 = [0, pi, 0, 0] + [1.5, deg2rad(-20), .3, 0]
```

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q2_1.svg" style="zoom:67%;" />

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q2_1.gif" style="zoom:50%;" />

#### Part B : Basin of Attarction吸引域分析

LQR控制器是基于系统在$(x_{goal},u_{goal})$处的线性近似设计的，当系统状态远离线性化点时，真实非线性动力学与线性模型差异变大，导致控制器性能下降甚至失效。

这里测试LQR控制器在不同初始条件下的稳定性，绘制吸引域，即能成功稳定的初始状态范围。

```julia
# create a span of initial configurations 
M=20
ps = LinRange(-7, 7, M)
thetas = LinRange(deg2rad(180-60), deg2rad(180+60), M)
```

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q2_2.svg" style="zoom:67%;" />

#### Part C : 无限时域LQR调参

通过调整$Q$和$R$中的权重参数，来实现5秒仿真结束时，系统状态与目标状态的2范数误差小于0.1$||x_{end}-x_{goal}||<0.1$，并且满足执行器限制$-3\le u\le 3$（限幅）。

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q2_3.svg" style="zoom:67%;" />

#### Part D: TVLQR for  trajectory tracking

在参考轨迹的每个点上线性化系统，设计时变反馈增益$K(t)$，使系统能稳定跟踪时变目标。

```julia
function TVlqr(A_list::Vector{Matrix{Float64}}, B_list::Vector{Matrix{Float64}}, Q::Matrix, R::Matrix,
        Qf::Matrix,N::Int64)::Tuple{Vector{Matrix{Float64}}, Vector{Matrix{Float64}}}
  
        nx, nu = size(B_list[1])
  
        P = [zeros(nx,nx) for i = 1:N]
        K = [zeros(nu,nx) for i = 1:N-1]
        P[N] = deepcopy(Qf)
        for i = N-1:-1:1
            A,B = A_list[i],B_list[i]
            K[i] = (R + B'*P[i+1]*B) \ (B'*P[i+1]*A)
            P[i] = Q + A'*P[i+1]*(A - B*K[i])  
        end
        return P,K
    end
```

![](https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q2_4.svg)

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q2_2.gif" style="zoom:50%;" />
