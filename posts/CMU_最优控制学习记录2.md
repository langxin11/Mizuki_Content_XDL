---
title: CMU 最优控制笔记 2：LQR、Riccati 递推与轨迹跟踪
author: Daliang
published: 2025-07-28
updated: 2026-10-04
description: 通过二次规划与 Riccati 递推理解 LQR，并整理非线性系统局部控制和 TVLQR 跟踪实例。
toc: true
toc-depth: 4
toc-title: Contents
tags:
  - 最优控制
  - LQR
  - Riccati
  - Julia
category: "CMU Optimal Control 16-745"
licenseName: "CC BY 4.0"
---

学习线性二次型调节器（LQR）时，我把二次规划和 Riccati 递推放在一起整理：同一个最优控制问题，既可以写成二次规划，也可以利用时间结构通过 Riccati 递推求解。

**先修知识：**离散状态空间模型、二次型、矩阵求导与等式约束优化。

**阅读路线：**

1. 先明确代价、动力学约束和初始状态，比较 QP 与 Riccati 两种求解思路。
2. 再区分有限时域、无限时域和平衡点附近的局部控制。
3. 最后阅读非线性系统案例及 TVLQR 轨迹跟踪，观察线性化的适用范围。

笔记结合 CMU 16-745 的课程与作业整理，相关材料见[课程作业页面](https://optimalcontrol.ri.cmu.edu/homeworks/)。文中的代码片段保留学习时的实现，运行时还需使用对应作业的依赖与上下文。

其他学习视角可参考[向阳的笔记](https://github.com/Zhihaibi/Optimal_control_16-745/blob/main/CMU16_745_Optimial%20control%20Lecture_Notes_zhihai%20Bi.pdf)和知乎[我爱科研](https://www.zhihu.com/column/c_1635315526615388160) 的整理

## LQR：二次规划与 Riccati 递推

按[2025 年课程目录](https://optimalcontrol.ri.cmu.edu/lectures/)，LQR in 3 Ways 对应 Lecture 8；此前的笔记记为 Lecture 9，此处按主题组织。

 线性二次型最优控制问题

$$
\begin{align}
\min_{x_{1:N},{u}_{1:N-1}}& \quad \sum_{k=1}^{N-1}(\frac{1}{2}{x_k}^TQx_k + \frac{1}{2}{u_k}^TRu_k)+\frac{1}{2}{x_N}^TQ_Nx_N \\
\text{s.t.}&\quad x_{k+1}=A_kx_k+B_ku_k,\quad x_1=x_{\mathrm{init}}
\end{align}
$$

其中，$Q\succeq 0,R\succ 0$。

- 根据$A,B,Q,R$是否随时间变化可以分为时不变和时变LQR，时不变 LQR 常用于平衡点附近的稳定控制，TVLQR 用于轨迹跟踪
- 可以在线性化点附近设计非线性系统的局部控制器；一次线性化得到的 LQR 解不等于原非线性最优控制问题的全局解。

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

  通过引入拉格朗日乘子给出拉格朗日函数，得到 KKT 线性方程组，可直接求解该等式约束二次规划；数值实现时还需检查相应矩阵的可解性

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

下面先用 $N=4$、时不变 $A,B,Q,R$ 的示意例子推导，终端权重为 $Q_N$。时变情形需要给各步矩阵加上下标。

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
Q_Nx_4-\lambda_4=0 \Longrightarrow\lambda_4  =Q_Nx_4
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

数值实现应求解线性方程组来计算增益，避免显式计算逆矩阵。时变形式为 $K_n=(R_n+B_n^\top P_{n+1}B_n)^{-1}B_n^\top P_{n+1}A_n$、$P_n=Q_n+A_n^\top P_{n+1}(A_n-B_nK_n)$。

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

在输入零阶保持、$A^2=0$ 时，$A_d=I+A\Delta t$、$B_d=(I\Delta t+A\Delta t^2/2)B$。原式遗漏右乘 $B$，会使输入矩阵的维度错误。对本例：

$$
A_d=\begin{bmatrix}I_2&\Delta t I_2\\0&I_2\end{bmatrix},\qquad
B_d=\begin{bmatrix}\frac12\Delta t^2 I_2\\\Delta t I_2\end{bmatrix}.
$$

#### Part B: Finite Horizon LQR via Convex Optimization

定义性能指标和约束方程，使用 `Convex.jl`求解得到 `Xcvx,Ucvx = convex_trajopt(A,B,Q,R,Qf,N,x_ic)`

$$
\begin{align} \min_{x_{1:N},u_{1:N-1}} \quad & \sum_{i=1}^{N-1} \bigg[ \frac{1}{2} x_i^TQx_i + \frac{1}{2} u_i^TRu_i \bigg] + \frac{1}{2}x_N^TQ_fx_N\\ 
 \text{st} \quad & x_1 = x_{\text{IC}} \\ 
 & x_{i+1} = A x_i + Bu_i \quad \text{for } i = 1,2,\ldots,N-1 
 \end{align}
$$

初态$x_{ic} = [5,7,2,-1.4]$

<img src="/images/blog/HW2_Q1_2.svg" style="zoom:67%;" />

**验证 Bellman 最优性原理：**从原最优轨迹在时刻 $L$ 的状态 $x_L^*$ 出发，保留剩余时域、代价和约束，原最优控制序列的后缀仍是这个子问题的最优解。这不表示系统在任意中间时刻都已到达平衡点；若解不唯一，也不能要求重新求解后的轨迹必然逐点相同。

$$
\begin{align} \min_{x_{L:N},u_{L:N-1}} \quad & \sum_{i=L}^{N-1} \bigg[ \frac{1}{2} x_i^TQx_i + \frac{1}{2} u_i^TRu_i \bigg] + \frac{1}{2}x_N^TQ_fx_N\\ 
 \text{st     } \quad & x_L = x^*_L \\ 
 & x_{i+1} = A x_i + Bu_i \quad \text{for } i = L,L + 1,\ldots,N-1 
 \end{align}
$$

<img src="/images/blog/HW2_Q1_3.svg" style="zoom:67%;" />

#### Part C：Finite-Horizon LQR via Riccati

使用Riccati 方程递推求解离散LQR问题的解析解并与convex.jl的求解结果进行对比，结果是一致的

<img src="/images/blog/HW2_Q1_4.svg" style="zoom:67%;" />

多次随机初始状态结果也是一致的

<img src="/images/blog/HW2_Q1_5.svg" style="zoom:67%;" />

#### Part D: Why LQR is so great LQR的优异性

求出控制序列后，给实际的动力学系统加入噪声

$$
x_{k+1} = Ax_k + Bu_k + \text{noise}
$$

```julia
noise = [.005*randn(2);.1*randn(2)]
```

此处比较的是一次求解后直接执行的开环控制序列，与每步根据实际状态计算的反馈控制 $u_k=-K_kx_k$。抗扰效果来自反馈；对于同一个无约束 LQR 问题，QP 与 Riccati 是等价的求解方法，不能据此认定 Riccati 求解器本身更鲁棒。QP 若每步重新求解，也能形成反馈。

<img src="/images/blog/HW2_Q1_6.svg" style="zoom:67%;" />

设定非零目标 $x_{goal}=[-3.5,-3.5,0,0]$。对本例双积分系统，它在零输入下仍是平衡点，因此可在误差坐标中使用 $u=-K(x-x_{goal})$。一般系统需要先求满足平衡条件的 $(x_{goal},u_{goal})$，再使用 $u=u_{goal}-K(x-x_{goal})$。

<img src="/images/blog/HW2_Q1_8.svg" style="zoom:67%;" />

![](/images/blog/HW2_Q1_7.svg)

#### Part E: Infinite -horizon LQR 无限时间二次型调节问题

<img src="/images/blog/HW2_Q1_9.svg" style="zoom:67%;" />

<img src="/images/blog/HW2_Q1_10.svg" style="zoom:67%;" />

矩阵$P K$各元素的变化如图所示，可以看出对于一个有限时间的LQR系统，矩阵$P(n_x\times n_x)$和$K(n_u\times n_x)$在Ricatti反向迭代的过程中，经过一段时间就很快收敛。

对于时不变无限时域 LQR，在 $(A,B)$ 可稳定、$(Q^{1/2},A)$ 可检测且 $R\succ0$ 等标准条件下，可得到稳定化 Riccati 解和常数增益 $K$。有限时域曲线在本例中趋于平稳，不表示任意系统都能快速收敛。

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
- 重力向量$G=\begin{bmatrix}0\\m_pgl\sin\theta\end{bmatrix}$（此处 $\theta=0$ 为摆向下）
- 输入映射矩阵$B=\begin{bmatrix}1\\0\end{bmatrix}$

<img src="/images/blog/cartpole.png" style="zoom:33%;" />

使用Infinite Horizon LQR将倒立摆小车稳定到平衡位置

```julia
xgoal = [0, pi, 0, 0]
x0 = [0, pi, 0, 0] + [1.5, deg2rad(-20), .3, 0]
```

<img src="/images/blog/HW2_Q2_1.svg" style="zoom:67%;" />

<img src="/images/blog/HW2_Q2_1.gif" style="zoom:50%;" />

#### Part B : Basin of Attraction吸引域分析

LQR控制器是基于系统在$(x_{goal},u_{goal})$处的线性近似设计的，当系统状态远离线性化点时，真实非线性动力学与线性模型差异变大，导致控制器性能下降甚至失效。

这里测试LQR控制器在不同初始条件下的稳定性，绘制吸引域，即能成功稳定的初始状态范围。

```julia
# create a span of initial configurations 
M=20
ps = LinRange(-7, 7, M)
thetas = LinRange(deg2rad(180-60), deg2rad(180+60), M)
```

<img src="/images/blog/HW2_Q2_2.svg" style="zoom:67%;" />

#### Part C : 无限时域LQR调参

本例通过调整 $Q,R$ 并对输入限幅，检查给定初态下 5 秒末的误差是否小于 0.1。LQR 本身不显式处理 $-3\le u\le3$；截断控制会改变闭环系统，某次仿真满足条件不能保证其他初态也满足。需要硬约束时，应采用相应的约束优化控制方法。

<img src="/images/blog/HW2_Q2_3.svg" style="zoom:67%;" />

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

![](/images/blog/HW2_Q2_4.svg)

<img src="/images/blog/HW2_Q2_2.gif" style="zoom:50%;" />

## 整理与核查说明

本笔记原有许可为 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)。引用的课程材料、代码和图片仍须遵守其各自的许可。

**核查状态：部分验证（2026-10-04）。** 已核对离散化、LQR 与二次规划关系及维度；完整摆杆实验未重新运行。

2026-10-04 整理时修正了已定位的公式和实现问题。文中的图片、动画和输出保留自学习时的实验记录，不代表修订后的代码已经完整重跑。作业片段依赖原项目环境，不能直接作为完整可运行教程；具体核查范围与尚未复现事项见正文。
