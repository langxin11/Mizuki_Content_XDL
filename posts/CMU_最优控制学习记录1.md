---
title: CMU Optimal control and reinforcement learning 16-745 2025学习简要记录1
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
# CMU Optimal control and reinforcement learning 16-745 2025学习简要记录1

主要是配合Homework来的（在[课程官网答案](https://optimalcontrol.ri.cmu.edu/homeworks/)有完整的代码）

较早的更完整的课程记录可以参考[向阳的笔记](https://github.com/Zhihaibi/Optimal_control_16-745/blob/main/CMU16_745_Optimial%20control%20Lecture_Notes_zhihai%20Bi.pdf)和知乎[我爱科研](https://www.zhihu.com/column/c_1635315526615388160) 的整理

## lecture 1: Introduction and Dynamic review

$\dot{x}=f(x,u)$例子-单摆 $m g l sin \theta+m \ddot {\theta} l^2=u$

1. 状态空间方程

   $\dot X=A X+B U$

   取状态变量$X=(\theta,\dot \theta)^T$

   $$
   \begin{bmatrix}
   \dot{x}_1 \\
   \dot{x}_2
   \end{bmatrix}
   =
   \begin{bmatrix}
   \dot{\theta} \\
   \ddot{\theta}
   \end{bmatrix}
   =
   \begin{bmatrix}
   x_2 \\
   -\frac{g}{l} \sin\theta + \frac{u}{m l^2}
   \end{bmatrix}
   $$
2. 仿射系统方程
3. 机械臂系统方程

   $M(q) v + C(q,u)=B u$

   $M$质量矩阵，$C$科氏力
4. 线性系统

   $\dot X=f(X,U)->\dot{X} =\frac{df} {dX}X+\frac{df} {dU}U$
5. 平衡点

   $\dot{X}=0$代入动力学方程所处的平衡状态，改变控制输入可以改变平衡点
6. 平衡点稳定性

   $R e [\text{eig}\frac{df}{dx}]<0$

## lecture 2: Dynamics Discretization

1. 显式欧拉法
   $x_{k+1}=x_k+T f(x_k,u_k)$
2. RK4

   $x_{k+1} = x_k + 1/6 T (k_1 +k_2 +k_3 +k_3)$
3. 隐式中点法
   $x_{k+1}=x_k+T f(x_{k+1},u_k)$

## Lecture3 optimization part 1

### 基础概念（梯度，雅可比矩阵，Hessian矩阵）

1. 标量函数$ f:\mathbb{R} ^n→\mathbb{R}$的梯度向量(Gradient)

$$
\nabla f(\mathbf{x} )=\left [  \frac{\partial f}{\partial x_1}, \frac{\partial f}{\partial x_2},..,\frac{\partial f}{\partial x_n}\right ]^\top
$$

示例$f(x,y)=x^2+y^3$的梯度为$\nabla{f}=\left[2x,3y^2\right]^\top$

2. 向量值函数$\mathbf{F}(\mathbf{x}):\mathbb{R} ^n→\mathbb{R}^m$的雅可比矩阵(Jacobain)

$$
\mathbf{J}_{\mathbf{F}} = \begin{bmatrix}
\frac{\partial f_1}{\partial x_1} & \cdots & \frac{\partial f_1}{\partial x_n} \\
\vdots & \ddots & \vdots \\
\frac{\partial f_m}{\partial x_1} & \cdots & \frac{\partial f_m}{\partial x_n}
\end{bmatrix}
$$

$\mathbf{F}(x, y) = \begin{bmatrix} x^2 y \\ \sin y \end{bmatrix}$的雅可比矩阵为

$$
\mathbf{J} = \begin{bmatrix} 2xy & x^2 \\ 0 & \cos y \end{bmatrix}
$$

3. 标量函数$ f:\mathbb{R} ^n→\mathbb{R}$的Hessian矩阵：：$n\times n$对称矩阵

   $$
   \mathbf{H}_f = \begin{bmatrix}
   \frac{\partial^2 f}{\partial x_1^2} & \cdots & \frac{\partial^2 f}{\partial x_1 \partial x_n} \\
   \vdots & \ddots & \vdots \\
   \frac{\partial^2 f}{\partial x_n \partial x_1} & \cdots & \frac{\partial^2 f}{\partial x_n^2}
   \end{bmatrix}
   $$

   - 标量函数梯度的雅可比矩阵即是Hessian矩阵
   - $m=1$时，雅可比矩阵退化为梯度的转置
   - 行主导和列主导向量函数的复合求导会导致链式法则的式子不一样，由于矩阵的维度不一致)

$$
\frac{dF(g(\mathbf{x}))}{d\mathbf{x}}=\frac{dF}{d\mathbf{x}}|_{g(\mathbf{x})}\frac{dG(\mathbf{x})}{d\mathbf{x}}|_\mathbf{x}
$$

### HW1-Q1

Julia 求导————————ForwardDiff.jl

1. 标量函数$f(x)$对标量$x$的导数

```julia
import ForwardDiff as FD
function f(x)
    return x^2
end
x=randn()
dx=FD.derivative(f,x)
```

1. 标量函数$f(x)$对向量$X=[x_1,x_2,...,x_n]$的导数--雅可比矩阵/梯度
2. 向量函数$f(X)=[f_1(X),f_2(X),...,f_m(X),]$对向量$X=[x_1,x_2,...,x_n]$的Jacobian矩阵

### 求解一个非线性方程$f(x)=0$的方法（求根）

- 不动点迭代法

  离散系统动力学求平衡点

  $$
  x_{k+1}=f(x_k,u_k)\\
  f^*=x-f
  $$
- Newton方法

  $$
  f(x+dx)=f(x)+\frac{df}{dx}\Delta x=0\\
  \Delta x= -\frac{df}{dx}^{-1}f\\
  x \leftarrow x+\Delta x\\
  \text{Loop until convergence}
  $$

在无约束优化问题中，可以用Newton法求解$\Delta g=0$（一阶必要条件）

### HW1_S25_Q3 QP求解器（Log-Domain Interior Point Method）

$$
\begin{align}
\min_x \quad & \frac{1}{2}x^TQx + q^Tx \\ 
\text{s.t.}\quad &  Ax -b = 0 \\ 
&  Gx - h \geq 0 
\end{align}
$$

引入拉格朗日乘子

- $\mu \in \mathcal{R}^p$  对应等式约束
- $\lambda \in \mathcal{R}^m$对应不等式约束（要求  $\lambda≥ 0$）

拉格朗日函数为：

$$
\mathcal{L}(x, \mu,\lambda) = \frac{1}{2}x^\top Q x + q^\top x 
+ \mu^\top (Ax - b)
- \lambda^\top (Gx - h)
$$

KKT条件：梯度条件、原始可行性、对偶可行性、互补松弛条件

$$
\begin{align}
Qx+q+A^\top\mu - G^\top \lambda&= 0 \quad \quad \text{(stationarity)} \\
Ax - b&= 0 \quad \quad \text{(primal feasibility)} \\
 Gx - h &\geq 0 \quad \quad \text{(primal feasibility)} \\
\lambda &\geq 0 \quad \quad \text{(dual feasibility)} \\
\lambda \circ(Gx - h) &= 0 \quad \quad \text{(complementarity)} 
\end{align}
$$

映入非负松弛变量$s>0$使得$Gx-h=s$

新的拉格朗日函数

$$
\mathcal{L}(x, \lambda, \mu) = \frac{1}{2}x^\top Q x + q^\top x 
+ \mu^\top (Ax - b)
+ \lambda^\top [s-(Gx - h)]
$$

$$
\begin{align}
Qx+q+A^\top\mu - G^\top \lambda&= 0 \quad \quad \text{(stationarity)} \\
Ax - b&= 0 \quad \quad \text{(primal feasibility)} \\
 Gx - h-s &=0 \quad \quad \text{(primal feasibility)} \\
\lambda &\geq 0 \quad \quad \text{(dual feasibility)} \\
s &\geq 0\\
\lambda^\top \circ s &= \mathbf{1}^\top \rho \quad \quad \text{(complementarity)} 
\end{align}
$$

令$\lambda=\sqrt{\rho}e^{-\sigma},s=\sqrt{\rho}e^{\sigma}$，得到转换后的ip_KKT条件方程

$$
\begin{align}
Qx+q+A^\top\mu - G^\top \sqrt{\rho}e^{-\sigma}&= 0 \quad \quad \text{(stationarity)} \\
Ax - b&= 0 \quad \quad \text{(primal feasibility)} \\
 Gx - h-\sqrt{\rho}e^{\sigma} &=0 \quad \quad \text{(primal feasibility)} \\
\end{align}
$$

新的拉格朗日函数$\mathcal{L}(x, \lambda, \sigma)$对应的Jacobian和Hessian矩阵如下

$$
\mathbf{J}=\begin{bmatrix}
Qx+q+A^\top\mu - G^\top \sqrt{\rho}e^{-\sigma}&\\
Ax - b\\
Gx - h-\sqrt{\rho}e^{\sigma}\\
\end{bmatrix}
$$

$$
\mathbf{H}=\begin{bmatrix}
Q& A^\top& G^\top \text{diag}(\sqrt{\rho}\odot e^{-\sigma})\\
A&\mathbf{0}&\mathbf{0}\\
G&\mathbf{0} &\text{diag}( \sqrt{\rho}\odot e^{-\sigma})  \\
\end{bmatrix}
$$

这样我们求解QP问题就转换为求解用Newton法求解关于$z=[x; \mu;\sigma]$的KKT方程（求根）

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/image-20250716230521679.png" style="zoom: 50%;" />

这里的$P_0$和$P_1$代表原始KKT和IP_KKT的残差，也就是

$$
\begin{align}
P_0=\begin{bmatrix}
  Qx+q+A^\top\mu - G^\top \lambda&\\
  Ax - b&\\
  min.(Gx - h,\mathbf{0})\\
  min.(\lambda,\mathbf{0})\\ 
  \lambda \circ(Gx - h) & 
\end{bmatrix}
\end{align}
$$

$$
\begin{align}
P_1=\begin{bmatrix}
  Qx+q+A^\top\mu - G^\top \lambda&\\
  Ax - b&\\
 Gx - h-s\\
\end{bmatrix}
\end{align}
$$

#### 砖块掉落仿真

不考虑砖块的旋转，一个掉落的砖块的动力学方程可以写成

$$
M \dot{v}  + M g = J^T \mu \\ \text{ where } M = mI_{2\times 2}, \; g = \begin{bmatrix} 0 \\ 9.81 \end{bmatrix},\; J = \begin{bmatrix} 0 & 1 \end{bmatrix}
$$

其中，$v=[v_x;v_z]$是砖块的水平速度和竖直速度，$q=[q_x;q_z]$是砖块的水平和竖直位移，$\lambda$是地面给砖块的法向接触力，通过backward Euler离散

$$
\begin{bmatrix} v_{k+1} \\ q_{k+1} \end{bmatrix} = \begin{bmatrix} v_k \\ q_k \end{bmatrix}+ \Delta t \cdot \begin{bmatrix} \frac{1}{m} J^T \mu_{k+1} - g \\ v_{k+1} \end{bmatrix}
$$

约束

$$
\begin{align}
J q_{k+1} &\geq 0 &&\text{(不会穿透地面)} \\
\mu_{k+1} &\geq 0 &&\text{(接触力方向只向上)} \\
\mu_{k+1} J q_{k+1} &= 0 &&\text{(没接触无接触力)}
\end{align}
$$

等价转换成如下的QP问题(可以通过KKT条件证明)

$$
\begin{align}
    &\text{minimize}_{v_{k+1}} && \frac{1}{2} v_{k+1}^T M v_{k+1} + [M (\Delta t \cdot g - v_k)]^Tv_{k+1} \\
    &\text{subject to} && J(q_k + \Delta t \cdot v_{k+1}) \geq 0 \\
\end{align}
$$

引入拉格朗日乘子$\mu \geq 0$

$$
\mathcal{L}=\frac{1}{2} v_{k+1}^T M v_{k+1} + [M (\Delta t \cdot g - v_k)]^Tv_{k+1} -\mu \cdot J(q_k + \Delta t \cdot v_{k+1})
$$

KKT 条件

$$
\begin{align}
Mv_{k+1} + M(\Delta{t}\cdot g-v_k)^\mathrm{T}-J^\mathrm{T}\mu\Delta t&=0\qquad\text{梯度条件}\\
J(q_k + \Delta t \cdot v_{k+1}) &\geq 0\qquad\text{原始可行性}\\
\mu &\geq 0\qquad\text{对偶可行性}\\
\mu \cdot J(q_k + \Delta t \cdot v_{k+1})&=0\qquad\text{互补松弛 }
\end{align}
$$

引入松弛能量$s=\sqrt{\rho}e^{-\sigma}$，转换

$$
\begin{align}
Mv_{k+1} + M(\Delta{t}\cdot g-v_k)^\mathrm{T}-J^\mathrm{T}\mu\Delta t&=0\\
J(q_k + \Delta t \cdot v_{k+1}) -\sqrt{\rho}e^{-\sigma}&= 0\\

\end{align}
$$

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/砖块掉落仿真.gif" style="zoom: 50%;" />

## Julia 简介

1. 下载：windows 通过winget下载juliaup下载指定julia版本

   在vscode中配置julia环境，安装插件Julia,通过juliaup 安装release版本以启用julia语言服务器

   ```powershell
   winget install julia -s msstore`#安装juliaup
   juliaup add release
   ```
2. julia虚拟环境创建

   ```julia
   #创建虚拟环境
   using Pkg
   Pkg.activate(@__DIR__)
   #初始化虚拟环境
   Pkg.instantiate()
   #安装包
   Pkg.add("Plots")
   #检查虚拟环境状态
   Pkg.status()
   ```
3. pyplot的使用

   ```julia
   # 在 Julia 中执行（替换为你的 Python 路径）
   ENV["PYTHON"] = raw"C:\Python39\python.exe"  # Windows 示例
   # ENV["PYTHON"] = "/usr/bin/python3"        # Linux/macOS 示例

   # 重新构建 PyCall
   using Pkg
   Pkg.build("PyCall")
   # 显示图片
   display(gcf())
   ```
4. eltype和typeof

   ```julia
   #返回容器（collection）里元素的类型（数组、向量、矩阵、迭代器）
   eltype([1, 2, 3])          # Int64
   eltype([1.0, 2.0])         # Float64
   eltype(["a", "b"])         # String
   eltype(1:10)               # Int64
   eltype(1.0:0.1:2.0)        # Float64
   #返回某个值的具体类型
   typeof(1)            # Int64
   typeof(1.0)          # Float64
   typeof("hello")      # String
   typeof(true)         # Bool
   ```
5. vector{vector}和Martix的转换

   ```julia

   ```
