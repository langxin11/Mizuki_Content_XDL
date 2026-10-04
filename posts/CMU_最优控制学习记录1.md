---
title: CMU 最优控制笔记 1：动力学、离散化与数值优化
author: Daliang
published: 2025-07-28
updated: 2026-10-04
description: 从动力学建模和数值积分出发，整理梯度、牛顿法、约束优化及 Julia 作业实践。
toc: true
toc-depth: 4
toc-title: Contents
tags:
  - 最优控制
  - 动力学
  - 数值优化
  - Julia
category: "CMU Optimal Control 16-745"
licenseName: "CC BY 4.0"
---

我从连续时间动力学开始整理这组学习笔记，梳理离散化、求导与数值优化的基础工具，为后续 LQR 和 MPC 做准备。

**先修知识：**线性代数、多元微积分，以及基本的 Julia 语法。

**阅读路线：**

1. 先建立状态、输入和动力学方程，再讨论平衡点附近的线性化。
2. 比较显式欧拉、RK4 和隐式中点法，关注离散化误差与稳定性。
3. 结合作业理解自动微分、牛顿法及二次规划中的约束处理。

笔记结合 CMU 16-745 的课程与作业整理，相关材料见[课程作业页面](https://optimalcontrol.ri.cmu.edu/homeworks/)。文中的代码片段保留学习时的实现，运行时还需使用对应作业的依赖与上下文。

其他学习视角可参考[向阳的笔记](https://github.com/Zhihaibi/Optimal_control_16-745/blob/main/CMU16_745_Optimial%20control%20Lecture_Notes_zhihai%20Bi.pdf)和知乎[我爱科研](https://www.zhihu.com/column/c_1635315526615388160) 的整理

## Lecture 1：动力学、平衡点与线性化

以质量为 $m$、长度为 $l$、输入为关节力矩 $u$ 的理想单摆为例：

$$
m l^2\ddot{\theta}+mgl\sin\theta=u.
$$

取状态 $x=[\theta,\dot\theta]^\top$，连续时间模型为

$$
\dot{x}=f(x,u)=
\begin{bmatrix}
x_2\\
-\frac{g}{l}\sin x_1+\frac{u}{ml^2}
\end{bmatrix}.
$$

它对状态是非线性的，对输入则是仿射的。更一般地，输入仿射系统可以写成 $\dot{x}=f_0(x)+G(x)u$。

机械系统常用的动力学形式为

$$
M(q)\ddot q+C(q,\dot q)\dot q+g(q)=Bu,
$$

其中 $M$ 为惯性矩阵，$C(q,\dot q)\dot q$ 表示科里奥利力与离心力项，$g(q)$ 为重力项。此前的笔记把加速度写成了速度，此处作了修正。

在平衡点 $(\bar x,\bar u)$ 附近，先令 $f(\bar x,\bar u)=0$，再定义扰动 $\delta x=x-\bar x$、$\delta u=u-\bar u$，得到一阶近似

$$
\delta\dot x\approx A\delta x+B\delta u,\qquad
A=\left.\frac{\partial f}{\partial x}\right|_{\bar x,\bar u},\quad
B=\left.\frac{\partial f}{\partial u}\right|_{\bar x,\bar u}.
$$

在输入固定、模型足够光滑时，若 $A$ 的所有特征值实部严格为负，可判断该平衡点局部渐近稳定；出现零实部特征值时，仅靠这个线性化判据不能下结论。参考[课程的动力学笔记](https://optimalcontrol.ri.cmu.edu/course_notes/dynamics/lec1/)。

## Lecture 2：动力学离散化

以下假设每个采样区间内输入保持为 $u_k$，采样周期为 $h$。

### 显式欧拉法

$$
x_{k+1}=x_k+h f(x_k,u_k).
$$

### 经典四阶 Runge–Kutta（RK4）

$$
\begin{aligned}
k_1&=f(x_k,u_k),\\
k_2&=f(x_k+\tfrac h2 k_1,u_k),\\
k_3&=f(x_k+\tfrac h2 k_2,u_k),\\
k_4&=f(x_k+h k_3,u_k),\\
x_{k+1}&=x_k+\tfrac h6(k_1+2k_2+2k_3+k_4).
\end{aligned}
$$

此前的笔记遗漏了中间两项的系数，并把最后一项误写为 $k_3$；上式为修正后的表达。

### 隐式中点法

$$
x_{k+1}=x_k+h f\left(\frac{x_k+x_{k+1}}2,u_k\right).
$$

由于右端含有未知的 $x_{k+1}$，每一步通常需要求解隐式方程。原先写的 $x_{k+1}=x_k+h f(x_{k+1},u_k)$ 是后向欧拉法，不是隐式中点法。

比较积分方法时，应同时考察步长、误差、稳定性和每一步的求解成本。课程材料见 [Dynamics Discretization & Stability](https://optimalcontrol.ri.cmu.edu/lectures/)。

## Lecture 3：数值优化基础

### 基础概念（梯度，雅可比矩阵，Hessian矩阵）

1. 标量函数$ f:\mathbb{R} ^n→\mathbb{R}$的梯度向量(Gradient)

$$
\nabla f(\mathbf{x} )=\left [  \frac{\partial f}{\partial x_1}, \frac{\partial f}{\partial x_2},..,\frac{\partial f}{\partial x_n}\right ]^\top
$$

示例$f(x,y)=x^2+y^3$的梯度为$\nabla{f}=\left[2x,3y^2\right]^\top$

2. 向量值函数$\mathbf{F}(\mathbf{x}):\mathbb{R} ^n→\mathbb{R}^m$的雅可比矩阵(Jacobian)

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

3. 标量函数$ f:\mathbb{R} ^n→\mathbb{R}$的Hessian 矩阵：$n\times n$对称矩阵

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
J_{F\circ g}(\mathbf{x})=J_F(g(\mathbf{x}))J_g(\mathbf{x})
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
  \begin{gathered}
  x_{k+1}=f(x_k,u_k)\\
  f^*=x-f
  \end{gathered}
  $$
- Newton方法

  $$
  \begin{gathered}
  f(x+dx)=f(x)+\frac{df}{dx}\Delta x=0\\
  \Delta x= -\frac{df}{dx}^{-1}f\\
  x \leftarrow x+\Delta x\\
  \text{Loop until convergence}
  \end{gathered}
  $$

在无约束优化问题中，可以用 Newton 法求解 $\nabla f(x)=0$（一阶必要条件）；驻点是否为局部极小值还需要进一步判断

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

引入非负松弛变量 $s\geq0$，使得 $Gx-h=s$；内点迭代时取 $s>0$。

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
\lambda \circ s &= \rho\mathbf{1} \quad \quad \text{(perturbed complementarity)}
\end{align}
$$

这里使用 $\rho>0$ 的扰动互补条件；原始 KKT 的右侧应为零。令 $\lambda=\sqrt{\rho}e^{-\sigma},s=\sqrt{\rho}e^{\sigma}$（指数逐元素计算），得到内点中心路径上的方程。需要逐步减小 $\rho$ 才能逼近原问题，不能固定一个正数就宣称解满足原始互补条件。

$$
\begin{align}
Qx+q+A^\top\mu - G^\top \sqrt{\rho}e^{-\sigma}&= 0 \quad \quad \text{(stationarity)} \\
Ax - b&= 0 \quad \quad \text{(primal feasibility)} \\
 Gx - h-\sqrt{\rho}e^{\sigma} &=0 \quad \quad \text{(primal feasibility)} \\
\end{align}
$$

定义关于 $z=[x;\mu;\sigma]$ 的残差向量 $r_\rho(z)$，及其 Jacobian $D r_\rho(z)$。残差向量不是 Jacobian，下面的分块矩阵也不是目标函数的 Hessian。

$$
r_\rho(z)=\begin{bmatrix}
Qx+q+A^\top\mu - G^\top \sqrt{\rho}e^{-\sigma}&\\
Ax - b\\
Gx - h-\sqrt{\rho}e^{\sigma}\\
\end{bmatrix}
$$

$$
D r_\rho(z)=\begin{bmatrix}
Q& A^\top& G^\top \text{diag}(\sqrt{\rho}\odot e^{-\sigma})\\
A&\mathbf{0}&\mathbf{0}\\
G&\mathbf{0} &-\text{diag}( \sqrt{\rho}e^{\sigma})  \\
\end{bmatrix}
$$

每个 Newton 步求解 $D r_\rho(z)\Delta z=-r_\rho(z)$，再配合线搜索和中心路径参数更新。这是算法推导，尚未构成完整求解器；终止时还需检查原始可行性、对偶可行性和互补残差。这里假设 $Q$ 对称半正定；一般非凸 QP 的 KKT 驻点不能直接认定为全局最优解。

<img src="/images/blog/image-20250716230521679.png" style="zoom: 50%;" />

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
\begin{gathered}
M \dot{v}  + M g = J^T \mu \\ \text{ where } M = mI_{2\times 2}, \; g = \begin{bmatrix} 0 \\ 9.81 \end{bmatrix},\; J = \begin{bmatrix} 0 & 1 \end{bmatrix}
\end{gathered}
$$

其中，$v=[v_x;v_z]$ 是速度，$q=[q_x;q_z]$ 是位置，竖直向上为正；$\mu$ 是法向接触力。这里忽略旋转、摩擦和反弹，采用非穿透接触与 backward Euler 离散，不能直接当作一般刚体碰撞模型。

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
Mv_{k+1} + M(\Delta{t}\cdot g-v_k)-J^\mathrm{T}\mu\Delta t&=0\qquad\text{梯度条件}\\
J(q_k + \Delta t \cdot v_{k+1}) &\geq 0\qquad\text{原始可行性}\\
\mu &\geq 0\qquad\text{对偶可行性}\\
\mu \cdot J(q_k + \Delta t \cdot v_{k+1})&=0\qquad\text{互补松弛 }
\end{align}
$$

引入间隙松弛变量 $s=J(q_k+\Delta t\,v_{k+1})$，并令 $\mu=\sqrt{\rho}e^{-\sigma}$、$s=\sqrt{\rho}e^{\sigma}$，使 $\mu s=\rho$。这是与上文一致的数值参数化；在本例中 $\rho$ 带有接触力乘间隙的单位，实际实现还应考虑尺度归一化。

$$
\begin{align}
Mv_{k+1} + M(\Delta{t}\cdot g-v_k)-J^\mathrm{T}\sqrt{\rho}e^{-\sigma}\Delta t&=0\\
J(q_k + \Delta t \cdot v_{k+1}) -\sqrt{\rho}e^{\sigma}&= 0\\

\end{align}
$$

<img src="/images/blog/%E7%A0%96%E5%9D%97%E6%8E%89%E8%90%BD%E4%BB%BF%E7%9C%9F.gif" style="zoom: 50%;" />

## Julia 简介

1. 下载：windows 通过winget下载juliaup下载指定julia版本

   在 VS Code 中安装 Julia 扩展，并检查自动发现的 Julia 路径。只有日志明确提示缺少 Juliaup `release` 通道时，才按提示补装；这不是所有补全故障的通用修复。

   ```powershell
   winget install --name Julia --id 9NJNWW8PVKMN -e -s msstore
   juliaup status
   # 若扩展提示缺少 release 通道，再执行 juliaup add release
   ```
   安装命令见 [Juliaup 官方说明](https://github.com/JuliaLang/juliaup)。`release` 随时间变化，课程复现应保留原项目的 `Project.toml`、`Manifest.toml` 和 Julia 版本。

2. Julia 项目环境创建（管理依赖，不等同于隔离整个解释器的 Python 虚拟环境）

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
5. 向量集合与矩阵的转换

   将等长向量按列拼成矩阵，再用 `eachcol` 拆回列向量：

   ```julia
   columns = [[1, 2], [3, 4]]
   A = hcat(columns...)
   restored = [collect(column) for column in eachcol(A)]
   ```

## 整理与核查说明

本笔记原有许可为 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)。引用的课程材料、代码和图片仍须遵守其各自的许可。

**核查状态：部分验证（2026-10-04）。** 已核对主要公式，并用小型数值例子检验 KKT Jacobian；整套课程作业未重新运行。

2026-10-04 整理时修正了已定位的公式和实现问题。文中的图片、动画和输出保留自学习时的实验记录，不代表修订后的代码已经完整重跑。作业片段依赖原项目环境，不能直接作为完整可运行教程；具体核查范围与尚未复现事项见正文。
