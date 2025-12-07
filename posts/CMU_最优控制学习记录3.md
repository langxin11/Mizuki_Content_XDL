---
title: CMU Optimal control and reinforcement learning 16-745 2025学习简要记录3
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
# CMU Optimal control and reinforcement learning 16-745 2025学习简要记录3

主要是配合Homework来的（在[课程官网答案](https://optimalcontrol.ri.cmu.edu/homeworks/)有完整的代码）

较早的更完整的课程记录可以参考[向阳的笔记](https://github.com/Zhihaibi/Optimal_control_16-745/blob/main/CMU16_745_Optimial%20control%20Lecture_Notes_zhihai%20Bi.pdf)和知乎[我爱科研](https://www.zhihu.com/column/c_1635315526615388160) 的整理

## Lecture10 Convex Model-Predictive Control

LQR（线性二次调节器）是控制理论中的经典方法，但存在一些局限

- 仅适用于**线性系统**和局部线性化的非线性系统
- 代价函数需要是**二次型**
- 无法直接处理**控制输入约束**或**状态约束**

MPC通过**滚动优化**克服LQR的局限性，在每一个时间步求解一个有限时域的优化问题，考虑未来若干步的动力学和约束，并仅应用优化结果的第一步控制输入，下一时间步重新优化，具有以下优势

- **显示处理约束**：将控制限幅、状态约束直接写入优化问题
- 兼容非线性：通过数值优化处理非线性动力学或代价函数
- **适应性**：可实时响应环境变化（如障碍物移动）

### HW2_Q3 Optimal Rendezvous and Docking航天器交汇

接下来将针对SpaceX Dragon飞船与国际空间站（ISS）的交会对接，使用**LQR**、**凸轨迹优化**、**凸MPC**三种控制方法

状态变量$x \in \mathbb{R}^6$为$x,y,z$的位置和速度，控制变量$u \in \mathbb{R}^3$为飞船三轴推力

$$
\begin{align}
x &= [r_x, r_y, r_z, v_x, v_y, v_z]^T,\\
u &= [t_x, t_y, t_z]^T \end{align}
$$

系统的连续时间动力学模型$\dot{x}=Ax+Bu$如下(Clohessy-Wiltshire 方程)

$$
\begin{align}
\dot{x} &= \begin{bmatrix}0  &   0 & 0  &  1 &  0 &  0 \\  
         0 &    0 & 0  &  0 &  1 &  0 \\ 
         0 &    0 & 0 &   0 &  0 &  1\\
         3n^2 &0 & 0  &  0 &  2n &0 \\
         0  &   0 & 0  & -2n &0  & 0\\
         0  &   0 &-n^2 & 0 &  0 &  0 \end{bmatrix} + \begin{bmatrix} 0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \\ 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix} u
\end{align}
$$

+ $A$矩阵包含轨道动力学效应（科里奥利力、离心力）
+ $n=\sqrt{u/a^3}$为轨道角速度，其中地球标准动力参数$\mu=3.986\times10^14\bf m^3/s^2$，ISS轨道的半长轴$a=6778 \bf km$

#### Part A: Discretize the dynamics系统离散化

使用增广矩阵进行系统离散化

```julia
function create_dynamics(dt::Real)::Tuple{Matrix,Matrix}
    mu = 3.986004418e14 # standard gravitational parameter
    a = 6971100.0       # semi-major axis of ISS
    n = sqrt(mu/a^3)    # mean motion

    # continuous time dynamics ẋ = Ax + Bu
    A = [0     0  0    1   0   0; 
         0     0  0    0   1   0;
         0     0  0    0   0   1;
         3*n^2 0  0    0   2*n 0;
         0     0  0   -2*n 0   0;
         0     0 -n^2  0   0   0]
   
    B = Matrix([zeros(3,3);0.1*I(3)])

    # TODO: convert to discrete time X_{k+1} = Ad*x_k + Bd*u_k
    nx,nu =size(B)
    M = exp([A B; zeros(3,6) zeros(3,3)] * dt)  # 增广矩阵
    Ad = M[1:nx, 1:nx]
    Bd = M[1:nx, nx+1:nx+nu]
    return Ad, Bd
end
```

#### Part B:LQR

使用有线时域LQR跟踪给定的参考轨迹

$$
\begin{align} \min_{x_{1:N},u_{1:N-1}} \quad & \sum_{i=1}^{N-1} \bigg[ \frac{1}{2} (x_i - x_{ref, i})^TQ(x_i - x_{ref, i}) + \frac{1}{2} u_i^TRu_i \bigg] + \frac{1}{2}(x_N- x_{ref, N})^TQ_f
(x_N- x_{ref, N})\\ 
 \text{st} \quad & x_1 = x_{\text{IC}} \\ 
 & x_{i+1} = A x_i + Bu_i \quad \text{for } i = 1,2,\ldots,N-1 
 \end{align}
$$

求得控制策略$u_i = -K_i(x_i - x_{ref, i})$，并进行限幅 `clamp.(u, u_min, u_max)`

```julia
# TODO: FHLQR 
function fhlqr(A::Matrix, # A matrix 
           B::Matrix, # B matrix 
           Q::Matrix, # cost weight 
           R::Matrix, # cost weight 
           Qf::Matrix,# term cost weight 
           N::Int64   # horizon size 
           )::Tuple{Vector{Matrix{Float64}}, Vector{Matrix{Float64}}} # return two matrices 

    # check sizes of everything 
    nx,nu = size(B)
    @assert size(A) == (nx, nx)
    @assert size(Q) == (nx, nx)
    @assert size(R) == (nu, nu)
    @assert size(Qf) == (nx, nx)

    # instantiate S and K 
    P = [zeros(nx,nx) for i = 1:N]
    K = [zeros(nu,nx) for i = 1:N-1]

    # initialize S[N] with Qf 
    P[N] = deepcopy(Qf)

    # Ricatti 
    for n in N-1:-1:1
        # TODO
        K[n] = (R+B'*P[n+1]B)\B'*P[n+1]*A
        P[n] = Q + A'*P[n+1]*(A-B*K[n])
    end

    return P, K 
end

# Solve LQR
_, K = fhlqr(A,B,Q,R,Qf,N)

# simulation 
X_sim = [zeros(nx) for i = 1:N]
U_sim = [zeros(nu) for i = 1:N-1]
X_sim[1] = x0 
for i = 1:(N-1) 
    # TODO: put LQR control law here 
    # make sure to clamp 
    U_sim[i] = clamp.(-K[i]*(X_sim[i]-X_ref[i]),u_min,u_max)

    # simulate 1 step 
    X_sim[i+1] = A*X_sim[i] + B*U_sim[i]
end
```

![](https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q3_LQR.svg)

#### Part C: Convex Trajectory Optimization

$$
\begin{align} \min_{x_{1:N},u_{1:N-1}} \quad & \sum_{i=1}^{N-1} \bigg[ \frac{1}{2} (x_i - x_{ref, i})^TQ(x_i - x_{ref, i}) + \frac{1}{2} u_i^TRu_i \bigg] \\ 

 \text{st} \quad & x_1 = x_{\text{IC}} \\ 

 & x_{i+1} = A x_i + Bu_i \quad \text{for } i = 1,2,\ldots,N-1  \\ 

 & u_{min} \leq u_i \leq u_{max} \quad \text{for } i = 1,2,\ldots,N-1 \\

 & x_i[2] \leq x_{goal} [2]\quad \text{for } i = 1,2,\ldots,N \\ 

 & x_N = x_{goal}

 \end{align}
$$

```julia
"""
Xcvx,Ucvx = convex_trajopt(A,B,X_ref,x0,xg,u_min,u_max,N)

setup and solve the above optimization problem, returning 
the solutions X and U, after first converting them to 
vectors of vectors with vec_from_mat(X.value)
"""
function convex_trajopt(A::Matrix, # discrete dynamics A 
                        B::Matrix, # discrete dynamics B 
                        X_ref::Vector{Vector{Float64}}, # reference trajectory 
                        x0::Vector, # initial condition 
                        xg::Vector, # goal state 
                        u_min::Vector, # lower bound on u 
                        u_max::Vector, # upper bound on u
                        N::Int64, # length of trajectory 
                        )::Tuple{Vector{Vector{Float64}}, Vector{Vector{Float64}}} # return Xcvx,Ucvx
  
    # get our sizes for state and control
    nx,nu = size(B)
  
    @assert size(A) == (nx, nx)
    @assert length(x0) == nx 
    @assert length(xg) == nx 
  
    # LQR cost
    Q = diagm(ones(nx))
    R = diagm(ones(nu))

    # variables we are solving for
    X = cvx.Variable(nx,N)
    U = cvx.Variable(nu,N-1)

    # TODO: implement cost
    obj = 0
    for k =1:N-1
        x_k,u_k = X[:,k]-X_ref[k],U[:,k]
        obj += 0.5*cvx.quadform(x_k,Q)+0.5*cvx.quadform(u_k,R)
    end

    # create problem with objective
    prob = cvx.minimize(obj)

    # TODO: add constraints with prob.constraints = vcat(prob.constraints, ...)
    prob.constraints = vcat(prob.constraints,(X[:,1]==x0))
    prob.constraints = vcat(prob.constraints,(X[:,end]==xg))
    for i =1:N-1
        #dynamics constraints
        prob.constraints = vcat(prob.constraints,(X[:,i+1]==A*X[:,i]+B*U[:,i]))
        # control constraint
        prob.constraints = vcat(prob.constraints,(U[:,i]<=u_max))
        prob.constraints = vcat(prob.constraints,(U[:,i]>=u_min))
    end

    for i = 1:N
        prob.constraints = vcat(prob.constraints,(X[2,i]<=xg[2]))
    end
    cvx.solve!(prob, ECOS.Optimizer; silent = true)

    X = X.value
    U = U.value
  
    Xcvx = vec_from_mat(X)
    Ucvx = vec_from_mat(U)
  
    return Xcvx, Ucvx
end
```

![](https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q3_convex.svg)

#### Part D: Convex MPC

在航天器交会对接任务中，（Part C）开环控制无法处理系统中的不确定性，MPC通过滚动时域优化使用反馈控制，弥补“sim-to-real gap”

给定当前时刻的参考轨迹窗口$\tilde{x}_{ref} = x_{ref}[i,(i + N_{mpc} - 1)]$，MPC将求解以下的凸优化问题

$$
\begin{align} \min_{x_{1:N},u_{1:N-1}} \quad & \sum_{i=1}^{N-1} \bigg[ \frac{1}{2} (x_i - \tilde{x}_{ref, i})^TQ({x}_i - \tilde{x}_{ref, i}) + \frac{1}{2} u_i^TRu_i \bigg] + \frac{1}{2}(x_N- \tilde{x}_{ref, N})^TQ
({x}_N- \tilde{x}_{ref, N})\\ 
 \text{st} \quad & x_1 = x_{\text{IC}} \\ 
 & x_{i+1} = A x_i + Bu_i \quad \text{for } i = 1,2,\ldots,N-1  \\ 
 & u_{min} \leq u_i \leq u_{max} \quad \text{for } i = 1,2,\ldots,N-1 \\
 & x_i[2] \leq x_{goal} [2]\quad \text{for } i = 1,2,\ldots,N 
 \end{align}
$$

参数说明：

- $Q,R,Q_f$:状态、控制输入和终端状态的权重矩阵
- $N_mpc$：预测时域长度
- $x_\mathbf{IC}$:当前状态估计（来自传感器滤波）

```julia
"""
`u = convex_mpc(A,B,X_ref_window,xic,xg,u_min,u_max,N_mpc)`

setup and solve the above optimization problem, returning the 
first control u_1 from the solution (should be a length nu 
Vector{Float64}).  
"""
function convex_mpc(A::Matrix, # discrete dynamics matrix A
                    B::Matrix, # discrete dynamics matrix B
                    X_ref_window::Vector{Vector{Float64}}, # reference trajectory for this window 
                    xic::Vector, # current state x 
                    xg::Vector, # goal state 
                    u_min::Vector, # lower bound on u 
                    u_max::Vector, # upper bound on u 
                    N_mpc::Int64,  # length of MPC window (horizon)
                    )::Vector{Float64} # return the first control command of the solved policy 
  
    # get our sizes for state and control
    nx,nu = size(B)
  
    # check sizes 
    @assert size(A) == (nx, nx)
    @assert length(xic) == nx 
    @assert length(xg) == nx 
    @assert length(X_ref_window) == N_mpc 
  
    # LQR cost
    Q = diagm(ones(nx))
    R = diagm(ones(nu))
    Qf = 10*Q

    # variables we are solving for
    X = cvx.Variable(nx,N_mpc)
    U = cvx.Variable(nu,N_mpc-1)

    # TODO: implement cost function
    obj = cvx.quadform(X[:,N_mpc]-X_ref_window[N_mpc],Qf)
    for i = 1:N_mpc-1
        obj +=cvx.quadform(X[:,i]-X_ref_window[i],Q)+cvx.quadform(U[:,i],R)
    end
    # create problem with objective
    prob = cvx.minimize(obj)

    # TODO: add constraints with prob.constraints = vcat(prob.constraints, ...)
    prob.constraints = vcat(prob.constraints,(X[:,1]==xic))
    for i =1:N_mpc-1
        #dynamics constraints
        prob.constraints = vcat(prob.constraints,(X[:,i+1]==A*X[:,i]+B*U[:,i]))
        # control constraint
        prob.constraints = vcat(prob.constraints,(U[:,i]<=u_max))
        prob.constraints = vcat(prob.constraints,(U[:,i]>=u_min))
    end

    for i = 1:N_mpc
        prob.constraints = vcat(prob.constraints,(X[2,i]<=xg[2]))
    end

    # solve problem 
    cvx.solve!(prob, ECOS.Optimizer; silent = true)

    # get X and U solutions 
    X = X.value
    U = U.value
  
    # return first control U 
    return U[:,1]
end
```

![](https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q3_MPC.svg)

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW2_Q3_MPC.gif" style="zoom: 33%;" />

### 无人机悬停案例

1.平面无人机动力学

$$
\begin{align}

\ddot{x}&=\frac{1}{m}(u_1+u_2)sin\theta\\
\ddot{y}&=\frac{1}{m}(u_1+u_2)cos\theta-g\\
\ddot{\theta}&=\frac{1}{J}l^2(u_2-u_1)\\
\end{align}
$$

在平衡点线性化$(u_1=u_2 =\frac{1}{2}mg,\theta=0)$写成矩阵形式

$$
\Longrightarrow
\begin{align}
\Delta\ddot{x}&=g\theta\\
\Delta\ddot{y}&=\frac{1}{m}(\Delta u_1+\Delta u_2)\\
\Delta\ddot{\theta}&=\frac{1}{J}l^2(\Delta u_2-\Delta u_1)\\
\end{align}
$$

$$
\begin{align}
\begin{bmatrix}\Delta\dot{x}\\ \Delta\dot{y} \\ \Delta\dot{\theta} \\\Delta\ddot{x}\\ \Delta\ddot{y} \\ \Delta\ddot{\theta}\end{bmatrix}=
\begin{bmatrix}0&0 &0 & 1&0 &0  \\ 0& 0&0 &0 &1 &0  \\0&0 &0 &0 &0 &1\\
0&0&g&0&0&0\\0&0&0&0&0&0\\0&0&0&0&0&0\end{bmatrix}

\begin{bmatrix}{\Delta x}\\ {\Delta y} \\ {\Delta \theta}\\\Delta \dot{x}\\ \Delta \dot{y} \\ \Delta\dot{\theta} \end{bmatrix}
+
\begin{bmatrix}0&0\\0&0 \\0&0 \\0&0 \\ \frac{1}{m}&\frac{1}{m}\\-\frac{l^2}{J}&\frac{l^2}{J}\end{bmatrix}
\begin{bmatrix}\Delta u_1\\ \Delta u_2\end{bmatrix}
\end{align}
$$

```julia
using LinearAlgebra
using ForwardDiff
using OSQP
#Model parameters
g = 9.81 #m/s^2
m = 1.0 #kg 
ℓ = 0.3 #meters
J = 0.2*m*ℓ*ℓ

h = 0.05 #time step (20 Hz)

#Planar Quadrotor Dynamics
function quad_dynamics(x,u)
    θ = x[3]
    ẍ = (1/m)*(u[1] + u[2])*sin(θ)
    ÿ = (1/m)*(u[1] + u[2])*cos(θ) - g
    θ̈ = (1/J)*(ℓ/2)*(u[2] - u[1])
    return [x[4:6]; ẍ; ÿ; θ̈]
end
function quad_dynamics_rk4(x,u)
    #RK4 integration with zero-order hold on u
    f1 = quad_dynamics(x, u)
    f2 = quad_dynamics(x + 0.5*h*f1, u)
    f3 = quad_dynamics(x + 0.5*h*f2, u)
    f4 = quad_dynamics(x + h*f3, u)
    return x + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)
end

#Linearized dynamics for hovering
x_hover = zeros(6)
u_hover = [0.5*m*g; 0.5*m*g]
A = ForwardDiff.jacobian(x->quad_dynamics_rk4(x,u_hover),x_hover);
B = ForwardDiff.jacobian(u->quad_dynamics_rk4(x_hover,u),u_hover);
quad_dynamics_rk4(x_hover, u_hover)
```

### MPC

离散时不变系统的状态空间方程

$$
{x}_{k+1}=Ax_k+Bu_k
$$

定义未来$p$个周期内预测的系统状态$(n_xn_h\times1)$

$$
X_k=[x_{k+1|k} ^\top,x_{k+2|k} ^\top,...,x_{k+p|k} ^\top]^\top
$$

定义未来到达$p$个周期内的系统输入$(n_un_h\times1)$

$$
U_k=[u_{k|k} ^\top,u_{k+1|k} ^\top,...,u_{k+p-1|k} ^\top]^\top
$$

由系统离散动力学方程可以得到$x_{k|k}\rightarrow x_{k+h|k}$的状态转移方程

$$
\begin{align}
x_{k+1|k}&=Ax_{k|k}+Bu_{k|k}\\
x_{k+2|k}&=A^2x_{k|k}+ABu_{k|k}+Bu_{k+1|k}\\
...&\\
x_{k+p|k}&=A^px_{k|k}+A^{p-1}Bu_{k|k}+...+ABu_{k+p-1|k}+Bu_{k+p|k}
\end{align}
$$

写成矩阵形式

$$
X_k=\Phi x_{k|k}+\Gamma U_k
$$

其中

$$
\begin{align}
\Phi = \begin{bmatrix}A\\A^2\\\vdots \\ A^p\end{bmatrix}\quad
\Gamma = \begin{bmatrix}B& \mathbf{0}&\dots&\mathbf{0}\\AB&B& \dots&\mathbf{0}\\\vdots &\vdots&&\vdots\\A^{p-1}B&  B&\dots&AB\end{bmatrix}
\end{align}
$$

二次型性能指标

$$
\begin{align}
J&=\sum_{i=1}^{p-1}(\frac{1}{2}{x_{k+i|k}^\top}Qx_{k+i|k} + \frac{1}{2}{u_{k+i|k}^\top}Ru_{k+i|k})+\frac{1}{2}{x_{k+p|k}^\top}Q_Nx_{k+p|k}\\
&=\frac{1}{2}{x_{k|k}^\top}Qx_{k|k}+\frac{1}{2}X_{k}^\top\Omega X_{k} + \frac{1}{2}U_{k}^\top\Psi U_{k}
\end{align}
$$

其中

$$
\begin{align}
\Omega = \begin{bmatrix}Q&\dots&\mathbf{0}\\\vdots&Q&\vdots \\ \mathbf{0}&\dots&Q_N\end{bmatrix}\quad
\Psi = \begin{bmatrix}R&\dots&\mathbf{0}\\\vdots&\ddots&\vdots \\ \mathbf{0}&\dots&R\end{bmatrix}\quad\end{align}
$$

在通过变换用$U_k$表示$X_k$（忽略$x_{k|k}$决定项）

$$
\begin{align}
J&=\frac{1}{2}U_k^\top \mathbf{H}U_k+U_k^\top \mathbf{F}x_{k|k}
\end{align}
$$

其中

$$
\begin{align}
\mathbf{H}=\Gamma ^\top\Omega\Gamma+\Psi, \quad \mathbf{F}=\Gamma ^\top\Omega\Phi
\end{align}
$$

### osqp求解器的使用

QSQP求解器是一个用于求解凸二次规划（形式如下）的数值优化软件包

$$
\begin{split}\begin{array}{ll}
  \text{minimize} & \frac{1}{2} x^T P x + q^T x \\
  \text{subject to} & l \leq A x \leq u
\end{array}\end{split}
$$

其中$x$是优化变量，$P\in \mathbf{S}_+^n$s是一个半正定矩阵。

考虑一个线性时不变动力学系统到某个参考状$x_r\in \mathcal{R}^{n_x}$的问题

$$
\begin{split}\begin{array}{ll}
  \text{minimize}   & (x_N-x_r)^T Q_N (x_N-x_r) + \sum_{k=0}^{N-1} (x_k-x_r)^T Q (x_k-x_r) + u_k^T R u_k \\
  \text{subject to} & x_{k+1} = A x_k + B u_k \\
                    & x_{\rm min} \le x_k  \le x_{\rm max} \\
                    & u_{\rm min} \le u_k  \le u_{\rm max} \\
                    & x_0 = \bar{x}
\end{array}\end{split}
$$

1.给出系统的$Q,R,Q_N,A,B$

```julia
Nx = 6     # number of state
Nu = 2     # number of controls
Tfinal = 10.0 # final time
Nt = Int(Tfinal/h)+1    # number of time steps
thist = Array(range(0,h*(Nt-1), step=h));

# Cost weights
Q = Array(1.0*I(Nx));
R = Array(.01*I(Nu));
Qn = Array(1.0*I(Nx));

#Thrust limits
umin = [0.2*m*g; 0.2*m*g]
umax = [0.6*m*g; 0.6*m*g]
```

2.转换成标准QP问题

优化变量

$$
z=[x_1^T,x_2^T,\dots,x_p^T,u_0^T,u_1^T,\dots,u_{p-1}^T]^\top
$$

$$
J=z^\top \text{diag}(Q,\dots,Q,Q_N,R,...,R)z+2[Qx_{r_1},\dots,Qx_{r_k},\mathbf{0},\dots,\mathbf{0}]z
$$

```julia
U = kron(Diagonal(I,Nh), [I zeros(Nu,Nx)]) #Matrix that picks out all u
Θ = kron(Diagonal(I,Nh), [0 0 0 0 1 0 0 0]) #Matrix that picks out all x3 (θ)
H = sparse([kron(Diagonal(I,Nh-1),[R zeros(Nu,Nx); zeros(Nx,Nu) Q]) zeros((Nx+Nu)*(Nh-1), Nx+Nu); zeros(Nx+Nu,(Nx+Nu)*(Nh-1)) [R zeros(Nu,Nx); zeros(Nx,Nu) P]])
b = zeros(Nh*(Nx+Nu))
C = sparse([[B -I zeros(Nx,(Nh-1)*(Nu+Nx))]; zeros(Nx*(Nh-1),Nu) [kron(Diagonal(I,Nh-1), [A B]) zeros((Nh-1)*Nx,Nx)] + [zeros((Nh-1)*Nx,Nx) kron(Diagonal(I,Nh-1),[zeros(Nx,Nu) Diagonal(-I,Nx)])]])

#Dynamics + Thrust limit constraints
D = [C; U]
lb = [zeros(Nx*Nh); kron(ones(Nh),umin-u_hover)]
ub = [zeros(Nx*Nh); kron(ones(Nh),umax-u_hover)]

rob = OSQP.Model()
OSQP.setup!(prob; P=H, q=b, A=D, l=lb, u=ub, verbose=false, eps_abs=1e-8, eps_rel=1e-8, polish=1);
```
