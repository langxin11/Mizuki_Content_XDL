---
title: CMU Optimal control and reinforcement learning 16-745 2025学习简要记录4
author: Daliang
published: 2025-07-28
toc: true
toc-depth: 4
toc-title: Contents
tags:
  - 优化控制
category: "CMU Optimal Control 16-745"
licenseName: "CC BY 4.0"
---
# CMU Optimal control and reinforcement learning 16-745 2025学习简要记录4

主要是配合Homework来的（在[课程官网答案](https://optimalcontrol.ri.cmu.edu/homeworks/)有完整的代码）

较早的更完整的课程记录可以参考[向阳的笔记](https://github.com/Zhihaibi/Optimal_control_16-745/blob/main/CMU16_745_Optimial%20control%20Lecture_Notes_zhihai%20Bi.pdf)和知乎[我爱科研](https://www.zhihu.com/column/c_1635315526615388160) 的整理

## Lecture 11-12 Nonlinear Trajectory Optimization/Differential Dynamic Programming

对于非线性轨迹优化问题，求解的方法分为shooting Method 和 collocation methods即直接法和间接法。

直接法通过将轨迹优化问题转化为标准的大规模非线性优化问题，利用现成的开源/善用的非线性求解器（IPOPT，mosek,yalmip...）。常见的非线性优化策略是基于序列二次规划的SQP方法。动力学约束推荐采用Hermite-Simpson 方法,比显示的RK4稳定性强、计算成本低。

**注意**

课程演示的代码 `dircol.ipynb`在我的本地运行到z_sol = solve(z0,prob) 给出报错

```markdown
ERROR: TypeError: in typeassert, expected Vector{Tuple{Int64, Int64}}, got a value of type Vector{Tuple{Any, Any}}
```

检查发现需要修改 `row_col!` 函数和 `sparsity_jacobian`稀疏矩阵生成部分，确保返回的是明确的 `Int64` 类型元组

```
row = Int64[]#而不是row=[]
col = Int64[]
```

同时在using Meshcat是给出报错

```
InitError: could not load library "C:\Users\26583\.julia\artifacts\8f71eb37d5b304026b6363d835f8c65ff1920339\bin\avdevice-61.dll"
The specified module could not be found.
......
during initialization of module FFMPEG_jll
```

表明无法找到 FFMPEG 的动态链接库文件，需要运行如下代码

```julia
Pkg.build("FFMPEG_jll")
Pkg.build("MeshCat")
Pkg.add("FFMPEG_jll")
using MeshCat
```

![](https://raw.githubusercontent.com/langxin11/Picture/main/blog/Lecture13_DIRCOL.gif)

微分动态规划（Differential Dynamic Programming, DDP）是一种用于求解非线性最优控制问题的迭代算法，结合动态规划和二阶泰勒展开的思想，通过局部近似和反向传播来高效地优化控制策略。

参考文章

[Differential Dynamic Programming with Nonlinear Constraints](https://zhaomingxie.github.io/projects/CDDP/CDDP.pdf)

[Second-Order Differential Dynamic Programming for Whole-Body MPC of Legged Robots](https://openreview.net/pdf?id=BNDO0dxvjD)

[iLQR Tutorial](http://roboticexplorationlab.org/papers/iLQR_Tutorial.pdf)

[AL-iLQR Tutorial](https://bjack205.github.io/papers/AL_iLQR_Tutorial.pdf)

考虑离散动力学系统，目标是找到控制序列$\mathbf{u}_{1:N-1}$，最小化总代价（通常假设代价函数和约束函数二阶可导）

$$
\begin{aligned}
	\min_{x_{1:N},u_{1:N-1}} J&=\sum_{k=1}^{N-1}{\ell (\mathbf{x}_k,\mathbf{u}_k)}+\ell _N(\mathbf{x}_N)\\
	\mathrm{s}.\mathrm{t}.\quad &
	\mathbf{x}_{k+1}=\mathbf{f}\left( \mathbf{x}_k,\mathbf{u}_k \right)\\
	&\mathbf{x}_k\in \mathbf{X}_k\\
	&\mathbf{u}_k\in \mathbf{U}_k\\
\end{aligned}
$$

### Solving the DDR/iLQR

**反向传播（Backward Pass）**

1. 定义值函数(value function)

   在时间步，值函数$V_k(\mathbf{x})$为从状态$\mathbf{x}_k$出发，采用最优控制策略$\pi^*$的最小总代价

   $$
   V_k(\mathbf{x})=\min_{\mathbf{u}_t,...,\mathbf{u}_{N-1}} \left( \sum_{j=k}^{N-1}{\ell (\mathbf{x}_j,\mathbf{u}_j)}+\ell _N(\mathbf{x}_N) \right)
   $$
2. 贝尔曼方程

   值函数满足贝尔曼方程(反映状态动作值函数$Q_k(\mathbf{x}_k,\mathbf{u}_k)$和状态值函数的关系)

   $$
   Q_k(\mathbf{x}_k,\mathbf{u}_k)={\ell (\mathbf{x}_k,\mathbf{u_k})}+V _{k+1}(\mathbf{x}_{k+1})
   $$

   $$
   V_k(\mathbf{x}_k)=\min_{\mathbf{u}} \left[ {\ell (\mathbf{x}_k,\mathbf{u_k})}+V _{k+1}(\mathbf{x}_{k+1}) \right]=\min_{\mathbf{u}}Q_k(\mathbf{x}_k,\mathbf{u}_k)
   $$
3. 二阶泰勒展开

   DDP 对Q 函数（即当前步代价+下一步值函数）进行二阶泰勒展开

   $$
   \begin{array}{l}
   	Q_k(\mathbf{x}+\Delta \mathbf{x},\mathbf{u}+\Delta \mathbf{u})\approx Q_k(\mathbf{x},\mathbf{u})+\left[ \begin{array}{c}
   	Q_{\mathbf{x}}\\
   	Q_{\mathbf{u}}\\
   \end{array} \right] ^{\mathrm{T}}\left[ \begin{array}{c}
   	\Delta \mathbf{x}\\
   	\Delta \mathbf{u}\\
   \end{array} \right]\\
   	+\frac{1}{2}\left[ \begin{array}{c}
   	\Delta \mathbf{x}\\
   	\Delta \mathbf{u}\\
   \end{array} \right] ^{\mathrm{T}}\left[ \begin{matrix}
   	Q_{\mathbf{xx}}&		Q_{\mathbf{xu}}\\
   	Q_{\mathbf{ux}}&		Q_{\mathbf{uu}}\\
   \end{matrix} \right] \left[ \begin{array}{c}
   	\Delta \mathbf{x}\\
   	\Delta \mathbf{u}\\
   \end{array} \right] ,\left( Q_{\mathbf{xu}}=Q_{\mathbf{ux}}^{\mathrm{T}} \right)\\
   \end{array}
   $$

   其中

   $$
   \begin{aligned}
   	&Q_{\mathbf{x}}=\ell _{\mathbf{x}}+\mathbf{f}_{\mathbf{x}}^{\top}V_{\mathbf{x}}^{\prime}\\
   	&Q_{\mathbf{u}}=\ell _{\mathbf{u}}+\mathbf{f}_{\mathbf{u}}^{\top}V_{\mathbf{x}}^{\prime}\\
   	&Q_{\mathbf{xx}}=\ell _{\mathbf{xx}}+\mathbf{f}_{\mathbf{x}}^{\top}V_{\mathbf{xx}}^{\prime}\mathbf{f}_{\mathbf{x}}+V_{\mathbf{x}}^{\prime}\cdot \mathbf{f}_{\mathbf{xx}}\\
   	&Q_{\mathbf{uu}}=\ell _{\mathbf{uu}}+\mathbf{f}_{\mathbf{u}}^{\top}V_{\mathbf{xx}}^{\prime}\mathbf{f}_{\mathbf{u}}+V_{\mathbf{x}}^{\prime}\cdot \mathbf{f}_{\mathbf{uu}}\\
   	&Q_{\mathbf{ux}}=\ell _{\mathbf{ux}}+\mathbf{f}_{\mathbf{u}}^{\top}V_{\mathbf{xx}}^{\prime}\mathbf{f}_{\mathbf{x}}+V_{\mathbf{x}}^{\prime}\cdot \mathbf{f}_{\mathbf{ux}}\\
   \end{aligned}
   $$

   iLQR的不同在于忽略$\bf f_{xx}$等二阶项
4. 最优控制修正

   通过最小化$V_k(\mathbf{x}_k)$也即$\nabla_\mathbf{u} V_k(\mathbf{x}_k+\Delta \mathbf{x})$得到最优控制修正

   $$
   \Delta \mathbf{u}_{k}^{\star}=-Q_{\mathbf{uu}}^{-1}Q_{\mathbf{u}}-Q_{\mathbf{uu}}^{-1}Q_{\mathbf{ux}}\Delta \mathbf{x}_k
   $$

   其中，反馈增益$K_k=-Q_{\mathbf{uu}}^{-1}Q_{\mathbf{ux}}$，前馈增益$j_k=Q_{\mathbf{uu}}^{-1}Q_{\mathbf{u}}$
5. 更新值函数的二次近似

   $$
   V_k\left( \mathbf{x}+\Delta \mathbf{x} \right) \leftarrow V_k\left( \mathbf{x} \right) +p_{k}^{\mathrm{T}}\Delta \mathbf{x}+\frac{1}{2}\Delta \mathbf{x}^{\mathrm{T}}P_k\Delta \mathbf{x}
   $$

   其中

   $$
   \begin{aligned}
   	P_k&=Q_{\mathbf{xx}}+K_{k}^{\mathrm{T}}Q_{\mathbf{uu}}K_k+Q_{xu}K_k+K_{k}^{\mathbf{T}}Q_{\mathbf{ux}},\\
   	p_k&=Q_{\mathbf{x}}+K_{k}^{\mathrm{T}}Q_{\mathbf{uu}}j_k+Q_{\mathbf{xu}}j_k+K_{k}^{\mathbf{T}}Q_{\mathbf{u}},\\
   \end{aligned}
   $$

   从$p_N=\nabla _xl_N(\mathbf{x}),P_N=\nabla _{xx}^{2}l_N(\mathbf{x})$开始，递推出$K_k,j_k,P_k$和$p_k$。

**前向更新（Forward Pass）**

更新控制序列$(\mathbf{x}_0^{new}=\mathbf{x}_0)$

$$
\begin{aligned}
	\mathbf{u}_{k}^{new}&=\mathbf{u}_k+K_k(\mathbf{x}_{k}^{new}-\mathbf{x}_k)+\alpha j_k\\
	\mathbf{x}_{k+1}^{new}&=f(\mathbf{x}_{k}^{new},\mathbf{u}_{k}^{new})\\
\end{aligned}
$$

通过$J_new-J$进行linesearch

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/image-20250805185855595.png" alt="image-20250805185855595" style="zoom: 50%;" />

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/image-20250805185947239.png" alt="image-20250805185947239" style="zoom:50%;" />

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/image-20250805190032028.png" alt="image-20250805190032028" style="zoom:50%;" />

### HW3_Q1 倒立摆小车DIRCOL示例

IPOPT是一个基于内点法的开源大规模非线性优化求解器，专门用于求解具有约束的连续优化问题。设定如下标准问题（这个HW自带了一个fmincon函数将IPOPT的接口进一步封装）

$$
\begin{align} \min_{x} \quad & \ell(x) & \text{cost function}\\ 
 \text{st} \quad & c_{eq}(x) = 0 & \text{equality constraint}\\
 & c_L \leq c_{ineq}(x) \leq c_U & \text{inequality constraint}\\
 & x_L \leq x \leq x_U & \text{primal bound constraint}
 \end{align}
$$

```julia
"""
x = fmincon(cost,equality_constraint,inequality_constraint,x_l,x_u,c_l,c_u,x0,params,diff_type)

This function uses IPOPT to minimize an objective function 

`cost(params, x)` 

With the following three constraints: 

`equality_constraint(params, x) = 0`
`c_l <= inequality_constraint(params, x) <= c_u` 
`x_l <= x <= x_u` 

Note that the constraint functions should return vectors. 

Problem specific parameters should be loaded into params::NamedTuple (things like 
cost weights, dynamics parameters, etc.). 

args:
    cost::Function                    - objective function to be minimzed (returns scalar)
    equality_constraint::Function     - c_eq(params, x) == 0 
    inequality_constraint::Function   - c_l <= c_ineq(params, x) <= c_u 
    x_l::Vector                       - x_l <= x <= x_u 
    x_u::Vector                       - x_l <= x <= x_u 
    c_l::Vector                       - c_l <= c_ineq(params, x) <= c_u 
    c_u::Vector                       - c_l <= c_ineq(params, x) <= c_u 
    x0::Vector                        - initial guess 
    params::NamedTuple                - problem parameters for use in costs/constraints 
    diff_type::Symbol                 - :auto for ForwardDiff, :finite for FiniteDiff 
    verbose::Bool                     - true for IPOPT output, false for nothing 

optional args:
    tol                               - optimality tolerance 
    c_tol                             - constraint violation tolerance 
    max_iters                         - max iterations 
    verbose                           - verbosity of IPOPT 

outputs:
    x::Vector                         - solution 

You should try and use :auto for your `diff_type` first, and only use :finite if you 
absolutely cannot get ForwardDiff to work. 

This function will run a few basic checks before sending the problem off to IPOPT to 
solve. The outputs of these checks will be reported as the following:

---------checking dimensions of everything----------
---------all dimensions good------------------------
---------diff type set to :auto (ForwardDiff.jl)----
---------testing objective gradient-----------------
---------testing constraint Jacobian----------------
---------successfully compiled both derivatives-----
---------IPOPT beginning solve----------------------

If you're getting stuck during the testing of one of the derivatives, try switching 
to FiniteDiff.jl by setting diff_type = :finite. 
""";
```

#### Part B:Cart pole Swingup

倒立摆小车的状态向量定义如下

$$
x = [p,\theta,\dot{p},\dot{\theta}]^T
$$

求解优化问题

$$
\begin{align} \min_{x_{1:N},u_{1:N-1}} \quad & \sum_{i=1}^{N-1} \bigg[ \frac{1}{2} (x_i - x_{goal})^TQ(x_i - x_{goal}) + \frac{1}{2} u_i^TRu_i \bigg] + \frac{1}{2}(x_N - x_{goal})^TQ_f(x_N - x_{goal})\\ 
 \text{st} \quad & x_1 = x_{\text{IC}} \\ 
 & x_N = x_{goal} \\ 
 & f_{hs}(x_i,x_{i+1},u_i,dt) = 0 \quad \text{for } i = 1,2,\ldots,N-1 \\
 & -10 \leq u_i \leq 10 \quad \text{for } i = 1,2,\ldots,N-1 
 \end{align}
$$

给定水平推力(限幅)使得倒立摆小车从$x_{IC} = [0,0,0,0]$到$x_{goal} = [0, \pi, 0, 0]$。

采用ZOH和Hermite Simpson来描述离散动力学系统，$f_{hs}(x_i,x_{i+1},u_i)$代表Hermite Simpson残差。

$$
\begin{align} x_{k+1/2} &= \frac{1}{2}(x_k + x_{k+1}) + \frac{\Delta t}{8}(\dot{x}_k - \dot{x}_{k+1})\\ f(x_k,x_{k+1},\Delta t) &= x_k + \frac{\Delta t}{6} \cdot (\dot{x}_k + 4\dot{x}_{k+1/2} + \dot{x}_{k+1}) - x_{k+1}= 0\quad \quad \text{Hermite-Simpson} \end{align}
$$

```julia
# cartpole 
function dynamics(params::NamedTuple, x::Vector, u)
    # cartpole ODE, parametrized by params. 

    # cartpole physical parameters 
    mc, mp, l = params.mc, params.mp, params.l
    g = 9.81
  
    q = x[1:2]
    qd = x[3:4]

    s = sin(q[2])
    c = cos(q[2])

    H = [mc+mp mp*l*c; mp*l*c mp*l^2]
    C = [0 -mp*qd[2]*l*s; 0 0]
    G = [0, mp*g*l*s]
    B = [1, 0]

    qdd = -H\(C*qd + G - B*u[1])
    xdot = [qd;qdd]
    return xdot 

end
function hermite_simpson(params::NamedTuple, x1::Vector, x2::Vector, u, dt::Real)::Vector
    # TODO: input hermite simpson implicit integrator residual 
    x1_dot=dynamics(params,x1,u)
    x2_dot=dynamics(params,x2,u)
    xm=(x1+x2)/2+dt*(x1_dot-x2_dot)/8
    xm_dot = dynamics(params,xm,u)
    return x1+dt/6*(x1_dot+4*xm_dot+x2_dot)-x2
end
```

定义优化变量

$$
Z = \begin{bmatrix}x_1 \\ u_1 \\ x_2 \\ u_2 \\ \vdots \\ x_{N-1} \\ u_{N-1} \\ x_N \end{bmatrix} \in \mathbb{R}^{N \cdot nx + (N-1)\cdot nu}
$$

```julia
function create_idx(nx,nu,N)
    # This function creates some useful indexing tools for Z 
    # x_i = Z[idx.x[i]]
    # u_i = Z[idx.u[i]]
    # Feel free to use/not use anything here.
    # our Z vector is [x0, u0, x1, u1, …, xN]
    nz = (N-1) * nu + N * nx # length of Z 
    x = [(i - 1) * (nx + nu) .+ (1 : nx) for i = 1:N]
    u = [(i - 1) * (nx + nu) .+ ((nx + 1):(nx + nu)) for i = 1:(N - 1)]
    # constraint indexing for the (N-1) dynamics constraints when stacked up
    c = [(i - 1) * (nx) .+ (1 : nx) for i = 1:(N - 1)]
    nc = (N - 1) * nx # (N-1)*nx 
    return (nx=nx,nu=nu,N=N,nz=nz,nc=nc,x= x,u = u,c = c)
end
function cartpole_cost(params::NamedTuple, Z::Vector)::Real
    idx, N, xg = params.idx, params.N, params.xg
    Q, R, Qf = params.Q, params.R, params.Qf
    # TODO: input cartpole LQR cost
    J = 0 
    for i = 1:(N-1)
        xi = Z[idx.x[i]]
        ui = Z[idx.u[i]]
        J += 0.5*(xi-xg)'*Q*(xi-xg)+0.5*ui'*R*ui 
    end
    xN =Z[idx.x[N]]
    J += 0.5*(xN-xg)'*Qf*(xN-xg)
    # dont forget terminal cost  
    return J 
end
function cartpole_dynamics_constraints(params::NamedTuple, Z::Vector)::Vector
    idx, N, dt = params.idx, params.N, params.dt  
    # TODO: create dynamics constraints using hermite simpson 
    # create c in a ForwardDiff friendly way (check HW0)
    c = zeros(eltype(Z), idx.nc)  
    for i = 1:(N-1)
        xi = Z[idx.x[i]]
        ui = Z[idx.u[i]] 
        xip1 = Z[idx.x[i+1]]     
        # TODO: hermite simpson 
        c[idx.c[i]] = hermite_simpson(params, xi,xip1, ui, dt)
    end
    return c 
end
function cartpole_equality_constraint(params::NamedTuple, Z::Vector)::Vector
    N, idx, xic, xg = params.N, params.idx, params.xic, params.xg   
    # TODO: return all of the equality constraints      
    return [
    Z[idx.x[1]] - xic;
    Z[idx.x[N]] - xg;
    cartpole_dynamics_constraints(params, Z)
    ]            # 10 is an arbitrary number 
end
function solve_cartpole_swingup(;verbose=true)   
    # problem size 
    nx = 4 
    nu = 1 
    dt = 0.05
    tf = 2.0 
    t_vec = 0:dt:tf 
    N = length(t_vec)   
    # LQR cost 
    Q = diagm(ones(nx))
    R = 0.1*diagm(ones(nu))
    Qf = 10*diagm(ones(nx))  
    # indexing 
    idx = create_idx(nx,nu,N)   
    # initial and goal states 
    xic = [0, 0, 0, 0]
    xg = [0, pi, 0, 0]  
    # load all useful things into params 
    params = (Q = Q, R = R, Qf = Qf, xic = xic, xg = xg, dt = dt, N = N, idx = idx,mc = 1.0, mp = 0.2, l = 0.5)
  
    # TODO: primal bounds 
    x_l = -Inf*ones(idx.nz)
    x_u = Inf*ones(idx.nz)
    for i = 1:N-1
        x_l[idx.u[i]].=-10
        x_u[idx.u[i]].=10
    end
    # inequality constraint bounds (this is what we do when we have no inequality constraints)
    c_l = zeros(0)
    c_u = zeros(0)
    function inequality_constraint(params, Z)
        return zeros(eltype(Z), 0)
    end 
    # initial guess 
    z0 = 0.001*randn(idx.nz)   
    # choose diff type (try :auto, then use :finite if :auto doesn't work)
    diff_type = :auto 
     #diff_type = :finite  
    Z = fmincon(cartpole_cost,cartpole_equality_constraint,inequality_constraint,
                x_l,x_u,c_l,c_u,z0,params, diff_type;
                tol = 1e-6, c_tol = 1e-6, max_iters = 10_000, verbose = verbose)  
    # pull the X and U solutions out of Z 
    X = [Z[idx.x[i]] for i = 1:N]
    U = [Z[idx.u[i]] for i = 1:(N-1)]  
    return X, U, t_vec, params 
end
@testset "cartpole swingup" begin 
  
    X, U, t_vec = solve_cartpole_swingup(verbose=true)   
    # --------------testing------------------
    @test isapprox(X[1],zeros(4), atol = 1e-4)
    @test isapprox(X[end], [0,pi,0,0], atol = 1e-4)
    Xm = hcat(X...)
    Um = hcat(U...)  
    # --------------plotting-----------------
    display(plot(t_vec, Xm', label = ["p" "θ" "ṗ" "θ̇"], xlabel = "time (s)", title = "State Trajectory"))
    display(plot(t_vec[1:end-1],Um',label="",xlabel = "time (s)", ylabel = "u",title = "Controls"))  
    # meshcat animation
    display(animate_cartpole(X, 0.05))
  
end
```

**输出**

```
---------checking dimensions of everything----------
---------all dimensions good------------------------
---------diff type set to :auto (ForwardDiff.jl)----
---------testing objective gradient-----------------
---------testing constraint Jacobian----------------
---------successfully compiled both derivatives-----
---------IPOPT beginning solve----------------------
This is Ipopt version 3.14.17, running with linear solver MUMPS 5.8.0.

Number of nonzeros in equality constraint Jacobian...:    34272
Number of nonzeros in inequality constraint Jacobian.:        0
Number of nonzeros in Lagrangian Hessian.............:        0

Total number of variables............................:      204
                     variables with only lower bounds:        0
                variables with lower and upper bounds:       40
                     variables with only upper bounds:        0
Total number of equality constraints.................:      168
Total number of inequality constraints...............:        0
        inequality constraints with only lower bounds:        0
   inequality constraints with lower and upper bounds:        0
        inequality constraints with only upper bounds:        0

iter    objective    inf_pr   inf_du lg(mu)  ||d||  lg(rg) alpha_du alpha_pr  ls
   0  2.4668025e+02 3.14e+00 3.14e-04   0.0 0.00e+00    -  0.00e+00 0.00e+00   0
   1  2.7495083e+02 2.38e+00 8.00e+00  -5.0 1.28e+01    -  4.90e-01 2.43e-01h  3
   2  2.9800850e+02 2.16e+00 1.03e+01  -0.5 1.05e+01    -  6.11e-01 9.26e-02h  4
   ..............

Number of Iterations....: 79

                                   (scaled)                 (unscaled)
Objective...............:   3.9344833576222919e+02    3.9344833576222919e+02
Dual infeasibility......:   7.5868099118641311e-07    7.5868099118641311e-07
Constraint violation....:   1.5276668818842154e-13    1.5276668818842154e-13
Variable bound violation:   9.9997231828297117e-08    9.9997231828297117e-08
Complementarity.........:   1.0000650824516029e-11    1.0000650824516029e-11
Overall NLP error.......:   7.5868099118641311e-07    7.5868099118641311e-07


Number of objective function evaluations             = 185
Number of objective gradient evaluations             = 80
Number of equality constraint evaluations            = 185
Number of inequality constraint evaluations          = 0
Number of equality constraint Jacobian evaluations   = 80
Number of inequality constraint Jacobian evaluations = 0
Number of Lagrangian Hessian evaluations             = 0
Total seconds in IPOPT                               = 1.664

EXIT: Optimal Solution Found.
```

#### Part C: Track DIRCOL Solution

类似HW2 Q2 PartC,使用DIRCOL得到的开环状态轨迹$X$和控制轨迹$U$，通过TVLQR跟踪该轨迹以补偿模型失配。DIRCOL的动力学约束使用了Hermite-Simpson积分方法，但闭环控制使用RK4进行数值积分，并且对闭环控制进行截断（clamp.(U[k],-10,10)）防止违约。

![](https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW3_Q1_C1.svg)

![](https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW3_Q1_C2.svg)

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW3_Q1_DIRCOL.gif" style="zoom:50%;" />

### HW3_Q2 四旋翼无人机iLQR 示例

使用iLQR求解四旋翼无人机(6DOF)的轨迹优化问题，使用如下特定的代价函数促使无人机完成指定的机动动作(∞)

四旋翼的连续时间动力学模型在 `quadrotor.jl`中完成，通过rk4进行离散化，四旋翼的状态向量表示如下

$x = [r,v,{}^Np^B{},\omega]$

其中 $r\in\mathbb{R}^3$ 是无人机在世界坐标系(N)的位置， $v\in\mathbb{R}^3$ 是无人机在世界坐标系(N)的速度(N)，  $^Np^B\in\mathbb{R}^3$ 是无人机姿态的修正罗德里格斯参数（Modified Rodrigues Parameter, MRP）表示（即机体坐标系相对于世界坐标系的旋转），$\omega\in\mathbb{R}^3$ 是无人机在机体坐标系的角速度。

补充：MRP使用三个参数表示旋转，在360°会遇到奇点（singularity）但在本次给定的规划动作不会使姿态旋转接近奇点。

```julia
include(joinpath(@__DIR__, "utils", "quadrotor.jl"))
functiondiscrete_dynamics(params::NamedTuple, x::Vector, u, k)
    # discrete dynamics
    # x - state
    # u - control
    # k - index of trajectory
    # dt comes from params.model.dt
    returnrk4(params.model, quadrotor_dynamics, x, u, params.model.dt)
end
```

#### Part A: iLQR for a quadrotor (25 pts)

iLQR用来解决如下的优化控制问题 :

$$
\begin{align} \min_{x_{1:N},u_{1:N-1}} \quad & \bigg[ \sum_{i=1}^{N-1} \ell(x_i,u_i)\bigg] + \ell_N(x_N)\\ 

 \text{st} \quad & x_1 = x_{{IC}} \\ 

 & x_{k+1} = f(x_k, u_k) \quad\text{for } i = 1,2,\ldots,N-1 \\

 \end{align}
$$

 $x_{IC}$是初始状态， $x_{k+1} = f(x_k, u_k)$是系统离散动力学模型， $\ell(x_i,u_i)$ 是状态代价函数，  $\ell_N(x_N)$ 是终端代价函数。由于优化问题可以是非凸的，因此不能保证收敛到全局最小值，但实际可以取得良好的效果。

针对此问题，选取如下的状态代价函数和终端代价函数来促使无人机跟踪设定的参考轨迹$x_{ref}$:

$$
\ell(x_i,u_i) = \frac{1}{2} (x_i - x_{ref,i})^TQ(x_i - x_{ref,i}) + \frac{1}{2}(u_i - u_{ref,i})^TR(u_i - u_{ref,i})\\

\ell_N(x_N) = \frac{1}{2}(x_N - x_{ref,N})^TQ_f(x_N - x_{ref,N})
$$

在这里我们需要补充 `iLQR`的实现过程，用反向传播（backward pass）计算的值函数增量来判断iLQR的收敛$\Delta J < \text{atol}$，并在solve_quadrotor_trajectory函数中调用。

**cost function的实现**

```julia
# starter code: feel free to use or not use 

function stage_cost(p::NamedTuple, x::Vector, u::Vector, k::Int)
    # TODO: return stage cost at time step k 
    Q, R, xref, uref = p.Q, p.R, p.Xref[k], p.Uref[k]
    return 0.5 * (x - xref)' * Q * (x - xref) + 0.5 * (u - uref)' * R * (u - uref)
end
function term_cost(p::NamedTuple, x)
    # TODO: return terminal cost
    Qf, xref = p.Qf, p.Xref[end]
    return 0.5 * (x - xref)' * Qf * (x - xref)
end
function stage_cost_expansion(p::NamedTuple, x::Vector, u::Vector, k::Int)
    # TODO: return stage cost expansion
    # if the stage cost is J(x,u), you can return the following
    # ∇ₓ²J, ∇ₓJ, ∇ᵤ²J, ∇ᵤJ
    Q, R, xref, uref = p.Q, p.R, p.Xref[k], p.Uref[k]
    Jxx = Q
    Jx = Q * (x - xref)
    Juu = R
    Ju = R * (u - uref)
    return Jxx, Jx, Juu, Ju
end
function term_cost_expansion(p::NamedTuple, x::Vector)
    # TODO: return terminal cost expansion
    # if the terminal cost is Jn(x,u), you can return the following
    # ∇ₓ²Jn, ∇ₓJn

    Qf, xref = p.Qf, p.Xref[end]
    Jn_xx = Qf
    Jn_x = Qf * (x - xref)
    return Jn_xx, Jn_x
end
```

**反向传播（backward pass）**

注意：我在参考课程给出的参考答案[Q3.ipynb](https://github.com/Optimal-Control-16-745/HW3_S25_Solutions/blob/main/Q3.ipynb)时，发现无法在我的电脑运行，调试之后发现在iLQR第一迭代时计算K[1]为空值因此加了一个判断进行修正

```julia
if any(isnan,K[k])
    K[k] = K[k+1] #k=1时出错K[1] =[NaN,...]
end
```

并且使用Qxu = Qux'（理论上成立）会报错，需使用Qxu = A_k'*P[k+1]*B_k来分别计算Qxu和Qux，具体原因还不太清楚，大概率是数值计算存在累计误差导致P[k+1]不完全对称。

```julia
function backward_pass(params::NamedTuple,          # useful params 
    X::Vector{Vector{Float64}},  # state trajectory 
    U::Vector{Vector{Float64}})  # control trajectory 
    # compute the iLQR backwards pass given a dynamically feasible trajectory X and U
    # return d, K, ΔJ  

    # outputs:
    #     d  - Vector{Vector} feedforward control  
    #     K  - Vector{Matrix} feedback gains 
    #     ΔJ - Float64        expected decrease in cost 

    nx, nu, N = params.nx, params.nu, params.N

    # vectors of vectors/matrices for recursion 
    P = [zeros(nx, nx) for i = 1:N]   # cost to go quadratic term
    p = [zeros(nx) for i = 1:N]   # cost to go linear term
    d = [zeros(nu) for i = 1:N-1] # feedforward control
    K = [zeros(nu, nx) for i = 1:N-1] # feedback gain

    # TODO: implement backwards pass and return d, K, ΔJ 
    N = params.N
    ΔJ = 0.0


    P[N], p[N] = term_cost_expansion(params, X[end])
    for k = N-1:-1:1
        Jxx, Jx, Juu, Ju = stage_cost_expansion(params, X[k], U[k], k)

        A_k = FD.jacobian(_x -> discrete_dynamics(params, _x, U[k], k), X[k])
        B_k = FD.jacobian(_u -> discrete_dynamics(params, X[k], _u, k), U[k])
        @assert all(isfinite, B_k) "B_k has NaN or Inf at step $k"
        gx = Jx + A_k' * p[k+1]
        gu = Ju + B_k' * p[k+1]
  
        Qxx = Jxx + A_k' * P[k+1] * A_k
        Quu = Juu + B_k' * P[k+1] * B_k
        Qux = B_k' * P[k+1] * A_k
        Qxu = A_k'*P[k+1]*B_k
        #Qxu = Qux'     #出错
  
        β = 1e-1
        Quu_reg = Juu + B_k' * (P[k+1]+β*I) * B_k
        Qux_reg = B_k' * (P[k+1]+β*I) * A_k
  
        for i =1:15
            if any(isnan, Quu_reg) || abs(det(Quu_reg)) < 1e-8
                @warn "Quu_reg is singular or has NaN at step $k"
                β = β*10
                Quu_reg = Juu + B_k' * (P[k+1]+β*I) * B_k  
            else
                break
            end
        end
  
        d[k] = -(Quu_reg) \ gu
        K[k] = -(Quu_reg) \ Qux_reg

        if any(isnan,K[k])
            K[k] = K[k+1] #k=1时出错K[1] =[NaN,...]
        end

        p[k] = gx + K[k]' * gu + K[k]' * Quu * d[k]  + Qxu * d[k]
        P[k] = Qxx + K[k]' * Quu * K[k] + Qxu * K[k] + K[k]' * Qux
        #ΔJ += -d[k]' * gu + 0.5 * d[k]' * Quu * d[k]
        ΔJ += -d[k]' * gu 
    end
    return d, K, ΔJ
end
```

前向更新（foward pass）

```julia
function trajectory_cost(params::NamedTuple,          # useful params 
    X::Vector{Vector{Float64}},  # state trajectory 
    U::Vector{Vector{Float64}}) # control trajectory 
    # compute the trajectory cost for trajectory X and U (assuming they are dynamically feasible)
    N = params.N
    cost = 0
    for i = 1:N-1
        cost += stage_cost(params, X[i], U[i], i)
    end
    cost += term_cost(params, X[N])
    # TODO: add trajectory cost 
    return cost
end

function forward_pass(params::NamedTuple,           # useful params 
    X::Vector{Vector{Float64}},   # state trajectory 
    U::Vector{Vector{Float64}},   # control trajectory 
    d::Vector{Vector{Float64}},   # feedforward controls 
    K::Vector{Matrix{Float64}};   # feedback gains
    max_linesearch_iters=20)    # max iters on linesearch 
    # forward pass in iLQR with linesearch 
    # use a line search where the trajectory cost simply has to decrease (no Armijo)

    # outputs:
    #     Xn::Vector{Vector}  updated state trajectory  
    #     Un::Vector{Vector}  updated control trajectory 
    #     J::Float64          updated cost  
    #     α::Float64.         step length 

    nx, nu, N = params.nx, params.nu, params.N
    Xn = [zeros(nx) for i = 1:N]      # new state history 
    Un = [zeros(nu) for i = 1:N-1]    # new control history 

    # initial condition 
    Xn[1] = 1 * X[1]
    # initial step length 
    α = 1.0
    # TODO: add forward pass 
    #current cost
    J = trajectory_cost(params, X, U)
    Jn = trajectory_cost(params, Xn, Un)
    for i = 1:max_linesearch_iters
        for k = 1:N-1
            Un[k] = U[k] + K[k] * (Xn[k] - X[k]) + α * d[k]
            Xn[k+1] = discrete_dynamics(params, Xn[k], Un[k], k)
        end
        Jn = trajectory_cost(params, Xn, Un)
        if Jn < J||i==max_linesearch_iters
            return Xn, Un, Jn, α
        else
            α = 0.5 * α
        end
    end
    error("forward pass failed")
end
```

**iLQR实现**

```julia
function iLQR(params::NamedTuple,         # useful params for costs/dynamics/indexing 
    x0::Vector,                 # initial condition 
    U::Vector{Vector{Float64}}; # initial controls 
    atol=1e-3,                  # convergence criteria: ΔJ < atol 
    max_iters=250,            # max iLQR iterations 
    verbose=true)             # print logging

    # iLQR solver given an initial condition x0, initial controls U, and a 
    # dynamics function described by `discrete_dynamics`

    # return (X, U, K) where 
    # outputs:
    #     X::Vector{Vector} - state trajectory 
    #     U::Vector{Vector} - control trajectory 
    #     K::Vector{Matrix} - feedback gains K 

    # first check the sizes of everything
    @assert length(U) == params.N - 1
    @assert length(U[1]) == params.nu
    @assert length(x0) == params.nx

    nx, nu, N = params.nx, params.nu, params.N
    # TODO: initial rollout
    X = [x0 for i = 1:N]
    for i = 1:N-1
        X[i+1] = discrete_dynamics(params, X[i], U[i], i)
    end

    for ilqr_iter = 1:max_iters
        d, K, ΔJ = backward_pass(params, X, U)
        Xn, Un, J, α = forward_pass(params, X, U, d, K)
        X, U = Xn, Un
        # termination criteria 
        if ΔJ < atol
            if verbose
                @info "iLQR converged"
            end
            return X, U, K
        end

        # ---------------logging -------------------
        if verbose
            dmax = maximum(norm.(d))
            if rem(ilqr_iter - 1, 10) == 0
                @printf "iter     J           ΔJ        |d|         α         \n"
                @printf "-------------------------------------------------\n"
            end
            @printf("%3d   %10.3e  %9.2e  %9.2e  %6.4f    \n",
                ilqr_iter, J, ΔJ, dmax, α)
        end

    end
    error("iLQR failed")
end
```

给出参考轨迹和控制，定义模型参数

```julia
function create_reference(N, dt)
    # create reference trajectory for quadrotor 
    R = 6
    Xref = [ [R*cos(t);R*cos(t)*sin(t);1.2 + sin(t);zeros(9)] for t = range(-pi/2,3*pi/2, length = N)]
    for i = 1:(N-1)
        Xref[i][4:6] = (Xref[i+1][1:3] - Xref[i][1:3])/dt
    end
    Xref[N][4:6] = Xref[N-1][4:6]
    Uref = [(9.81*0.5/4)*ones(4) for i = 1:(N-1)]
    return Xref, Uref
end
function solve_quadrotor_trajectory(;verbose = true)
  
    # problem size 
    nx = 12
    nu = 4
    dt = 0.05 
    tf = 5 
    t_vec = 0:dt:tf 
    N = length(t_vec)

    # create reference trajectory 
    Xref, Uref = create_reference(N, dt)
  
    # tracking cost function
    Q = 1*diagm([1*ones(3);.1*ones(3);1*ones(3);.1*ones(3)])
    R = .1*diagm(ones(nu))
    Qf = 10*Q 

    # dynamics parameters (these are estimated)
    model = (mass=0.5,
            J=Diagonal([0.0023, 0.0023, 0.004]),
            gravity=[0,0,-9.81],
            L=0.1750,
            kf=1.0,
            km=0.0245,dt = dt)

  
    # the params needed by iLQR 
    params = (
        N = N, 
        nx = nx, 
        nu = nu, 
        Xref = Xref, 
        Uref = Uref, 
        Q = Q, 
        R = R, 
        Qf = Qf, 
        model = model
    )

    # initial condition 
    x0 = 1*Xref[1]
  
    # initial guess controls 
    U = [(uref + .0001*randn(nu)) for uref in Uref]
  
    # solve with iLQR
    X, U, K = iLQR(params,x0,U;atol=1e-4,max_iters = 250,verbose = verbose)
  
    return X, U, K, t_vec, params
end
```

结果（迭代过程，位置，速度，姿态，角速度和控制量变化）

```
iter     J           ΔJ        |d|         α       
-------------------------------------------------
  1    2.988e+02   1.37e+05   2.88e+01  1.0000  
  2    1.075e+02   5.31e+02   1.35e+01  0.5000  
  3    4.903e+01   1.33e+02   4.73e+00  1.0000  
  4    4.429e+01   1.15e+01   2.48e+00  1.0000  
  5    4.402e+01   8.05e-01   2.51e-01  1.0000  
  6    4.398e+01   1.44e-01   8.32e-02  1.0000  
  7    4.396e+01   3.78e-02   7.28e-02  1.0000  
  8    4.396e+01   1.29e-02   3.76e-02  1.0000  
  9    4.396e+01   5.06e-03   3.19e-02  1.0000  
 10    4.396e+01   2.28e-03   1.94e-02  1.0000  
iter     J           ΔJ        |d|         α       
-------------------------------------------------
 11    4.396e+01   1.14e-03   1.61e-02  1.0000  
 12    4.395e+01   6.21e-04   1.09e-02  1.0000  
 13    4.395e+01   3.62e-04   8.94e-03  1.0000  
 14    4.395e+01   2.22e-04   6.61e-03  1.0000  
 15    4.395e+01   1.40e-04   5.39e-03  1.0000
```

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW3_Q2_1.svg" />

![](https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW3_Q2_2.svg)

![](https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW3_Q2_3.svg)

![HW3_Q2_4](https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW3_Q2_4.svg)

![HW3_Q2_5](https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW3_Q2_5.svg)

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW3_Q2_iLQR.gif" style="zoom:50%;" />

#### Part B:Tracking solution with TVLQR

通过iLQR生成的轨迹是开环，实际中会因模型误差（如风扰、参数不准）而偏离，这里使用iLQR得到的参考轨迹$\{\mathbf{x}_{ilqr,k},\mathbf{U}_{ilqr,k}\}_{k=1}^{N}$和时变增益矩阵$\{\mathbf{K}_{ilqr,k}\}_{k=1}^{N-1}$获得**TVLQR**反馈控制(注意K的正负号，这里输出的k=-Quu\Qux,自带了负号)

$$
\mathbf{u}_{sim,k}=\mathbf{u}_{ilqr,k}+K_{ilqr,k}(\mathbf{x}_{sim,k}-\mathbf{x}_{ilqr,k})\\
\mathbf{x}_{sim,k+1} =rk4(\text{real  model},\mathbf{x}_{ilqr,k})
$$

```julia
# set verbose to false when you submit 
    Xilqr, Uilqr, Kilqr, t_vec, params =  solve_quadrotor_trajectory(verbose = false)
  
    # real model parameters for dynamics 
    model_real = (mass=0.5,
            J=Diagonal([0.0025, 0.002, 0.0045]),
            gravity=[0,0,-9.81],
            L=0.1550,
            kf=0.9,
            km=0.0365,dt = 0.05)
  
    # simulate closed loop system 
    nx, nu, N = params.nx, params.nu, params.N
    Xsim = [zeros(nx) for i = 1:N]
    Usim = [zeros(nx) for i = 1:(N-1)]
  
    # initial condition 
    Xsim[1] = 1*Xilqr[1]
  
    # TODO: simulate with closed loop control 

    for i = 1:(N-1) 
        Usim[i] = Uilqr[i]+Kilqr[i]*(Xsim[i]-Xilqr[i])
        Xsim[i+1] = rk4(model_real, quadrotor_dynamics, Xsim[i], Usim[i], model_real.dt)
    end
```

TVLQR结果如下

![](https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW3_Q2_6.svg)

![HW3_Q2_7](https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW3_Q2_7.svg)

### HW3_Q3:Quadrotor Reorientation (40 pts)

$$
\begin{align} x = \begin{bmatrix} p_x \\ p_z \\ \theta \\ v_x \\ v_z \\ \omega \end{bmatrix}, &\quad \quad  \end{align} \dot{x} = \begin{bmatrix}v_x \\ v_z \\ \omega \\ \frac{1}{m}(u_1 + u_2)\sin\theta \\ \frac{1}{m}(u_1 + u_2)\cos\theta \\ \frac{\ell}{2J}(u_2 - u_1)\end{bmatrix}
$$

**问题要求**

实现三架平面无人机的无碰撞的轨迹优化问题，要求如下

- 无人机的初始位置$x1ic,x2ic,x3ic$和末端期望位置$x1g,x2g,x3g$给定
- 无人机彼此的距离不能小于$0.8\mathrm{m}$

一般的iLQR无法直接处理碰撞约束，需要在代价函数中纳入距离成本，但无法保证完全满足距离约束，这里选择使用直接配点法DIRCOL，求解问题的数学形式设计如下

$$
\begin{align} \min_{x_{1:N},u_{1:N-1}} \quad & \sum_{i=1}^{N-1} \bigg[ \frac{1}{2} (x_i - x_{goal})^TQ(x_i - x_{goal}) + \frac{1}{2} u_i^TRu_i \bigg] + \frac{1}{2}(x_N - x_{goal})^TQ_f(x_N - x_{goal})\\ 
 \text{st} \quad & x_1 = x_{\text{IC}} \\ 
 & x_N = x_{goal} \\ 
 & f_{hs}(x_i,x_{i+1},u_i,dt) = 0 \quad \text{for } i = 1,2,\ldots,N-1 \\
 &||p_{1,i}-p_{2,i}||>=0.8,||p_{1,i}-p_{3,i}||>=0.8,||p_{2,i}-p_{3,i}||>=0.8\\ 
 \end{align}
$$

实际中将三架无人机各自的状态变量和控制变量进行合并，$z$为整个要求解的优化变量

$$
x_i =[x1_{i};x2_{i};x3_{i}]\\
u = [u1_{i};u2_{i};u3_{i}]\\
z=[x_1;u_1;...;u_{N-1};x_N ]
$$

并且在处理碰撞约束的时候使用 `norm()^2>=R^2`的形式，保证可微性。

初始轨迹的生成可以使用 `x_initialize = range(xic, xg, length = N)`

整体的代码框架与HW3_Q1基本一致，结果如下

<img src="https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW3_Q3_DIRCOL.gif" style="zoom:50%;" />

![](https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW3_Q3_1.svg)

![HW3_Q3_2](https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW3_Q3_2.svg)

![HW3_Q3_3](https://raw.githubusercontent.com/langxin11/Picture/main/blog/HW3_Q3_3.svg)
