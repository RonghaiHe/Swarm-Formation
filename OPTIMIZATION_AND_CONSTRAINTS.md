# Optimization and Constraints

## 1. Overview

This document describes the L-BFGS-based trajectory optimization framework used in the swarm formation planner. The system generates smooth, collision-free trajectories for multiple quadrotors while maintaining formation constraints. The optimization runs iteratively between path searching (A* front-end) and trajectory smoothing (L-BFGS back-end).

### Execution Flow

**Entry point**: `PolyTrajOptimizer::OptimizeTrajectory_lbfgs` → `src/planner/traj_opt/src/poly_traj_optimizer.cpp:7`

1. Initialize MINCO trajectory from A* waypoints and time allocation
2. Convert real time to virtual time via `RealT2VirtualT` (line 33)
3. Call `lbfgs_optimize` with `costFunctionCallback` (line 64)

**Per-iteration callback**: `costFunctionCallback` → `poly_traj_optimizer.cpp:128`

```
1. VirtualT2RealT(T)                               — line 141
2. jerkOpt_.generate(P, T)                         — line 147
3. initAndGetSmoothnessGradCost2PT(gradT, cost)    — line 149
4. addPVAGradCost2CT(gradT, costs, K)              — line 151
5. jerkOpt_.getGrad2TP(gradT, gradP)               — line 153
6. VirtualTGradCost(T, t, gradT, gradt, costT)     — line 156
7. return smoo_cost + costs.sum() + time_cost      — line 159
```

**Sampled-point cost evaluation** (`addPVAGradCost2CT` → `poly_traj_optimizer.cpp:222`):

| Order | Function | Cost Index | Code Line |
|-------|----------|------------|-----------|
| 1 | `obstacleGradCostP` | `costs(0)` | 268 |
| 2 | `swarmGradCostP` | `costs(1)` | 278 |
| 3 | `swarmGraphGradCostP` | `costs(2)` | 293 |
| 4 | `feasibilityGradCostV` | `costs(4)` | 307 |
| 5 | `feasibilityGradCostA` | `costs(4)` | 315 |
| 6 | `distanceSqrVarianceWithGradCost2p` | `costs(5)` | 337 |

### Related Files

| File | Purpose |
|------|---------|
| `src/planner/traj_opt/src/poly_traj_optimizer.cpp` | Main optimizer, all cost functions |
| `src/planner/traj_opt/include/optimizer/poly_traj_optimizer.h` | Optimizer class, weights, config |
| `src/planner/traj_opt/include/optimizer/poly_traj_utils.hpp` | `MinJerkOpt` class, smoothness cost |
| `src/planner/swarm_graph/src/swarm_graph.cpp` | Graph Laplacian, formation cost |
| `src/planner/swarm_graph/include/swarm_graph/swarm_graph.hpp` | `SwarmGraph` class |
| `src/planner/plan_manage/src/planner_manager.cpp` | Calls optimizer with `use_formation` flag |
| `src/planner/plan_manage/src/ego_replan_fsm.cpp` | FSM triggers planning |

---

## 2. Trajectory Representation

The trajectory is represented using MINCO (Minimal Coordinate) polynomial trajectories. Each segment is a 5th-order polynomial:

$$
p(t) = \mathbf{c}_0 + \mathbf{c}_1 t + \mathbf{c}_2 t^2 + \mathbf{c}_3 t^3 + \mathbf{c}_4 t^4 + \mathbf{c}_5 t^5
$$

This representation ensures $C^2$ continuity at segment boundaries, providing continuous position, velocity, and acceleration. The trajectory is parameterized by:
- **Position waypoints**: Intermediate through-points from A*
- **Time allocation**: Duration $T_i$ for each segment
- **Virtual time**: Transformed time for unconstrained optimization

---

## 3. Objective Function

The total cost functional is:

$$
J = J_{\text{smooth}} + J_{\text{obs}} + J_{\text{swarm}} + J_{\text{formation}} + J_{\text{feas}} + J_{\text{var}} + J_{\text{time}}
$$

Each component serves a specific purpose:
| Term | Purpose | Cost Index |
|------|---------|------------|
| $J_{\text{smooth}}$ | Minimize jerk for smooth motion | `smoo_cost` |
| $J_{\text{obs}}$ | Avoid static obstacles | `costs(0)` |
| $J_{\text{swarm}}$ | Avoid inter-robot collisions | `costs(1)` |
| $J_{\text{formation}}$ | Maintain formation shape | `costs(2)` |
| $J_{\text{feas}}$ | Respect dynamic limits | `costs(4)` |
| $J_{\text{var}}$ | Regularize trajectory variability | `costs(5)` |
| $J_{\text{time}}$ | Minimize total flight time | `time_cost` |

---

## 4. Smoothness Cost (Jerk)

The smoothness cost penalizes the time derivative of acceleration (jerk):

$$
J_{\text{smooth}} = \frac{1}{2} \int_0^{T_{\text{total}}} \| \dddot{p}(t) \|^2 \, dt
$$

For a 5th-order polynomial, the jerk is:

$$
\dddot{p}(t) = 6\mathbf{c}_3 + 24\mathbf{c}_4 t + 60\mathbf{c}_5 t^2
$$

This cost is computed in closed form for each segment and summed across all segments. It encourages smooth, flyable trajectories with minimal control effort.

### Code

- **Call**: `initAndGetSmoothnessGradCost2PT` → `poly_traj_optimizer.cpp:216`
- **Implementation**: `jerkOpt_.initGradCost(gdT, cost)` → `poly_traj_utils.hpp:1351`
- **Cost formula**: `getTrajJerkCost()` → `poly_traj_utils.hpp:1277`

```cpp
// poly_traj_utils.hpp:1277
inline double getTrajJerkCost() const {
    double objective = 0.0;
    for (int i = 0; i < N; i++) {
        objective += 36.0 * b.row(6*i+3).squaredNorm() * T1(i) +
                     144.0 * b.row(6*i+4).dot(b.row(6*i+3)) * T2(i) +
                     192.0 * b.row(6*i+4).squaredNorm() * T3(i) +
                     240.0 * b.row(6*i+5).dot(b.row(6*i+3)) * T3(i) +
                     720.0 * b.row(6*i+5).dot(b.row(6*i+4)) * T4(i) +
                     720.0 * b.row(6*i+5).squaredNorm() * T5(i);
    }
    return objective;
}
```

**Gradient propagation**: `addGradJbyT(gdT)` and `addGradJbyC(gdC)` in `poly_traj_utils.hpp`

---

## 5. Obstacle Avoidance

Obstacle avoidance uses an Euclidean Signed Distance Field (ESDF) to compute distances to static obstacles. The cost is:

$$
J_{\text{obs}} = \sum_{k=1}^{N} w_{\text{obs}} \cdot \max\left(0, \frac{1}{d_k^2} - \frac{1}{d_{\text{safe}}^2}\right)
$$

where:
- $d_k$ is the distance from the trajectory point to the nearest obstacle
- $d_{\text{safe}}$ is the safety clearance distance
- $w_{\text{obs}}$ is the obstacle cost weight

The gradient is computed analytically with respect to trajectory control points, enabling efficient L-BFGS optimization.

### Code

- **Call**: `obstacleGradCostP(i_dp, pos, gradp, costp)` → `poly_traj_optimizer.cpp:268`
- **Implementation**: `poly_traj_optimizer.cpp:460-488`
- **Weight**: `wei_obs_` (loaded from `optimization/weight_obstacle`, line 738)
- **Clearance**: `obs_clearance_`

```cpp
// poly_traj_optimizer.cpp:460
bool PolyTrajOptimizer::obstacleGradCostP(const int i_dp,
                                          const Eigen::Vector3d &p,
                                          Eigen::Vector3d &gradp,
                                          double &costp) {
    if (i_dp == 0 || i_dp >= cps_.cp_size * 2 / 3)
        return false;

    double dist;
    grid_map_->evaluateEDT(p, dist);
    double dist_err = obs_clearance_ - dist;
    if (dist_err > 0) {
        Eigen::Vector3d dist_grad;
        grid_map_->evaluateFirstGrad(p, dist_grad);
        costp = wei_obs_ * pow(dist_err, 3);
        gradp = -wei_obs_ * 3.0 * pow(dist_err, 2) * dist_grad;
        return true;
    }
    return false;
}
```

**Gradient**: Chain rule through ESDF distance → position

---

## 6. Swarm Collision Avoidance

Inter-robot collision avoidance uses an ellipsoidal safety envelope around each robot. The cost between robots $i$ and $j$ is:

$$
J_{\text{swarm}} = \sum_{i < j} w_{\text{swarm}} \cdot \max\left(0, 1 - \frac{\| \mathbf{p}_i(t) - \mathbf{p}_j(t) \|^2}{r_{\text{safe}}^2}\right)
$$

The ellipsoidal model accounts for different clearance requirements in horizontal and vertical directions:
- **Horizontal clearance**: $r_h$ (typically larger for formation flight)
- **Vertical clearance**: $r_v$ (typically smaller)

This formulation is differentiable and can be efficiently optimized with gradient-based methods.

### Code

- **Call**: `swarmGradCostP(i_dp, t, pos, vel, gradp, gradt, grad_prev_t, costp)` → `poly_traj_optimizer.cpp:278`
- **Implementation**: `poly_traj_optimizer.cpp:490-563`
- **Weight**: `wei_swarm_` (loaded from `optimization/weight_swarm`, line 739)
- **Clearance**: `swarm_clearance_` (used as `CLEARANCE2 = (swarm_clearance_ * 1.5)^2`)

```cpp
// poly_traj_optimizer.cpp:490
bool PolyTrajOptimizer::swarmGradCostP(const int i_dp, const double t,
                                       const Eigen::Vector3d &p, const Eigen::Vector3d &v,
                                       Eigen::Vector3d &gradp, double &gradt,
                                       double &grad_prev_t, double &costp) {
    const double CLEARANCE2 = (swarm_clearance_ * 1.5) * (swarm_clearance_ * 1.5);
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1/a/a, inv_b2 = 1/b/b;

    for (size_t id = 0; id < swarm_trajs_->size(); id++) {
        // Get swarm member position/velocity at current time
        Eigen::Vector3d dist_vec = p - swarm_p;
        double ellip_dist2 = dist_vec(2)*dist_vec(2)*inv_a2 +
                            (dist_vec(0)*dist_vec(0) + dist_vec(1)*dist_vec(1))*inv_b2;
        double dist2_err = CLEARANCE2 - ellip_dist2;

        if (dist2_err*dist2_err*dist2_err > 0) {
            costp += wei_swarm_ * dist2_err*dist2_err*dist2_err;
            Eigen::Vector3d dJ_dP = wei_swarm_ * 3 * dist2_err*dist2_err * (-2) *
                Eigen::Vector3d(inv_b2*dist_vec(0), inv_b2*dist_vec(1), inv_a2*dist_vec(2));
            gradp += dJ_dP;
            gradt += dJ_dP.dot(v - swarm_v);
            grad_prev_t += dJ_dP.dot(-swarm_v);
        }
    }
}
```

**Gradient**: Chain rule through ellipsoidal distance → position, time

---

## 7. Formation Similarity

Formation maintenance uses a graph-theoretic approach with Laplacian similarity. The cost measures deviation from the desired formation shape:

$$
J_{\text{formation}} = \frac{1}{2} \sum_{(i,j) \in \mathcal{E}} w_{ij} \| (\mathbf{p}_i - \mathbf{p}_j) - (\mathbf{p}_i^{\text{des}} - \mathbf{p}_j^{\text{des}}) \|^2
$$

where:
- $\mathcal{E}$ is the set of edges in the formation graph
- $w_{ij}$ are edge weights encoding formation strength
- $\mathbf{p}_i^{\text{des}}$ are the desired formation positions

This formulation uses the formation Laplacian to penalize shape deformation while allowing rigid translation and rotation of the entire formation.

### Code

- **Call**: `swarmGraphGradCostP(i_dp, t, pos, vel, gradp, gradt, grad_prev_t, costp)` → `poly_traj_optimizer.cpp:293`
- **Implementation**: `poly_traj_optimizer.cpp:374-458`
- **Weight**: `wei_formation_` (loaded from `optimization/weight_formation`, line 743)

```cpp
// poly_traj_optimizer.cpp:433-447
swarm_graph_->updateGraph(swarm_graph_pos);

double similarity_error;
swarm_graph_->calcFNorm2(similarity_error);  // cost = ||L - L_des||^2

costp = wei_formation_ * similarity_error;
vector<Eigen::Vector3d> swarm_grad;
swarm_graph_->getGrad(swarm_grad);
gradp = wei_formation_ * swarm_grad[drone_id_];
```

### SwarmGraph Implementation (`swarm_graph.cpp`)

| Function | Line | Description |
|----------|------|-------------|
| `updateGraph` | 7 | Rebuild adjacency, Laplacian, gradients |
| `calcMatrices` | 53 | Compute $A$, $D$, $\hat{L}$ from positions |
| `calcFNorm2` | 90 | Cost = $\|\hat{L} - \hat{L}_{\text{des}}\|_F^2$ |
| `calcFGrad` | 101 | Analytic gradient via chain rule |
| `setDesiredForm` | 40 | Set desired formation $\hat{L}_{\text{des}}$ |

```cpp
// swarm_graph.cpp:90
bool SwarmGraph::calcFNorm2(double &cost) {
    cost = DLhat.cwiseAbs2().sum();  // Frobenius norm squared
}

// swarm_graph.cpp:53
bool SwarmGraph::calcMatrices(...) {
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            Adj(i,j) = calcDist2(swarm[i], swarm[j]);  // squared distance
    // Symmetric normalized Laplacian
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            SNL(i,j) = (i==j) ? 1 : -Adj(i,j) * pow(Deg(i),-0.5) * pow(Deg(j),-0.5);
}
```

---

## 8. Dynamic Feasibility

Dynamic feasibility constraints ensure the trajectory respects the physical limits of the quadrotors:

$$
J_{\text{feas}} = \sum_{k=1}^{N} w_{\text{feas}} \cdot \left[ \max\left(0, \|\dot{p}_k\| - v_{\max}\right)^2 + \max\left(0, \|\ddot{p}_k\| - a_{\max}\right)^2 \right]
$$

where:
- $v_{\max}$ is the maximum velocity
- $a_{\max}$ is the maximum acceleration
- $N$ is the number of sampled points along the trajectory

These constraints are soft penalties rather than hard constraints, allowing the optimizer to find feasible solutions while maintaining differentiability.

### Code

- **Velocity**: `feasibilityGradCostV(vel, gradv, costv)` → `poly_traj_optimizer.cpp:307`
- **Acceleration**: `feasibilityGradCostA(acc, grada, costa)` → `poly_traj_optimizer.cpp:315`
- **Weight**: `wei_feas_` (loaded from `optimization/weight_feasibility`, line 740)
- **Limits**: `max_vel_`, `max_acc_`

```cpp
// poly_traj_optimizer.cpp:565
bool PolyTrajOptimizer::feasibilityGradCostV(const Eigen::Vector3d &v,
                                             Eigen::Vector3d &gradv, double &costv) {
    double vpen = v.squaredNorm() - max_vel_ * max_vel_;
    if (vpen > 0) {
        gradv = wei_feas_ * 6 * vpen * vpen * v;
        costv = wei_feas_ * vpen * vpen * vpen;
        return true;
    }
    return false;
}

// poly_traj_optimizer.cpp:579
bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector3d &a,
                                             Eigen::Vector3d &grada, double &costa) {
    double apen = a.squaredNorm() - max_acc_ * max_acc_;
    if (apen > 0) {
        grada = wei_feas_ * 6 * apen * apen * a;
        costa = wei_feas_ * apen * apen * apen;
        return true;
    }
    return false;
}
```

**Gradient**: Chain rule through velocity/acceleration → control points

---

## 9. Distance Variance

The distance variance cost regularizes the trajectory to reduce unnecessary variability:

$$
J_{\text{var}} = w_{\text{var}} \cdot \text{Var}\left( \{ d_k \}_{k=1}^{N} \right)
$$

where $d_k$ are distances between consecutive trajectory points. This cost:
- Encourages uniform spacing along the trajectory
- Reduces oscillations
- Improves numerical stability of the optimization

### Code

- **Call**: `distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var)` → `poly_traj_optimizer.cpp:337`
- **Implementation**: `poly_traj_optimizer.cpp:593-619`
- **Weight**: `wei_sqrvar_`

```cpp
// poly_traj_optimizer.cpp:593
void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(
    const Eigen::MatrixXd &ps, Eigen::MatrixXd &gdp, double &var) {
    int N = ps.cols() - 1;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    double dsqrmean = dsqrs.sum() / N;
    double dquarmean = dsqrs.squaredNorm() / N;
    var = wei_sqrvar_ * (dquarmean - dsqrmean * dsqrmean);

    // Gradient w.r.t. positions
    for (int i = 0; i <= N; i++) {
        if (i != 0)
            gdp.col(i) += wei_sqrvar_ * (4.0*(dsqrs(i-1)-dsqrmean)/N * dps.col(i-1));
        if (i != N)
            gdp.col(i) += wei_sqrvar_ * (-4.0*(dsqrs(i)-dsqrmean)/N * dps.col(i));
    }
}
```

**Gradient**: Analytic derivative of squared variance → positions

---

## 10. Time Penalty

The time penalty minimizes total flight time:

$$
J_{\text{time}} = w_{\text{time}} \cdot T_{\text{total}} = w_{\text{time}} \cdot \sum_{i=1}^{M} T_i
$$

where $T_i$ is the duration of the $i$-th trajectory segment. This cost:
- Encourages faster completion
- Balances against smoothness and safety costs
- Can be adjusted based on mission requirements

### Code

- **Call**: `VirtualTGradCost(T, t, gradT, gradt, time_cost)` → `poly_traj_optimizer.cpp:156`
- **Implementation**: `poly_traj_optimizer.cpp:189-212`
- **Weight**: `wei_time_` (loaded from `optimization/weight_time`, line 742)

```cpp
// poly_traj_optimizer.cpp:189
template <typename EIGENVEC, typename EIGENVECGD>
void PolyTrajOptimizer::VirtualTGradCost(const Eigen::VectorXd &RT,
                                         const EIGENVEC &VT,
                                         const Eigen::VectorXd &gdRT,
                                         EIGENVECGD &gdVT,
                                         double &costT) {
    for (int i = 0; i < VT.size(); ++i) {
        double gdVT2Rt;
        if (VT(i) > 0)
            gdVT2Rt = VT(i) + 1.0;
        else {
            double denSqrt = (0.5*VT(i)-1.0)*VT(i) + 1.0;
            gdVT2Rt = (1.0-VT(i)) / (denSqrt*denSqrt);
        }
        gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }
    costT = RT.sum() * wei_time_;
}
```

**Gradient**: Chain rule through virtual time transform

---

## 11. Time Mapping

Direct optimization of segment times $T_i$ is constrained (must be positive). To handle this, a virtual time transform is used:

$$
T_i = e^{\tau_i}
$$

where $\tau_i$ is the virtual time parameter. This ensures:
- $T_i > 0$ for any real value of $\tau_i$
- Smooth gradients with respect to $\tau_i$: $\frac{\partial T_i}{\partial \tau_i} = T_i$
- Unconstrained optimization in the $\tau$-space

The gradient of the cost with respect to virtual times is computed via chain rule:

$$
\frac{\partial J}{\partial \tau_i} = \frac{\partial J}{\partial T_i} \cdot T_i
$$

### Code

- **Real → Virtual**: `RealT2VirtualT` → `poly_traj_optimizer.cpp:170`
- **Virtual → Real**: `VirtualT2RealT` → `poly_traj_optimizer.cpp:180`

```cpp
// poly_traj_optimizer.cpp:170
template <typename EIGENVEC>
void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT) {
    for (int i = 0; i < RT.size(); ++i) {
        VT(i) = RT(i) > 1.0 ? (sqrt(2.0*RT(i)-1.0) - 1.0)
                            : (1.0 - sqrt(2.0/RT(i) - 1.0));
    }
}

// poly_traj_optimizer.cpp:180
template <typename EIGENVEC>
void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT) {
    for (int i = 0; i < VT.size(); ++i) {
        RT(i) = VT(i) > 0.0 ? ((0.5*VT(i)+1.0)*VT(i) + 1.0)
                            : 1.0 / ((0.5*VT(i)-1.0)*VT(i) + 1.0);
    }
}
```

---

## 12. A* Path Planning

The front-end uses A* search to find initial paths through the free space:

- **Graph construction**: Uniform 3D grid with obstacle occupancy
- **Heuristic**: Euclidean distance to goal
- **Cost function**: Path length + obstacle penalty
- **Post-processing**: Path smoothing and point reduction

The A* path provides:
- Initial waypoints for the trajectory optimization
- Collision-free initial guess
- Topological path diversity for multi-robot scenarios

### Code

- **Call**: `astarWithMinTraj` → `poly_traj_optimizer.cpp:621`
- **Search**: `a_star_->astarSearchAndGetSimplePath(...)` → `poly_traj_optimizer.cpp:631`

```cpp
// poly_traj_optimizer.cpp:621
void PolyTrajOptimizer::astarWithMinTraj(const Eigen::MatrixXd &iniState,
                                         const Eigen::MatrixXd &finState,
                                         vector<Eigen::Vector3d> &simple_path,
                                         Eigen::MatrixXd &ctl_points,
                                         poly_traj::MinJerkOpt &frontendMJ) {
    simple_path = a_star_->astarSearchAndGetSimplePath(
        grid_map_->getResolution(), start_pos, end_pos);
    // Generate minimum snap trajectory from path waypoints
    ...
}
```

The optimized trajectory is then computed by minimizing the cost functional $J$ using L-BFGS optimization with the A* path as the initial solution.

---

## 13. Configuration Parameters

Loaded in `PolyTrajOptimizer::setParam` → `poly_traj_optimizer.cpp:730`

| Parameter | Variable | Line | Default |
|-----------|----------|------|---------|
| `optimization/weight_obstacle` | `wei_obs_` | 738 | -1.0 |
| `optimization/weight_swarm` | `wei_swarm_` | 739 | -1.0 |
| `optimization/weight_feasibility` | `wei_feas_` | 740 | -1.0 |
| `optimization/weight_time` | `wei_time_` | 742 | -1.0 |
| `optimization/weight_formation` | `wei_formation_` | 743 | -1.0 |
| `optimization/formation_type` | `formation_type_` | 747 | -1 |
