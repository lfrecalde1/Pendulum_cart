from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from model_pendulum import export_pendulum_model
import scipy.linalg
import numpy as np
import time
import matplotlib.pyplot as plt
from casadi import Function
from casadi import MX
from fancy_plots import fancy_plots_2, fancy_plots_1


def f_d(x, u, ts, f_system):
    k1 = f_system(x, u)
    k2 = f_system(x+(ts/2)*k1, u)
    k3 = f_system(x+(ts/2)*k2, u)
    k4 = f_system(x+(ts)*k3, u)
    x = x + (ts/6)*(k1 +2*k2 +2*k3 +k4)
    aux_x = np.array(x[:,0]).reshape((4,))
    return aux_x

def create_ocp_solver_description(x0, N_horizon, t_horizon, F_max, F_min) -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    model, f_system = export_pendulum_model()
    ocp.model = model
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    # set dimensions
    ocp.dims.N = N_horizon

    # set cost
    Q_mat = 1 * np.diag([1, 1, 0.0, 0.0])  # [x,th,dx,dth]
    R_mat = 1 * np.diag([0.1])

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    ny = nx + nu
    ny_e = nx

    ocp.cost.W_e = Q_mat
    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    # set constraints
    ocp.constraints.lbu = np.array([F_min])
    ocp.constraints.ubu = np.array([F_max])
    ocp.constraints.idxbu = np.array([0])

    ocp.constraints.x0 = x0

    # set options
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # SQP_RTI, SQP
    ocp.solver_options.nlp_solver_max_iter = 400
    # ocp.solver_options.levenberg_marquardt = 1e-2

    # set prediction horizon
    ocp.solver_options.tf = t_horizon

    return ocp

def main():
    # Initial Values System
    # Simulation Time
    t_final = 20
    # Sample time
    t_s = 0.05
    # Prediction Time
    t_prediction= 1;

    # Nodes inside MPC
    N = np.arange(0, t_prediction + t_s, t_s)
    N_prediction = N.shape[0]

    # Time simulation
    t = np.arange(0, t_final + t_s, t_s)

    # Sample time vector
    delta_t = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    t_sample = t_s*np.ones((1, t.shape[0] - N_prediction), dtype=np.double)

    # Vector Initial conditions
    x = np.zeros((4, t.shape[0]+1-N_prediction), dtype = np.double)
    x[0,0] = 5.0
    x[1,0] = 10*(np.pi/180)
    x[2,0] = 0.0
    x[3,0] = 0.0
    # Initial Control values
    u_control = np.zeros((1, t.shape[0]-N_prediction), dtype = np.double)

    # Reference Signal of the system
    xref = np.zeros((5, t.shape[0]), dtype = np.double)
    xref[0,:] = 5*np.sin(0.5*t)
    xref[1,:] = 30*(np.pi/180)*np.cos(0.5*t)
    xref[2,:] = 0.0
    xref[3,:] = 0.0 
    xref[4,:] = 0.0
    #Constraints 
    f_max = 20
    f_min = -20
    # Create Model of the system
    model, f_system = export_pendulum_model()

    # Optimization Problem
    ocp = create_ocp_solver_description(x[:,0], N_prediction, t_prediction, f_max, f_min)
    acados_ocp_solver = AcadosOcpSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )
    acados_integrator = AcadosSimSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )

    # Dimentions System
    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]

    # Initial States Acados
    for stage in range(N_prediction + 1):
        acados_ocp_solver.set(stage, "x", 0.0 * np.ones(x[:,0].shape))
    for stage in range(N_prediction):
        acados_ocp_solver.set(stage, "u", np.zeros((nu,)))
    # Simulation System

    for k in range(0, t.shape[0]- N_prediction):
        acados_ocp_solver.set(0, "lbx", x[:,k])
        acados_ocp_solver.set(0, "ubx", x[:,k])

        # update yref
        for j in range(N_prediction):
            yref = xref[:,k+j]
            acados_ocp_solver.set(j, "yref", yref)
        yref_N = xref[:,k+N_prediction]
        acados_ocp_solver.set(N_prediction, "yref", yref_N[0:4])

        # Get Computational Time
        tic = time.time()
        # solve ocp
        status = acados_ocp_solver.solve()

        toc = time.time()- tic
        print(toc)

        # Get Control Signal
        u_control[:, k] = acados_ocp_solver.get(0, "u")

        # System Evolution
        x[:, k+1] = f_d(x[:, k], u_control[:,k], t_s, f_system)
        x[:,k+1] = x[:,k+1] +  np.random.uniform(low=-0.05, high=0.05, size=(4,))
        delta_t[:, k] = toc

    # Systems Results
    print(f'Mean iteration time with MLP Model: {1000*np.mean(delta_t):.1f}ms -- {1/np.mean(delta_t):.0f}Hz)')

if __name__ == '__main__':
    #main()
    main()