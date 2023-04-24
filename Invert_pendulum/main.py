from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from model_pendulum import export_pendulum_model
import scipy.linalg
import numpy as np
import time
import matplotlib.pyplot as plt
from casadi import Function
from casadi import MX
from fancy_plots import fancy_plots_2, fancy_plots_1
#from c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython


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
    Q_mat = 1 * np.diag([1, 2, 0.0, 0.0])  # [x,th,dx,dth]
    R_mat = 1 * np.diag([0.3])

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
    #ocp.solver_options.nlp_solver_max_iter = 400
    # ocp.solver_options.levenberg_marquardt = 1e-2

    # set prediction horizon
    ocp.solver_options.tf = t_horizon

    return ocp

def main():
    # Initial Values System
    # Simulation Time
    t_final = 150
    # Sample time
    t_s = 0.05
    # Prediction Time
    t_prediction= 5;

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
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json", build= False, generate= False)

    #solver_json = 'acados_ocp_' + model.name + '.json'
    #AcadosOcpSolver.generate(ocp, json_file=solver_json)
    #AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
    #acados_ocp_solver = AcadosOcpSolver.create_cython_solver(solver_json)
    #acados_ocp_solver = AcadosOcpSolverCython(ocp.model.name, ocp.solver_options.nlp_solver_type, ocp.dims.N)

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
        x[:, k+1] = x[:, k+1] +  np.random.uniform(low=-0.02, high=0.02, size=(4,))

        delta_t[:, k] = toc

    fig1, ax1, ax2 = fancy_plots_2()
    ## Axis definition necesary to fancy plots
    ax1.set_xlim((t[0], t[-1]))
    ax2.set_xlim((t[0], t[-1]))
    ax1.set_xticklabels([])

    state_1, = ax1.plot(t[0:x.shape[1]],x[0,:],
                    color='#00429d', lw=2, ls="-")
    desired_1, = ax1.plot(t[0:x.shape[1]],xref[0,0:x.shape[1]],
                    color='#00429d', lw=2, ls="--")
    state_3, = ax1.plot(t[0:x.shape[1]],x[2,:],
                    color='#9e4941', lw=2, ls="-.")

    state_2, = ax2.plot(t[0:x.shape[1]],x[1,:],
                    color='#ac7518', lw=2, ls="-")
    desired_2, = ax2.plot(t[0:x.shape[1]],xref[1,0:x.shape[1]],
                    color='#ac7518', lw=2, ls="--")
    state_4, = ax2.plot(t[0:x.shape[1]],x[3,:],
                    color='#97a800', lw=2, ls="-.")

    ax1.set_ylabel(r"$[m],[m/s]$", rotation='vertical')
    ax1.legend([state_1,state_3,desired_1],
            [r'$x_1$', r'$x_3$',r'$x_{1d}$'],
            loc="best",
            frameon=True, fancybox=True, shadow=False, ncol=2,
            borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
            borderaxespad=0.3, columnspacing=2)
    ax1.grid(color='#949494', linestyle='-.', linewidth=0.5)

    ## Figure 2
    #fig2, ax2 = fancy_plots()
    ax2.set_ylabel(r"$[rad],[rad/s]$", rotation='vertical')
    ax2.set_xlabel(r"$\textrm{Time}[s]$", labelpad=5)
    ax2.legend([state_2, state_4,desired_2],
            [r'$x_2$', r'$x_4$', r'$x_{2d}$'],
            loc="best",
            frameon=True, fancybox=True, shadow=False, ncol=2,
            borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
            borderaxespad=0.3, columnspacing=2)
    #ax2.axis([t[0], t[-1], x[1,:].min(), x[1,:].max()])
    ax2.grid(color='#949494', linestyle='-.', linewidth=0.5)
    fig1.savefig("system_states.eps")
    fig1.savefig("system_states.png")
    fig1
    plt.show()

    fig2, ax11 = fancy_plots_1()
    ## Axis definition necesary to fancy plots
    ax11.set_xlim((t[0], t[-1]))

    control_1, = ax11.plot(t[0:u_control.shape[1]],u_control[0,:],
                    color='#00429d', lw=2, ls="-")

    ax11.set_ylabel(r"$[N]$", rotation='vertical')
    ax11.set_xlabel(r"$\textrm{Time}[s]$", labelpad=5)
    ax11.legend([control_1],
            [r'$u$'],
            loc="best",
            frameon=True, fancybox=True, shadow=False, ncol=2,
            borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
            borderaxespad=0.3, columnspacing=2)
    ax11.grid(color='#949494', linestyle='-.', linewidth=0.5)

    fig2.savefig("control_actions.eps")
    fig2.savefig("control_actions.png")
    fig2
    plt.show()

    fig3, ax13 = fancy_plots_1()
    ## Axis definition necesary to fancy plots
    ax13.set_xlim((t[0], t[-1]))

    time_1, = ax13.plot(t[0:delta_t.shape[1]],delta_t[0,:],
                    color='#00429d', lw=2, ls="-")
    tsam1, = ax13.plot(t[0:t_sample.shape[1]],t_sample[0,:],
                    color='#9e4941', lw=2, ls="-.")

    ax13.set_ylabel(r"$[s]$", rotation='vertical')
    ax13.set_xlabel(r"$\textrm{Time}[s]$", labelpad=5)
    ax13.legend([time_1,tsam1],
            [r'$t_{compute}$',r'$t_{sample}$'],
            loc="best",
            frameon=True, fancybox=True, shadow=False, ncol=2,
            borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
            borderaxespad=0.3, columnspacing=2)
    ax13.grid(color='#949494', linestyle='-.', linewidth=0.5)

    fig3.savefig("time.eps")
    fig3.savefig("time.png")
    fig3
    plt.show()

    # Systems Results
    print(f'Mean iteration time with MLP Model: {1000*np.mean(delta_t):.1f}ms -- {1/np.mean(delta_t):.0f}Hz)')

if __name__ == '__main__':
    #main()
    main()