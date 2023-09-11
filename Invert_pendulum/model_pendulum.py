from acados_template import AcadosModel
from casadi import MX, vertcat, sin, cos
from casadi import Function

def export_pendulum_model() -> AcadosModel:

    model_name = 'pendulum_car_ode'

    # constants
    M = 1. # mass of the cart [kg] -> now estimated
    m = 0.1 # mass of the ball [kg]
    g = 9.81 # gravity constant [m/s^2]
    l = 0.8 # length of the rod [m]

    # set up states & controls
    x1      = MX.sym('x1')
    theta   = MX.sym('theta')
    v1      = MX.sym('v1')
    dtheta  = MX.sym('dtheta')

    x = vertcat(x1, theta, v1, dtheta)

    F = MX.sym('F')
    u = vertcat(F)

    # xdot
    x1_dot      = MX.sym('x1_dot')
    theta_dot   = MX.sym('theta_dot')
    v1_dot      = MX.sym('v1_dot')
    dtheta_dot  = MX.sym('dtheta_dot')

    xdot = vertcat(x1_dot, theta_dot, v1_dot, dtheta_dot)

    # dynamics
    cos_theta = cos(theta)
    sin_theta = sin(theta)
    denominator = M + m - m*cos_theta*cos_theta
    f_expl = vertcat(v1,
                     dtheta,
                     (-m*l*sin_theta*dtheta*dtheta + m*g*cos_theta*sin_theta+F)/denominator,
                     (-m*l*cos_theta*sin_theta*dtheta*dtheta + F*cos_theta+(M+m)*g*sin_theta)/(l*denominator)
                     )

    f_impl = xdot - f_expl

    x_ref = MX.sym('x_ref')
    theta_ref = MX.sym('theta_ref')
    v_ref = MX.sym('v_ref')
    thetap_ref = MX.sym('thetap_ref')

    p = vertcat(x_ref, theta_ref, v_ref, thetap_ref)

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = model_name
    model.p = p

    f_system = Function('system',[x, u], [f_expl])
    return model, f_system
