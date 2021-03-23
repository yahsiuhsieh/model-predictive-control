import do_mpc
import numpy as np
from casadi import *
from casadi.tools import *
import matplotlib.pyplot as plt
import matplotlib.patches as plt_patches
import pdb
import sys
import globals

sys.path.append('../../')


class MPC:
    def __init__(self, vehicle, use_obstacles):

        self.vehicle = vehicle
        self.model = vehicle.model
        self.use_obstacles = use_obstacles

        self.horizon = 15
        globals.horizon = self.horizon  # for model.py use

        self.Ts = 0.05
        self.length = 0.12
        self.width = 0.06

        self.current_prediction = None

        self.mpc = do_mpc.controller.MPC(self.model)
        setup_mpc = {
            'n_robust': 0,
            'n_horizon': self.horizon,
            't_step': self.Ts,
            'state_discretization': 'collocation',
            'store_full_solution': True,
        }
        self.mpc.set_param(**setup_mpc)

        # define the objective function and constriants
        self.objective_function_setup()
        self.constraints_setup()

        # provide time-varing parameters: setpoints/references
        self.tvp_template = self.mpc.get_tvp_template()
        self.mpc.set_tvp_fun(self.tvp_fun)

        self.mpc.setup()

    def tvp_fun(self, t_now):
        '''
        provides data into time-varying parameters 
        '''
        ey_ub, ey_lb, _ = self.update_new_bound()
        for k in range(self.horizon):
            # extract information from current waypoint
            current_waypoint = self.vehicle.reference_path.get_waypoint(
                self.vehicle.wp_id + k
            )
            self.tvp_template['_tvp', k, 'x_ref'] = current_waypoint.x
            self.tvp_template['_tvp', k, 'y_ref'] = current_waypoint.y
            self.tvp_template['_tvp', k, 'psi_ref'] = current_waypoint.psi
            self.tvp_template['_tvp', k, 'ey_lb'] = ey_lb[k]
            self.tvp_template['_tvp', k, 'ey_ub'] = ey_ub[k]
            if current_waypoint.v_ref is not None:
                self.tvp_template['_tvp', k,
                                  'vel_ref'] = current_waypoint.v_ref
            else:
                self.tvp_template['_tvp', k, 'vel_ref'] = 0

        return self.tvp_template

    def objective_function_setup(self):
        # obstacle avoidance
        if self.use_obstacles:
            lterm = (100000 * (self.model.x['e_y'] - (self.model.tvp['ey_lb'] + self.model.tvp['ey_ub']) / 2) ** 2
                + self.model.aux['psi_diff'] ** 2
            )
            mterm = (100000 * (self.model.x['e_y'] - (self.model.tvp['ey_lb'] + self.model.tvp['ey_ub']) / 2) ** 2
                + 0.1 * (self.model.x['vel'] - self.model.tvp['vel_ref']) ** 2)
        # lane following
        else:
            lterm = (self.model.aux['psi_diff'] ** 2
                + (self.model.x['pos_x'] - self.model.tvp['x_ref']) ** 2
                + (self.model.x['pos_y'] - self.model.tvp['y_ref']) ** 2
            )
            mterm = ((self.model.x['pos_x'] - self.model.tvp['x_ref']) ** 2
                + (self.model.x['pos_y'] - self.model.tvp['y_ref']) ** 2
                + 0.1 * (self.model.x['vel'] - self.model.tvp['vel_ref']) ** 2)

        self.mpc.set_objective(mterm=mterm, lterm=lterm)
        # self.mpc.set_rterm(acc=0.01, delta=0.01)

    def constraints_setup(
        self, vel_bound=[0.0, 1.0], reset=False
    ):

        # states constraints
        self.mpc.bounds['lower', '_x', 'pos_x'] = -np.inf
        self.mpc.bounds['upper', '_x', 'pos_x'] = np.inf
        self.mpc.bounds['lower', '_x', 'pos_y'] = -np.inf
        self.mpc.bounds['upper', '_x', 'pos_y'] = np.inf
        self.mpc.bounds['lower', '_x', 'psi'] = - 2 * np.pi
        self.mpc.bounds['upper', '_x', 'psi'] = 2 * np.pi
        self.mpc.bounds['lower', '_x', 'vel'] = vel_bound[0]
        self.mpc.bounds['upper', '_x', 'vel'] = vel_bound[1]
        self.mpc.bounds['lower', '_x', 'e_y'] = -2
        self.mpc.bounds['upper', '_x', 'e_y'] = 2

        # input constraints
        self.mpc.bounds['lower', '_u', 'acc'] = -0.5
        self.mpc.bounds['upper', '_u', 'acc'] = 0.5
        self.mpc.bounds['lower', '_u', 'delta'] = -1
        self.mpc.bounds['upper', '_u', 'delta'] = 1

        if reset is True:
            self.mpc.setup()

    def update_new_bound(self, ay_max=4.0):
        # Compute dynamic constraints on e_y
        ey_ub, ey_lb, _ = self.vehicle.reference_path.update_path_constraints(
            self.vehicle.wp_id + 1,
            globals.horizon,
            2 * self.vehicle.safety_margin,
            self.vehicle.safety_margin,
        )

        # Get curvature predictions from previous control signals
        kappa_pred = 0
        #np.tan(np.array(self.mpc.data['_u', 'delta'][0])) / self.vehicle.length

        # Constrain maximum speed based on predicted car curvature
        vel_ub = np.sqrt(ay_max / (np.abs(kappa_pred) + 1e-12))

        return ey_ub, ey_lb, vel_ub

    def get_control(self, x0):

        # update current waypoint
        self.vehicle.get_current_waypoint()

        # solve optization problem
        u0 = self.mpc.make_step(x0)

        return np.array([u0[0], u0[1]])

    def distance_update(self, states):
        vel, psi = states[3], states[2]

        # Compute velocity along path
        s_dot = vel * np.cos(self.mpc.data['_aux', 'psi_diff'][0])

        # Update distance travelled along reference path
        globals.s += s_dot * self.Ts
