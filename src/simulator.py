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

# Colors
CAR = '#F1C40F'
CAR_OUTLINE = '#B7950B'


class Simulator:

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.simulator = do_mpc.simulator.Simulator(vehicle.model)

        # provide time-varing parameters: setpoints/references
        self.tvp_template = self.simulator.get_tvp_template()
        self.simulator.set_tvp_fun(self.tvp_fun)

        self.simulator.set_param(t_step=0.05)

        self.simulator.setup()

    def tvp_fun(self, t_now):
        # extract information from current waypoint
        current_waypoint = self.vehicle.reference_path.get_waypoint(
            self.vehicle.wp_id
        )
        self.tvp_template['x_ref'] = current_waypoint.x
        self.tvp_template['y_ref'] = current_waypoint.y
        self.tvp_template['psi_ref'] = current_waypoint.psi
        if current_waypoint.v_ref is not None:
            self.tvp_template['vel_ref'] = current_waypoint.v_ref
        else:
            self.tvp_template['vel_ref'] = 0

        return self.tvp_template

    #################################################################
    #                                                               #
    #   This plotting function is cited from:                       #
    #   the file "spatial_bicycle_models.py" of matssteinweg, ZTH   #
    #   to use the simulator created by the author                  #
    #                                                               #
    #################################################################
    def show(self, states):
        '''
        Display car on current axis.
        '''
        x, y, psi = states[0], states[1], states[2]

        # Get car's center of gravity
        cog = (x, y)
        # Get current angle with respect to x-axis
        yaw = np.rad2deg(psi)
        # Draw rectangle
        car = plt_patches.Rectangle(
            cog,
            width=self.vehicle.length,
            height=self.vehicle.width,
            angle=yaw,
            facecolor=CAR,
            edgecolor=CAR_OUTLINE,
            zorder=20,
        )

        # Shift center rectangle to match center of the car
        car.set_x(
            car.get_x()
            - (
                self.vehicle.length / 2 * np.cos(psi)
                - self.vehicle.width / 2 * np.sin(psi)
            )
        )
        car.set_y(
            car.get_y()
            - (
                self.vehicle.width / 2 * np.cos(psi)
                + self.vehicle.length / 2 * np.sin(psi)
            )
        )

        # Add rectangle to current axis
        ax = plt.gca()
        ax.add_patch(car)
