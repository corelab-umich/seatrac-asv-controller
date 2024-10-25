import rclpy
from rclpy.node import Node
from juliacall import Main as jl
import numpy as np
from datetime import datetime
from tzlocal import get_localzone

from messages.msg import Commands, SensorData

from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

K_KTS2MS = 0.5144444444 # kts -> m/s conversion
K_MS2KTS = 1.9438444924 # m/s -> kts conversion
K_M2KM = 0.001 # meters -> kilometers

class ASVErgoControl(Node):

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'controller_enable' and param.type_ == Parameter.Type.BOOL:
                self.controller_enable = param.value
                if self.controller_enable:
                    self.controller_init()
            if param.name == 'mission_duration_hrs' and param.type_ == Parameter.Type.DOUBLE:
                self.mission_duration_hrs = param.value
            if param.name == 'terminal_soc' and param.type_ == Parameter.Type.DOUBLE:
                self.terminal_soc = param.value
            if param.name == 'speed_kp' and param.type_ == Parameter.Type.DOUBLE:
                self.kp = param.value
            if param.name == 'speed_ki' and param.type_ == Parameter.Type.DOUBLE:
                self.ki = param.value
            if param.name == 'speed_kd' and param.type_ == Parameter.Type.DOUBLE:
                self.kd = param.value
        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__('asv_ergo_control')

        """ Timezone/Clock Setup """
        self.local_tz = get_localzone()

        """ Declare User Parameters """
        self.declare_parameter('controller_enable', False)
        self.controller_enable = False
        self.declare_parameter('mission_duration_hrs', 3.0)
        self.mission_duration_hrs = 3.0
        self.declare_parameter('terminal_soc', 3000.0)
        self.terminal_soc = 3000.0
        self.declare_parameter('speed_kp', 0.5)
        self.kp = 0.5
        self.declare_parameter('speed_ki', 0.01)
        self.ki = 0.01
        self.declare_parameter('speed_kd', 0.5)
        self.kd = 0.5
        # TODO: Add Speed & Heading Override Parameters
        # TODO: Make rated speed a parameter

        """ Parameter Update Function """
        self.add_on_set_parameters_callback(self.parameter_callback)

        """ Julia Imports """
        # Creating variable storage function
        self.jlstore = jl.seval("(k, v) -> (@eval $(Symbol(k)) = $v; return)")

        # Importing packages
        jl.seval("using LinearAlgebra, Random, Statistics, Plots, StaticArrays, Interpolations, LazySets, SpatiotemporalGPs, JLD2, LinearInterpolations")

        # Importing modules
        self.soc_controller = jl.include("src/asv_controller/jl_src/SOC_Controller.jl")
        self.domain = jl.include("src/asv_controller/jl_src/jordan_lake_domain.jl")
        self.kf = jl.include("src/asv_controller/jl_src/kf.jl")
        self.ngpkf = jl.include("src/asv_controller/jl_src/ngpkf.jl")
        self.synthetic = jl.include("src/asv_controller/jl_src/SyntheticData.jl")
        self.ergodic = jl.include("src/asv_controller/jl_src/ergodic.jl")
        self.vario = jl.include("src/asv_controller/jl_src/variograms.jl")
        self.boundary_avoid = jl.include("src/asv_controller/jl_src/Convex_bound_avoidance.jl")
        self.control_class = jl.include("src/asv_controller/jl_src/Controller.jl")
        self.sim_vars = jl.include("src/asv_controller/jl_src/simulator_ST.jl")

        # JLD2 Saving
        current_time = datetime.now()
        time_string = current_time.strftime("%Y-%m-%d_%H-%M-%S")
        self.filename = f"/root/jld2_files/{time_string}_jlvars.jld2"

        """ Subscribe to Sensor Data """
        self.subscription = self.create_subscription(
            SensorData,
            'measurement_packet',
            self.measurement_aggregator,
            10)
        self.subscription  # prevent unused variable warning
        self.state_of_charge = None
        self.position_xy = None

        """ SOC Controller Variables """
        # Time variables
        self.dt_sec = 2.5
        # dt_min = self.dt_sec/(60.0)
        self.dt_hrs = self.dt_sec/(60.0 * 60.0)
        self.T_begin = None
        self.T_end = None
        self.ts_hrs = None
        self.soc_target = None

        # Speed Controller variables
        self.error_sum = 0.0
        self.error = 0.0

        """ Ergodic Controller Variables """
        # KF Variables
        self.stgpkfprob = None
        self.kt = None
        self.ks = None
        self.dx = None
        self.xs = None
        self.ys = None
        self.grid_points = None
        self.ngpkf_grid = None
        self.ergo_grid = None
        self.stgpkf_state = None
        self.Nx = None
        self.Ny = None

        # KF Estimates
        self.M = None
        self.w_hat = None
        self.qs = None
        self.ergo_q_map = None

        # Clarity matrix and functions
        self.target_q = 0.95
        self.target_q_matrix = None

        # Rated Speed
        self.w_rated = 2.25 # TODO: Make a parameter

        """ Control Loop on Timer """
        self.publisher_ = self.create_publisher(Commands, 'asv_command', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        """ Create control command message """
        msg = Commands()

        """ Compute control if enabled """
        if self.controller_enable:
            """ Compute Speed Command """
            try:
                speed, target_soc = self.speed_controller()
            except:
                speed = 0.0
                target_soc = -6908.00
                self.get_logger().error('Speed Controller Error')

            """ Get speeds from ergo controller """
            
            try:
               speed_x, speed_y = self.ergo_controller(speed)
            except:
               speed_x = speed
               speed_y = 0.0
               self.get_logger().error('Ergodic Controller Error')

            """ Convert speeds to heading """
            try:
                heading = self.heading_calc(speed_x, speed_y)
            except:
                heading = 0
                self.get_logger().error('Heading Calculation Error')

            """ Publish message with control command """
            msg.speed_kts = speed * K_MS2KTS
            msg.heading = heading
            msg.target_bat_soc = target_soc
            self.publisher_.publish(msg)
        else:
            msg.speed_kts = 0.0
            msg.heading = 0.0
            self.publisher_.publish(msg)

        """ Update JLD2 File with workspace vars """
        # try:
        #     self.save_all_julia_vars()
        # except:
        #     self.get_logger().error('JLD2 Logging Error')
        pass
    
    def controller_init(self):
        self.get_logger().debug('Initializing Controller')

        """ Initialize SOC Controller"""

        # Define variables
        self.get_logger().debug('Initializing SOC Controller')
        now = datetime.now(self.local_tz)
        day_of_year = self.get_day_of_year(now)
        self.T_begin = self.get_fractional_hours(now)
        self.T_end = self.T_begin + self.mission_duration_hrs
        self.ts_hrs = [self.T_begin + i * self.dt_hrs for i in range(int((self.T_end - self.T_begin)/ self.dt_hrs) + 1)]
        soc_begin = self.state_of_charge
        soc_end = self.terminal_soc
        # TODO: Create a reference plot for SOC vs Time under ideal case to determine final SOC target

        # Compute SOC barriers and target profile
        ucbf = self.soc_controller.compute_ucbf(self.ts_hrs, self.dt_hrs)
        lcbf = self.soc_controller.compute_lcbf(self.ts_hrs, self.dt_hrs)
        try:
            self.soc_target = self.soc_controller.generate_SOC_target(lcbf, ucbf, soc_begin, soc_end, self.ts_hrs, self.dt_hrs)
            self.get_logger().debug('Generated SOC Target: {self.soc_target[2]}')
        except:
            self.get_logger().error('Failed to generate SOC Target')
            controller_disable = Parameter('controller_enable', Parameter.Type.BOOL, False)
            self.set_parameters([controller_disable])
            return
        
        """ Initialize Ergodic Controller """
        self.get_logger().debug('Initializing Ergodic Controller')
        try:
            # Hyperparameter Initialization
            sigma_t = 2.0
            sigma_s = 1.0
            lt = 0.75 * 60.0 # minutes
            ls = 0.75 # km

            # Create domain arrays
            self.kt = self.jlstore("kt", jl.Matern(1/2, sigma_t, lt))
            self.ks = self.jlstore("ks", jl.Matern(1/2, sigma_s, ls))
            self.dx = self.jlstore("dx", 0.10)
            self.xs = jl.seval("xs = 0:dx:1.4")
            self.ys = jl.seval("ys = 0:dx:6.5")

            # Create grid point variable
            self.grid_points = jl.seval("grid_points = vec([@SVector[x, y] for x in xs, y in ys])")

            # Initialize STGPKF Problem
            self.jlstore("dt_min", self.dt_sec/(60.0))
            self.stgpkfprob = jl.seval("problem = STGPKFProblem(grid_points, ks, kt, dt_min)")

            # Initialize ngpkf_grid
            self.ngpkf_grid = jl.seval("ngpkf_grid = NGPKF.NGPKFGrid(xs, ys, ks)")

            # Initialize target clarity matrix
            jl.seval("Nx, Ny = length(xs), length(ys)")
            self.Nx = jl.seval("Nx")
            self.Ny = jl.seval("Ny")
            self.jlstore("w_rated", self.w_rated)
            jl.seval("M = ones(Nx, Ny) * w_rated")
            self.M = jl.seval("M")
            jl.seval("ergo_grid = SimulatorST.ErgoGrid(ngpkf_grid, (256,256))")
            self.ergo_grid = jl.seval("ergo_grid")

            # Initialize STGPKF Problem and get first estimate
            jl.seval("state = stgpkf_initialize(problem)")
            self.stgpkf_state = jl.seval("state")
            jl.seval("est = STGPKF.get_estimate(problem, state)")
            jl.seval("w_hat = reshape(est, length(xs), length(ys))")
            self.w_hat = jl.seval("w_hat")

            # Initialize Clarity Map
            jl.seval("qs = STGPKF.get_estimate_clarity(problem, state)")
            self.qs = jl.seval("qs")
            jl.seval("q_map = reshape(qs, length(xs), length(ys))")
            jl.seval("ergo_q_map = SimulatorST.ngpkf_to_ergo(ngpkf_grid, ergo_grid, q_map)")
            self.ergo_q_map = jl.seval("ergo_q_map")
            self.jlstore("target_q", self.target_q)
            
            # Initialize trajectory
            self.jlstore("current_x", self.position_xy[0])
            self.jlstore("current_y", self.position_xy[1])
            jl.seval("coords = [[@SVector[current_x, current_y]]]")
        except:
            self.get_logger().error("Ergodic Initialization Failed")
        pass

    def ergo_controller(self, speed):
        self.jlstore("current_x", self.position_xy[0])
        self.jlstore("current_y", self.position_xy[1])
        jl.seval("push!(coords, [@SVector[current_x, current_y]])")
        jl.seval("traj = vcat(coords...)")

        self.jlstore("speed", speed)
        jl.seval("speeds, new_q_target = Controller.ergo_controller_weighted_2(coords[end], M, w_rated, JordanLakeDomain.convex_polygon, target_q, Nx, Ny, xs, ys; ergo_grid=ergo_grid, ergo_q_map=ergo_q_map, traj=traj, umax=speed)")


        self.q_target = jl.seval("new_q_target")
        speeds = jl.seval("speeds[end]")
        
        return speeds[0], speeds[1]

    def speed_controller(self):
        # Get current SOC level - TODO: Implement this routine
        current_soc = self.state_of_charge

        now = datetime.now(self.local_tz)
        current_hrs = self.get_fractional_hours(now)
        idx = min(range(len(self.ts_hrs)), key=lambda i: abs(self.ts_hrs[i] - current_hrs))
        target_soc = self.soc_target[idx]

        # PID
        prev_error = self.error
        self.error = current_soc - target_soc
        self.error_sum += self.error
        difference = self.error - prev_error
        speed = self.kp*self.error + self.ki*self.error_sum + self.kd*difference
        speed = max(self.soc_controller.boat.v_min, min(speed, self.soc_controller.boat.v_max))
        return speed, target_soc
    
    def measurement_aggregator(self, msg):
        self.state_of_charge = msg.stateofcharge
        self.position_xy = [msg.pose_x, msg.pose_y]
        pass

    def get_day_of_year(self, now):
        """
            Return the current day of the year as a numerical value from 1-366
        """
        return float(now.timetuple().tm_yday)

    def get_fractional_hours(self, now):
        """
            Return the current time as a fractional hour (hrs.decimal)
        """
        hours = now.hour
        minutes_fraction = now.minute / 60.0
        seconds_fraction = now.second / 3600.0
        return hours + minutes_fraction + seconds_fraction

   
    def heading_calc(self, ux, uy):
        heading = np.degrees(np.atan2(ux, uy))
        return (heading + 360) % 360
    
    def save_all_julia_vars(self):
        """
            Function to save all Julia workspace variables to a JLD2 file
        """
        # Get all variable names in the Julia `Main` workspace
        var_names = jl.names(jl.Main, imported=True, all=True)
        
        # Create a dictionary with variable names as keys and their values as values
        vars_dict = {str(var): jl.seval(var) for var in var_names}
        
        # Save the dictionary to the JLD2 file
        jl.JLD2.save(self.filename, vars_dict)
        pass

def main(args=None):
    rclpy.init(args=args)
    
    controller = ASVErgoControl()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
