import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from juliacall import Main as jl
import numpy as np
from datetime import datetime
from tzlocal import get_localzone

from messages.msg import Commands, SensorData, ParamEst

from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

K_KTS2MS = 0.5144444444 # kts -> m/s conversion
K_MS2KTS = 1.9438444924 # m/s -> kts conversion
K_M2KM = 0.001 # meters -> kilometers

class TransectControl(Node):

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'controller_enable' and param.type_ == Parameter.Type.BOOL:
                self.controller_enable = param.value
                if self.controller_enable:
                    self.controller_init()
                else:
                    self.controller_initalized = False
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
        super().__init__('transect_control')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,  # Adjust depth based on your needs
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

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

        """ Parameter Update Function """
        self.add_on_set_parameters_callback(self.parameter_callback)

        """ Julia Imports """
        # Creating variable storage function
        self.jlstore = jl.seval("(k, v) -> (@eval $(Symbol(k)) = $v; return)")

        # Importing packages
        jl.seval("using LinearAlgebra, Random, Statistics, Plots, StaticArrays, Interpolations, LazySets, SpatiotemporalGPs, JLD2, LinearInterpolations, Dates")

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
        self.transects = jl.include('src/asv_controller/jl_src/transects.jl')
        self.trajlib = jl.include('src/asv_controller/jl_src/TrajLib.jl')

        # JLD2 Saving
        self.file_counter = 0
        current_time = datetime.now()
        self.time_string = current_time.strftime("%Y-%m-%d_%H-%M-%S")
        self.filename = f"/root/jld2_files/{self.time_string}_transect_jlvars_{self.file_counter}.jld2"
        filename_timer = 60*15 # minutes -> seconds
        self.filetimer = self.create_timer(filename_timer, self.filename_update)
        jl.seval("""
            function save_selected_variables(fname, vars_to_save)
                group_name = Dates.format(now(), "yyyymmdd_HH-MM-ss.sSSS")

                # Convert variable names to Symbols and filter defined variables
                workspace_vars = Dict(
                    string(var) => getfield(Main, Symbol(var))
                    for var in vars_to_save
                    if isdefined(Main, Symbol(var)) && !(typeof(getfield(Main, Symbol(var))) <: Function || typeof(getfield(Main, Symbol(var))) <: Module)
                )
                
                # Save to a JLD2 file with compression, under a specified group
                jldopen(fname, "a", compress=true) do fid
                    group = JLD2.Group(fid, group_name)
                    for (name, value) in workspace_vars
                        group[name] = value
                    end
                end
            end
            """)


        """ Subscribe to Sensor Data """
        self.subscription = self.create_subscription(
            SensorData,
            'measurement_packet',
            self.measurement_aggregator,
            qos_profile)
        self.subscription  # prevent unused variable warning
        self.state_of_charge = None
        self.position_xy = None

        """ Subscribe to Parameter Estimates """
        self.param_sub = self.create_subscription(
            ParamEst,
            'param_estimates',
            self.param_update,
            qos_profile)
        self.spatial_length = 0.0
        self.temporal_length = 0.0
        self.spatial_deviation = 0.0
        self.temporal_deviation = 0.0

        """ SOC Controller Variables """
        # Time variables
        self.dt_sec = 2.5
        # dt_min = self.dt_sec/(60.0)
        self.dt_hrs = self.dt_sec/(60.0 * 60.0)
        self.T_begin = None
        self.T_end = None
        self.ts_hrs = None
        self.soc_target = None
        self.target_soc = None

        # Speed Controller variables
        self.error_sum = 0.0
        self.error = 0.0

        """ Transect Controller Variables """
        self.controller_initalized = False
        self.current_wp_index = 0

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
        self.w_rated = None

        """ Filter Updates """
        timer_filter = 5 # seconds
        self.filter_timer = self.create_timer(timer_filter, self.filter_update)

        # Create array to store measurements for filtering
        self.jlstore("sigma_meas", 0.25) # TODO: Make adjustable parameter for sensor noise
        self.sigma_meas = jl.seval("sigma_meas")
        jl.seval("measurement_pts = Vector{SVector{2, Float64}}()")
        jl.seval("measurement_w = Vector{Float64}()")
        jl.seval("measure_sigma = sigma_meas")

        """ Control Loop on Timer """
        self.publisher_ = self.create_publisher(Commands, 'asv_command', 10)
        timer_period = 1 # seconds
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

            """ Get heading from transect controller """
            heading = self.transect_controller(speed)
            # speed_x, speed_y = self.transect_controller(speed)
            # try:
            #    speed_x, speed_y = self.transect_controller(speed)
            # except:
            #    speed_x = speed
            #    speed_y = 0.0
            #    self.get_logger().error('Ergodic Controller Error')

            """ Convert speeds to heading """
            # try:
            #     heading = self.heading_calc(speed_x, speed_y)
            # except:
            #     heading = 0
            #     self.get_logger().error('Heading Calculation Error')

            """ Publish message with control command """
            msg.speed_kts = speed * K_MS2KTS
            msg.heading = heading
            msg.target_bat_soc = target_soc
            self.publisher_.publish(msg)

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
            self.dx = self.jlstore("dx", 0.05)
            self.xs = jl.seval("xs = 0:dx:1.6")
            self.ys = jl.seval("ys = 0:dx:1.9")

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

            self.transect_xs = jl.seval("transect_xs = 0:0.3:2")
            self.transect_ys = jl.seval("transect_ys = 0:0.3:2")
            self.pts = jl.seval("pts = vec([[x, y] for x in xs, y in ys])")
            self.transect_pts = jl.seval("transect_pts = Transects.create_points(pts)")

            
            # Clear measure vecs
            jl.seval("measurement_pts = Vector{SVector{2, Float64}}()")
            jl.seval("measurement_w = Vector{Float64}()")

            # Let us know controller finished enabling
            self.controller_initalized = True
        except:
            self.get_logger().error("Ergodic Initialization Failed")
        
        pass

    def transect_controller(self, speed):
        self.jlstore("current_x", self.position_xy[0])
        self.jlstore("current_y", self.position_xy[1])
        jl.seval("push!(coords, [@SVector[current_x, current_y]])")
        jl.seval("traj = vcat(coords...)")

        self.jlstore("speed", speed)
        pos_x = self.position_xy[0]
        pos_y = self.position_xy[1]

        target = self.transect_pts[self.current_wp_index]
        x_target = target[0]
        y_target = target[1]
        if np.sqrt(np.power(pos_x - x_target, 2) + np.power(pos_y - y_target, 2))<= 0.05:
            self.current_wp_index += 1
            self.current_wp_index %= len(self.transect_pts)
        
        target = self.transect_pts[self.current_wp_index]
        x_target = target[0]
        y_target = target[1]

        heading = self.compute_heading(pos_x, pos_y, x_target, y_target)
        return heading[1]

    def speed_controller(self):
        current_soc = self.state_of_charge

        now = datetime.now(self.local_tz)
        current_hrs = self.get_fractional_hours(now)
        idx = min(range(len(self.ts_hrs)), key=lambda i: abs(self.ts_hrs[i] - current_hrs))
        target_soc = self.soc_target[idx]
        self.target_soc = target_soc

        # PID
        prev_error = self.error
        self.error = current_soc - target_soc
        self.error_sum += self.error
        difference = self.error - prev_error
        speed = self.kp*self.error + self.ki*self.error_sum + self.kd*difference
        speed = max(self.soc_controller.boat.v_min, min(speed, self.soc_controller.boat.v_max))
        return speed, target_soc
    
    def measurement_aggregator(self, msg):
        # Store state of charge
        self.state_of_charge = msg.stateofcharge

        # Store Position
        self.position_xy = [msg.pose_x, msg.pose_y]
        
        # Add position to measurement vector for KF
        self.jlstore("temp_x", self.position_xy[0])
        self.jlstore("temp_y", self.position_xy[1])
        jl.seval("push!(measurement_pts, @SVector[temp_x, temp_y])")

        # Add windspeed to measurement vector for KF
        self.jlstore("temp_speed", msg.windspeed)
        jl.seval("push!(measurement_w, temp_speed)")

        self.w_rated = msg.ratedwind

        pass

    def param_update(self, msg):
        self.spatial_length = msg.spatial_length
        self.temporal_length = msg.temporal_length
        self.spatial_deviation = msg.spatial_deviation
        self.temporal_deviation = msg.temporal_deviation

        # Generate new STGPKF state
        try:
            self.kt = self.jlstore("kt", jl.Matern(1/2, self.temporal_deviation, self.temporal_length))
            self.ks = self.jlstore("ks", jl.Matern(1/2, self.spatial_deviation, self.spatial_length))
            self.dx = self.jlstore("dx", 0.05)
            self.stgpkfprob = jl.seval("problem = STGPKFProblem(grid_points, ks, kt, dt_min)")
            self.ngpkf_grid = jl.seval("ngpkf_grid = NGPKF.NGPKFGrid(xs, ys, ks)")
            jl.seval("ergo_grid = SimulatorST.ErgoGrid(ngpkf_grid, (256,256))")
            self.ergo_grid = jl.seval("ergo_grid")  
        except:
            self.get_logger().error('Param Estimation Failure')
        pass

    def filter_update(self):
        if self.controller_initalized:
            jl.seval("measure_sigma = (sigma_meas^2) * I(length(measurement_w))")

            # Update estimate using KF
            jl.seval("state_correction = stgpkf_correct(problem, state, measurement_pts, measurement_w, measure_sigma)")
            jl.seval("state = stgpkf_predict(problem, state_correction)")
            self.stgpkf_state = jl.seval("state")
            jl.seval("est = STGPKF.get_estimate(problem, state)")
            jl.seval("w_hat = reshape(est, length(xs), length(ys))")
            self.w_hat = jl.seval("w_hat")

            # update the clarity map
            jl.seval("qs = STGPKF.get_estimate_clarity(problem, state)")
            self.qs = jl.seval("qs")
            jl.seval("q_map = reshape(qs, length(xs), length(ys))")
            jl.seval("ergo_q_map = SimulatorST.ngpkf_to_ergo(ngpkf_grid, ergo_grid, q_map)")
            self.ergo_q_map = jl.seval("ergo_q_map")
            self.jlstore("target_q_matrix", self.target_q_matrix)

            jl.seval("M = w_hat")
            self.M = jl.seval("M")

            self.jlstore("current_soc", self.state_of_charge)
            self.jlstore("soc_target", self.target_soc)

            # Save variable states to JLD2 file
            variables_to_save = ["state", "est", "w_hat", "q_map", "ergo_q_map", "target_q_matrix", "measurement_pts", "measurement_w", "current_soc", "soc_target"]
            jl.save_selected_variables(self.filename, variables_to_save)  

            # Save plots
            jl.seval("""
                        polygon_vertices = hcat(JordanLakeDomain.convex_polygon.vertices, JordanLakeDomain.convex_polygon.vertices[:, 1])
                        heatmap(xs, ys, q_map', clims=(0, 1))
                        plot!(polygon_vertices[1, :], polygon_vertices[2, :], seriestype=:shape, fillalpha=0.0, label="", lw = 2, linecolor = "green")
                        plot!(legend=false)
                        xlabel!("x [km]")
                        ylabel!("y [km]")
                        title!("Clarity")
                        savefig("/root/images/q_map.png")
                     """)      
            jl.seval("""
                        polygon_vertices = hcat(JordanLakeDomain.convex_polygon.vertices, JordanLakeDomain.convex_polygon.vertices[:, 1])
                        heatmap(xs, ys, w_hat', cmap = :balance, clims=(-5, 5))
                        plot!(polygon_vertices[1, :], polygon_vertices[2, :], seriestype=:shape, fillalpha=0.0, label="", lw = 3, linecolor = "green")
                        plot!(legend=false)
                        xlabel!("x [km]")
                        ylabel!("y [km]")
                        title!("Estimate")
                        savefig("/root/images/w_hat.png")
                     """)

        # Clear measure vecs
        jl.seval("measurement_pts = Vector{SVector{2, Float64}}()")
        jl.seval("measurement_w = Vector{Float64}()")
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

    def filename_update(self):
        self.file_counter += 1
        self.filename = f"/root/jld2_files/{self.time_string}_jlvars_{self.file_counter}.jld2"
        pass

    def heading_calc(self, ux, uy):
        heading = np.degrees(np.arctan2(ux, uy))
        return (heading + 360) % 360
    
    def compute_heading(self, x_current, y_current, x_target, y_target):
        # Calculate the differences in x and y coordinates
        delta_x = x_target - x_current
        delta_y = y_target - y_current
        
        # Compute the heading angle in radians
        heading_radians = np.arctan2(delta_x, delta_y)
        
        # Convert the angle to degrees
        heading_degrees = np.degrees(heading_radians)
        
        # Ensure heading is within the range 0 to 360
        if heading_degrees < 0:
            heading_degrees += 360
        
        return heading_radians, heading_degrees

def main(args=None):
    rclpy.init(args=args)
    
    controller = TransectControl()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
