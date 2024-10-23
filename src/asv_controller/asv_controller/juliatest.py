import rclpy
from rclpy.node import Node
from juliacall import Main as jl
import numpy as np
from datetime import datetime
from tzlocal import get_localzone

from std_msgs.msg import String
from messages.msg import Commands, SensorData

from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

K_KTS2MS = 0.5144444444 # kts -> m/s conversion
K_MS2KTS = 1.9438444924 # m/s -> kts conversion
K_M2KM = 0.001 # meters -> kilometers

class JuliaPublisher(Node):

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'sim_enable' and param.type_ == Parameter.Type.BOOL:
                self.sim_enable = param.value
            if param.name == 'sim_soc_begin' and param.type_ == Parameter.Type.DOUBLE:
                if not self.sim_enable: # update SOC only if sim has not begun
                    self.state_of_charge = param.value
        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__('synthetic_data_pub')

        """ Timezone/Clock Setup """
        self.local_tz = get_localzone()

        """ Declare User Parameters """
        self.declare_parameter('sim_soc_begin', 6500.0)
        self.declare_parameter('sim_enable', False)
        self.sim_enable = False

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

        # """ SOC Controller Testing"""
        dt_sec = 2.5
        dt_min = dt_sec/(60.0)
        self.jlstore("dt_min", dt_min)
        self.dt_hrs = dt_sec/(60.0 * 60.0)
        self.T_begin = 9.0
        self.jlstore("T_begin", self.T_begin)
        self.T_end = 12.0
        self.jlstore("T_end", self.T_end)
        self.ts_hrs = [self.T_begin + i * self.dt_hrs for i in range(int((self.T_end - self.T_begin)/ self.dt_hrs) + 1)]
        
        """ Generate Synthetic Data """
        sigma_t = 2.0
        sigma_s = 1.0
        lt = 0.75 * 60.0 # minutes
        ls = 0.75 # km

        kt = self.jlstore("kt", jl.Matern(1/2, sigma_t, lt))
        ks = self.jlstore("ks", jl.Matern(1/2, sigma_s, ls))
        dx = self.jlstore("dx", 0.10)
        xs = jl.seval("xs = 0:dx:1.4")
        ys = jl.seval("ys = 0:dx:6.5")

        self.synthetic_data = jl.seval("STGPKF.generate_spatiotemporal_process(xs, ys, dt_min, (T_end - T_begin)*60.0, ks, kt)")
        # print(self.synthetic_data.ts[2])

        """ Other Variables """
        # ASV Params
        self.boat = self.soc_controller.ASV_Params()

        # Commands from controller
        self.speed_command = 0.0
        self.heading_command = 0.0

        # State variables
        self.latitude = 0.0
        self.longitude = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.state_of_charge = 0.0
        self.windspeed = 0.0
        self.sim_time = self.synthetic_data.ts[0]

        """ Subscribe to Sensor Data """
        self.subscription = self.create_subscription(
            Commands,
            'asv_command',
            self.command_parser,
            10)
        self.subscription  # prevent unused variable warning

        """ Simulation Update Step """
        self.synth_publisher_ = self.create_publisher(SensorData, 'measurement_packet', 10)
        self.timer_period = 0.5 # seconds
        self.timer = self.create_timer(self.timer_period, self.sim_update)

    
    def command_parser(self, msg):
        self.speed_command = msg.speed_kts
        self.heading_command = msg.heading
        

    def sim_update(self):
        """
            Update state of the simulation
        """
        if self.sim_enable:

            """ Create Message """
            new_data = SensorData()

            """ Get Time """
            now = datetime.now(self.local_tz)
            day_of_year = self.get_day_of_year(now)
            self.sim_time += (self.timer_period / 60.0) # (time step [s] -> min)

            """ Update Position """
            # Convert speed from kts to ms & pull heading
            speed = self.speed_command * K_KTS2MS
            self.get_logger().debug('ASV Speed: {new_data.windspeed}')
            heading = self.heading_command

            # update x/y position
            self.pose_x += (speed*np.cos(np.radians(heading))) * self.timer_period * K_M2KM
            self.pose_y += (speed*np.sin(np.radians(heading))) * self.timer_period * K_M2KM
            new_data.pose_x = self.pose_x
            new_data.pose_y = self.pose_y

            """ Update Vehicle State of Charge """
            # TODO: FIGURE OUT HOW TO GET THIS TO WORK
            try:
                self.state_of_charge = self.soc_controller.batterymodel(int(day_of_year), self.sim_time / 60.0, 35.75, speed, self.state_of_charge)
            except:
                self.get_logger().error('Battery Model Failure')
                            
            new_data.stateofcharge = self.state_of_charge
            """ Update Windspeed """
            self.get_logger().debug('Interpolating wind data')
            try:
                new_data.windspeed = self.synthetic_data.itp(self.pose_x, self.pose_y, self.sim_time)
            except:
                self.get_logger().error('Wind Interpolation Out Of Bounds')
            self.get_logger().debug('Measured wind: {new_data.windspeed}')

            """ Publish Simulation Update & Synthetic Data """
            self.synth_publisher_.publish(new_data)
    
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

def main(args=None):
    rclpy.init(args=args)
    
    jl_publisher = JuliaPublisher()

    rclpy.spin(jl_publisher)

    jl_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()