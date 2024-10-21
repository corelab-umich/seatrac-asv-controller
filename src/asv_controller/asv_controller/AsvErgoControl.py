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
        # Define variables
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
            speed, target_soc = self.speed_controller()
            self.get_logger().info('Compute Speed {speed}')

            """ Get speeds from ergo controller """
            # TODO: Call actual ergodic controller
            # self.control_class.ergo_controller_weighted_2()
            

            
            """ Convert speeds to heading """
            # heading = heading_calc(4,3)

            """ Publish message with control command """
            
            msg.speed_kts = speed * K_MS2KTS
            # msg.heading = heading
            msg.target_bat_soc = target_soc
            self.publisher_.publish(msg)
        else:
            msg.speed_kts = 0.0
            msg.heading = 0.0
            self.publisher_.publish(msg)
        pass
    
    def controller_init(self):
        self.get_logger().debug('Initializing Controller')

        # Define variables
        now = datetime.now(self.local_tz)
        day_of_year = self.get_day_of_year(now)
        self.T_begin = self.get_fractional_hours(now)
        self.T_end = self.T_begin + 3.0 # TODO: Make this adjustable
        self.ts_hrs = [self.T_begin + i * self.dt_hrs for i in range(int((self.T_end - self.T_begin)/ self.dt_hrs) + 1)]
        soc_begin = self.state_of_charge
        soc_end = soc_begin - 500 # TODO: Make this a value that is initialized based on T_end

        # Compute SOC barriers and target profile
        ucbf = self.soc_controller.compute_ucbf(self.ts_hrs, self.dt_hrs)
        lcbf = self.soc_controller.compute_lcbf(self.ts_hrs, self.dt_hrs)
        self.soc_target = self.soc_controller.generate_SOC_target(lcbf, ucbf, soc_begin, soc_end, self.ts_hrs, self.dt_hrs)

        self.get_logger().debug('Generated SOC Target: {self.soc_target[2]}')
        # Initialize ergodic control

        pass

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
        self.get_logger().debug('Computed Speed Command: {speed}')
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

   
    # def heading_calc(self, ux, uy):
    #     heading = np.arctan2(uy, ux) * (180/np.pi)
    #     return (heading + 360) % 360

def main(args=None):
    rclpy.init(args=args)
    
    controller = ASVErgoControl()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()