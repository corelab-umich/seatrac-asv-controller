import rclpy
from rclpy.node import Node
from juliacall import Main as jl
import numpy as np
from datetime import datetime
from tzlocal import get_localzone

from std_msgs.msg import String
from messages.msg import Commands, SensorData

class JuliaPublisher(Node):

    def __init__(self):
        super().__init__('julia_publisher')

        """ Timezone/Clock Setup """
        self.local_tz = get_localzone()

        """ Julia Imports """
        # Creating variable storage function
        self.jlstore = jl.seval("(k, v) -> (@eval $(Symbol(k)) = $v; return)")

        # Importing packages
        jl.seval("using SpatiotemporalGPs")

        # Importing modules
        self.soc_controller = jl.include("src/asv_controller/jl_src/SOC_Controller.jl")

        # """ SOC Controller Testing"""
        dt_sec = 2.5
        dt_min = dt_sec/(60.0)
        self.dt_hrs = dt_sec/(60.0 * 60.0)
        self.dt_hrs = self.jlstore("")
        T_begin = 9.0
        T_end = 12.0
        self.ts_hrs = [T_begin + i * self.dt_hrs for i in range(int((T_end - T_begin)/ self.dt_hrs) + 1)]
        
        """ Generate Synthetic Data """
        sigma_t = 2.0
        sigma_s = 1.0
        lt = 0.75 * 60.0 # minutes
        ls = 0.75 # km

        kt = jl.Matern(1/2, sigma_t, lt)
        ks = jl.Matern(1/2, sigma_s, ls)
        dx = 0.1
        xs = np.arange(0, 1.4 + dx, dx)
        ys = np.arange(0, 6.5 + dx, dx)

        synthetic_data = jl.STGPKF.generate_spatiotemporal_process(xs, ys, dt_min, (T_end - T_begin)*60.0, ks, kt)

        """ Subscribe to Sensor Data """
        self.subscription = self.create_subscription(
            Commands,
            'asv_command',
            self.sim_update,
            10)
        self.subscription  # prevent unused variable warning

        self.synth_publisher_ = self.create_publisher(SensorData, 'measurement_packet', 10)

        """ ROS Publisher"""
        self.publisher_ = self.create_publisher(String, 'julia_msg', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):

        # ucbf = self.soc_controller.compute_ucbf(self.ts_hrs, self.dt_hrs)
        msg = String()
        # msg.data = str(ucbf[self.i])
        now = datetime.now(self.local_tz)
        day_of_year = self.get_day_of_year(now)
        fractional_hour = self.get_fractional_hours(now)
        self.get_logger().info(f"Day of year: {day_of_year:.2f}")
        msg.data = str(fractional_hour)
        # self.get_logger().info(f"Time: {fractional_hour:.6f}")
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def sim_update(self, msg, heading):
        """ Update Position """
        new_data = SensorData()
        new_data.pose_x = msg.pose_x + msg.speed*np.cos(np.radians(heading))
        new_data.pose_y = msg.pose_y + msg.speed*np.sin(np.radians(heading))
        new_data.windspeed = self.synthetic_data(msg.pose_x, msg.pose_y, self.get_fractional_hours(now))
        self.synth_publisher_.publish(new_data)


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