import rclpy
from rclpy.node import Node
from juliacall import Main as jl
import numpy as np

from std_msgs.msg import String

class JuliaPublisher(Node):

    def __init__(self):
        super().__init__('julia_publisher')

        """ SOC Controller Testing"""
        self.soc_controller = jl.include("src/asv_controller/jl_src/SOC_Controller.jl")
        dt_sec = 2.5
        dt_min = dt_sec/(60.0)
        self.dt_hrs = dt_sec/(60.0 * 60.0)
        T_begin = 9.0
        T_end = 12.0
        self.ts_hrs = [T_begin + i * self.dt_hrs for i in range(int((T_end - T_begin)/ self.dt_hrs) + 1)]

        """ ROS Publisher"""
        self.publisher_ = self.create_publisher(String, 'julia_msg', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):

        ucbf = self.soc_controller.compute_ucbf(self.ts_hrs, self.dt_hrs)
        msg = String()
        msg.data = str(ucbf[self.i])
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    
    jl_publisher = JuliaPublisher()

    rclpy.spin(jl_publisher)

    jl_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()