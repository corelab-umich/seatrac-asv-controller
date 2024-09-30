import rclpy
from rclpy.node import Node
from juliacall import Main as jl
import numpy as np

from messages.msg import Commands

class ASVErgoControl(Node):

    def __init__(self):
        super().__init__('asv_ergo_control')

        """ SOC Controller """
        # Create python instance of Julia SOC Controller
        self.get_logger().debug('Initializing SOC Controller')
        self.soc_controller = jl.include("src/asv_controller/jl_src/SOC_Controller.jl")

        # Define variables
        dt_sec = 2.5
        dt_min = dt_sec/(60.0)
        self.dt_hrs = dt_sec/(60.0 * 60.0)
        T_begin = 9.0 # TODO: Switch to initialize based on startup time
        T_end = 12.0 # TODO: Make this a parameter that is initialized at startup
        self.ts_hrs = [T_begin + i * self.dt_hrs for i in range(int((T_end - T_begin)/ self.dt_hrs) + 1)]
        soc_begin = 3000 # TODO: Initialize based on startup value
        soc_end = 3500 # TODO: Make this a value that is initialized based on T_end

        # Compute SOC barriers and target profile
        ucbf = self.soc_controller.compute_ucbf(self.ts_hrs, self.dt_hrs)
        lcbf = self.soc_controller.compute_lcbf(self.ts_hrs, self.dt_hrs)
        self.soc_target = self.soc_controller.generate_SOC_target(lcbf, ucbf, soc_begin, soc_end, self.ts_hrs, self.dt_hrs)

        # Speed Controller variables
        self.error_sum = 0.0
        self.error = 0.0

        """ Initialize Ergodic Controller """


        """ Control Loop on Timer """
        self.publisher_ = self.create_publisher(Commands, 'asv_command', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        self.get_logger().debug('Computing speed command')
        """ Compute Speed Command """
        speed = self.speed_controller()

        """ Get speeds from ergo controller """
        # TODO: Call actual ergodic controller

        
        """ Convert speeds to heading """
        # heading = heading_calc(4,3)

        """ Create and publish message with control command """
        msg = Commands()
        msg.speed_kts = speed
        # msg.heading = heading
        self.publisher_.publish(msg)
        pass

    def speed_controller(self):
        # Speed Control PID Gains - TODO: Make these parameters to adjust in real time
        kp = 0.5
        ki = 0.01
        kd = 0.5

        # Get current SOC level - TODO: Implement this routine
        current_soc = 150.0
        target_soc = 0.0

        # PID
        prev_error = self.error
        self.error = current_soc - target_soc
        self.error_sum += self.error
        difference = self.error - prev_error
        speed = kp*self.error + ki*self.error_sum + kd*difference
        speed = max(self.soc_controller.boat.v_min, min(speed, self.soc_controller.boat.v_max))
        return speed

   
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