import rclpy
from rclpy.node import Node
from juliacall import Main as jl
import numpy as np
from datetime import datetime
from tzlocal import get_localzone

from messages.msg import Commands, SensorData, ParamEst

class ParamEstimator(Node):

    def __init__(self):
        super().__init__('param_estimator')

        """ Timezone/Clock Setup """
        self.local_tz = get_localzone()

        """ Instantiate Variogram Module """
        # Create python instance of Julia variogram functions
        self.get_logger().debug('Initializing Param Estimator')
        self.variograms = jl.include("src/asv_controller/jl_src/variograms.jl") 

        # Create python instance of MeasurementSpatial struct
        self.measurements = jl.seval("measurements = []")
        jl.seval("position = [0.0, 0.0]")
        jl.seval("time = 0.0")
        jl.seval("speed = 0.0")
        self.jlstore = jl.seval("(k, v) -> (@eval $(Symbol(k)) = $v; return)")

        """ Subscribe to Sensor Data """
        self.subscription = self.create_subscription(
            SensorData,
            'measurement_packet',
            self.measurement_aggregator,
            10)
        self.subscription  # prevent unused variable warning

        """ Parameter Estimate Publisher"""
        self.publisher_ = self.create_publisher(ParamEst, 'param_estimates', 10)
        timer_period = 60 # seconds
        self.timer = self.create_timer(timer_period, self.timed_estimator)
    
    def timed_estimator(self):
        self.get_logger().debug('Estimating New Parameters')

        """ Create and publish message with parameter estimates """
        msg = ParamEst()
        msg.spatial_length = 0.0
        self.publisher_.publish(msg)

        """ Remove oldest measurement """
        if len(self.measurements) >= (60*30):
            jl.seval("popfirst!(measurements)")
        pass

    def measurement_aggregator(self, msg):
        """ Convert lat/long to x/y positions """
        # TODO: Replace with actual message data
        jl.position = [1.3, 2.2]

        """ Get current time """
        now = datetime.now(self.local_tz)
        fractional_hour = self.get_fractional_hours(now)
        jl.time = fractional_hour

        """ Get current wind speed """
        # TODO: Replace with live sensor data
        # jl.speed = msg.speedoverground
        jl.speed = 2.0

        """ Push sensor data into Measurement Struct """
        jl.seval("push!(measurements, MeasurementSpatial(time, position, speed))")

        pass

def main(args=None):
    rclpy.init(args=args)
    
    estimator = ParamEstimator()

    rclpy.spin(estimator)

    estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()