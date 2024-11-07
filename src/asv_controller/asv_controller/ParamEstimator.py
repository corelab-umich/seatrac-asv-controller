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

class ParamEstimator(Node):

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'manual_hyperparam_override' and param.type_ == Parameter.Type.BOOL:
                self.manual_param_override = param.value
            if param.name == 'spatial_length' and param.type_ == Parameter.Type.DOUBLE:
                self.spatial_length = param.value
            if param.name == 'temporal_length' and param.type_ == Parameter.Type.DOUBLE:
                self.temporal_length = param.value
            if param.name == 'spatial_deviation' and param.type_ == Parameter.Type.DOUBLE:
                self.spatial_deviation = param.value
            if param.name == 'temporal_deviation' and param.type_ == Parameter.Type.DOUBLE:
                self.temporal_deviation = param.value
            if param.name == 'measurement_window_sec' and param.type_ == Parameter.Type.DOUBLE:
                self.measure_window = param.value
                self.jlstore("measure_window", self.measure_window)
        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__('param_estimator')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,  # Adjust depth based on your needs
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
      
        """ Timezone/Clock Setup """
        self.local_tz = get_localzone()

        """ Declare User Parameters """
        self.declare_parameter('manual_hyperparam_override', False)
        self.manual_param_override = False
        self.declare_parameter('spatial_length', 2.0)
        self.spatial_length = 2.0
        self.declare_parameter('temporal_length', 5.0)
        self.temporal_length = 5.0
        self.declare_parameter('spatial_deviation', 1.0)
        self.spatial_deviation = 1.0
        self.declare_parameter('temporal_deviation', 1.0)
        self.temporal_deviation = 1.0
        self.declare_parameter('measurement_window_sec', 60.0 * 30.0)
        self.measure_window = 60.0 * 30.0

        """ Parameter Update Function """
        self.add_on_set_parameters_callback(self.parameter_callback)

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
        self.jlstore("measure_window", self.measure_window)

        """ Subscribe to Sensor Data """
        self.subscription = self.create_subscription(
            SensorData,
            'measurement_packet',
            self.measurement_aggregator,
            qos_profile)
        self.subscription  # prevent unused variable warning

        """ Parameter Estimate Publisher"""
        self.publisher_ = self.create_publisher(ParamEst, 'param_estimates', qos_profile)
        timer_period = 30 # seconds
        self.timer = self.create_timer(timer_period, self.timed_estimator)
    
    def timed_estimator(self):
        self.get_logger().debug('Estimating New Parameters')

        """ Estimate Parameters """
        if len(self.measurements) >= 100:
            try:
                params = self.variograms.hp_fit(self.measurements)
            except:
                params = [self.spatial_deviation * self.temporal_deviation, self.spatial_length, self.temporal_length]
                self.get_logger().error('Param Estimation Failure')

            """ Create and publish message with parameter estimates """
            msg = ParamEst()
            if self.manual_param_override:
                msg.spatial_length = self.spatial_length
                msg.temporal_length = self.temporal_length
                msg.temporal_deviation = self.temporal_deviation
                msg.spatial_deviation = self.spatial_deviation
            else:
                msg.spatial_length = params[1]
                msg.temporal_length = params[2]
                msg.temporal_deviation = np.sqrt(params[0])
                msg.spatial_deviation = np.sqrt(params[0])
            self.publisher_.publish(msg)

        """ Remove oldest measurement """
        if len(self.measurements) > (self.measure_window):
            jl.seval("measurements = measurements[(length(measurements) - Int(measure_window)) : end]")
        # TODO: Make the measurement window a parameter
        # try:
        #     if len(self.measurements) > (self.measure_window):
        #         jl.seval("measurements = [length(measurements) - measure_window:end]")
        # except:
        #     self.get_logger().error('Measurement Trimming Failed')
        # pass

    def measurement_aggregator(self, msg):
        """ Convert lat/long to x/y positions """
        jl.position = [msg.pose_x, msg.pose_y]

        """ Get current time """
        now = datetime.now(self.local_tz)
        fractional_hour = self.get_fractional_hours(now)
        jl.time = fractional_hour

        """ Get current wind speed """
        jl.speed = msg.windspeed

        """ Push sensor data into Measurement Struct """
        measure_struct = self.variograms.MeasurementSpatial(jl.time, jl.position, jl.speed)
        self.jlstore("measure_struct", measure_struct)
        jl.seval("push!(measurements, measure_struct)")

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
    
    estimator = ParamEstimator()

    rclpy.spin(estimator)

    estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()