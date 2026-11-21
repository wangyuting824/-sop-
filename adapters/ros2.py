"""Import ROS only on the robot; QoS selected from actual publisher endpoints."""
import time


class Subscription:
    def __init__(self,adapter,on_message):
        import rclpy
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from rosidl_runtime_py.utilities import get_message
        from rosidl_runtime_py.convert import message_to_ordereddict
        self.rclpy=rclpy;self.owned=not rclpy.ok()
        if self.owned:rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
        self.node=rclpy.create_node('robotcal_readonly_'+str(time.time_ns()))
        self.topic=adapter.config['feedback']['topic'];self.type=adapter.config['feedback']['type']
        deadline=time.monotonic()+10
        try:
            while rclpy.ok():
                entries=dict(self.node.get_topic_names_and_types())
                if self.topic in entries:break
                if time.monotonic()>deadline:raise RuntimeError(f'ROS topic not found: {self.topic}')
                rclpy.spin_once(self.node,timeout_sec=.1)
            if self.type not in entries.get(self.topic,[]):raise RuntimeError('Live topic type differs from YAML')
            qos=QoSProfile(depth=100)
            qos.durability=DurabilityPolicy.VOLATILE
            publishers=self.node.get_publishers_info_by_topic(self.topic)
            qos.reliability=(ReliabilityPolicy.BEST_EFFORT if any(p.qos_profile.reliability==ReliabilityPolicy.BEST_EFFORT for p in publishers)
                             else ReliabilityPolicy.RELIABLE)
            def callback(message):
                clocks={'received_unix_ns':time.time_ns(),'received_monotonic_ns':time.monotonic_ns(),
                        'subscriber_ros_time_ns':self.node.get_clock().now().nanoseconds}
                on_message(message_to_ordereddict(message),clocks)
            self.sub=self.node.create_subscription(get_message(self.type),self.topic,callback,qos)
        except BaseException:
            self.close();raise

    def spin(self):
        if not self.rclpy.ok():raise RuntimeError('ROS context stopped')
        self.rclpy.spin_once(self.node,timeout_sec=.1)

    def close(self):
        if getattr(self,'node',None) is not None:self.node.destroy_node();self.node=None
        if self.owned and self.rclpy.ok():self.rclpy.shutdown()


def read_once(adapter):
    received=[]
    def receive(message,clocks):received.append({'clocks':clocks,'raw':message,'states':adapter.decode(message)})
    sub=Subscription(adapter,receive)
    try:
        deadline=time.monotonic()+10
        while not received:
            if time.monotonic()>deadline:raise TimeoutError('No valid feedback in 10 seconds')
            sub.spin()
        return received[-1]
    finally:sub.close()
