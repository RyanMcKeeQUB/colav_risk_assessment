import rclpy
from rclpy.node import Node
from unsafe_set_gen.unsafe_set import create_unsafe_set_polyshape
from unsafe_set_gen.objects import DynamicObject, DynamicObstacle, Configuration
from colav_interfaces.msg import AgentConfig, ObstaclesConfig, UnsafeSet
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray

class UnsafeSetGeneratorNode(Node):
    def __init__(self, node_name = 'risk_assesment_node', namespace='colav_risk_assesment', dt_global_update_tolerance:float=10):

        super().__init__(node_name, namespace=namespace)
        self.get_logger().info('starting colav/risk_assesment_node')

        self.unsafe_set_publisher = self.create_publisher(
            topic='/unsafe_set',
            msg_type=UnsafeSet,
            qos_profile=10
        )

        self.agent_sub = Subscriber(self, AgentConfig, '/agent_config')
        self.obstacles_sub = Subscriber(self, ObstaclesConfig, '/obstacles_config')

        # global update syncronised, when /agent_config and /obstacles_config both get updates in alloted time then
        # the sync_callback function is called!.
        self.sync = ApproximateTimeSynchronizer(
            [self.agent_sub, self.obstacles_sub],
            queue_size=10,
            slop=dt_global_update_tolerance
        )
        self.sync.registerCallback(self.sync_callback)

        # Store last update time
        self.last_update_time = self.get_clock().now()

        # Timer to check for timeouts
        self.timeout_tolerance = dt_global_update_tolerance * 2  # Give some extra margin
        self.timer = self.create_timer(0.1, self.check_for_timeout)  # Check every 0.1s
        self.dsf = 10

 

    def sync_callback(self, agent_msg, obstacles_msg): 
        self.get_logger().info("Received synchronized message: ")        
        self.get_logger().info(f"Agent Config: {agent_msg}")
        self.get_logger().info(f"Obstacles Config: {obstacles_msg}")

        self.process_data(agent_msg, obstacles_msg)
    
    def check_for_timeout(self):
        """Checks if an update has not been received within the allowed tolerance."""
        current_time = self.get_clock().now()
        time_since_last_update = (current_time - self.last_update_time).nanoseconds * 1e-9  # Convert to seconds

        if time_since_last_update > self.timeout_tolerance:
            self.get_logger().warn(f"No global update received in {time_since_last_update:.2f} seconds!")
            self.last_update_time = self.get_clock().now()  # Reset to prevent spamming



    def process_data(self, agent_msg:AgentConfig, obstacles_msg:ObstaclesConfig):
        self.get_logger().info('processing data!')
        agent_config = self.extract_agent_config(agent_config=agent_msg)
        obstacles_config = self.extract_dynamic_obstacles(dynamic_obstacles=obstacles_msg.dynamic_obstacles)
        unsafe_set_vertices = create_unsafe_set_polyshape(agent_vessel=agent_config, dynamic_obstacles=obstacles_config, dsf=self.dsf)
        unsafe_set_msg = UnsafeSet(
            header = Header(
                stamp = self.get_clock().now().to_msg(),
                frame_id = 'map'
            ),
            vertices = Float64MultiArray(
                data=[float(i) for i in unsafe_set_vertices]
            )
        )
        self.unsafe_set_publisher.publish(unsafe_set_msg)

    def extract_agent_config(self, agent_config:AgentConfig):
        return DynamicObject(
            configuration=Configuration(
                pose=Configuration.Pose(
                    position=Configuration.Pose.Position(
                        x = agent_config.pose.position.x,
                        y = agent_config.pose.position.y,
                        z = agent_config.pose.position.z
                    ),
                    orientation=Configuration.Pose.Orientation(
                        x = agent_config.pose.orientation.x,
                        y = agent_config.pose.orientation.y,
                        z = agent_config.pose.orientation.z,
                        w = agent_config.pose.orientation.w
                    )
                ),
                yaw_rate=agent_config.yaw_rate,
                velocity=agent_config.velocity
            ), 
            safety_radius=agent_config.safety_radius
        )

    def extract_dynamic_obstacles(self, dynamic_obstacles: list):
        obstacles_list = []
        for obstacle in dynamic_obstacles:
            obstacle_config = DynamicObstacle(
                id = obstacle.id,
                object=DynamicObject(
                    configuration=Configuration(
                        pose=Configuration.Pose(
                            position=Configuration.Pose.Position(
                                x = obstacle.pose.position.x,
                                y = obstacle.pose.position.y,
                                z = obstacle.pose.position.z
                            ),
                            orientation=Configuration.Pose.Orientation(
                                x = obstacle.pose.orientation.x,
                                y = obstacle.pose.orientation.y,
                                z = obstacle.pose.orientation.z,
                                w = obstacle.pose.orientation.w
                            )
                        ),
                        yaw_rate=obstacle.yaw_rate,
                        velocity=obstacle.velocity
                    ),
                    safety_radius=obstacle.safety_radius
                ),
            )
            obstacles_list.append(obstacle_config)
        
        return obstacles_list

def main(args = None):
    rclpy.init()
    node = UnsafeSetGeneratorNode()
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()