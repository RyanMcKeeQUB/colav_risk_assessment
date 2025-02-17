import rclpy
from scripts.unsafe_set_generator_node import UnsafeSetGeneratorNode

def main(args = None):
    rclpy.init()
    node = UnsafeSetGeneratorNode()
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()