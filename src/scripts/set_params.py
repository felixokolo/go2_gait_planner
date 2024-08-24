import rclpy
from rclpy.node import Node
import json

# Import the custom message type
from go2_scratch.msg import ParamsSet

class ParamsPublisher(Node):
    def __init__(self):
        super().__init__('params_publisher')
        
        # Create a publisher for the custom message
        self.publisher_ = self.create_publisher(ParamsSet, '/go2_scratch/params', 10)
        
        # Read parameters from the JSON file
        with open('../assets/params.json', 'r') as f:
            params = json.load(f)
        
        # Create a message and assign the parameters
        msg = ParamsSet()
        # msg.q = params['q']
        # msg.dq = params['dq']
        # msg.tau = params['tau']
        msg.kp = params['kp']
        msg.kd = params['kd']
        msg.mask = params['mask']
        
        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info('Parameters published to /go2_scratch/params topic')

def main(args=None):
    rclpy.init(args=args)
    
    # Create the ParamsPublisher node
    params_publisher = ParamsPublisher()
    
    # Spin the node to keep it alive
    rclpy.spin(params_publisher)
    
    # Destroy the node explicitly
    params_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
