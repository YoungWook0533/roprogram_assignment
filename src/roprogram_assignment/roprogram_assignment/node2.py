import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from multiplier_interfaces.srv import Multiply
from multiplier_interfaces.msg import Uid
from multiplier_interfaces.action import AddDigits

class MinimalSubscriberClientAction(Node):

    def __init__(self):
        super().__init__('minimal_subscriber_client_action')
        self.subscription = self.create_subscription(Uid,'University_ID',self.listener_callback,10) #subscribing uid with customized interface
        self.cli = self.create_client(Multiply, 'multiply')                                         #service client
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Multiply.Request()
        self._action_client = ActionClient(self, AddDigits, 'add_digits')                           #action client
        self.declare_parameter('my_uid', '2020741046')                                              #parameter

    def listener_callback(self, msg):
        self.get_logger().info('Recieved: "%s"' % msg.uid)

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def send_goal(self):
        my_param = self.get_parameter('my_uid').get_parameter_value().string_value      #set parameters
        
        my_new_param = rclpy.parameter.Parameter(
            'my_uid',
            rclpy.Parameter.Type.STRING,
            '2020741046'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

        goal_msg = AddDigits.Goal()                                                     #set action goals
        goal_msg.uid = my_param

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result : %d' % result.sequence[0])
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: [First element is partial sum] {0}'.format(feedback.partial_sequence))
           


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber_client_action = MinimalSubscriberClientAction()
    response = minimal_subscriber_client_action.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_subscriber_client_action.get_logger().info(
        'Result of multipy: %d * %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.product))
    
    minimal_subscriber_client_action.send_goal()

    rclpy.spin(minimal_subscriber_client_action)

    minimal_subscriber_client_action.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
