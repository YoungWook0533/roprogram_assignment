import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from std_msgs.msg import String

from multiplier_interfaces.srv import Multiply
from multiplier_interfaces.msg import Uid
from multiplier_interfaces.action import AddDigits


class MinimalPublisherServiceAction(Node):

    def __init__(self):
        super().__init__('minimal_publisher_service_action')
        self.publisher = self.create_publisher(Uid, 'University_ID', 10)                        #publishing uid with customized interface
        self.srv = self.create_service(Multiply, 'multiply', self.multiply_callback)            #service server
        self.action_server = ActionServer(self, AddDigits, 'add_digits', self.execute_callback) #action server
        self.declare_parameter('my_uid', '2020741046')                                                    #parameter
        
        timer_period = 0.5  #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0  #counter used in the callback
    
    def timer_callback(self):   #publish uid with parameter

        my_param = self.get_parameter('my_uid').get_parameter_value().string_value      #set parameters
        
        my_new_param = rclpy.parameter.Parameter(
            'my_uid',
            rclpy.Parameter.Type.STRING,
            '2020741046'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

        msg = Uid()     #publish uid
        msg.uid = my_param + ': %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.uid)
        self.i += 1

    def multiply_callback(self, request, response): #handle multiply
        response.product = request.a * request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response
    
    def execute_callback(self, goal_handle):    #add digits in uid
        self.get_logger().info('Executing goal...')

        feedback_msg = AddDigits.Feedback()
        uid = goal_handle.request.uid
        feedback_msg.partial_sequence = [int(digit) for digit in uid]

        tmp_sum = 0

        for digit in feedback_msg.partial_sequence:
            tmp_sum += digit
            feedback_msg.partial_sequence[0] = tmp_sum  #save added results in first element of partial_sequence(to show both recieved uid and added result)
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = AddDigits.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
        

    

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher_service_action = MinimalPublisherServiceAction()

    rclpy.spin(minimal_publisher_service_action)

    minimal_publisher_service_action.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()