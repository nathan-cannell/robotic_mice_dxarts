# 
# @file: id_srv.py
# @author: Ethan Widger
# @breif: A service to give out mouse name spaces.
# @version: 0.1
# @date: 2024-05-02
#
from custom_msgs.srv import Id 

import rclpy
from rclpy.node import Node
from mouse_global.mouse_utils import get_mouse_namespace

# The ID assigning class grabs the mouse name spaces and gives out a new one with a higher ID number
class ID_Assigner(Node):

    def __init__(self):
        super().__init__('mouse_global')
        self.srv = self.create_service(Id, 'id_assign', self.id_request_callback)

    # Callback which sends off the response message
    def id_request_callback(self, request, response):
        response.req_id = request.req_id

        ids = [int(id.split('_', 1)[1]) for id in get_mouse_namespace(node=self)]
        if (len(ids) != 0):
            for id in range(1, max(ids, default=1) + 1):
                # Look for the first missing id in the series
                if not id in ids:
                    response.id = id # Set the first missing response
                    break
                response.id = id + 1 # Otherwise exit the for loop with max + 1
        else:
            response.id = 1 # Initate mouse 1

        self.get_logger().info('Request: %d -> Id: %d' % (request.req_id, response.id))

        return response

def main(args=None):
    rclpy.init(args=args)

    id_assign = ID_Assigner()
    id_assign.get_logger().info('Mouse id assign service started.')

    rclpy.spin(id_assign)

    rclpy.shutdown()

if __name__ == '__main__':
    main()