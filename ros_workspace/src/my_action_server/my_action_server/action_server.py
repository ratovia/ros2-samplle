import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from my_action_server_interface import action

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node is started')

        self.action_server = ActionServer(
            self,
            action.Notice,
            '/Notice',
            execute_callback=self.execute_callback
        )

        self.get_logger().info('ActionServer is started')

    def execute_callback(self, goal_handle):
        order_id = goal_handle.request.order_id

        self.get_logger().info('execute_callback is called, order_id: %d' % order_id)
        # order_idに基づいた処理を実行する
        # ...

        result = action.Notice.Result()
        result.success = True
        result.result_message = "success"
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # シャットダウン時にアクションサーバを破棄する
    node.action_server.destroy()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
