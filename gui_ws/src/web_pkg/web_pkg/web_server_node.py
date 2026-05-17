"""
Serves the static web controller UI on http://0.0.0.0:8080.
Open http://<workstation-ip>:8080 from any device on the same network.
"""
import os
import threading
import http.server
import functools

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.declare_parameter('port', 8080)
        port = self.get_parameter('port').value

        static_dir = os.path.join(
            get_package_share_directory('web_pkg'), 'static')

        handler = functools.partial(
            http.server.SimpleHTTPRequestHandler,
            directory=static_dir)

        self._server = http.server.HTTPServer(('0.0.0.0', port), handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

        self.get_logger().info(f'Web controller available at http://0.0.0.0:{port}')

    def destroy_node(self):
        self._server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
