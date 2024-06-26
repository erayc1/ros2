import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymavlink import mavutil

class WarehouseRobotController(Node):
    def __init__(self):
        super().__init__('warehouse_robot_controller')
        self.publisher_ = self.create_publisher(String, 'robot_commands', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Warehouse Robot Controller has started.')
        self.master = mavutil.mavlink_connection('udp:localhost:14550')
        self.master.wait_heartbeat()

    def timer_callback(self):
        msg = String()
        msg.data = 'Move to position (x, y, z)'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.master.mav.command_long_send(
            1, 1, 
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,
            0, 0, 0, 0, 0, 0, 0)

class ControlPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        rclpy.init()
        self.node = WarehouseRobotController()
        
    def initUI(self):
        self.setWindowTitle('Warehouse Robot Control Panel')
        layout = QVBoxLayout()
        
        self.start_button = QPushButton('Start Robot', self)
        self.start_button.clicked.connect(self.start_robot)
        layout.addWidget(self.start_button)
        
        self.stop_button = QPushButton('Stop Robot', self)
        self.stop_button.clicked.connect(self.stop_robot)
        layout.addWidget(self.stop_button)
        
        self.setLayout(layout)
        
    def start_robot(self):
        print('Robot started')
        rclpy.spin(self.node)
        
    def stop_robot(self):
        print('Robot stopped')
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    app = QApplication(sys.argv)
    control_panel = ControlPanel()
    control_panel.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
