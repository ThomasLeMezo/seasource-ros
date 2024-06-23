import rclpy
from rclpy.node import Node
from digi.xbee.devices import *
import socket

from seabot2_safety.msg import SafetyStatus
from pressure_bme280_driver.msg import Bme280Data
from seabot2_power_driver.msg import PowerState
from gpsd_client.msg import GpsFix
from seabot2_lambert.msg import GnssPose
from seabot2_depth_filter.msg import DepthPose
from seabot2_mission.msg import MissionState


def serialize_data(data, val, nb_bit, start_bit, value_min=None, value_max=None, flag_debug=False):
    if (flag_debug):
        print("------")
        print("val init =", val)
    if (value_min != None and value_max != None):
        scale = ((1 << nb_bit) - 1) / (value_max - value_min)
        val = int(round((val - value_min) * scale))
    else:
        value_min = 0.0
        scale = 1.0
    mask = ((1 << nb_bit) - 1) << start_bit
    data = data | (mask & (val << start_bit))

    if (flag_debug):
        print("val_min =", value_min)
        print("scale = ", scale)
        print("val = ", val)
    return data, nb_bit + start_bit, val / scale + value_min

def deserialize_data(data, nb_bit, start_bit, value_min=None, value_max=None):
    mask = ((1 << nb_bit) - 1) << start_bit
    v = (data & mask) >> start_bit
    if (value_min != None and value_max != None):
        scale = ((1 << nb_bit) - 1) / (value_max - value_min)
        v = v / scale + value_min
    return v, start_bit + nb_bit


class XbeeNode(Node):

    def __init__(self):
        super().__init__('xbee_node')

        # Interfaces
        self.subscription_mission = None
        self.subscription_depth = None
        self.subscription_gnss_pose = None
        self.subscription_gnss_data = None
        self.subscription_power_data = None
        self.subscription_internal_sensor_filter = None
        self.subscription_safety_data = None

        # Get hostname of device
        self.hostname = socket.gethostname()
        # limit size
        self.hostname = self.hostname[:20] if len(self.hostname) >= 21 else self.hostname

        # Test if hostname starts with "seabot"
        self.hostname_is_seabot = True
        if not self.hostname.startswith("seabot"):
            self.hostname_is_seabot = False

        # Parameters
        self.xbee_network_id = 0x42
        self.xbee_encryption_key = "ABCDEFGHIFKLMNOP"
        self.xbee_node_id = self.hostname
        self.time_between_communication = 5
        self.serial_baudrate = 9600

        self.serial_port = "/dev/ttyMAX0" if self.hostname_is_seabot else "/dev/ttyUSB0"

        # Initialization
        self.init_interfaces()
        self.init_parameters()

        self.timer = self.create_timer(self.time_between_communication, self.timer_callback)

        # Connect to xbee
        # https://xbplib.readthedocs.io/en/latest/index.html
        self.xbee = XBeeDevice(self.serial_port, self.serial_baudrate)
        self.xbee.open()

        self.configure_xbee()

        self.safety_global_safety_valid = False
        self.safety_published_frequency = False
        self.safety_depth_limit = False
        self.safety_batteries_limit = False
        self.safety_depressurization = False
        self.safety_seafloor = False
        self.safety_piston = False
        self.safety_zero_depth = False

        self.internal_pressure = 0.0
        self.internal_temperature = 0.0
        self.internal_humidity = 0.0

        self.battery = 0.0

        self.valid_fix = False
        self.fix_latitude = 0.0
        self.fix_longitude = 0.0

        self.gnss_heading = 0.0
        self.gnss_speed = 0.0
        self.gnss_mean_east = 0.0
        self.gnss_mean_north = 0.0

        self.depth = 0.0

        self.current_waypoint = 0
        self.mission_mode = 0

        self.CMD_MSG_TYPE = {"LOG_STATE": 0, "CMD_SLEEP": 1, "CMD_PARAMETERS": 2, "CMD_MISSION_NEW": 3,
                             "CMD_MISSION_KEEP": 4}
        self.last_cmd_received = 0

    def __del__(self):
        self.xbee.close()

    def configure_xbee(self):
        self.xbee.read_device_info()
        self.xbee.set_node_id(self.xbee_node_id)

        # Set network id
        print("Set network ID")
        self.xbee.set_parameter('ID', self.xbee_network_id.to_bytes(2, 'big'))

        # Set encryption key
        print("Set encryption key")
        self.xbee.set_parameter('KY', bytearray(self.xbee_encryption_key, 'utf-8'))

        # Set encryption enable
        print("Enable Encryption")
        self.xbee.set_parameter('EE', b'\x01')

        # Apply changes.
        print("Apply changes & write")
        self.xbee.apply_changes()
        # Write changes (to flash).
        self.xbee.write_changes()

        self.xbee.add_data_received_callback(self.data_received_callback)

    def data_received_callback(self, xbee_message):
        # ToDo : process the callback
        address = xbee_message.remote_device.get_64bit_addr()
        data = xbee_message.data.decode("utf8")
        print("Received data from %s: %s" % (address, data))

    def init_parameters(self):
        self.declare_parameter('serial_port', self.serial_port)
        self.declare_parameter('serial_baudrate', self.serial_baudrate)
        self.declare_parameter('xbee_node_id', self.xbee_node_id)
        self.declare_parameter('time_between_communication', self.time_between_communication)  # in seconds
        self.declare_parameter('xbee_encryption_key', self.xbee_encryption_key)  # 16 bytes
        self.declare_parameter('xbee_network_id', self.xbee_network_id)  # between 0x0 and 0x7FFF

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.serial_baudrate = self.get_parameter('serial_baudrate').get_parameter_value().integer_value
        self.time_between_communication = self.get_parameter(
            'time_between_communication').get_parameter_value().integer_value
        self.xbee_node_id = self.get_parameter('xbee_node_id').get_parameter_value().string_value
        self.xbee_encryption_key = self.get_parameter('xbee_encryption_key').get_parameter_value().string_value
        self.xbee_network_id = self.get_parameter('xbee_network_id').get_parameter_value().integer_value

    def init_interfaces(self):
        self.subscription_safety_data = self.create_subscription(SafetyStatus,
                                                                 '/safety/safety', self.safety_callback, 10)
        self.subscription_internal_sensor_filter = self.create_subscription(Bme280Data, '/observer/pressure_internal',
                                                                            self.internal_sensor_callback, 10)
        self.subscription_power_data = self.create_subscription(PowerState, '/observer/power', self.power_callback, 10)
        self.subscription_gnss_data = self.create_subscription(GpsFix, '/driver/fix', self.gpsd_callback, 10)
        self.subscription_gnss_pose = self.create_subscription(GnssPose,
                                                               '/observer/pose_mean', self.gnss_pose_callback, 10)
        self.subscription_depth = self.create_subscription(DepthPose, '/observer/depth', self.depth_callback, 10)
        self.subscription_mission_state = self.create_subscription(MissionState, '/mission/mission_state',
                                                                    self.mission_callback, 10)

    def safety_callback(self, msg):
        self.safety_global_safety_valid = msg.global_safety_valid
        self.safety_published_frequency = msg.published_frequency
        self.safety_depth_limit = msg.depth_limit
        self.safety_batteries_limit = msg.batteries_limit
        self.safety_depressurization = msg.depressurization
        self.safety_seafloor = msg.seafloor
        self.safety_piston = msg.piston
        self.safety_zero_depth = msg.zero_depth

    def internal_sensor_callback(self, msg):
        self.internal_pressure = msg.pressure
        self.internal_temperature = msg.temperature
        self.internal_humidity = msg.humidity

    def gpsd_callback(self, msg):
        self.valid_fix = msg.mode > GpsFix.MODE_NO_FIX
        self.fix_latitude = msg.latitude
        self.fix_longitude = msg.longitude

    def power_callback(self, msg):
        self.battery = msg.battery_volt

    def gnss_pose_callback(self, msg):
        self.gnss_heading = msg.heading
        self.gnss_speed = msg.velocity
        self.gnss_mean_east = msg.east
        self.gnss_mean_north = msg.north

    def depth_callback(self, msg):
        self.depth = msg.depth

    def mission_callback(self, msg):
        self.current_waypoint = msg.waypoint_id
        self.mission_mode = msg.mission_mode

    def timer_callback(self):
        # Send data to xbee
        if self.depth < 0.5 or self.mission_mode == 0:
            self.xbee.send_data_broadcast(self.serialize_log_state()[0])

    def serialize_log_state(self):
        bit_position = 0
        data = 0b0
        message_type = self.CMD_MSG_TYPE["LOG_STATE"]
        state = 0
        state |= (self.safety_global_safety_valid & 0x1) << 0
        state |= (self.safety_published_frequency & 0x1) << 1
        state |= (self.safety_depth_limit & 0x1) << 2
        state |= (self.safety_batteries_limit & 0x1) << 3
        state |= (self.safety_depressurization & 0x1) << 4
        state |= (self.safety_seafloor & 0x1) << 5
        state |= (self.safety_piston & 0x1) << 6
        state |= (self.safety_zero_depth & 0x1) << 7

        data, bit_position, _ = serialize_data(data, message_type, 4, bit_position)
        data, bit_position, _ = serialize_data(data, self.fix_latitude, 25, bit_position, value_min=-90.0,
                                               value_max=90.0, flag_debug=False)
        data, bit_position, _ = serialize_data(data, self.fix_longitude, 25, bit_position, value_min=-180.0,
                                               value_max=180.0, flag_debug=False)
        data, bit_position, _ = serialize_data(data, self.gnss_speed, 8, bit_position, value_min=0, value_max=5.0,
                                               flag_debug=False)
        data, bit_position, _ = serialize_data(data, self.gnss_heading, 8, bit_position, value_min=0, value_max=359.0,
                                               flag_debug=False)

        data, bit_position, _ = serialize_data(data, state, 8, bit_position, flag_debug=False)

        data, bit_position, _ = serialize_data(data, self.battery, 8, bit_position, value_min=12.0, value_max=16.8,
                                               flag_debug=False)
        data, bit_position, _ = serialize_data(data, self.internal_pressure, 6, bit_position, value_min=680.0,
                                               value_max=800.0, flag_debug=False)
        data, bit_position, _ = serialize_data(data, self.internal_temperature, 6, bit_position, value_min=8.0,
                                               value_max=50.0, flag_debug=False)
        data, bit_position, _ = serialize_data(data, self.internal_humidity, 6, bit_position, value_min=50.0,
                                               value_max=100.0, flag_debug=False)

        data, bit_position, _ = serialize_data(data, self.current_waypoint, 8, bit_position, flag_debug=False)
        data, bit_position, _ = serialize_data(data, self.last_cmd_received, 4, bit_position, flag_debug=False)

        return data.to_bytes(int(bit_position / 8), byteorder='little'), message_type


def main(args=None):
    rclpy.init(args=args)

    xbee_node = XbeeNode()
    rclpy.spin(xbee_node)

    xbee_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
