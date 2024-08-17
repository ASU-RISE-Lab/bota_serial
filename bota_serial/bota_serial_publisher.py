import rclpy

from rclpy.node import Node

from builtin_interfaces.msg import Time
from geometry_msgs.msg import WrenchStamped

from collections import namedtuple
import serial, time, struct
from crc import Calculator, Configuration

class BotaSerialPublisher(Node):

    BOTA_PRODUCT_CODE = 123456
    # Leave this baud rate alone, it is as fast as it can already be @ 460800
    BAUDERATE = 460800
    # Check out user manual for SINC_LENGTH.
    SINC_LENGTH = 512
    # Don't enable CHOP for real-time applications. 
    CHOP_ENABLE = 0
    # Enabling FAST allows for high sampling rate in exchange for 2x-3x the noise.
    FAST_ENABLE = 0
    # I don't think FIR should be disabled.
    FIR_DISABLE = 1
    # Documentation says that temperature drift is negligible.
    TEMP_COMPENSATION = 0 # 0: Disabled (recommended), 1: Enabled
    # IDK, leave this on i guess. User manual says to send sensor back to them if we want to recalibrate.
    USE_CALIBRATION = 1 # 1: calibration matrix active, 0: raw measurements
    DATA_FORMAT = 0 # 0: binary, 1: CSV
    BAUDERATE_CONFIG = 4 # 0: 9600, 1: 57600, 2: 115200, 3: 230400, 4: 460800
    FRAME_HEADER = b'\xAA'
    # Note that the time step is set according to the sinc filter size!
    # I guess this doesn't do anything?
    # time_step = 0.01;

    def __init__(self):
        super().__init__('bota_serial_publisher')
        self.publisher_ = self.create_publisher(WrenchStamped, 'bota_serial/wrench_stamped', 10)
        # timer_period = 0.01  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        # FT sensor setup
        self._port = "/dev/ttyUSB0" # TODO: Run or launch argument?
        self._ser = serial.Serial()
        # self._pd_thread_stop_event = threading.Event()
        DeviceSet = namedtuple('DeviceSet', 'name product_code config_func')
        self._expected_device_layout = {0: DeviceSet('BFT-SENS-SER-M8', self.BOTA_PRODUCT_CODE, self.bota_sensor_setup)}
        self._status = None
        self._fx = 0.0
        self._fy = 0.0
        self._fz = 0.0
        self._mx = 0.0
        self._my = 0.0
        self._mz = 0.0
        self._timestamp = 0.0
        self._old_timestamp = 0.0
        self._temperature = 0.0

    def bota_sensor_setup(self):
        self._ser.baudrate = self.BAUDERATE
        self._ser.port = self._port
        self._ser.timeout = 10
        self.get_logger().info("Trying to open serial port %s" % self._port)
        try:
            self._ser.open()
            self.get_logger().info("Opened serial port %s" % self._port)
        except:
            self.get_logger().error('Could not open port (1)')

        if not self._ser.is_open:
            self.get_logger().error('Could not open port (2)')

        self.get_logger().info("Trying to setup the sensor.")
        # Wait for streaming of data
        out = self._ser.read_until(bytes('App Init', 'ascii'))
        if not self.contains_bytes(bytes('App Init', 'ascii'), out):
            self.get_logger().error("Sensor not streaming, check if correct port selected!")
            return False
        time.sleep(0.5)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

        # Go to CONFIG mode
        cmd = bytes('C', 'ascii')
        self._ser.write(cmd)
        out = self._ser.read_until(bytes('r,0,C,0', 'ascii'))
        if not self.contains_bytes(bytes('r,0,C,0', 'ascii'), out):
            self.get_logger().error("Failed to go to CONFIG mode.")
            return False

        # Communication setup
        comm_setup = f"c,{self.TEMP_COMPENSATION},{self.USE_CALIBRATION},{self.DATA_FORMAT},{self.BAUDERATE_CONFIG}"
        #print(comm_setup)
        cmd = bytes(comm_setup, 'ascii')
        self._ser.write(cmd)
        out = self._ser.read_until(bytes('r,0,c,0', 'ascii'))
        if not self.contains_bytes(bytes('r,0,c,0', 'ascii'), out):
            self.get_logger().error("Failed to set communication setup.")
            return False
        self.time_step = 0.00001953125*self.SINC_LENGTH
        self.get_logger().info("Timestep: %5.3f" % self.time_step)

        # Filter setup
        filter_setup = f"f,{self.SINC_LENGTH},{self.CHOP_ENABLE},{self.FAST_ENABLE},{self.FIR_DISABLE}"
        #print(filter_setup)
        cmd = bytes(filter_setup, 'ascii')
        self._ser.write(cmd)
        out = self._ser.read_until(bytes('r,0,f,0', 'ascii'))
        if not self.contains_bytes(bytes('r,0,f,0', 'ascii'), out):
            self.get_logger().error("Failed to set filter setup.")
            return False

        # Go to RUN mode
        cmd = bytes('R', 'ascii')
        self._ser.write(cmd)
        out = self._ser.read_until(bytes('r,0,R,0', 'ascii'))
        if not self.contains_bytes(bytes('r,0,R,0', 'ascii'), out):
            self.get_logger().error("Failed to go to RUN mode.")
            return False

        return True

    def contains_bytes(self, subsequence, sequence):
        return subsequence in sequence
    
    def run(self):
        # while not self._pd_thread_stop_event.is_set():
        frame_synced = False
        crc16X25Configuration = Configuration(16, 0x1021, 0xFFFF, 0xFFFF, True, True)
        crc_calculator = Calculator(crc16X25Configuration)

        # while not frame_synced and not self._pd_thread_stop_event.is_set():
        while not frame_synced:
            possible_header = self._ser.read(1)
            if self.FRAME_HEADER == possible_header:
                #print(possible_header)
                data_frame = self._ser.read(34)
                crc16_ccitt_frame = self._ser.read(2)

                crc16_ccitt = struct.unpack_from('H', crc16_ccitt_frame, 0)[0]
                checksum = crc_calculator.checksum(data_frame)
                if checksum == crc16_ccitt:
                    self.get_logger().info("Frame synced, publishing data")
                    frame_synced = True
                else:
                    self._ser.read(1)

        # while frame_synced and not self._pd_thread_stop_event.is_set():
        while frame_synced:
            start_time = time.perf_counter()
            frame_header = self._ser.read(1)

            if frame_header != self.FRAME_HEADER:
                self.get_logger().warn("Lost sync")
                frame_synced = False
                break

            data_frame = self._ser.read(34)
            crc16_ccitt_frame = self._ser.read(2)

            crc16_ccitt = struct.unpack_from('H', crc16_ccitt_frame, 0)[0]
            checksum = crc_calculator.checksum(data_frame)
            if checksum != crc16_ccitt:
                self.get_logger().warn("CRC mismatch received")
                break

            self._status = struct.unpack_from('H', data_frame, 0)[0]

            self._fx = struct.unpack_from('f', data_frame, 2)[0]
            self._fy = struct.unpack_from('f', data_frame, 6)[0]
            self._fz = struct.unpack_from('f', data_frame, 10)[0]
            self._mx = struct.unpack_from('f', data_frame, 14)[0]
            self._my = struct.unpack_from('f', data_frame, 18)[0]
            self._mz = struct.unpack_from('f', data_frame, 22)[0]

            self._timestamp = struct.unpack_from('I', data_frame, 26)[0]

            self._temperature = struct.unpack_from('f', data_frame, 30)[0]

            wrench = WrenchStamped()
            wrench.header.stamp = self.get_clock().now().to_msg()
            wrench.wrench.force.x = self._fx
            wrench.wrench.force.y = self._fy
            wrench.wrench.force.z = self._fz
            wrench.wrench.torque.x = self._mx
            wrench.wrench.torque.y = self._my
            wrench.wrench.torque.z = self._mz
            self.publisher_.publish(wrench)
            
            # time_diff = time.perf_counter() - start_time


def main(args=None):
    rclpy.init(args=args)

    bota_serial_publisher = BotaSerialPublisher()
    bota_serial_publisher.bota_sensor_setup()
    bota_serial_publisher.run()

    rclpy.spin(bota_serial_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bota_serial_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()