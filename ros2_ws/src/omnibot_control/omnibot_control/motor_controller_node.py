import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

try:
    import gpiod  # type: ignore
except Exception:  # pragma: no cover
    gpiod = None

try:
    import spidev  # type: ignore
except Exception:  # pragma: no cover
    spidev = None

try:
    from smbus2 import SMBus  # type: ignore
except Exception:  # pragma: no cover
    SMBus = None


@dataclass
class MotorConfig:
    pwm_channel: int
    in1_gpio: int
    in2_gpio: int
    enc_a_ch: int
    enc_b_ch: int
    direction_sign: int


class PCA9685:
    MODE1 = 0x00
    MODE2 = 0x01
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, bus_num: int, address: int = 0x40, pwm_hz: int = 1000):
        self._enabled = False
        self._bus = None
        self._address = address
        self._last_error = ''

        if SMBus is None:
            self._last_error = 'smbus2 not installed'
            return

        try:
            self._bus = SMBus(bus_num)
            # Restart + auto-increment
            self._write_reg(self.MODE1, 0x00)
            # Totem-pole output
            self._write_reg(self.MODE2, 0x04)
            self.set_pwm_freq(pwm_hz)
            # Disable all channels initially
            for ch in range(16):
                self.set_channel_duty(ch, 0.0)
            self._enabled = True
        except Exception as e:
            self._last_error = str(e)
            self._enabled = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def last_error(self) -> str:
        return self._last_error

    def _write_reg(self, reg: int, value: int) -> None:
        if self._bus is None:
            return
        self._bus.write_byte_data(self._address, reg, value & 0xFF)

    def set_pwm_freq(self, pwm_hz: int) -> None:
        if self._bus is None:
            return
        hz = max(24, min(1526, int(pwm_hz)))
        prescale = int(round(25_000_000.0 / (4096.0 * float(hz)) - 1.0))

        old_mode = self._bus.read_byte_data(self._address, self.MODE1)
        sleep_mode = (old_mode & 0x7F) | 0x10
        self._write_reg(self.MODE1, sleep_mode)
        self._write_reg(self.PRESCALE, prescale)
        self._write_reg(self.MODE1, old_mode)
        self._write_reg(self.MODE1, old_mode | 0xA1)

    def set_channel_duty(self, channel: int, duty: float) -> None:
        if not self._enabled or self._bus is None or channel < 0 or channel > 15:
            return

        base = self.LED0_ON_L + 4 * channel
        d = max(0.0, min(1.0, duty))
        if d <= 0.0:
            # FULL OFF
            self._write_reg(base + 0, 0x00)
            self._write_reg(base + 1, 0x00)
            self._write_reg(base + 2, 0x00)
            self._write_reg(base + 3, 0x10)
            return
        if d >= 1.0:
            # FULL ON
            self._write_reg(base + 0, 0x00)
            self._write_reg(base + 1, 0x10)
            self._write_reg(base + 2, 0x00)
            self._write_reg(base + 3, 0x00)
            return

        off = int(d * 4095.0)
        self._write_reg(base + 0, 0x00)
        self._write_reg(base + 1, 0x00)
        self._write_reg(base + 2, off & 0xFF)
        self._write_reg(base + 3, (off >> 8) & 0x0F)

    def close(self) -> None:
        if self._bus is not None:
            self._bus.close()


class MCP3008:
    def __init__(self, bus: int, device: int, max_speed_hz: int = 1_000_000):
        self._enabled = False
        self._spi = None
        if spidev is None:
            return
        try:
            self._spi = spidev.SpiDev()
            self._spi.open(bus, device)
            self._spi.max_speed_hz = max_speed_hz
            self._enabled = True
        except Exception:
            self._enabled = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    def read_channel(self, channel: int) -> int:
        if not self._enabled or self._spi is None or channel < 0 or channel > 7:
            return 0
        raw = self._spi.xfer2([1, (8 + channel) << 4, 0])
        return ((raw[1] & 3) << 8) + raw[2]

    def close(self) -> None:
        if self._spi is not None:
            self._spi.close()


class GPIOController:
    """Wrapper around libgpiod for direction and OE pins."""

    def __init__(self, motors: List[MotorConfig], oe_gpio: int = -1) -> None:
        self._chip = None
        self._lines: Dict[int, any] = {}
        self._enabled = False
        self._last_error = ''
        self._oe_gpio = oe_gpio

        if gpiod is None:
            return

        try:
            self._chip = gpiod.Chip("gpiochip0")
            # Configure only direction pins (and optional OE) as GPIO outputs.
            all_pins = set()
            for m in motors:
                all_pins.add(m.in1_gpio)
                all_pins.add(m.in2_gpio)
            if oe_gpio >= 0:
                all_pins.add(oe_gpio)

            # Request lines for digital outputs
            for pin in all_pins:
                try:
                    line = self._chip.get_line(pin)
                    line.request(consumer="omnibot", type=gpiod.LINE_REQ_DIR_OUT)
                    line.set_value(0)
                    self._lines[pin] = line
                except Exception as e:
                    self._last_error = f'line {pin}: {e}'

            self._enabled = len(self._lines) > 0

        except Exception as e:
            self._last_error = str(e)
            self._enabled = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def last_error(self) -> str:
        return self._last_error

    def set_output(self, pin: int, value: int) -> None:
        """Set a GPIO line to 0 or 1."""
        if not self._enabled or pin not in self._lines:
            return
        try:
            self._lines[pin].set_value(1 if value else 0)
        except Exception:
            pass

    def set_oe_enabled(self, enabled: bool) -> None:
        # OE is active-low on PCA9685 breakout boards.
        if self._oe_gpio < 0:
            return
        self.set_output(self._oe_gpio, 0 if enabled else 1)

    def close(self) -> None:
        """Release all GPIO lines."""
        for line in self._lines.values():
            try:
                line.release()
            except Exception:
                pass
        self._lines.clear()
        if self._chip is not None:
            try:
                self._chip.close()
            except Exception:
                pass


class MotorControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('motor_controller')

        self.declare_parameter('wheel_radius_m', 0.05)
        self.declare_parameter('wheel_base_m', 0.11)
        self.declare_parameter('max_wheel_rad_s', 25.0)
        self.declare_parameter('cmd_frame_rotation_deg', 0.0)
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('encoder_rate_hz', 200.0)
        self.declare_parameter('encoder_threshold', 512)
        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('pca9685.i2c_bus', 1)
        self.declare_parameter('pca9685.address', 0x40)
        self.declare_parameter('pca9685.pwm_hz', 1000)
        self.declare_parameter('pca9685.oe_gpio', 16)

        # Requested mapping from user.
        self.declare_parameter('motor1.pwm_channel', 0)
        self.declare_parameter('motor1.in1_gpio', 5)
        self.declare_parameter('motor1.in2_gpio', 6)
        self.declare_parameter('motor1.enc_a_ch', 0)
        self.declare_parameter('motor1.enc_b_ch', 1)
        self.declare_parameter('motor1.direction_sign', 1)

        self.declare_parameter('motor2.pwm_channel', 1)
        self.declare_parameter('motor2.in1_gpio', 17)
        self.declare_parameter('motor2.in2_gpio', 22)
        self.declare_parameter('motor2.enc_a_ch', 3)
        self.declare_parameter('motor2.enc_b_ch', 4)
        self.declare_parameter('motor2.direction_sign', 1)

        self.declare_parameter('motor3.pwm_channel', 2)
        self.declare_parameter('motor3.in1_gpio', 26)
        self.declare_parameter('motor3.in2_gpio', 27)
        self.declare_parameter('motor3.enc_a_ch', 5)
        self.declare_parameter('motor3.enc_b_ch', 6)
        self.declare_parameter('motor3.direction_sign', -1)

        self.wheel_radius = float(self.get_parameter('wheel_radius_m').value)
        self.wheel_base = float(self.get_parameter('wheel_base_m').value)
        self.max_wheel_rad_s = float(self.get_parameter('max_wheel_rad_s').value)
        self.cmd_frame_rotation_deg = float(self.get_parameter('cmd_frame_rotation_deg').value)
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        encoder_rate_hz = float(self.get_parameter('encoder_rate_hz').value)
        self.encoder_threshold = int(self.get_parameter('encoder_threshold').value)

        self._motors = [
            MotorConfig(
                pwm_channel=int(self.get_parameter('motor1.pwm_channel').value),
                in1_gpio=int(self.get_parameter('motor1.in1_gpio').value),
                in2_gpio=int(self.get_parameter('motor1.in2_gpio').value),
                enc_a_ch=int(self.get_parameter('motor1.enc_a_ch').value),
                enc_b_ch=int(self.get_parameter('motor1.enc_b_ch').value),
                direction_sign=int(self.get_parameter('motor1.direction_sign').value),
            ),
            MotorConfig(
                pwm_channel=int(self.get_parameter('motor2.pwm_channel').value),
                in1_gpio=int(self.get_parameter('motor2.in1_gpio').value),
                in2_gpio=int(self.get_parameter('motor2.in2_gpio').value),
                enc_a_ch=int(self.get_parameter('motor2.enc_a_ch').value),
                enc_b_ch=int(self.get_parameter('motor2.enc_b_ch').value),
                direction_sign=int(self.get_parameter('motor2.direction_sign').value),
            ),
            MotorConfig(
                pwm_channel=int(self.get_parameter('motor3.pwm_channel').value),
                in1_gpio=int(self.get_parameter('motor3.in1_gpio').value),
                in2_gpio=int(self.get_parameter('motor3.in2_gpio').value),
                enc_a_ch=int(self.get_parameter('motor3.enc_a_ch').value),
                enc_b_ch=int(self.get_parameter('motor3.enc_b_ch').value),
                direction_sign=int(self.get_parameter('motor3.direction_sign').value),
            ),
        ]

        spi_bus = int(self.get_parameter('spi_bus').value)
        spi_device = int(self.get_parameter('spi_device').value)
        self.mcp3008 = MCP3008(spi_bus, spi_device)

        pca_bus = int(self.get_parameter('pca9685.i2c_bus').value)
        pca_addr = int(self.get_parameter('pca9685.address').value)
        pca_hz = int(self.get_parameter('pca9685.pwm_hz').value)
        pca_oe = int(self.get_parameter('pca9685.oe_gpio').value)
        self.pca = PCA9685(pca_bus, pca_addr, pca_hz)

        # Initialize GPIO via libgpiod
        self.gpio = GPIOController(self._motors, pca_oe)
        if self.gpio.enabled:
            self.gpio.set_oe_enabled(True)

        self.hardware_enabled = self.gpio.enabled and self.pca.enabled

        if self.hardware_enabled:
            self.get_logger().info('GPIO (libgpiod) + PWM (PCA9685) initialized.')
        else:
            gerr = self.gpio.last_error
            perr = self.pca.last_error
            detail = ', '.join([e for e in [gerr, perr] if e])
            if detail:
                self.get_logger().warn(f'Hardware unavailable ({detail}). Running in dry-run mode.')
            else:
                self.get_logger().warn('Hardware unavailable. Running in dry-run mode.')

        if not self.mcp3008.enabled:
            self.get_logger().warn('MCP3008 SPI unavailable. Encoder counts will not update.')

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 20)
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/wheel_encoder_counts', 10)

        self._target_wheels = [0.0, 0.0, 0.0]
        self._last_cmd = Twist()
        self._enc_counts = [0, 0, 0]
        self._enc_state = [0, 0, 0]

        self.control_timer = self.create_timer(1.0 / control_rate_hz, self._control_loop)
        self.encoder_timer = self.create_timer(1.0 / encoder_rate_hz, self._encoder_loop)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg
        # Optional command frame rotation to align keyboard frame and robot frame.
        a = math.radians(self.cmd_frame_rotation_deg)
        ca = math.cos(a)
        sa = math.sin(a)
        vx = ca * msg.linear.x - sa * msg.linear.y
        vy = sa * msg.linear.x + ca * msg.linear.y
        self._target_wheels = self._cmd_vel_to_wheels(vx, vy, msg.angular.z)

    def _cmd_vel_to_wheels(self, vx: float, vy: float, wz: float) -> List[float]:
        r = self.wheel_radius
        l = self.wheel_base
        sqrt3 = math.sqrt(3.0)

        w1 = (-0.5 * vx + (sqrt3 / 2.0) * vy + l * wz) / r
        w2 = (-0.5 * vx - (sqrt3 / 2.0) * vy + l * wz) / r
        w3 = (vx + l * wz) / r
        return [w1, w2, w3]

    def _wheel_to_duty(self, wheel_omega: float) -> float:
        if self.max_wheel_rad_s <= 0.0:
            return 0.0
        duty = wheel_omega / self.max_wheel_rad_s
        return max(-1.0, min(1.0, duty))

    def _apply_motor(self, idx: int, duty: float) -> None:
        motor = self._motors[idx]
        if not self.hardware_enabled or self.gpio is None:
            return

        duty = duty * (1.0 if motor.direction_sign >= 0 else -1.0)

        if duty >= 0.0:
            self.gpio.set_output(motor.in1_gpio, 1)
            self.gpio.set_output(motor.in2_gpio, 0)
        else:
            self.gpio.set_output(motor.in1_gpio, 0)
            self.gpio.set_output(motor.in2_gpio, 1)

        self.pca.set_channel_duty(motor.pwm_channel, abs(duty))

    def _control_loop(self) -> None:
        for i, wheel_speed in enumerate(self._target_wheels):
            duty = self._wheel_to_duty(wheel_speed)
            self._apply_motor(i, duty)

    def _encoder_loop(self) -> None:
        if self.mcp3008.enabled:
            for i, motor in enumerate(self._motors):
                a_raw = self.mcp3008.read_channel(motor.enc_a_ch)
                b_raw = self.mcp3008.read_channel(motor.enc_b_ch)
                a = 1 if a_raw >= self.encoder_threshold else 0
                b = 1 if b_raw >= self.encoder_threshold else 0
                new_state = (a << 1) | b

                old_state = self._enc_state[i]
                delta = self._quadrature_delta(old_state, new_state)
                self._enc_counts[i] += delta
                self._enc_state[i] = new_state

        msg = Int32MultiArray()
        msg.data = list(self._enc_counts)
        self.encoder_pub.publish(msg)

    @staticmethod
    def _quadrature_delta(old: int, new: int) -> int:
        table: Dict[Tuple[int, int], int] = {
            (0, 1): 1,
            (1, 3): 1,
            (3, 2): 1,
            (2, 0): 1,
            (1, 0): -1,
            (3, 1): -1,
            (2, 3): -1,
            (0, 2): -1,
        }
        return table.get((old, new), 0)

    def destroy_node(self) -> bool:
        for i in range(3):
            self._apply_motor(i, 0.0)
        if self.gpio is not None:
            self.gpio.set_oe_enabled(False)
            self.gpio.close()
        self.pca.close()
        self.mcp3008.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
