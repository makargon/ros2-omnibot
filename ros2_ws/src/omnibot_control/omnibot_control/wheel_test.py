import time
import errno
from dataclasses import dataclass
from typing import Dict, List

try:
    from .gpio_compat import resolve_gpiochip
except Exception:
    # Allows direct execution: python3 wheel_test.py
    from gpio_compat import resolve_gpiochip  # type: ignore

try:
    import gpiod  # type: ignore
except Exception:
    gpiod = None

try:
    from smbus2 import SMBus  # type: ignore
except Exception:
    SMBus = None


@dataclass
class Motor:
    name: str
    pwm_channel: int
    in1_gpio: int
    in2_gpio: int


MOTORS: List[Motor] = [
    Motor('motor1', 0, 5, 6),
    Motor('motor2', 1, 17, 22),
    Motor('motor3', 2, 26, 27),
]

OE_GPIO = 16


class PCA9685:
    MODE1 = 0x00
    MODE2 = 0x01
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, bus_num: int = 1, address: int = 0x40, pwm_hz: int = 1000):
        if SMBus is None:
            raise RuntimeError('smbus2 is not available')
        self.bus = SMBus(bus_num)
        self.address = address
        self.bus.write_byte_data(self.address, self.MODE1, 0x00)
        self.bus.write_byte_data(self.address, self.MODE2, 0x04)
        self.set_pwm_freq(pwm_hz)

    def set_pwm_freq(self, pwm_hz: int) -> None:
        hz = max(24, min(1526, int(pwm_hz)))
        prescale = int(round(25_000_000.0 / (4096.0 * float(hz)) - 1.0))
        old_mode = self.bus.read_byte_data(self.address, self.MODE1)
        sleep_mode = (old_mode & 0x7F) | 0x10
        self.bus.write_byte_data(self.address, self.MODE1, sleep_mode)
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode | 0xA1)

    def set_duty(self, channel: int, duty: float) -> None:
        d = max(0.0, min(1.0, duty))
        base = self.LED0_ON_L + 4 * channel
        if d <= 0.0:
            self.bus.write_byte_data(self.address, base + 0, 0x00)
            self.bus.write_byte_data(self.address, base + 1, 0x00)
            self.bus.write_byte_data(self.address, base + 2, 0x00)
            self.bus.write_byte_data(self.address, base + 3, 0x10)
            return
        off = int(d * 4095.0)
        self.bus.write_byte_data(self.address, base + 0, 0x00)
        self.bus.write_byte_data(self.address, base + 1, 0x00)
        self.bus.write_byte_data(self.address, base + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, base + 3, (off >> 8) & 0x0F)

    def close(self) -> None:
        self.bus.close()


def main() -> None:
    if gpiod is None:
        raise RuntimeError('gpiod is not available in this Python environment')

    pca = PCA9685(bus_num=1, address=0x40, pwm_hz=1000)

    chip_name = resolve_gpiochip()
    print(f'Using GPIO chip: {chip_name}')
    chip = gpiod.Chip(chip_name)
    lines: Dict[int, object] = {}

    def request(pin: int):
        if pin in lines:
            return
        line = chip.get_line(pin)
        try:
            line.request(consumer='wheel_test', type=gpiod.LINE_REQ_DIR_OUT)
        except OSError as e:
            if e.errno == errno.EBUSY:
                raise RuntimeError(
                    f'GPIO{pin} is busy. Stop motor_controller first (it holds the same GPIO lines), then run wheel_test.'
                ) from e
            raise
        line.set_value(0)
        lines[pin] = line

    def write(pin: int, value: int):
        if pin in lines:
            lines[pin].set_value(1 if value else 0)

    def read(pin: int):
        if pin not in lines:
            return None
        try:
            return int(lines[pin].get_value())
        except Exception:
            return None

    def gpio_self_check() -> None:
        print('\nGPIO self-check (toggle 0 -> 1 -> 0):')
        pins = []
        for m in MOTORS:
            pins.append((f'{m.name}.in1', m.in1_gpio))
            pins.append((f'{m.name}.in2', m.in2_gpio))
        pins.append(('pca_oe', OE_GPIO))

        for label, pin in pins:
            write(pin, 0)
            r0 = read(pin)
            time.sleep(0.03)

            write(pin, 1)
            r1 = read(pin)
            time.sleep(0.03)

            write(pin, 0)
            r2 = read(pin)

            if r0 is None or r1 is None or r2 is None:
                print(f'  [WARN] GPIO{pin:>2} ({label}): write OK, readback unavailable on this kernel/driver')
                continue

            ok = (r0 == 0 and r1 == 1 and r2 == 0)
            status = 'OK' if ok else 'MISMATCH'
            print(f'  [{status}] GPIO{pin:>2} ({label}): readback {r0}->{r1}->{r2}')

    try:
        # Request IN lines (mandatory)
        for m in MOTORS:
            request(m.in1_gpio)
            request(m.in2_gpio)

        # Enable PCA9685 outputs (OE is active-low)
        request(OE_GPIO)
        write(OE_GPIO, 0)

        gpio_self_check()

        # PWM lines are optional because PWM0/1/2 may not be gpiochip offsets.
        def stop_all():
            for m in MOTORS:
                write(m.in1_gpio, 0)
                write(m.in2_gpio, 0)
                pca.set_duty(m.pwm_channel, 0.0)

        stop_all()
        print('Start wheel test: each wheel forward 2s, stop 1s, reverse 2s, stop 1s')

        for m in MOTORS:
            print(f'\n=== {m.name} ===')

            print('forward')
            write(m.in1_gpio, 1)
            write(m.in2_gpio, 0)
            pca.set_duty(m.pwm_channel, 0.6)
            time.sleep(2.0)

            stop_all()
            time.sleep(1.0)

            print('reverse')
            write(m.in1_gpio, 0)
            write(m.in2_gpio, 1)
            pca.set_duty(m.pwm_channel, 0.6)
            time.sleep(2.0)

            stop_all()
            time.sleep(1.0)

        print('\nWheel test completed')

    finally:
        for m in MOTORS:
            try:
                pca.set_duty(m.pwm_channel, 0.0)
            except Exception:
                pass
        pca.close()
        for line in lines.values():
            try:
                line.set_value(0)
                line.release()
            except Exception:
                pass
        chip.close()


if __name__ == '__main__':
    main()
