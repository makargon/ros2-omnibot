from smbus2 import SMBus 
import spidev

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
        self._bus.write_byte_data(self._address, reg, value & 0xFF)

    def set_pwm_freq(self, pwm_hz: int) -> None:
        hz = max(24, min(1526, int(pwm_hz)))
        prescale = int(round(25_000_000.0 / (4096.0 * float(hz)) - 1.0))

        old_mode = self._bus.read_byte_data(self._address, self.MODE1)
        sleep_mode = (old_mode & 0x7F) | 0x10
        self._write_reg(self.MODE1, sleep_mode)
        self._write_reg(self.PRESCALE, prescale)
        self._write_reg(self.MODE1, old_mode)
        self._write_reg(self.MODE1, old_mode | 0xA1)

    def set_channel_duty(self, channel: int, duty: float) -> None:
        if not self._enabled or channel < 0 or channel > 15:
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
        self._bus.close()