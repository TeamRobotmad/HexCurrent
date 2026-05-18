"""HexCurrent Hexpansion App."""
# To install on a badge:
# Create a directory named "TeamRobotmad_hexCurrent" on the badge's internal storage under apps
# Copy the app.mpy file to this directory

import asyncio
import time

try:
    from micropython import const
except ImportError:
    # CPython / simulator fallback – const() is an identity function on MicroPython
    const = lambda x: x  # noqa: E731

import ota
import settings as platform_settings
import vfs
from app_components import Menu, button_labels, clear_background, label_font_size
from app_components.notification import Notification
from events.input import BUTTON_TYPES, Buttons
from machine import I2C
from system.eventbus import eventbus
from system.hexpansion.config import HexpansionConfig
from system.hexpansion.events import HexpansionMountedEvent, HexpansionRemovalEvent
from system.hexpansion.util import (
    detect_eeprom_addr,
    get_hexpansion_block_devices,
    read_hexpansion_header,
)
from system.scheduler.events import (
    RequestForegroundPopEvent,
    RequestForegroundPushEvent,
    RequestStopAppEvent,
)

import app

PRE= "hexcurrent"

SLOTS = const(6)
FILE = "current.csv"
_FILE_DEST_LABELS = ("Badge FS", "Hex FS")

_DEFAULT_CAPTURE_SECONDS = const(30)
_MIN_CAPTURE_SECONDS = const(5)
_MAX_CAPTURE_SECONDS = const(600)

STATE_MENU = const(0)
STATE_MESSAGE = const(1)
STATE_SETTINGS = const(2)
STATE_MONITOR = const(3)

MINIMISE_VALID_STATES = [STATE_MENU, STATE_MESSAGE]

MAIN_MENU_ITEMS = ["Monitor", "Settings", "About", "Exit"]
MENU_ITEM_MONITOR = const(0)
MENU_ITEM_SETTINGS = const(1)
MENU_ITEM_ABOUT = const(2)
MENU_ITEM_EXIT = const(3)

DEFAULT_BACKGROUND_UPDATE_PERIOD = const(100)
_LOGGING = True

_AUTO_REPEAT_MS = const(200)
_AUTO_REPEAT_COUNT_THRES = const(10)
_AUTO_REPEAT_SPEED_LEVEL_MAX = const(4)
_AUTO_REPEAT_LEVEL_MAX = const(3)

# avoid whiteas that is used for button labels (and we have no control over this)
_CURRENT_COLOUR = (0.0, 1.0, 0.5)
_VOLTAGE_COLOUR = (0.2, 0.7, 1.0)
_TITLE_COLOUR = (1.0, 1.0, 0.0)
_TEXT_COLOUR = (1.0, 0.5, 1.0)
_RATE_COLOUR = (1.0, 0.5, 0.5)


def _setting_key(name: str) -> str:
    return f"{PRE}.{name}"


def _format_voltage_mv(voltage_mv: int | None) -> str:
    if voltage_mv is None:
        return "--"
    sign = "-" if voltage_mv < 0 else ""
    absolute = abs(int(voltage_mv))
    whole = absolute // 1000
    fraction = (absolute % 1000) // 10
    return f"{sign}{whole}.{fraction:02d}V"


class MySetting:
    """Single persisted setting with optional labels for display."""

    def __init__(self, container, default, minimum, maximum, labels=None):
        self._container = container
        self.d = default
        self.v = default
        self._min = minimum
        self._max = maximum
        self._labels = labels

    def _index(self):
        for key, value in self._container.items():
            if value == self:
                return key
        return None

    def label(self, index=None):
        if index is None:
            index = self.v
        if self._labels is not None and 0 <= int(index) < len(self._labels):
            return self._labels[int(index)]
        return str(index)

    def inc(self, value, level=0):
        if isinstance(self.v, bool):
            return not value
        if isinstance(self.v, int):
            if level == 0:
                value += 1
            else:
                step = 10 ** level
                value = ((value // step) + 1) * step
            if value > self._max:
                if self._labels is not None:
                    value = self._min
                else:
                    value = self._max
        return value

    def dec(self, value, level=0):
        if isinstance(self.v, bool):
            return not value
        if isinstance(self.v, int):
            if level == 0:
                value -= 1
            else:
                step = 10 ** level
                value = (((value + (9 * (10 ** (level - 1)))) // step) - 1) * step
            if value < self._min:
                if self._labels is not None:
                    value = self._max
                else:
                    value = self._min
        return value

    def persist(self):
        index = self._index()
        if index is None:
            return
        key = _setting_key(index)
        try:
            platform_settings.set(key, self.v if self.v != self.d else None)
        except Exception as exc:      # pylint: disable=broad-exception-caught
            print(f"HC:Failed to persist setting {key}: {exc}")


class SensorBase:
    """Abstract base class for I2C-backed sensors used by HexCurrent."""

    I2C_ADDR = 0x00
    NAME = "Unknown"
    READ_INTERVAL_MS = 250

    def __init__(self, i2c_addr=None, logging=False):
        self._i2c = None
        self._ready = False
        self._i2c_addr = self.I2C_ADDR if i2c_addr is None else i2c_addr
        self._logging = logging

    def begin(self, i2c) -> bool:
        self._i2c = i2c
        self._ready = False
        try:
            self._ready = self._init()
        except Exception as exc:      # pylint: disable=broad-exception-caught
            print(f"S:{self.NAME} begin error: {exc}")
            self._ready = False
        return self._ready

    def reset(self):
        try:
            self._shutdown()
        except Exception as exc:      # pylint: disable=broad-exception-caught
            print(f"S:{self.NAME} reset error: {exc}")
        self._ready = False

    @property
    def i2c_addr(self):
        return self._i2c_addr

    def _read_reg(self, reg, length=1):
        if self._i2c is None:
            raise RuntimeError("I2C not initialised")
        return self._i2c.readfrom_mem(self._i2c_addr, reg, length)

    def _write_reg(self, reg, data):
        if self._i2c is None:
            raise RuntimeError("I2C not initialised")
        self._i2c.writeto_mem(self._i2c_addr, reg, data)

    def _read_u16_be(self, reg):
        data = self._read_reg(reg, 2)
        return (data[0] << 8) | data[1]

    def _read_s16_be(self, reg):
        value = self._read_u16_be(reg)
        if value & 0x8000:
            value -= 0x10000
        return value

    def _write_u16_be(self, reg, value):
        self._write_reg(reg, bytes([(value >> 8) & 0xFF, value & 0xFF]))

    def read_sample_if_ready(self) -> dict | None:
        return None

    def _init(self):
        raise NotImplementedError

    def _shutdown(self):
        return


_REG_CONFIGURATION = const(0x00)
_REG_BUS_VOLTAGE = const(0x02)
_REG_CURRENT = const(0x04)
_REG_CALIBRATION = const(0x05)
_REG_MASK_ENABLE = const(0x06)
_REG_MANUFACTURER_ID = const(0xFE)

_CFG_AVG_SHIFT = const(9)
_CFG_VBUSCT_SHIFT = const(6)
_CFG_VSHCT_SHIFT = const(3)
_CFG_MODE_SHIFT = const(0)

_CFG_AVG_16 = const(0b010)
_CFG_CT_1100US = const(0b100)
_CFG_CT_8244US = const(0b111)
_CFG_MODE_POWER_DOWN = const(0b000)
_CFG_MODE_SHUNT_BUS_CONT = const(0b111)

_MASK_CNVR = const(0x0400)
_MASK_CVRF = const(0x0008)
_MASK_OVF = const(0x0004)
_MASK_LEN = const(0x0001)

_MANUFACTURER_ID_TI = const(0x5449)
_CALIBRATION_VALUE = const(0x0200)
_CURRENT_LSB_UA = const(100)

_DEFAULT_CONFIGURATION = (
    (_CFG_AVG_16 << _CFG_AVG_SHIFT)
    | (_CFG_CT_1100US << _CFG_VBUSCT_SHIFT)
    | (_CFG_CT_8244US << _CFG_VSHCT_SHIFT)
    | (_CFG_MODE_SHUNT_BUS_CONT << _CFG_MODE_SHIFT)
)


class INA226(SensorBase):
    """INA226 current and voltage sensor driver."""

    I2C_ADDR = 0x40
    I2C_ADDRS = tuple(range(0x40, 0x50))
    NAME = "INA226"
    READ_INTERVAL_MS = 150

    def _init(self):
        if self._read_u16_be(_REG_MANUFACTURER_ID) != _MANUFACTURER_ID_TI:
            return False
        self._write_u16_be(_REG_CONFIGURATION, _DEFAULT_CONFIGURATION)
        self._write_u16_be(_REG_CALIBRATION, _CALIBRATION_VALUE)
        self._write_u16_be(_REG_MASK_ENABLE, _MASK_CNVR | _MASK_LEN)
        return True

    def _shutdown(self):
        self._write_u16_be(_REG_CONFIGURATION, _CFG_MODE_POWER_DOWN)

    def read_sample_if_ready(self):
        if not self._ready:
            return None
        status = self._read_u16_be(_REG_MASK_ENABLE)
        if (status & _MASK_CVRF) == 0:
            return None
        if (status & _MASK_OVF) != 0:
            print(f"S:{self.NAME} math overflow (status=0x{status:04X})")
            return None
        bus_raw = self._read_u16_be(_REG_BUS_VOLTAGE)
        current_raw = self._read_s16_be(_REG_CURRENT)
        return {
            "mV": (bus_raw * 125) // 100,
            "mA": (current_raw * _CURRENT_LSB_UA) // 1000,
        }


ALL_SENSOR_CLASSES = [INA226]


class SensorManager:
    """Detect and manage the INA226 on a hexpansion port."""

    def __init__(self, logging=False):
        self._logging = logging
        self._i2c = None
        self._port = None
        self._sensors = []
        self._read_interval_ms = 250

    @property
    def read_interval(self):
        return self._read_interval_ms

    def open(self, port):
        self.close()
        self._port = port
        try:
            self._i2c = I2C(port)
            found_addrs = set(self._i2c.scan())
        except Exception as exc:      # pylint: disable=broad-exception-caught
            if self._logging:
                print(f"HC:Cannot open I2C port {port}: {exc}")
            self.close()
            return False

        if self._logging:
            print(f"HC:Port {port} scan: {[hex(addr) for addr in found_addrs]}")

        for cls in ALL_SENSOR_CLASSES:
            for address in getattr(cls, "I2C_ADDRS", (cls.I2C_ADDR,)):
                if address not in found_addrs:
                    continue
                sensor = cls(i2c_addr=address, logging=self._logging)
                if sensor.begin(self._i2c):
                    self._sensors.append(sensor)
                    self._read_interval_ms = getattr(sensor, "READ_INTERVAL_MS", 250)
                    if self._logging:
                        print(f"HC:Found {cls.NAME} @ 0x{address:02X} on port {port}")
                    return True
        self.close()
        return False

    def close(self):
        for sensor in self._sensors:
            try:
                sensor.reset()
            except Exception:      # pylint: disable=broad-exception-caught
                pass
        self._sensors = []
        self._i2c = None
        self._port = None
        self._read_interval_ms = 250

    def get_sensor_by_name(self, name):
        for sensor in self._sensors:
            if sensor.NAME == name:
                return sensor
        return None


class HexCurrentApp(app.App):         # pylint: disable=no-member
    """Monitor current and voltage from a HexCurrent INA226 sensor."""

    VERSION = "0.1"

    def __init__(self, config: HexpansionConfig | None = None):
        super().__init__()

        try:
            if self._parse_version(ota.get_version()) < [1, 9, 0]:
                raise RuntimeError("HexCurrent requires BadgeOS upgrade")
        except Exception as exc:      # pylint: disable=broad-exception-caught
            print(f"HC:Version check failed {exc}")

        self.config: HexpansionConfig | None = config
        self._logging = True
        self._foreground = False

        self.button_states = Buttons(self)
        self.current_state = STATE_MENU
        self.previous_state = self.current_state
        self.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD

        self.refresh = True
        self.notification = None
        self.message = []
        self.message_colours = []
        self.message_type = None
        self.message_return_state = None
        self.current_menu = None
        self.menu = None

        self.settings = {}
        self.edit_setting = None
        self.edit_setting_value = None
        self._auto_repeat_intervals = [
            _AUTO_REPEAT_MS,
            _AUTO_REPEAT_MS // 2,
            _AUTO_REPEAT_MS // 4,
            _AUTO_REPEAT_MS // 8,
            _AUTO_REPEAT_MS // 16,
        ]
        self._auto_repeat = 0
        self._auto_repeat_count = 0
        self.auto_repeat_level = 0

        self._sensor_mgr = None
        self._ina226 = None
        self._monitor_port = getattr(config, "port", None)
        self._reading = {}

        self._show_chart = False
        self._capture_mode = False
        self._capture_done = False
        self._unsaved_data = False
        self._capture_elapsed_ms = 0
        self._capture_data = []
        self._last_current_ma = 0
        self._last_voltage_mv = 0
        self._max_current_ma = 0
        self._max_voltage_mv = 0

        self.settings["logging"] = MySetting(self.settings, _LOGGING, False, True)
        self.settings["rate_hz"] = MySetting(self.settings, 4, 1, 20)
        self.settings["duration_s"] = MySetting(
            self.settings,
            _DEFAULT_CAPTURE_SECONDS,
            _MIN_CAPTURE_SECONDS,
            _MAX_CAPTURE_SECONDS,
        )
        if self.config is None:
            self.settings["path"] = MySetting(
                self.settings,
                0,
                0,
                len(_FILE_DEST_LABELS) - 1,
                labels=_FILE_DEST_LABELS,
            )
        self.update_settings()

        location = f"port {self._monitor_port}" if self._monitor_port is not None else "badge install"
        print(f"HC:HexCurrent App V{self.VERSION} on {location}")

        eventbus.on_async(RequestForegroundPushEvent, self._gain_focus, self)
        eventbus.on_async(RequestForegroundPopEvent, self._lose_focus, self)
        eventbus.on_async(RequestStopAppEvent, self._handle_stop_app, self)
        eventbus.on_async(HexpansionMountedEvent, self._handle_mounted, self)
        eventbus.on_async(HexpansionRemovalEvent, self._handle_removal, self)

        if self.config is None:
            # running from badge rather than hexpansion EEPROM
            # We start with focus on launch, without an event emmited
            # This version is compatible with the simulator
            asyncio.get_event_loop().create_task(self._gain_focus(RequestForegroundPushEvent(self)))
            self._foreground = True

        self.show_message(
            ["HexCurrent", f"V{self.VERSION}", "INA226 Monitor", "By RobotMad"],
            [(0.2, 1.0, 0.2), _TITLE_COLOUR, _TEXT_COLOUR, _TEXT_COLOUR],
            return_state=STATE_MENU,
        )

    @property
    def logging(self):
        return self.settings["logging"].v if "logging" in self.settings else True

    @property
    def capture_seconds(self):
        return int(self.settings["duration_s"].v)

    def update_settings(self):
        for name, setting in self.settings.items():
            setting.v = platform_settings.get(_setting_key(name), setting.d)

    def set_logging(self, state):
        self._logging = state

    def auto_repeat_check(self, delta, speed_up=True):
        self._auto_repeat += delta
        level = self.auto_repeat_level if speed_up else 0
        if self._auto_repeat > self._auto_repeat_intervals[level]:
            self._auto_repeat = 0
            self._auto_repeat_count += 1
            threshold = (_AUTO_REPEAT_COUNT_THRES * _AUTO_REPEAT_MS) // self._auto_repeat_intervals[level]
            if self._auto_repeat_count > threshold:
                self._auto_repeat_count = 0
                max_level = _AUTO_REPEAT_SPEED_LEVEL_MAX if speed_up else _AUTO_REPEAT_LEVEL_MAX
                if self.auto_repeat_level < max_level:
                    self.auto_repeat_level += 1
            return True
        return False

    def auto_repeat_clear(self):
        self._auto_repeat = 1 + self._auto_repeat_intervals[0]
        self._auto_repeat_count = 0
        self.auto_repeat_level = 0

    async def background_task(self):
        last_time = time.ticks_ms()
        while True:
            now = time.ticks_ms()
            delta = time.ticks_diff(now, last_time)
            self._background_update(delta)
            await asyncio.sleep_ms(max(1, self.update_period - time.ticks_diff(time.ticks_ms(), now)))
            last_time = now

    def _background_update(self, _delta):
        if self._capture_mode and not self._capture_done:
            self._capture_elapsed_ms = self._capture_elapsed_ms + _delta
        if self._ina226 is None:
            if self._capture_mode and not self._capture_done and self._capture_elapsed_ms >= self.capture_seconds * 1000:
                self._finish_capture()
            return
        self._sample_sensor_in_background()

    def _candidate_ports(self):
        # if a port is specified in the config then put this first in the list
        # of ports to check before the others, otherwise check all ports in order
        if self.config is not None and getattr(self.config, "port", None) is not None:
            return [self.config.port] + [p for p in range(1, SLOTS + 1) if p != self.config.port]
        return list(range(1, SLOTS + 1))

    def _connect_monitor(self):
        if self._ina226 is not None:
            return True

        self._disconnect_monitor(clear_capture=False)

        for port in self._candidate_ports():
            mgr = SensorManager(logging=self._logging)
            if not mgr.open(port):
                continue
            sensor = mgr.get_sensor_by_name("INA226")
            if sensor is None:
                mgr.close()
                continue

            self._sensor_mgr = mgr
            self._ina226 = sensor
            self._monitor_port = port
            self.update_period = 1000 // self.settings["rate_hz"].v if "rate_hz" in self.settings else sensor.read_interval
            sample = sensor.read_sample_if_ready()
            if sample is not None:
                self._apply_reading(sample)
            if self._logging:
                print(f"HC:Using HexCurrent on port {port}")
            return True

        self._sensor_mgr = None
        self._ina226 = None
        self.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD
        return False

    def _disconnect_monitor(self, clear_capture=True):
        if self._sensor_mgr is not None:
            try:
                self._sensor_mgr.close()
            except Exception as exc:      # pylint: disable=broad-exception-caught
                if self._logging:
                    print(f"HC:Sensor manager close failed: {exc}")
        self._sensor_mgr = None
        self._ina226 = None
        self._reading = {}
        self.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD
        if clear_capture:
            self._clear_capture()

    def _apply_reading(self, sample):
        current_ma = int(sample.get("mA", 0))
        voltage_mv = int(sample.get("mV", 0))
        self._reading = {
            "mA": current_ma,
            "mV": voltage_mv,
        }
        self._last_current_ma = current_ma
        self._last_voltage_mv = voltage_mv

    def _sample_sensor_in_background(self):
        sample = self._ina226.read_sample_if_ready() if self._ina226 is not None else None
        capture_complete = self._capture_mode and not self._capture_done and self._capture_elapsed_ms >= self.capture_seconds * 1000
        if sample is not None:
            self._apply_reading(sample)
            self.refresh = True
            if self._capture_mode and not self._capture_done:
                self._record_sample(self._capture_elapsed_ms, sample)
                if capture_complete:
                    self._finish_capture()
        elif capture_complete:
            self._finish_capture()

    def _record_sample(self, elapsed_ms, sample):
        current_ma = int(sample.get("mA", 0))
        voltage_mv = int(sample.get("mV", 0))
        self._capture_data.append((elapsed_ms, current_ma, voltage_mv))
        self._max_current_ma = max(self._max_current_ma, abs(current_ma))
        self._max_voltage_mv = max(self._max_voltage_mv, abs(voltage_mv))
        if self._unsaved_data:
            self._unsaved_data = True

    def _start_capture(self):
        if self._ina226 is None and not self._connect_monitor():
            self.show_message(["HexCurrent", "not found"], [_TITLE_COLOUR, _TEXT_COLOUR], msg_type="warning")
            return False
        print("HC:Starting auto capture")
        self._capture_mode = True
        self._capture_done = False
        self._capture_elapsed_ms = 0
        self._capture_data = []
        self._unsaved_data = False
        self._max_current_ma = abs(int(self._reading.get("mA", 0)))
        self._max_voltage_mv = abs(int(self._reading.get("mV", 0)))
        if self._reading:
            self._record_sample(0, self._reading)
        self.refresh = True
        return True

    def _finish_capture(self):
        if not self._capture_mode or self._capture_done:
            return
        if not self._capture_data and self._reading:
            self._record_sample(self._capture_elapsed_ms, self._reading)
        print("HC:Finishing data capture")
        self._capture_done = True
        self._save_capture_data_csv()
        self.refresh = True

    def _clear_capture(self):
        print("HC:clearing capture data")
        self._show_chart = False
        self._capture_mode = False
        self._capture_done = False
        self._capture_elapsed_ms = 0
        self._capture_data = []
        self._max_current_ma = 0
        self._max_voltage_mv = 0
        self.refresh = True


    def _enter_background_mode(self):
        # if not already capturing then start now
        print("HC:Entering background mode")
        # minimise app to run in background while capturing
        self.minimise()


    def _start_monitor_mode(self):
        if not self._connect_monitor():
            self.show_message(
                ["HexCurrent", "not found", "Insert board"],
                [_TITLE_COLOUR, _TEXT_COLOUR, _TEXT_COLOUR],
                msg_type="warning",
                return_state=STATE_MENU,
            )
            return False
        print("HC:Starting monitor mode")
        self.set_menu(None)
        self.current_state = STATE_MONITOR
        self.refresh = True
        return True

    def _leave_monitor_mode(self):
        print("HC:Leaving monitor mode")
        if self._capture_mode and not self._capture_done:
            self._finish_capture()
        self._disconnect_monitor(clear_capture=True)
        self.return_to_menu()

    def _data_save_path_option(self):
        try:
            return int(self.settings["path"].v)
        except Exception:      # pylint: disable=broad-exception-caught
            return 0

    def _mount_current_fs(self):
        if self.config is None or getattr(self.config, "port", None) is None:
            print("HC:Hex fs save unavailable when running from badge")
            return None, False
        mountpoint = "/hexcurrent"
        eeprom_addr, addr_len = detect_eeprom_addr(self.config.i2c)
        if eeprom_addr is None or addr_len is None:
            print("HC:No EEPROM found on HexCurrent port")
            return None, False
        header = read_hexpansion_header(self.config.i2c, eeprom_addr=eeprom_addr, addr_len=addr_len)
        if header is None:
            print("HC:Failed to read HexCurrent EEPROM header")
            return None, False
        try:
            _, partition = get_hexpansion_block_devices(self.config.i2c, header, eeprom_addr, addr_len=addr_len)
        except RuntimeError as exc:
            print(f"HC:Failed to get block device: {exc}")
            return None, False

        mounted_here = True
        try:
            vfs.mount(partition, mountpoint, readonly=False)
        except OSError as exc:
            if exc.args and exc.args[0] == 1:
                mounted_here = False
            else:
                print(f"HC:Failed to mount {mountpoint}: {exc}")
                return None, False
        except Exception as exc:      # pylint: disable=broad-exception-caught
            print(f"HC:Failed to mount {mountpoint}: {exc}")
            return None, False
        return mountpoint, mounted_here

    def _data_save_path(self):
        if self._data_save_path_option() == 1:
            mountpoint, mounted_here = self._mount_current_fs()
            if mountpoint is None:
                return None, None, False
            return f"{mountpoint}/{FILE}", mountpoint, mounted_here
        return f"/{FILE}", None, False

    def _save_capture_data_csv(self):
        if not self._capture_data and not self._unsaved_data:
            return False

        output_path, mountpoint, mounted_here = self._data_save_path()
        if output_path is None:
            self.notification = Notification(" Save Failed ")
            return False

        try:
            with open(output_path, "wb") as csv_file:
                csv_file.write(b"ms,mA,mV\n")
                for elapsed_ms, current_ma, voltage_mv in self._capture_data:
                    row = f"{elapsed_ms},{current_ma},{voltage_mv}\n"
                    csv_file.write(row.encode())
        except Exception as exc:      # pylint: disable=broad-exception-caught
            print(f"HC:Failed to save CSV {output_path}: {exc}")
            self.notification = Notification(" Save Failed ")
            return False
        finally:
            if mounted_here and mountpoint is not None:
                try:
                    vfs.umount(mountpoint)
                except Exception as exc:      # pylint: disable=broad-exception-caught
                    print(f"HC:Failed to unmount {mountpoint}: {exc}")
        self._unsaved_data = False
        print(f"HC:Saved CSV to {output_path}")
        self.notification = Notification("  CSV Saved  ")
        return True

    def update(self, delta):
        if not self._foreground and not self._capture_mode:
            # for hexpansion launch we wait for the event to set foreground, for badge launch we start in foreground without an event
            eventbus.emit(RequestForegroundPushEvent(self))
            self._foreground = True

        if self.notification:
            self.notification.update(delta)
            try:
                if self.notification._is_closed():  # pylint: disable=protected-access
                    self.notification = None
            except Exception as exc:      # pylint: disable=broad-exception-caught
                if self.logging:
                    print(f"HC:Notification status error: {exc}")

        if self.update_period >= DEFAULT_BACKGROUND_UPDATE_PERIOD:
            self.refresh = True

        if self.current_state == STATE_MENU:
            if self.current_menu is None:
                self.set_menu()
                self.refresh = True
            else:
                menu = self.menu
                if menu is None:
                    self.set_menu()
                    self.refresh = True
                    return
                menu.update(delta)
                if menu.is_animating != "none":
                    self.refresh = True
        elif self.button_states.get(BUTTON_TYPES["CANCEL"]) and self.current_state in MINIMISE_VALID_STATES:
            self.button_states.clear()
            self.minimise()
        elif self.current_state == STATE_MESSAGE:
            self._update_state_message()
        elif self.current_state == STATE_SETTINGS:
            self._settings_mgr_update(delta)
        elif self.current_state == STATE_MONITOR:
            self._monitor_update()

        if self.current_state != self.previous_state:
            self.previous_state = self.current_state
            self.refresh = True

    def _update_state_message(self):
        if not self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            return
        self.button_states.clear()
        if self.message_return_state is not None:
            self.current_state = self.message_return_state
        else:
            self.set_menu()
            self.current_state = STATE_MENU
        self.message = []
        self.message_colours = []
        self.message_type = None
        self.message_return_state = None
        self.refresh = True

    def _monitor_update(self):
        if not self._foreground and self._capture_mode:
            print("HC:Didn't expect to be called while in background")

        if self.button_states.get(BUTTON_TYPES["CANCEL"]):
            self.button_states.clear()
            self._leave_monitor_mode()
            return

        if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            self.button_states.clear()
            if not self._capture_mode:
                self._start_capture()
                self._show_chart = True
            elif self._show_chart:
                self._show_chart = False
            else:
                self._show_chart = True
            self.refresh = True
            return

        if self.button_states.get(BUTTON_TYPES["RIGHT"]):
            self.button_states.clear()
            if not self._capture_mode:
                self._start_capture()
            elif not self._capture_done:
                self._finish_capture()
            else:
                self._clear_capture()
            self.refresh = True
            return

        if self.button_states.get(BUTTON_TYPES["LEFT"]):
            self.button_states.clear()
            if not self._capture_mode:
                self._start_capture()
            if not self._capture_done:
                self._enter_background_mode()


    def draw(self, ctx):
        if self.current_state == STATE_MENU and self.menu is not None:
            clear_background(ctx)
            self.menu.draw(ctx)
        elif self.refresh or self.notification:
            self.refresh = False
            clear_background(ctx)
            ctx.font_size = label_font_size
            if ctx.text_align != ctx.LEFT:
                ctx.text_align = ctx.LEFT
            ctx.text_baseline = ctx.BOTTOM

            if self.current_state == STATE_MESSAGE:
                if not self.message_colours:
                    self.message_colours = [(1, 0, 0)] * len(self.message)
                self.draw_message(ctx, self.message, self.message_colours, label_font_size)
                button_labels(ctx, confirm_label="OK", cancel_label="Exit")
            elif self.current_state == STATE_SETTINGS:
                self.settings_mgr_draw(ctx)
            elif self.current_state == STATE_MONITOR:
                self._monitor_draw(ctx)

        if self.notification:
            self.notification.draw(ctx)

    def _monitor_draw(self, ctx):
        if self._show_chart:
            self._draw_chart_monitor(ctx)
        else:
            self._draw_manual_monitor(ctx)

    def _draw_manual_monitor(self, ctx):
        ctx.font_size = label_font_size
        if self._capture_mode and not self._capture_done:
            progress = min(100, (self._capture_elapsed_ms * 100) // max(self.capture_seconds * 1000, 1))
            ctx.rgb(*_TEXT_COLOUR).move_to(-45, 85).text(f"Rec {progress}%")

        current_ma = self._reading.get("mA")
        voltage_mv = self._reading.get("mV")
        port_label = self._monitor_port if self._monitor_port is not None else "--"
        lines = [
            "HexCurrent",
            f"Port:{port_label}",
            f"I:{current_ma if current_ma is not None else '--'}mA",
            f"V:{_format_voltage_mv(voltage_mv)}",
            f"Rate:{1000//self.update_period}.{((1000 % self.update_period) // 10):02d}Hz",
        ]
        colours = [_TITLE_COLOUR, _TEXT_COLOUR, _CURRENT_COLOUR, _VOLTAGE_COLOUR, _RATE_COLOUR]
        self.draw_message(ctx, lines, colours, label_font_size)

        if self._capture_done:
            right_label = "Clear"
            left_label = None
        else:
            left_label = "Background"
            if self._capture_mode:
                right_label = "Stop"
            else:
                right_label = "Record"
        button_labels(ctx, cancel_label="Back", confirm_label="Chart", right_label=right_label, left_label=left_label)


    def _draw_chart_monitor(self, ctx):
        chart_left = -95
        chart_right = 95
        chart_top = -45
        chart_bottom = 55
        chart_w = chart_right - chart_left
        chart_h = chart_bottom - chart_top

        ctx.rgb(0.05, 0.05, 0.05).rectangle(chart_left - 5, chart_top - 5, chart_w + 10, chart_h + 10).fill()
        ctx.rgb(0.4, 0.4, 0.4)
        ctx.move_to(chart_left, chart_bottom).line_to(chart_right, chart_bottom).stroke()
        ctx.move_to(chart_left, chart_bottom).line_to(chart_left, chart_top).stroke()

        duration_ms = max(self.capture_seconds * 1000, 1)
        if self._capture_data:
            duration_ms = max(duration_ms, self._capture_data[-1][0], 1)

        self._plot_capture_series(ctx, chart_left, chart_bottom, chart_w, chart_h, duration_ms, self._max_current_ma, 1, _CURRENT_COLOUR)
        self._plot_capture_series(ctx, chart_left, chart_bottom, chart_w, chart_h, duration_ms, self._max_voltage_mv, 2, _VOLTAGE_COLOUR)

        ctx.font_size = label_font_size
        if self._capture_done:
            ctx.rgb(*_TITLE_COLOUR).move_to(-30, chart_top - 45).text("Chart")
        else:
            progress = min(100, (self._capture_elapsed_ms * 100) // max(self.capture_seconds * 1000, 1))
            ctx.rgb(*_TEXT_COLOUR).move_to(-45, chart_top - 40).text(f"Rec {progress}%")

        ctx.font_size = label_font_size - 8
        ctx.rgb(*_TEXT_COLOUR).move_to(-15, chart_top - 5).text("Max")
        ctx.rgb(*_CURRENT_COLOUR).move_to(chart_left + 10, chart_top - 5).text(f"I:{self._max_current_ma}mA")
        ctx.rgb(*_VOLTAGE_COLOUR).move_to(32, chart_top - 5).text(f"V:{_format_voltage_mv(self._max_voltage_mv)}")

        if self._capture_done:
            right_label = "Clear"
            left_label = None
            ctx.rgb(*_TEXT_COLOUR).move_to(chart_left + 30, chart_bottom + 45).text("Key:")
            ctx.rgb(*_CURRENT_COLOUR).move_to(-30, chart_bottom + 45).text("Current")
            ctx.rgb(*_VOLTAGE_COLOUR).move_to(-30, chart_bottom + 63).text("Voltage")
        else:
            right_label = "Stop"
            left_label = "Background"
            ctx.rgb(*_CURRENT_COLOUR).move_to(chart_left + 65, chart_bottom + 40).text(f"{self._last_current_ma}mA")
            ctx.rgb(*_VOLTAGE_COLOUR).move_to(chart_left + 65, chart_bottom + 60).text(_format_voltage_mv(self._last_voltage_mv))
        button_labels(ctx, cancel_label="Back", confirm_label="Data", right_label=right_label, left_label=left_label)


    def _plot_capture_series(self, ctx, chart_left, chart_bottom, chart_w, chart_h, duration_ms, max_value, value_index, colour):
        if len(self._capture_data) == 0 or max_value <= 0 or duration_ms <= 0:
            return

        previous = None
        ctx.rgb(*colour)
        for elapsed_ms, current_ma, voltage_mv in self._capture_data:
            value = current_ma if value_index == 1 else voltage_mv
            scaled = abs(value) if value_index == 1 else max(value, 0)
            x = chart_left + (elapsed_ms * chart_w) // duration_ms
            y = chart_bottom - (scaled * chart_h) // max_value
            if previous is None:
                ctx.rectangle(x, y, 2, 2).fill()
            else:
                ctx.move_to(previous[0], previous[1]).line_to(x, y).stroke()
            previous = (x, y)

    @staticmethod
    def draw_message(ctx, message, colours, size=label_font_size):
        ctx.font_size = size
        num_lines = len(message)
        for index, line in enumerate(message):
            text_line = str(line)
            width = ctx.text_width(text_line)
            colour = colours[index] if index < len(colours) else _TEXT_COLOUR
            if num_lines == 1:
                y_position = int(0.35 * ctx.font_size)
            else:
                y_position = int((index - ((num_lines - 2) / 2)) * ctx.font_size - 2)
            ctx.rgb(*colour).move_to(-width // 2, y_position).text(text_line)

    def show_message(self, msg_content, msg_colours, msg_type=None, return_state=None):
        self.message = msg_content
        self.message_colours = msg_colours
        self.message_type = msg_type
        self.message_return_state = return_state
        self.current_state = STATE_MESSAGE
        self.refresh = True

    def return_to_menu(self, menu_name="main"):
        if menu_name is not None:
            self.set_menu(menu_name)
        self.current_state = STATE_MENU
        self.refresh = True

    def set_menu(self, menu_name: str | None ="main"):
        if self.menu is not None:
            try:
                self.menu._cleanup()        # pylint: disable=protected-access
            except Exception:               # pylint: disable=broad-exception-caught
                pass
        self.current_menu = menu_name
        if menu_name == "main":
            self.menu = Menu(
                self,
                MAIN_MENU_ITEMS.copy(),
                select_handler=self._main_menu_select_handler,
                back_handler=self._menu_back_handler,
            )
        elif menu_name == MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS]:
            items = ["SAVE ALL", "DEFAULT ALL"]
            items.extend(self.settings.keys())
            self.menu = Menu(
                self,
                items,
                select_handler=self._settings_menu_select_handler,
                back_handler=self._menu_back_handler,
            )
        else:
            self.menu = None

    def _main_menu_select_handler(self, item, _idx):
        if item == MAIN_MENU_ITEMS[MENU_ITEM_MONITOR]:
            self.button_states.clear()
            self._start_monitor_mode()
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS]:
            self.set_menu(MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS])
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_ABOUT]:
            self.button_states.clear()
            self.set_menu(None)
            self.show_message(
                ["HexCurrent", f"V{self.VERSION}", "INA226 current", "& voltage monitor", "By RobotMad"],
                [_TITLE_COLOUR, _TEXT_COLOUR, _CURRENT_COLOUR, _CURRENT_COLOUR, _TEXT_COLOUR],
            )
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_EXIT]:
            self._exit_app()

    def _settings_menu_select_handler(self, _item, idx):
        if idx == 0:
            platform_settings.save()
            self.notification = Notification("  Settings  Saved")
            self.set_menu()
        elif idx == 1:
            for setting in self.settings.values():
                setting.v = setting.d
                setting.persist()
            self.notification = Notification("  Settings Defaulted")
            self.set_menu()
        else:
            name = list(self.settings.keys())[idx - 2]
            if self.settings_mgr_start(name):
                self.current_state = STATE_SETTINGS

    def settings_mgr_start(self, item):
        self.set_menu(None)
        self.button_states.clear()
        self.refresh = True
        self.auto_repeat_clear()
        self.edit_setting = item
        self.edit_setting_value = self.settings[item].v
        return True

    def _settings_mgr_update(self, delta):
        if self.button_states.get(BUTTON_TYPES["UP"]):
            if self.auto_repeat_check(delta, speed_up=False):
                self.edit_setting_value = self.settings[self.edit_setting].inc(self.edit_setting_value, self.auto_repeat_level)
                self.refresh = True
        elif self.button_states.get(BUTTON_TYPES["DOWN"]):
            if self.auto_repeat_check(delta, speed_up=False):
                self.edit_setting_value = self.settings[self.edit_setting].dec(self.edit_setting_value, self.auto_repeat_level)
                self.refresh = True
        else:
            self.auto_repeat_clear()
            if self.button_states.get(BUTTON_TYPES["RIGHT"]) or self.button_states.get(BUTTON_TYPES["LEFT"]):
                self.button_states.clear()
                self.edit_setting_value = self.settings[self.edit_setting].d
                self.refresh = True
                self.notification = Notification("Default")
            elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
                self.button_states.clear()
                self.return_to_menu(MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS])
            elif self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.button_states.clear()
                self.settings[self.edit_setting].v = self.edit_setting_value
                self.settings[self.edit_setting].persist()
                self.notification = Notification(f"  {self.edit_setting} set")
                self.return_to_menu(MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS])

    def settings_mgr_draw(self, ctx):
        display_value = self.settings[self.edit_setting].label(self.edit_setting_value)
        self.draw_message(
            ctx,
            ["Edit Setting", f"{self.edit_setting}:", f"{display_value}"],
            [_TITLE_COLOUR, _TEXT_COLOUR, _CURRENT_COLOUR],
            label_font_size,
        )
        button_labels(ctx, up_label="+", down_label="-", confirm_label="Set", cancel_label="Cancel", right_label="Default")

    def _menu_back_handler(self):
        if self.current_menu == "main":
            self.minimise()
        self.set_menu()

    def _parse_version(self, version):
        if "+" in version:
            version, _build = version.split("+", 1)
        if "-" in version:
            version, _pre = version.split("-", 1)
        return [int(item) if item.isdigit() else item for item in version.strip("v").split(".")]

    def deinitialise(self):
        eventbus.remove(HexpansionMountedEvent, self._handle_mounted, self)
        eventbus.remove(HexpansionRemovalEvent, self._handle_removal, self)
        eventbus.remove(RequestForegroundPushEvent, self._gain_focus, self)
        eventbus.remove(RequestForegroundPopEvent, self._lose_focus, self)
        self._disconnect_monitor(clear_capture=True)
        return True

    def _exit_app(self):
        eventbus.emit(RequestStopAppEvent(self))

    async def _handle_removal(self, event):
        if event.port != self._monitor_port:
            return
        self.notification = Notification("HexCurrent Removed")
        self._disconnect_monitor(clear_capture=True)
        if self.current_state == STATE_MONITOR:
            self.return_to_menu()

    async def _handle_mounted(self, event):
        if self.current_state != STATE_MONITOR or self._ina226 is not None:
            return
        if self.config is not None and getattr(self.config, "port", None) != event.port:
            return
        if self._connect_monitor():
            self.notification = Notification(f"HexCurrent {event.port}")

    async def _gain_focus(self, event):
        if event.app is self:
            print("HC:Gained focus")
            self._foreground = True

    async def _lose_focus(self, event):
        if event.app is self:
            print("HC:Lost focus")
            self._foreground = False

    async def _handle_stop_app(self, event):
        try:
            if event.app is self:
                print("HC:Stopping app")
                self.deinitialise()
        except (AttributeError, TypeError):
            pass


__app_export__ = HexCurrentApp
