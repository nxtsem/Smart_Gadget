###############################################################################
# Smart_gadget.py    Date: 25-Jul-2022
#
# This example finds and connects to a nRF SOC running the Sensiron temperature
# and humidity BLE service using SHT-31 sensor with Sensiron Firmware Rev 1.3
#
# BLE client is Unexpected Maker Tiny Pico (ESP32-PICO-D4) running 
# MicroPython V1.17 on ESP32 (build esp32-20210902-v1.17.bin)
#
# The sensors' BLE MAC addresses must be supplied by the user
# Sensiron nRF Connect or BLE sniffer apps can find MAC addresses
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# References:
#  - https://sensirion.com/products/catalog/SHT31-Smart-Gadget/
#  - https://github.com/Sensirion/SmartGadget-Firmware
#
# This program is based on code developed by 
# Matthias Prinke for the Xiaomi Mi Flora BLE plant sensors
#   ( https://github.com/matthias-bs/MicroPython-MiFlora  )
# 
###############################################################################

import machine
import bluetooth
import time
import struct
import binascii
# import ble_advertising
from ble_advertising import decode_services, decode_name

from micropython import const

VERBOSITY = 0  #3 = most debug statements, 0 = least

# Bluetooth MAC address of a Sensiron Smart Gadget
Sensiron_mac = [bytes(b"\xDE\x15\xE4\xB1\xD6\x67")]

# If SCAN_DEVICES == True, the demo functions start with scanning for devices and the
# device name and the RSSI are retrieved from the scan results. Otherwise the demo
# functions start with connecting to the devices.
SCAN_DEVICES = True

# If _DISCOVER_SERVICES == True, BLE.gattc_discover_services() is called in
# _IRQ_PERIPHERAL_CONNECT state,
# if DISCOVER_SERVICES == True, BLE.discover_services() is called from application.
_DISCOVER_SERVICES = False
DISCOVER_SERVICES = False   # Emabled this one

# If _DISCOVER_CHARACTERISTICS == True, BLE.gattc_discover_characteristics() is called
# in _IRQ_GATTC_SERVICE_DONE state,
# if  DISCOVER_CHARACTERISTICS == True, BLE.discover_characteristics() is called
# from the application
_DISCOVER_CHARACTERISTICS = False
DISCOVER_CHARACTERISTICS = False  # Enabled this one

# If AUTO_MODE is enabled, the BLE state machine is started with
# scan() or gap_connect() - as needed - and automatically progresses
# through the following stages (AUTO_MODE transitions marked with '*'):
#  1. _IRQ_SCAN_RESULT
#     if completed: -> _IRQ_SCAN_DONE
#  2. _IRQ_SCAN_DONE
#     -> _IRQ_PERIPHERAL_CONNECT
#  3. _IRQ_PERIPHERAL_CONNECT
#     if _DISCOVER_SERVICES: -> _IRQ_GATTC_SERVICE_RESULT
#     else S_READ_FIRMWARE_DONE
#  4. _IRQ_GATTC_SERVICE_RESULT
#     if completed: -> _IRQ_GATTC_SERVICE_DONE
#  5. _IRQ_GATTC_SERVICE_DONE
#     if _DISCOVER_CHARACTERISTICS: -> _IRQ_GATTC_CHARACTERISTIC_RESULT
#  6. _IRQ_GATTC_CHARACTERISTIC_RESULT
#     if completed: -> _IRQ_GATTC_CHARACTERISTIC_DONE
#  7. _IRQ_GATTC_CHARACTERISTIC_DONE
#     -> S_READ_BATTERY_LEVEL_DONE
#  8. S_READ_BATTERY_LEVEL_DONE
#     -> S_READ_TEMP_SENSOR_DONE
#  9. S_READ_TEMP_SENSOR_DONE
#     -> S_READ_HUMID_SENSOR_DONE
# 10. S_READ_HUMID_SENSOR_DONE
#
# The application can start the state machine, perform other tasks, eventually wait
# until the state S_READ_SENSOR_DONE is reached (or a timeout occurred) and finally
# disconnect from the peripheral (.disconnect()).
AUTO_MODE = 0  # Auto Mode not implemented, True = 1

# Time constants (T_WAIT: ms / others: s)
_T_WAIT = const(100)
_T_RETRY = const(10)
_T_CYCLE = const(20)

# Interrupt request IDs (https://docs.micropython.org/en/latest/library/bluetooth.html)
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)
_IRQ_GATTS_READ_REQUEST = const(4)
_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_DONE = const(6)
_IRQ_PERIPHERAL_CONNECT = const(7)
_IRQ_PERIPHERAL_DISCONNECT = const(8)
_IRQ_GATTC_SERVICE_RESULT = const(9)
_IRQ_GATTC_SERVICE_DONE = const(10)
_IRQ_GATTC_CHARACTERISTIC_RESULT = const(11)
_IRQ_GATTC_CHARACTERISTIC_DONE = const(12)
_IRQ_GATTC_DESCRIPTOR_RESULT = const(13)
_IRQ_GATTC_DESCRIPTOR_DONE = const(14)
_IRQ_GATTC_READ_RESULT = const(15)
_IRQ_GATTC_READ_DONE = const(16)
_IRQ_GATTC_WRITE_DONE = const(17)
_IRQ_GATTC_NOTIFY = const(18)
_IRQ_GATTC_INDICATE = const(19)
_IRQ_GATTS_INDICATE_DONE = const(20)
_IRQ_MTU_EXCHANGED = const(21)
_IRQ_L2CAP_ACCEPT = const(22)
_IRQ_L2CAP_CONNECT = const(23)
_IRQ_L2CAP_DISCONNECT = const(24)
_IRQ_L2CAP_RECV = const(25)
_IRQ_L2CAP_SEND_READY = const(26)
_IRQ_CONNECTION_UPDATE = const(27)
_IRQ_ENCRYPTION_UPDATE = const(28)
_IRQ_GET_SECRET = const(29)
_IRQ_SET_SECRET = const(30)

# Address types (cf. https://docs.micropython.org/en/latest/library/bluetooth.html)
ADDR_TYPE_PUBLIC = const(0x00)
ADDR_TYPE_RANDOM = const(0x01)

# Advertising types 
_ADV_IND = const(0x00)          # connectable, scannable, undirectable
_ADV_DIRECT_IND = const(0x01)   # directed to specific scanner, no advertising data
_ADV_SCAN_IND = const(0x02)     # nonconnectable but scannable (up to 31 bytes in advert & scan response)  
_ADV_NONCONN_IND = const(0x03)  # nonconnectable, nonscannable (no response to scan)
_ADV_SCAN_RSP = const(0x04)

# Adverstising data types
_DATA_FLAGS = const(0x01)       # Bit1=Discov mode, Bit2=Classic mode not supported
_DATA_UUID_LIST = const(0x02)   # List of 16-bit Service Class UUIDs
_DATA_SHORT_NAME = const(0x08)  # Shortened Local Name
_DATA_FULL_NAME = const(0x09)   # Complete Local Name

# Sensiron Gadget Service / Characteristics UUIDs
_GENERIC_ACCESS_SERVICE_UUID = bluetooth.UUID(0x1800)  # Generic Access
_GENERIC_ATTRIBUTE_SERVICE_UUID =  bluetooth.UUID(0x1801)  # Generic Attribute
_GADGET_DEVICE_INFO_UUID  =  bluetooth.UUID(0x180A)
_GADGET_BATTERY_SERV_UUID =  bluetooth.UUID(0x180F)
_GADGET_LOG_SERV_UUID     =  bluetooth.UUID("0000F234-B38D-4985-720E-0F993A68EE41")
_GADGET_TEMP_SERV_UUID    =  bluetooth.UUID("00001234-B38D-4985-720E-0F993A68EE41")
_GADGET_TEMP_CHAR_UUID    =  bluetooth.UUID("00001235-B38D-4985-720E-0F993A68EE41")
_GADGET_HUMID_SERV_UUID   =  bluetooth.UUID("00002234-B38D-4985-720E-0F993A68EE41")
_GADGET_HUMID_CHAR_UUID   =  bluetooth.UUID("00002235-B38D-4985-720E-0F993A68EE41")

# Value handles numbers from DISCOVER_CHARACTERISTICS
# Handle definitions by Measurement Standards Laboratory of New Zealand
# ( https://github.com/MSLNZ/rpi-smartgadget/tree/main/smartgadget  )

_HANDLE_DEVICE_NAME = 3    # 0x03 
_HANDLE_APPEARANCE =  5    # 0x05 
_HANDLE_CONN_PARAM =  7    # 0x07 
_HANDLE_UNKNOWN  = 10      # 0x0A 
_HANDLE_SYSTEM_ID = 14     # 0x0e 
_HANDLE_MFG_NAME = 16      # 0x10 
_HANDLE_MODEL_NUM_STR = 18 # 0x12 
_HANDLE_SER_NUM_STR = 20   # 0x14 
_HANDLE_HDW_REV_STR =  22  # 0x16 
_HANDLE_FIRM_REV_STR = 24  # 0x18 
_HANDLE_SOFT_REV_STR = 26  # 0x1A 
_HANDLE_BATT_LEVEL = 29    # 0x1D - 1 byte, 0x64 = 100% dec
_HANLDE_LOG_SERVICE = 31   # 0x1F 
_HANDLE_SYNC_TIME_MS = 33  # 0x21 
_HANDLE_OLD_TSTAMP_MS = 36 # 0x24 
_HANDLE_NEW_TSTAMP_MS = 39 # 0x27 
_HANDLE_START_LOG = 42     # 0x2A 
_HANDLE_LOG_INTRVL_MS = 46 # 0x2E 
_HANDLE_HUMID_SERVICE = 48 # 0X39 
_HANDLE_HUMIDITY =  50     # 0x32 - 4 byte float (Little Endian)
_HANDLE_TEMPERATURE = 55   # 0x37 - 4 byte float (Little Endian)

# States of state machine
S_INIT = const(0)
S_SCAN_DONE = const(1)
S_SERVICE_DONE = const(2)
S_CHARACTERISTIC_DONE = const(3)
S_READ_BATTERY_DONE = const(4)
S_READ_TSENSOR_DONE = const(5)
S_READ_HSENSOR_DONE = const(6)


class Sensiron:
    """
    Sensiron Gadget Bluetooth Low Energy (BLE) Driver

    Attributes:
        _ble (object):               bluetooth.BLE object (see BLE docs)
        state (int):                state of Sensiron state machine
        search_addr (bytes):        BLE MAC address of device to search for
        addr_found (bool):          flag indicating whether device was found
        name (string):              device name
        rssi (int):                 Received Signal Strength Indicator
        version (string):           Sensiron firmware version
        battery (int):              battery status [%]
        temp (float):               temperature [°C]
        humid (float):              humidity [%]
        _addr_type (int):           address type (PUBLIC or RANDOM) (see BLE docs)
        _addr (bytes):              BLE MAC address
        _value (memoryview):        cached data value (payload)
        _scan_callback (func):      callback for event _IRQ_SCAN_DONE
        _conn_callback (func):      callback for event _IRQ_PERIPHERAL_CONNECT
        _serv_done_callback (func): callback for event _IRQ_GATTC_SERVICE_DONE
        _char_done_callback (func): callback for event _IRQ_GATTC_CHARACTERISTIC_DONE
        _read_callback (func):      callback for event _IRQ_GATTC_READ_RESULT
        _write_callback (func):     callback for event _IRQ_GATTC_WRITE_DONE
        _notify_callback (func):    callback for event _IRQ_GATTC_NOTIFY
        _conn_handle (int):         connection handle
        _start_handle (int):        start handle (for characteristic discovery)
        _end_handle (int):          end   handle (for characteristic discovery)
        _value_handle (int):        value handle (for gattc_read()/gattc_write())
    """

    def __init__(self, ble):
        """
        The constructor for Sensiron class.
        Parameters:
            ble ( bluetooth.BLE):   bluetooth.BLE object
        """
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        self._reset()

    def _reset(self):
        # Init public members.
        self.state = S_INIT
        self.search_addr = None
        self.addr_found = False
        self.name = None
        self.rssi = 0
        self.version = None
        self.battery = None
        self.temperature = None
        self.humidity = None

        # Cached name and address from a successful scan.
        self._addr_type = None
        self._addr = None

        # Cached value (if we have one).
        self._value = None

        # Callbacks for completion of various operations.
        # These reset back to None after being invoked.
        self._scan_callback = None
        self._conn_callback = None
        self._serv_done_callback = None
        self._char_done_callback = None
        self._read_callback = None
        self._write_callback = None

        # Persistent callback for when new data is notified from the device.
        self._notify_callback = None

        # Connected device.
        self._conn_handle = None
        self._start_handle = None
        self._end_handle = None
        self._value_handle = None

    def _debug(self, text, debuglevel=0):
        """Debug output.
        Parameters:
            text (str): string to be printed (including formatted variables, if desired)
            debuglevel (int): must be <= VERBOSITY, or text will not be printed
        """
        if debuglevel <= VERBOSITY:
            print(text)

    def _irq(self, event, data):
        """
        Interrupt request handler.   See
        https://docs.micropython.org/en/latest/library/ bluetooth.html for description
        Parameters:
            event (int):  interrupt request ID
            data (tuple): event specific data as tuple
        """
        self._debug("bt_irq - event: {}".format(event), 3)

        if event == _IRQ_SCAN_RESULT:
            self._debug("bt irq - scan result", 2)
            # A single scan result.
            addr_type, addr, adv_type, rssi, adv_data = data
            _addr_type = "Public" if (addr_type == ADDR_TYPE_PUBLIC) else "Random"
            _addr = bytes(addr)
            _addr = binascii.hexlify(_addr)
            if adv_type == _ADV_IND:
                _adv_type = "ADV_IND"
            elif adv_type == _ADV_DIRECT_IND:
                _adv_type = "ADV_DIRECT_IND"
            elif adv_type == _ADV_SCAN_IND:
                _adv_type = "ADV_SCAN_IND"
            elif adv_type == _ADV_NONCONN_IND:
                _adv_type = "ADV_NONCONN_IND"
            else:
                _adv_type = "SCAN_RSP"

            _adv_data = bytes(adv_data)
            _name = decode_name(_adv_data) or "?"
            _services = decode_services(adv_data)
            self._debug(
                "addr_typ: {}; addr: {}; adv_typ: {}; rssi: {} dBm; name: {}; serv: {}"
                .format(_addr_type, _addr, _adv_type, rssi, _name, _services), 1
            )

            if (
                adv_type in (_ADV_IND, _ADV_DIRECT_IND, _ADV_SCAN_RSP)
                and bytes(addr) == self.search_addr
            ):
                # Found a potential device, remember it and stop scanning.
                self._addr_type = addr_type
                self.rssi = rssi
                self.addr_found = True
                self._addr = bytes(
                    addr
                )  # Note: addr buffer is owned by caller so need to copy it.
                if _name != "?":
                    self.name = _name
                self._debug("Device name: {}".format(_name), 1)
                self._ble.gap_scan(None)

        elif event == _IRQ_SCAN_DONE:
            self._debug("bt irq - scan done", 2)
            if self._scan_callback:
                if self._addr:
                    # Found a device during scan (and the scan was explicitly stopped).
                    self._scan_callback(self._addr_type, self._addr, self.name)
                    self._scan_callback = None
                    if AUTO_MODE:
                        self.gap_connect(self._addr_type, self._addr)
                else:
                    # Scan timed out.
                    self._scan_callback(None, None, None)

        elif event == _IRQ_PERIPHERAL_CONNECT:
            # gap_connect() successful.
            self._debug("bt irq - peripheral connect", 2)         
            conn_handle, addr_type, addr = data
            
            self._debug(
                "addr_type: {}; addr: {}; conn_handle: {}"
                .format(addr_type, addr, conn_handle), 0
            )            
            
            if addr_type == self._addr_type and addr == self._addr:
                self._conn_handle = conn_handle
                if self._conn_callback:
                    self._conn_callback()
                    self._conn_callback = None
#                if AUTO_MODE:
#                    if _DISCOVER_SERVICES:
#                        self.discover_services()
#                    else:
#                        self.read_firmware(callback=self.read_firmware_done)

        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            # Disconnect (either initiated by us or the remote end).
            conn_handle, _, _ = data
            if conn_handle == self._conn_handle:
                # If it was initiated by us, it'll already be reset.
                self._reset()

        elif event == _IRQ_GATTC_SERVICE_RESULT:
            # Connected device returned a service.
            self._debug("bt irq - gattc service result", 2)
            conn_handle, start_handle, end_handle, uuid = data

            if conn_handle == self._conn_handle:
                self._debug("{} -> service handle: {}...{}".format(uuid, start_handle, end_handle), 1)
                self.services[str(uuid)] = start_handle, end_handle
                if uuid == self.search_service:
                    self._debug("Wanted service {} has been discovered!".format(self.search_service), 1)
                    self._start_handle = start_handle
                    self._end_handle = end_handle

        elif event == _IRQ_GATTC_SERVICE_DONE:
            # Service query complete.
            self._debug("bt irq - gattc service done", 2)
            self.state = S_SERVICE_DONE
            if self._serv_done_callback:
                self._serv_done_callback()
                self._serv_done_callback = None
            if AUTO_MODE:
                if _DISCOVER_CHARACTERISTICS:
                    # Note: In AUTO_MODE _start_handle/_end_handle should have been
                    # set according to desired service
                    #       in _IRQ_GATTC_SERVICE_RESULT.
                    if self._start_handle and self._end_handle:
                        self.discover_characteristics(
                            self._start_handle, self._end_handle
                        )
                    else:
                        self._debug("Failed to find gattc service.", 3)
                else:
                    self.read_firmware(callback=self.read_firmware_done)

        elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
            # Connected device returned a characteristic.
            self._debug("bt irq - gattc characteristic result", 2)
            conn_handle, def_handle, value_handle, properties, uuid = data

            if conn_handle == self._conn_handle:
                self._debug(
                    "{}; def_handle: {}; value_handle: {}; properties: {}".format(
                        uuid, def_handle, value_handle, properties), 1
                )
                self.characteristics[str(uuid)] = def_handle, value_handle, properties

        elif event == _IRQ_GATTC_CHARACTERISTIC_DONE:
            # Characteristic query complete.
            self._debug("bt irq - gattc characteristic done", 2)
            self.state = S_CHARACTERISTIC_DONE
            if self._char_done_callback:
                self._char_done_callback()
                self._char_done_callback = None
 #           if AUTO_MODE:
 #               self.read_firmware(callback=self.read_firmware_done)

        elif event == _IRQ_GATTC_READ_RESULT:
            # A read completed successfully.
            self._debug("bt irq - gattc read result", 2)
            conn_handle, value_handle, char_data = data
            if conn_handle == self._conn_handle and value_handle == self._value_handle:
                self._update_value(char_data)
                if self._read_callback:
                    self._read_callback(self._value)
                    self._read_callback = None

        elif event == _IRQ_GATTC_READ_DONE:
            # Read completed (no-op).
            self._debug("bt irq - gattc read done", 2)
            conn_handle, value_handle, status = data
#            if AUTO_MODE and self.state == S_READ_FIRMWARE_DONE:
#               self.mode_change(self.mode_change_done)

        elif event == _IRQ_GATTC_WRITE_DONE:
            # A gattc_write() has completed.
            # Note: The value_handle will be zero on btstack (but present on NimBLE).
            # Note: Status is zero on success, implementation-specific value otherwise.
            self._debug("bt irq - gattc write done", 2)
            conn_handle, value_handle, status = data
            if conn_handle == self._conn_handle and value_handle == self._value_handle:
                self._debug("status: {}".format(status), 3)
                if self._write_callback:
                    self._write_callback()
                    self._write_callback = None
#               if AUTO_MODE and self.state == S_MODE_CHANGE_DONE:
#                   self.read_sensor(callback=self.read_sensor_done)

        elif event == _IRQ_GATTC_NOTIFY:
            self._debug("bt irq - gattc notify", 2)

            conn_handle, value_handle, notify_data = data
            if conn_handle == self._conn_handle and value_handle == self._value_handle:
                self._update_value(notify_data)
                if self._notify_callback:
                    self._notify_callback(self._value)


#  Action trigger methods
    def scan(self, callback=None):
        """
        Find all available devices.
        See https://docs.micropython.org/en/latest/library/ bluetooth.html
        for gap_scan() parameters.
        Parameters:
            callback (function): callback to be invoked in _IRQ_SCAN_DONE
            if the desired device was found in _IRQ_SCAN_RESULT
        """
        self._addr_type = None
        self._addr = None
        self._scan_callback = callback
        try:
            self._ble.gap_scan(2000, 30000, 30000, True)
        except OSError as e:
            pass

    def gap_connect(self, addr_type=None, addr=None, callback=None):
        """
        Connect to the specified device (otherwise use cached address from a scan).
        See https://docs.micropython.org/en/latest/library/ bluetooth.html for
        gap_connect().
        Parameters:
            addr_type (int):     address type (PUBLIC or RANDOM)
            addr (bytes):        BLE MAC address
            callback (function): callback to be invoked in _IRQ_PERIPHERAL_CONNECT
        Returns:
            bool: True  if valid address type and address was available and
            gap_connect() was called without error (not connected yet!),
                  False otherwise
        """
        self._debug("gap_connect()", 1)
        if not (addr_type is None) and not (addr is None):
            # if provided, use address type and address provided as parameters
            # (otherwise use address type and address from preceeding scan)
            self._addr_type = addr_type
            self._addr = addr
        self._conn_callback = callback
        if self._addr_type is None or self._addr is None:
            self.debug(
                "gap_connect(): Parameter error! _addr_type: {}; _addr: {}".format(
                    self._addr_type, self._addr), 1
            )
            return False
        try:
            self._ble.gap_connect(self._addr_type, self._addr)
            return True
        except OSError as e:
            return False

    def disconnect(self):
        """
        Disconnect from current device and reset object's attributes.
        """
        self._debug("disconnect()", 1)
        if not self._conn_handle:
            return
        try:
            self._ble.gap_disconnect(self._conn_handle)
        except OSError as e:
            pass
        self._reset()

    def discover_services(self, callback=None):
        """
        Discover services provided by connected device.
        All discovered services are stored in 'services'.
        For if service with UUID provided in 'search_service' was discovered,
        '_start_handle' and '_end_handle' for this service are stored.

        See https://docs.micropython.org/en/latest/library/ bluetooth.html
        for gattc_discover_services().
        Parameters:
            callback (function): callback to be invoked in _IRQ_GATTC_SERVICE_DONE
        """
        self._debug("discover_services()", 1)
        self.services = {}
        if not self.is_connected():
            return
        self._serv_done_callback = callback
        try:
            self._ble.gattc_discover_services(self._conn_handle)
        except OSError as e:
            pass

    def discover_characteristics(self, start_handle, end_handle, callback=None):
        """
        Discover characteristics of connected device in range specified by
        start_handle/end_handle.
        All discovered characteristics are stored in 'characteristics'.
        See https://docs.micropython.org/en/latest/library/ bluetooth.html for
        gattc_discover_services().
        Parameters:
            start_handle (int):  start of characteristic range
            end_handle (int):    end of characteristic range
            callback (function): callback to be invoked in
            _IRQ_GATTC_CHARACTERISTIC_DONE
        """
        self._debug("discover_characteristics()", 1)
        self.characteristics = {}
        if not self.is_connected():
            return
        self._char_done_callback = callback
        try:
            self._ble.gattc_discover_characteristics(
                self._conn_handle, start_handle, end_handle
            )
        except OSError as e:
            pass

    def read(self, callback):
        """
        Generic read access.
        Issues an (asynchronous) read of _value_handle, will invoke callback with data.
        See https://docs.micropython.org/en/latest/library/ bluetooth.html
        for gattc_read()
        Parameters:
            callback (function): callback to be invoked in _IRQ_GATTC_READ_RESULT
        """
        self._debug("read()", 1)
        if not self.is_connected():
            return
        self._read_callback = callback
        try:
            self._ble.gattc_read(self._conn_handle, self._value_handle)
        except OSError as e:
            pass

    def read_battery_level(self, callback):
        """
        Read Battery Level.
        Issues an (asynchronous) read from _value_handle = _HANDLE_FIRM_REV_STR,
        will invoke callback with data.
        See https://docs.micropython.org/en/latest/library/ bluetooth.html
        for gattc_read()
        Parameters:
            callback (function): callback to be invoked in _IRQ_GATTC_READ_RESULT
        """
        self._debug("read_battery_level()", 1)
        if not self.is_connected():
            return
        self._read_callback = callback
        self._value_handle = _HANDLE_BATT_LEVEL
        try:
            self._ble.gattc_read(self._conn_handle, self._value_handle)
        except OSError as e:
            pass

    def read_temp_sensor(self, callback):
        self._debug("read_temperature sensor()", 1)
        if not self.is_connected():
            return
        self._read_callback = callback         
        self._value_handle = _HANDLE_TEMPERATURE
        try:
            self._ble.gattc_read(self._conn_handle, self._value_handle)
        except OSError as e:
            pass
    
    def read_humid_sensor(self, callback):
        self._debug("read_humidity_sensor()", 1)
        if not self.is_connected():
            return
        self._read_callback = callback   
        self._value_handle = _HANDLE_HUMIDITY       
        try:
            self._ble.gattc_read(self._conn_handle, self._value_handle)
        except OSError as e:
            pass

    # Callback methods
    def scan_done(self, addr_type, addr, name):
        """
        Callback for scan().
        The parameters are those of the device which matched the search criterion.
        (In this case the address.)
        Parameters:
            addr_type (int): address type (PUBLIC or RANDOM)
            addr (bytes):    BLE MAC address
            name (string):   device name
        """
        self._debug("scan_done()", 1)
        self.state = S_SCAN_DONE

    def read_battery_level_done(self, data):
        """
        Callback for read_battery_level().
        The battery level and the firmware version are copied from the read data.
        Parameters: data (memoryview): read data
        """
        self._debug("read_battery_level_done()", 1)
        data = bytes(data) 
        self._debug("data(): {}".format(binascii.hexlify(data)), 2)            
        self.battery = struct.unpack("B", data[0:1])  # 1 byte, 0x64 = 100%(dec)
        self._debug("read_battery_level-Level: {}%".format(self.battery[0]), 1)        
        self.state = S_READ_BATTERY_DONE

    def read_temp_sensor_done(self, data):
        """Callback for read_temp_sensor().
        The sensor data - temperature is copied from the read data.
        Parameters:  data (memoryview): read data  
        """
        self._debug("read_temp_sensor_done()", 1)
        data = bytes(data)
        self._debug("data(): {}".format(binascii.hexlify(data)), 2)        
        # Note 1: ustruct.unpack() does not support padding
        #         (cf. https://docs.micropython.org/en/latest/library/struct.html)
        # Note 2: (u)struct() returns a tuple, even if result is a single element        
        self.temperature = struct.unpack("<f", data[0:4])    # 4 byte float
        self._debug("read_temp_sensor-Temperature: {}°C".format(self.temperature[0]), 1)
        self.state = S_READ_TSENSOR_DONE

    def read_humid_sensor_done(self, data):
        """Callback for read_humid_sensor().
        The sensor data - humidity is copied from the read data.
        Parameters:  data (memoryview): read data  
        """
        self._debug("read_humid_sensor_done()", 1)
        data = bytes(data)
        self._debug("data(): {}".format(binascii.hexlify(data)), 2)
        self.humidity = struct.unpack("<f", data[0:4])    # 4 byte float   
        self._debug("read_humid sensor-Humidity: {}%".format(self.humidity[0]), 1)    
        self.state = S_READ_HSENSOR_DONE

    def is_connected(self):
        """Check if connected to device.
        Returns:
            bool: True  if connected,
                  False otherwise.
        """
        return self._conn_handle is not None

    def wait_for_connection(self, status, timeout_ms):
        """
        Wait until connection reaches 'status' or a timeout occurrs.
        The connection status is polled in _T_WAIT intervals.
        Parameters:
            status (bool):     expected connection status
            timeout_ms (int) : timeout in ms
        Returns:
            bool: True  desired status occurred,
                  False timeout ocurred.
        """
        t0 = time.ticks_ms()

        while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
            if self.is_connected() == status:
                return True
            time.sleep_ms(_T_WAIT)
        return False

    def wait_for(self, state, timeout_ms):
        """
        Wait until 'state' is reached or a timeout occurrs.
        The state is polled in _T_WAIT intervals.
        Parameters:
            status (bool):     expected connection status
            timeout_ms (int) : timeout in ms
        Returns:
            bool: True  desired status occurred,
                  False timeout ocurred.
        """
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
            if self.state == state:
                return True
            time.sleep_ms(_T_WAIT)
        return False

    # Helper methods
    def on_notify(self, callback):
        """
        Set a callback for device notifications.
        Parameters:
            callback (function): callback to be invoked in _IRQ_GATTC_NOTIFY
        """
        self._debug("on_notify()", 0)
        self._notify_callback = callback

    def _update_value(self, data):
        """
        Update value from a notification or a read access.
        Parameters:
            data (memoryview): notification/read data (payload)
        Returns:
            memoryview: object in memory containing payload data
        """
        self._debug("_update_value()", 1)
        self._value = data
        return self._value

    def value(self):
        """
        Read access function for '_value'. (?)
        """
        return self._value

#  END class Sensiron


# Stand-alone version of Sensiron.wait_for() - obsolete!
def wait_for(obj, state, timeout_ms):

    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        if getattr(obj, "state") == state:
            return True
        time.sleep_ms(_T_WAIT)
    return False


# Stand-alone version of Sensiron.wait_for_connection() - obsolete!
def wait_for_connection(obj, status, timeout_ms):
    t0 = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        if obj.is_connected() == status:
            return True
        time.sleep_ms(_T_WAIT)
    return False


###############################################################################
# demo_man() - test function for class Sensiron gadget ("manual" mode)
#
# Sensor access is controlled step-by-step from the application. Typically an
# action is trigged, a callback function is invoked upon completion and the
# application waits until the expected state of the Sensiron class is reached
# (or a timeout occurred).
#
###############################################################################
def demo_man():
    ble = bluetooth.BLE()
    gadget = Sensiron(ble)
    print("demo_man()")

#    while True:
    for addr in Sensiron_mac:
        gadget.search_addr = addr
        gadget.search_service = _GENERIC_ACCESS_SERVICE_UUID     
#       gadget.search_service = _GADGET_TEMP_SERV_UUID   
#       gadget.search_service = _GADGET_DEVICE_INFO_UUID            

        if SCAN_DEVICES:
            print("Searching for device with MAC address {}...".format(binascii.hexlify(addr)))

            gadget.scan(callback=gadget.scan_done)

            if gadget.wait_for(S_SCAN_DONE, 2500):
                print("Scan done.")
            else:
                print("Scan timeout!")
                continue

            if not gadget.addr_found:
                print("Sensor not found!")
                continue

            print("Sensor '{}' found.".format(gadget.name))
            print("RSSI: {}dbB".format(gadget.rssi))

        print(
            "Trying to connect to device with MAC address {}...".format(binascii.hexlify(addr)))           
#        rc = gadget.gap_connect(ADDR_TYPE_PUBLIC, addr)
        rc = gadget.gap_connect(ADDR_TYPE_RANDOM, addr)                       
        print("gap_connect() = ", rc)

        if gadget.wait_for_connection(True, 5000):
            print("Connected")
        else:
            print("Connection failed!")
            continue

        if DISCOVER_SERVICES:
            gadget.discover_services()

            if gadget.wait_for(S_SERVICE_DONE, 2500):
                print(gadget.services)
            else:
                print("discover_services failed!")

        if DISCOVER_CHARACTERISTICS:
            gadget.discover_characteristics(1, 0xff)
 
            if gadget.wait_for(S_CHARACTERISTIC_DONE, 2500):
                print(gadget.characteristics)
            else:
                print("discover_characteristics failed!")

        gadget.read_battery_level(callback=gadget.read_battery_level_done)
        if gadget.wait_for(S_READ_BATTERY_DONE, 2000):
            print("Battery Level: {}%".format(gadget.battery))
        else:
            print("Reading battery level failed!")

        gadget.read_temp_sensor(callback=gadget.read_temp_sensor_done)
        if gadget.wait_for(S_READ_TSENSOR_DONE, 2000):
            print("Temperature: {}°C".format(gadget.temperature))
        else:
            print("Reading temperature sensor data failed!")
            
        gadget.read_humid_sensor(callback=gadget.read_humid_sensor_done)
        if gadget.wait_for(S_READ_HSENSOR_DONE, 2000):
            print("Humidity: {}%".format(gadget.humidity))
        else:
            print("Reading humidity sensor data failed!")               
                         
        gadget.disconnect()  # does not work for some reason

        if gadget.wait_for_connection(False, 10000):
            print("Disconnected")
        else:
            print("Disconnect failed!")
            gadget._reset()

demo_man()
machine.reset()   # BLE disconnect does not work so reset for now


