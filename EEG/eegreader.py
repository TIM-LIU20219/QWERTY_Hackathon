######Helper

from sys import platform
import warnings


def warn_bluemuse_not_supported(extra_text = ''):
    warnings.warn('Operation not supported by bluemuse backend.' + extra_text,
                  RuntimeWarning)


def resolve_backend(backend):
    if backend in ['auto', 'gatt', 'bgapi', 'bluemuse']:
        #platformName = platform.system().lower()
        """platformName = 'windows'
        if backend == 'auto':
            if platformName == 'linux' or platformName == 'linux2':
                backend = 'gatt'
            elif platformName == 'windows' and int(platform.version().replace('.', '')) >= 10015063:
                backend = 'bluemuse'
            else:
                backend = 'bgapi'"""
        backend='bluemuse'
        return backend
    else:
        raise(ValueError('Backend must be one of: auto, gatt, bgapi, bluemuse.'))
#####

#####Constants

MUSE_NB_EEG_CHANNELS = 5
MUSE_SAMPLING_EEG_RATE = 256
LSL_EEG_CHUNK = 12

MUSE_NB_PPG_CHANNELS = 3
MUSE_SAMPLING_PPG_RATE = 64
LSL_PPG_CHUNK = 6

MUSE_NB_ACC_CHANNELS = 3
MUSE_SAMPLING_ACC_RATE = 52
LSL_ACC_CHUNK = 1

MUSE_NB_GYRO_CHANNELS = 3
MUSE_SAMPLING_GYRO_RATE = 52
LSL_GYRO_CHUNK = 1

# 00001800-0000-1000-8000-00805f9b34fb Generic Access 0x05-0x0b
# 00001801-0000-1000-8000-00805f9b34fb Generic Attribute 0x01-0x04
MUSE_GATT_ATTR_SERVICECHANGED = '00002a05-0000-1000-8000-00805f9b34fb' # ble std 0x02-0x04
# 0000fe8d-0000-1000-8000-00805f9b34fb Interaxon Inc. 0x0c-0x42
MUSE_GATT_ATTR_STREAM_TOGGLE = '273e0001-4c4d-454d-96be-f03bac821358' # serial 0x0d-0x0f
MUSE_GATT_ATTR_LEFTAUX = '273e0002-4c4d-454d-96be-f03bac821358' # not implemented yet 0x1c-0x1e
MUSE_GATT_ATTR_TP9 = '273e0003-4c4d-454d-96be-f03bac821358' # 0x1f-0x21
MUSE_GATT_ATTR_AF7 = '273e0004-4c4d-454d-96be-f03bac821358' # fp1 0x22-0x24
MUSE_GATT_ATTR_AF8 = '273e0005-4c4d-454d-96be-f03bac821358' # fp2 0x25-0x27
MUSE_GATT_ATTR_TP10 = '273e0006-4c4d-454d-96be-f03bac821358' # 0x28-0x2a
MUSE_GATT_ATTR_RIGHTAUX = '273e0007-4c4d-454d-96be-f03bac821358' #0x2b-0x2d
MUSE_GATT_ATTR_REFDRL = '273e0008-4c4d-454d-96be-f03bac821358' # not implemented yet 0x10-0x12
MUSE_GATT_ATTR_GYRO = '273e0009-4c4d-454d-96be-f03bac821358' # 0x13-0x15
MUSE_GATT_ATTR_ACCELEROMETER = '273e000a-4c4d-454d-96be-f03bac821358' # 0x16-0x18
MUSE_GATT_ATTR_TELEMETRY = '273e000b-4c4d-454d-96be-f03bac821358' # 0x19-0x1b
#MUSE_GATT_ATTR_MAGNETOMETER = '273e000c-4c4d-454d-96be-f03bac821358' # 0x2e-0x30
#MUSE_GATT_ATTR_PRESSURE = '273e000d-4c4d-454d-96be-f03bac821358' # 0x31-0x33
#MUSE_GATT_ATTR_ULTRAVIOLET = '273e000e-4c4d-454d-96be-f03bac821358' # 0x34-0x36
MUSE_GATT_ATTR_PPG1 = "273e000f-4c4d-454d-96be-f03bac821358" # ambient 0x37-0x39
MUSE_GATT_ATTR_PPG2 = "273e0010-4c4d-454d-96be-f03bac821358" # infrared 0x3a-0x3c
MUSE_GATT_ATTR_PPG3 = "273e0011-4c4d-454d-96be-f03bac821358" # red 0x3d-0x3f
MUSE_GATT_ATTR_THERMISTOR = "273e0012-4c4d-454d-96be-f03bac821358" # muse S only, not implemented yet 0x40-0x42

MUSE_ACCELEROMETER_SCALE_FACTOR = 0.0000610352
MUSE_GYRO_SCALE_FACTOR = 0.0074768

MUSE_SCAN_TIMEOUT = 10.5
AUTO_DISCONNECT_DELAY = 3

LSL_SCAN_TIMEOUT = 5
LSL_BUFFER = 360

VIEW_SUBSAMPLE = 2
VIEW_BUFFER = 12

#####

##### Muse

import bitstring
import pygatt
import numpy as np
from time import time, sleep
from sys import platform
import subprocess

class Muse():
    """Muse 2016 headband"""

    def __init__(self,
                 address,
                 callback_eeg=None,
                 callback_control=None,
                 callback_telemetry=None,
                 callback_acc=None,
                 callback_gyro=None,
                 callback_ppg=None,
                 backend='auto',
                 interface=None,
                 time_func=time,
                 name=None,
                 preset=None,
                 disable_light=False):
        """Initialize

        callback_eeg -- callback for eeg data, function(data, timestamps)
        callback_control -- function(message)
        callback_telemetry -- function(timestamp, battery, fuel_gauge,
                                       adc_volt, temperature)

        callback_acc -- function(timestamp, samples)
        callback_gyro -- function(timestamp, samples)
        - samples is a list of 3 samples, where each sample is [x, y, z]
        """

        self.address = address
        self.name = name
        self.callback_eeg = callback_eeg
        self.callback_telemetry = callback_telemetry
        self.callback_control = callback_control
        self.callback_acc = callback_acc
        self.callback_gyro = callback_gyro
        self.callback_ppg = callback_ppg

        self.enable_eeg = not callback_eeg is None
        self.enable_control = not callback_control is None
        self.enable_telemetry = not callback_telemetry is None
        self.enable_acc = not callback_acc is None
        self.enable_gyro = not callback_gyro is None
        self.enable_ppg = not callback_ppg is None

        self.interface = interface
        self.time_func = time_func
        self.backend = resolve_backend(backend)
        self.preset = preset
        self.disable_light = disable_light

    def connect(self, interface=None, backend='auto'):
        """Connect to the device"""
        try:
            if self.backend == 'bluemuse':
                print('Starting BlueMuse.')
                subprocess.call('start bluemuse:', shell=True)
                self.last_timestamp = self.time_func()
            else:
                print('Connecting to %s: %s...' % (self.name
                                                   if self.name else 'Muse',
                                                   self.address))
                if self.backend == 'gatt':
                    self.interface = self.interface or 'hci0'
                    self.adapter = pygatt.GATTToolBackend(self.interface)
                else:
                    self.adapter = pygatt.BGAPIBackend(
                        serial_port=self.interface)

                self.adapter.start()
                self.device = self.adapter.connect(self.address)
                if(self.preset != None):
                    self.select_preset(self.preset)

                # subscribes to EEG stream
                if self.enable_eeg:
                    self._subscribe_eeg()

                if self.enable_control:
                    self._subscribe_control()

                if self.enable_telemetry:
                    self._subscribe_telemetry()

                if self.enable_acc:
                    self._subscribe_acc()

                if self.enable_gyro:
                    self._subscribe_gyro()

                if self.enable_ppg:
                    self._subscribe_ppg()
                
                if self.disable_light:
                    self._disable_light()

                self.last_timestamp = self.time_func()

            return True

        except pygatt.exceptions.BLEError as error:
            if ("characteristic" in str(error)):
                self.ask_reset()
                sleep(2)
                self.device = self.adapter.connect(self.address)
                self.select_preset(self.preset)

                # subscribes to EEG stream
                if self.enable_eeg:
                    self._subscribe_eeg()

                if self.enable_control:
                    self._subscribe_control()

                if self.enable_telemetry:
                    self._subscribe_telemetry()

                if self.enable_acc:
                    self._subscribe_acc()

                if self.enable_gyro:
                    self._subscribe_gyro()

                if self.enable_ppg:
                    self._subscribe_ppg()
                
                if self.disable_light:
                    self._disable_light()

                self.last_timestamp = self.time_func()

                return True

            else:
                print('Connection to', self.address, 'failed')
                return False

    def _write_cmd(self, cmd):
        """Wrapper to write a command to the Muse device.
        cmd -- list of bytes"""
        self.device.char_write_handle(0x000e, cmd, False)

    def _write_cmd_str(self, cmd):
        """Wrapper to encode and write a command string to the Muse device.
        cmd -- string to send"""
        self._write_cmd([len(cmd) + 1, *(ord(char) for char in cmd), ord('\n')])

    def ask_control(self):
        """Send a message to Muse to ask for the control status.

        Only useful if control is enabled (to receive the answer!)

        The message received is a dict with the following keys:
        "hn": device name
        "sn": serial number
        "ma": MAC address
        "id":
        "bp": battery percentage
        "ts":
        "ps": preset selected
        "rc": return status, if 0 is OK
        """
        if self.backend == 'bluemuse':
            warn_bluemuse_not_supported('Control information available manually by using the BlueMuse GUI.')
            return
        self._write_cmd_str('s')

    def ask_device_info(self):
        """Send a message to Muse to ask for the device info.

        The message received is a dict with the following keys:
        "ap":
        "sp":
        "tp": firmware type, e.g: "consumer"
        "hw": hardware version?
        "bn": build number?
        "fw": firmware version?
        "bl":
        "pv": protocol version?
        "rc": return status, if 0 is OK
        """
        if self.backend == 'bluemuse':
            warn_bluemuse_not_supported('Device information available manually by using the BlueMuse GUI.')
            return
        self._write_cmd_str('v1')

    def ask_reset(self):
        """Undocumented command reset for '*1'
        The message received is a singleton with:
        "rc": return status, if 0 is OK
        """
        self._write_cmd_str('*1')

    def start(self):
        """Start streaming."""
        if self.backend == 'bluemuse':
            address = self.address if self.address is not None else self.name
            if address is None:
                subprocess.call(
                    'start bluemuse://start?streamfirst=true', shell=True)
            else:
                subprocess.call(
                    'start bluemuse://start?addresses={0}'.format(address),
                    shell=True)
            return

        self.first_sample = True
        self._init_sample()
        self._init_ppg_sample()
        self.last_tm = 0
        self.last_tm_ppg = 0
        self._init_control()
        self.resume()

    def resume(self):
        """Resume streaming, sending 'd' command"""
        self._write_cmd_str('d')

    def stop(self):
        """Stop streaming."""
        if self.backend == 'bluemuse':
            address = self.address if self.address is not None else self.name
            if address is None:
                subprocess.call('start bluemuse://stopall', shell=True)
            else:
                subprocess.call(
                    'start bluemuse://stop?addresses={0}'.format(address),
                    shell=True)
            return

        self._write_cmd_str('h')

    def keep_alive(self):
        """Keep streaming, sending 'k' command"""
        self._write_cmd_str('k')

    def select_preset(self, preset=21):
        """Set preset for headband configuration

        See details here https://articles.jaredcamins.com/figuring-out-bluetooth-low-energy-part-2-750565329a7d
        For 2016 headband, possible choice are 'p20' and 'p21'.
        Untested but possible values include 'p22','p23','p31','p32','p50','p51','p52','p53','p60','p61','p63','pAB','pAD'
        Default is 'p21'."""

        if type(preset) is int:
            preset = str(preset)
        if preset[0] == 'p':
            preset = preset[1:]
        if str(preset) != '21':
            print('Sending command for non-default preset: p' + preset)
        preset = bytes(preset, 'utf-8')
        self._write_cmd([0x04, 0x70, *preset, 0x0a])

    def disconnect(self):
        """disconnect."""
        if self.backend == 'bluemuse':
            subprocess.call('start bluemuse://shutdown', shell=True)
            return

        self.device.disconnect()
        if self.adapter:
            self.adapter.stop()

    def _subscribe_eeg(self):
        """subscribe to eeg stream."""
        self.device.subscribe(MUSE_GATT_ATTR_TP9, callback=self._handle_eeg)
        self.device.subscribe(MUSE_GATT_ATTR_AF7, callback=self._handle_eeg)
        self.device.subscribe(MUSE_GATT_ATTR_AF8, callback=self._handle_eeg)
        self.device.subscribe(MUSE_GATT_ATTR_TP10, callback=self._handle_eeg)
        self.device.subscribe(
            MUSE_GATT_ATTR_RIGHTAUX, callback=self._handle_eeg)

    def _unpack_eeg_channel(self, packet):
        """Decode data packet of one EEG channel.

        Each packet is encoded with a 16bit timestamp followed by 12 time
        samples with a 12 bit resolution.
        """
        aa = bitstring.Bits(bytes=packet)
        pattern = "uint:16,uint:12,uint:12,uint:12,uint:12,uint:12,uint:12, \
                   uint:12,uint:12,uint:12,uint:12,uint:12,uint:12"

        res = aa.unpack(pattern)
        packetIndex = res[0]
        data = res[1:]
        # 12 bits on a 2 mVpp range
        data = 0.48828125 * (np.array(data) - 2048)
        return packetIndex, data

    def _init_sample(self):
        """initialize array to store the samples"""
        self.timestamps = np.full(5, np.nan)
        self.data = np.zeros((5, 12))

    def _init_ppg_sample(self):
        """ Initialise array to store PPG samples

            Must be separate from the EEG packets since they occur with a different sampling rate. Ideally the counters
            would always match, but this is not guaranteed
        """
        self.timestamps_ppg = np.full(3, np.nan)
        self.data_ppg = np.zeros((3, 6))

    def _init_timestamp_correction(self):
        """Init IRLS params"""
        # initial params for the timestamp correction
        # the time it started + the inverse of sampling rate
        self.sample_index = 0
        self.sample_index_ppg = 0
        self._P = 1e-4
        t0 = self.time_func()
        self.reg_params = np.array([t0, 1. / MUSE_SAMPLING_EEG_RATE])
        self.reg_ppg_sample_rate = np.array([t0, 1. / MUSE_SAMPLING_PPG_RATE])

    def _update_timestamp_correction(self, t_source, t_receiver):
        """Update regression for dejittering

        This is based on Recursive least square.
        See https://arxiv.org/pdf/1308.3846.pdf.
        """

        # remove the offset
        t_receiver = t_receiver - self.reg_params[0]

        # least square estimation
        P = self._P
        R = self.reg_params[1]
        P = P - ((P**2) * (t_source**2)) / (1 - (P * (t_source**2)))
        R = R + P * t_source * (t_receiver - t_source * R)

        # update parameters
        self.reg_params[1] = R
        self._P = P

    def _handle_eeg(self, handle, data):
        """Callback for receiving a sample.

        samples are received in this order : 44, 41, 38, 32, 35
        wait until we get 35 and call the data callback
        """
        if self.first_sample:
            self._init_timestamp_correction()
            self.first_sample = False

        timestamp = self.time_func()
        index = int((handle - 32) / 3)
        tm, d = self._unpack_eeg_channel(data)

        if self.last_tm == 0:
            self.last_tm = tm - 1

        self.data[index] = d
        self.timestamps[index] = timestamp
        # last data received
        if handle == 35:
            if tm != self.last_tm + 1:
                if (tm - self.last_tm) != -65535:  # counter reset
                    print("missing sample %d : %d" % (tm, self.last_tm))
                    # correct sample index for timestamp estimation
                    self.sample_index += 12 * (tm - self.last_tm + 1)

            self.last_tm = tm

            # calculate index of time samples
            idxs = np.arange(0, 12) + self.sample_index
            self.sample_index += 12

            # update timestamp correction
            # We received the first packet as soon as the last timestamp got
            # sampled
            self._update_timestamp_correction(idxs[-1], np.nanmin(
                self.timestamps))

            # timestamps are extrapolated backwards based on sampling rate
            # and current time
            timestamps = self.reg_params[1] * idxs + self.reg_params[0]

            # push data
            self.callback_eeg(self.data, timestamps)

            # save last timestamp for disconnection timer
            self.last_timestamp = timestamps[-1]

            # reset sample
            self._init_sample()

    def _init_control(self):
        """Variable to store the current incoming message."""
        self._current_msg = ""

    def _subscribe_control(self):
        self.device.subscribe(
            MUSE_GATT_ATTR_STREAM_TOGGLE, callback=self._handle_control)

        self._init_control()

    def _handle_control(self, handle, packet):
        """Handle the incoming messages from the 0x000e handle.

        Each message is 20 bytes
        The first byte, call it n, is the length of the incoming string.
        The rest of the bytes are in ASCII, and only n chars are useful

        Multiple messages together are a json object (or dictionary in python)
        If a message has a '}' then the whole dict is finished.

        Example:
        {'key': 'value',
        'key2': 'really-long
        -value',
        'key3': 'value3'}

        each line is a message, the 4 messages are a json object.
        """
        if handle != 14:
            return

        # Decode data
        bit_decoder = bitstring.Bits(bytes=packet)
        pattern = "uint:8,uint:8,uint:8,uint:8,uint:8,uint:8,uint:8,uint:8,uint:8,uint:8, \
                    uint:8,uint:8,uint:8,uint:8,uint:8,uint:8,uint:8,uint:8,uint:8,uint:8"

        chars = bit_decoder.unpack(pattern)

        # Length of the string
        n_incoming = chars[0]

        # Parse as chars, only useful bytes
        incoming_message = "".join(map(chr, chars[1:]))[:n_incoming]

        # Add to current message
        self._current_msg += incoming_message

        if incoming_message[-1] == '}':  # Message ended completely
            self.callback_control(self._current_msg)

            self._init_control()

    def _subscribe_telemetry(self):
        self.device.subscribe(
            MUSE_GATT_ATTR_TELEMETRY, callback=self._handle_telemetry)

    def _handle_telemetry(self, handle, packet):
        """Handle the telemetry (battery, temperature and stuff) incoming data
        """

        if handle != 26:  # handle 0x1a
            return
        timestamp = self.time_func()

        bit_decoder = bitstring.Bits(bytes=packet)
        pattern = "uint:16,uint:16,uint:16,uint:16,uint:16"  # The rest is 0 padding
        data = bit_decoder.unpack(pattern)

        battery = data[1] / 512
        fuel_gauge = data[2] * 2.2
        adc_volt = data[3]
        temperature = data[4]

        self.callback_telemetry(timestamp, battery, fuel_gauge, adc_volt,
                                temperature)

    def _unpack_imu_channel(self, packet, scale=1):
        """Decode data packet of the accelerometer and gyro (imu) channels.

        Each packet is encoded with a 16bit timestamp followed by 9 samples
        with a 16 bit resolution.
        """
        bit_decoder = bitstring.Bits(bytes=packet)
        pattern = "uint:16,int:16,int:16,int:16,int:16, \
                   int:16,int:16,int:16,int:16,int:16"

        data = bit_decoder.unpack(pattern)

        packet_index = data[0]

        samples = np.array(data[1:]).reshape((3, 3), order='F') * scale

        return packet_index, samples

    def _subscribe_acc(self):
        self.device.subscribe(
            MUSE_GATT_ATTR_ACCELEROMETER, callback=self._handle_acc)

    def _handle_acc(self, handle, packet):
        """Handle incoming accelerometer data.

        sampling rate: ~17 x second (3 samples in each message, roughly 50Hz)"""
        if handle != 23:  # handle 0x17
            return
        timestamps = [self.time_func()] * 3

        # save last timestamp for disconnection timer
        self.last_timestamp = timestamps[-1]

        packet_index, samples = self._unpack_imu_channel(
            packet, scale=MUSE_ACCELEROMETER_SCALE_FACTOR)

        self.callback_acc(samples, timestamps)

    def _subscribe_gyro(self):
        self.device.subscribe(MUSE_GATT_ATTR_GYRO, callback=self._handle_gyro)

    def _handle_gyro(self, handle, packet):
        """Handle incoming gyroscope data.

        sampling rate: ~17 x second (3 samples in each message, roughly 50Hz)"""
        if handle != 20:  # handle 0x14
            return

        timestamps = [self.time_func()] * 3

        # save last timestamp for disconnection timer
        self.last_timestamp = timestamps[-1]

        packet_index, samples = self._unpack_imu_channel(
            packet, scale=MUSE_GYRO_SCALE_FACTOR)

        self.callback_gyro(samples, timestamps)

    def _subscribe_ppg(self):
        try:
            """subscribe to ppg stream."""
            self.device.subscribe(
                MUSE_GATT_ATTR_PPG1, callback=self._handle_ppg)
            self.device.subscribe(
                MUSE_GATT_ATTR_PPG2, callback=self._handle_ppg)
            self.device.subscribe(
                MUSE_GATT_ATTR_PPG3, callback=self._handle_ppg)

        except pygatt.exceptions.BLEError as error:
            raise Exception(
                'PPG data is not available on this device. PPG is only available on Muse 2'
            )

    def _handle_ppg(self, handle, data):
        """Callback for receiving a sample.

        samples are received in this order : 56, 59, 62
        wait until we get x and call the data callback
        """
        timestamp = self.time_func()
        index = int((handle - 56) / 3)
        tm, d = self._unpack_ppg_channel(data)

        if self.last_tm_ppg == 0:
            self.last_tm_ppg = tm - 1

        self.data_ppg[index] = d
        self.timestamps_ppg[index] = timestamp
        # last data received
        if handle == 62:
            if tm != self.last_tm_ppg + 1:
                print("missing sample %d : %d" % (tm, self.last_tm_ppg))
            self.last_tm_ppg = tm

            # calculate index of time samples
            idxs = np.arange(0, LSL_PPG_CHUNK) + self.sample_index_ppg
            self.sample_index_ppg += LSL_PPG_CHUNK

            # timestamps are extrapolated backwards based on sampling rate and current time
            timestamps = self.reg_ppg_sample_rate[1] * \
                idxs + self.reg_ppg_sample_rate[0]

            # save last timestamp for disconnection timer
            self.last_timestamp = timestamps[-1]

            # push data
            if self.callback_ppg:
                self.callback_ppg(self.data_ppg, timestamps)

            # reset sample
            self._init_ppg_sample()

    def _unpack_ppg_channel(self, packet):
        """Decode data packet of one PPG channel.
        Each packet is encoded with a 16bit timestamp followed by 3
        samples with an x bit resolution.
        """

        aa = bitstring.Bits(bytes=packet)
        pattern = "uint:16,uint:24,uint:24,uint:24,uint:24,uint:24,uint:24"
        res = aa.unpack(pattern)
        packetIndex = res[0]
        data = res[1:]

        return packetIndex, data
    
    def _disable_light(self):
        self._write_cmd_str('L0')

#####
        
#####Stream
import re
import subprocess
from sys import platform
from time import time, sleep
from functools import partial
from shutil import which

from pylsl import StreamInfo, StreamOutlet
import pygatt

def _print_muse_list(muses):
    for m in muses:
        print(f'Found device {m["name"]}, MAC Address {m["address"]}')
    if not muses:
        print('No Muses found.')


# Returns a list of available Muse devices.
def list_muses(backend='auto', interface=None):
    if backend == 'auto' and which('bluetoothctl') is not None:
        print("Backend was 'auto' and bluetoothctl was found, using to list muses...")
        return _list_muses_bluetoothctl(MUSE_SCAN_TIMEOUT)

    backend = resolve_backend(backend)

    if backend == 'gatt':
        interface = interface or 'hci0'
        adapter = pygatt.GATTToolBackend(interface)
    elif backend == 'bluemuse':
        print('Starting BlueMuse, see BlueMuse window for interactive list of devices.')
        subprocess.call('start bluemuse:', shell=True)
        return
    else:
        adapter = pygatt.BGAPIBackend(serial_port=interface)

    try:
        adapter.start()
        print('Searching for Muses, this may take up to 10 seconds...')
        devices = adapter.scan(timeout=MUSE_SCAN_TIMEOUT)
        adapter.stop()
    except pygatt.exceptions.BLEError as e:
        if backend == 'gatt':
            print('pygatt failed to scan for BLE devices. Trying with '
                  'bluetoothctl.')
            return _list_muses_bluetoothctl(MUSE_SCAN_TIMEOUT)
        else:
            raise e

    muses = [d for d in devices if d['name'] and 'Muse' in d['name']]
    _print_muse_list(muses)

    return muses


def _list_muses_bluetoothctl(timeout, verbose=False):
    """Identify Muse BLE devices using bluetoothctl.

    When using backend='gatt' on Linux, pygatt relies on the command line tool
    `hcitool` to scan for BLE devices. `hcitool` is however deprecated, and
    seems to fail on Bluetooth 5 devices. This function roughly replicates the
    functionality of `pygatt.backends.gatttool.gatttool.GATTToolBackend.scan()`
    using the more modern `bluetoothctl` tool.

    Deprecation of hcitool: https://git.kernel.org/pub/scm/bluetooth/bluez.git/commit/?id=b1eb2c4cd057624312e0412f6c4be000f7fc3617
    """
    try:
        import pexpect
    except (ImportError, ModuleNotFoundError):
        msg = ('pexpect is currently required to use bluetoothctl from within '
               'a jupter notebook environment.')
        raise ModuleNotFoundError(msg)

    # Run scan using pexpect as subprocess.run returns immediately in jupyter
    # notebooks
    print('Searching for Muses, this may take up to 10 seconds...')
    scan = pexpect.spawn('bluetoothctl scan on')
    try:
        scan.expect('foooooo', timeout=timeout)
    except pexpect.EOF:
        before_eof = scan.before.decode('utf-8', 'replace')
        msg = f'Unexpected error when scanning: {before_eof}'
        raise ValueError(msg)
    except pexpect.TIMEOUT:
        if verbose:
            print(scan.before.decode('utf-8', 'replace').split('\r\n'))

    # List devices using bluetoothctl
    list_devices_cmd = ['bluetoothctl', 'devices']
    devices = subprocess.run(
        list_devices_cmd, stdout=subprocess.PIPE).stdout.decode(
            'utf-8').split('\n')
    muses = [{
            'name': re.findall('Muse.*', string=d)[0],
            'address': re.findall(r'..:..:..:..:..:..', string=d)[0]
        } for d in devices if 'Muse' in d]
    _print_muse_list(muses)

    return muses


# Returns the address of the Muse with the name provided, otherwise returns address of first available Muse.
def find_muse(name=None, backend='auto'):
    muses = list_muses(backend)
    if name:
        for muse in muses:
            if muse['name'] == name:
                return muse
    elif muses:
        return muses[0]


# Begins LSL stream(s) from a Muse with a given address with data sources determined by arguments
def stream(
    address,
    backend='auto',
    interface=None,
    name=None,
    ppg_enabled=False,
    acc_enabled=False,
    gyro_enabled=False,
    eeg_disabled=False,
    preset=None,
    disable_light=False,
    timeout=AUTO_DISCONNECT_DELAY,
):
    # If no data types are enabled, we warn the user and return immediately.
    if eeg_disabled and not ppg_enabled and not acc_enabled and not gyro_enabled:
        print('Stream initiation failed: At least one data source must be enabled.')
        return

    # For any backend except bluemuse, we will start LSL streams hooked up to the muse callbacks.
    if backend != 'bluemuse':
        if not address:
            found_muse = find_muse(name, backend)
            if not found_muse:
                return
            else:
                address = found_muse['address']
                name = found_muse['name']

        if not eeg_disabled:
            eeg_info = StreamInfo('Muse', 'EEG', MUSE_NB_EEG_CHANNELS, MUSE_SAMPLING_EEG_RATE, 'float32',
                                'Muse%s' % address)
            eeg_info.desc().append_child_value("manufacturer", "Muse")
            eeg_channels = eeg_info.desc().append_child("channels")

            for c in ['TP9', 'AF7', 'AF8', 'TP10', 'Right AUX']:
                eeg_channels.append_child("channel") \
                    .append_child_value("label", c) \
                    .append_child_value("unit", "microvolts") \
                    .append_child_value("type", "EEG")

            eeg_outlet = StreamOutlet(eeg_info, LSL_EEG_CHUNK)

        if ppg_enabled:
            ppg_info = StreamInfo('Muse', 'PPG', MUSE_NB_PPG_CHANNELS, MUSE_SAMPLING_PPG_RATE,
                                'float32', 'Muse%s' % address)
            ppg_info.desc().append_child_value("manufacturer", "Muse")
            ppg_channels = ppg_info.desc().append_child("channels")

            for c in ['PPG1', 'PPG2', 'PPG3']:
                ppg_channels.append_child("channel") \
                    .append_child_value("label", c) \
                    .append_child_value("unit", "mmHg") \
                    .append_child_value("type", "PPG")

            ppg_outlet = StreamOutlet(ppg_info, LSL_PPG_CHUNK)

        if acc_enabled:
            acc_info = StreamInfo('Muse', 'ACC', MUSE_NB_ACC_CHANNELS, MUSE_SAMPLING_ACC_RATE,
                                'float32', 'Muse%s' % address)
            acc_info.desc().append_child_value("manufacturer", "Muse")
            acc_channels = acc_info.desc().append_child("channels")

            for c in ['X', 'Y', 'Z']:
                acc_channels.append_child("channel") \
                    .append_child_value("label", c) \
                    .append_child_value("unit", "g") \
                    .append_child_value("type", "accelerometer")

            acc_outlet = StreamOutlet(acc_info, LSL_ACC_CHUNK)

        if gyro_enabled:
            gyro_info = StreamInfo('Muse', 'GYRO', MUSE_NB_GYRO_CHANNELS, MUSE_SAMPLING_GYRO_RATE,
                                'float32', 'Muse%s' % address)
            gyro_info.desc().append_child_value("manufacturer", "Muse")
            gyro_channels = gyro_info.desc().append_child("channels")

            for c in ['X', 'Y', 'Z']:
                gyro_channels.append_child("channel") \
                    .append_child_value("label", c) \
                    .append_child_value("unit", "dps") \
                    .append_child_value("type", "gyroscope")

            gyro_outlet = StreamOutlet(gyro_info, LSL_GYRO_CHUNK)

        def push(data, timestamps, outlet):
            for ii in range(data.shape[1]):
                outlet.push_sample(data[:, ii], timestamps[ii])

        push_eeg = partial(push, outlet=eeg_outlet) if not eeg_disabled else None
        push_ppg = partial(push, outlet=ppg_outlet) if ppg_enabled else None
        push_acc = partial(push, outlet=acc_outlet) if acc_enabled else None
        push_gyro = partial(push, outlet=gyro_outlet) if gyro_enabled else None

        muse = Muse(address=address, callback_eeg=push_eeg, callback_ppg=push_ppg, callback_acc=push_acc, callback_gyro=push_gyro,
                    backend=backend, interface=interface, name=name, preset=preset, disable_light=disable_light)

        didConnect = muse.connect()

        if(didConnect):
            print('Connected.')
            muse.start()

            eeg_string = " EEG" if not eeg_disabled else ""
            ppg_string = " PPG" if ppg_enabled else ""
            acc_string = " ACC" if acc_enabled else ""
            gyro_string = " GYRO" if gyro_enabled else ""

            print("Streaming%s%s%s%s..." %
                (eeg_string, ppg_string, acc_string, gyro_string))

            while time() - muse.last_timestamp < timeout:
                try:
                    sleep(1)
                except KeyboardInterrupt:
                    muse.stop()
                    muse.disconnect()
                    break

            print('Disconnected.')

    # For bluemuse backend, we don't need to create LSL streams directly, since these are handled in BlueMuse itself.
    else:
        # Toggle all data stream types in BlueMuse.
        subprocess.call('start bluemuse://setting?key=eeg_enabled!value={}'.format('false' if eeg_disabled else 'true'), shell=True)
        subprocess.call('start bluemuse://setting?key=ppg_enabled!value={}'.format('true' if ppg_enabled else 'false'), shell=True)
        subprocess.call('start bluemuse://setting?key=accelerometer_enabled!value={}'.format('true' if acc_enabled else 'false'), shell=True)
        subprocess.call('start bluemuse://setting?key=gyroscope_enabled!value={}'.format('true' if gyro_enabled else 'false'), shell=True)

        muse = Muse(address=address, callback_eeg=None, callback_ppg=None, callback_acc=None, callback_gyro=None,
                    backend=backend, interface=interface, name=name)
        muse.connect()

        if not address and not name:
            print('Targeting first device BlueMuse discovers...')
        else:
            print('Targeting device: '
                  + ':'.join(filter(None, [name, address])) + '...')
        print('\n*BlueMuse will auto connect and stream when the device is found. \n*You can also use the BlueMuse interface to manage your stream(s).')
        muse.start()

#####

#####StartMuseStream

muses = list_muses()

if not muses:
    print('No Muses found')
else:
    stream(muses[0]['address'])

    # Note: Streaming is synchronous, so code here will not execute until the stream has been closed
    print('Stream has ended')
#####

#####utils
import os
import sys
from tempfile import gettempdir
from subprocess import call

import matplotlib.pyplot as plt
import numpy as np
from sklearn import svm
from scipy.signal import butter, lfilter, lfilter_zi


NOTCH_B, NOTCH_A = butter(4, np.array([55, 65]) / (256 / 2), btype='bandstop')


def epoch(data, samples_epoch, samples_overlap=0):
    """Extract epochs from a time series.

    Given a 2D array of the shape [n_samples, n_channels]
    Creates a 3D array of the shape [wlength_samples, n_channels, n_epochs]

    Args:
        data (numpy.ndarray or list of lists): data [n_samples, n_channels]
        samples_epoch (int): window length in samples
        samples_overlap (int): Overlap between windows in samples

    Returns:
        (numpy.ndarray): epoched data of shape
    """

    if isinstance(data, list):
        data = np.array(data)

    n_samples, n_channels = data.shape

    samples_shift = samples_epoch - samples_overlap

    n_epochs = int(
        np.floor((n_samples - samples_epoch) / float(samples_shift)) + 1)

    # Markers indicate where the epoch starts, and the epoch contains samples_epoch rows
    markers = np.asarray(range(0, n_epochs + 1)) * samples_shift
    markers = markers.astype(int)

    # Divide data in epochs
    epochs = np.zeros((samples_epoch, n_channels, n_epochs))

    for i in range(0, n_epochs):
        epochs[:, :, i] = data[markers[i]:markers[i] + samples_epoch, :]

    return epochs


def compute_band_powers(eegdata, fs):
    """Extract the features (band powers) from the EEG.

    Args:
        eegdata (numpy.ndarray): array of dimension [number of samples,
                number of channels]
        fs (float): sampling frequency of eegdata

    Returns:
        (numpy.ndarray): feature matrix of shape [number of feature points,
            number of different features]
    """
    # 1. Compute the PSD
    winSampleLength, nbCh = eegdata.shape

    # Apply Hamming window
    w = np.hamming(winSampleLength)
    dataWinCentered = eegdata - np.mean(eegdata, axis=0)  # Remove offset
    dataWinCenteredHam = (dataWinCentered.T * w).T

    NFFT = nextpow2(winSampleLength)
    Y = np.fft.fft(dataWinCenteredHam, n=NFFT, axis=0) / winSampleLength
    PSD = 2 * np.abs(Y[0:int(NFFT / 2), :])
    f = fs / 2 * np.linspace(0, 1, int(NFFT / 2))

    # SPECTRAL FEATURES
    # Average of band powers
    # Delta <4
    ind_delta, = np.where(f < 4)
    meanDelta = np.mean(PSD[ind_delta, :], axis=0)
    # Theta 4-8
    ind_theta, = np.where((f >= 4) & (f <= 8))
    meanTheta = np.mean(PSD[ind_theta, :], axis=0)
    # Alpha 8-12
    ind_alpha, = np.where((f >= 8) & (f <= 12))
    meanAlpha = np.mean(PSD[ind_alpha, :], axis=0)
    # Beta 12-30
    ind_beta, = np.where((f >= 12) & (f < 30))
    meanBeta = np.mean(PSD[ind_beta, :], axis=0)
    # Gamma >30
    ind_gamma, = np.where(f >= 30)
    meanGamma = np.mean(PSD[ind_gamma, :], axis=0)

    feature_vector = np.concatenate((meanDelta, meanTheta, meanAlpha,
                                     meanBeta,meanGamma), axis=0)

    feature_vector = np.log10(feature_vector)

    return feature_vector


def nextpow2(i):
    """
    Find the next power of 2 for number i
    """
    n = 1
    while n < i:
        n *= 2
    return n


def compute_feature_matrix(epochs, fs):
    """
    Call compute_feature_vector for each EEG epoch
    """
    n_epochs = epochs.shape[2]

    for i_epoch in range(n_epochs):
        if i_epoch == 0:
            feat = compute_band_powers(epochs[:, :, i_epoch], fs).T
            # Initialize feature_matrix
            feature_matrix = np.zeros((n_epochs, feat.shape[0]))

        feature_matrix[i_epoch, :] = compute_band_powers(
            epochs[:, :, i_epoch], fs).T

    return feature_matrix


def get_feature_names(ch_names):
    """Generate the name of the features.

    Args:
        ch_names (list): electrode names

    Returns:
        (list): feature names
    """
    bands = ['delta', 'theta', 'alpha', 'beta','gamma']

    feat_names = []
    for band in bands:
        for ch in range(len(ch_names)):
            feat_names.append(band + '-' + ch_names[ch])

    return feat_names


def update_buffer(data_buffer, new_data, notch=False, filter_state=None):
    """
    Concatenates "new_data" into "data_buffer", and returns an array with
    the same size as "data_buffer"
    """
    if new_data.ndim == 1:
        new_data = new_data.reshape(-1, data_buffer.shape[1])

    if notch:
        if filter_state is None:
            filter_state = np.tile(lfilter_zi(NOTCH_B, NOTCH_A),
                                   (data_buffer.shape[1], 1)).T
        new_data, filter_state = lfilter(NOTCH_B, NOTCH_A, new_data, axis=0,
                                         zi=filter_state)

    new_buffer = np.concatenate((data_buffer, new_data), axis=0)
    new_buffer = new_buffer[new_data.shape[0]:, :]

    return new_buffer, filter_state


def get_last_data(data_buffer, newest_samples):
    """
    Obtains from "buffer_array" the "newest samples" (N rows from the
    bottom of the buffer)
    """
    new_buffer = data_buffer[(data_buffer.shape[0] - newest_samples):, :]

    return new_buffer
#####

#####neurofeedback
import numpy as np  # Module that simplifies computations on matrices
import matplotlib.pyplot as plt  # Module used for plotting
from pylsl import StreamInlet, resolve_byprop  # Module to receive EEG data
import pandas as pd
# Handy little enum to make code more readable


class Band:
    Delta = 0
    Theta = 1
    Alpha = 2
    Beta = 3
    Gamma =4


""" EXPERIMENTAL PARAMETERS """
# Modify these to change aspects of the signal processing

# Length of the EEG data buffer (in seconds)
# This buffer will hold last n seconds of data and be used for calculations
BUFFER_LENGTH = 5

# Length of the epochs used to compute the FFT (in seconds)
EPOCH_LENGTH = 1

# Amount of overlap between two consecutive epochs (in seconds)
OVERLAP_LENGTH = 0.5

# Amount to 'shift' the start of each next consecutive epoch
SHIFT_LENGTH = EPOCH_LENGTH - OVERLAP_LENGTH

# Index of the channel(s) (electrodes) to be used
# 0 = left ear, 1 = left forehead, 2 = right forehead, 3 = right ear
INDEX_CHANNEL = [0]

if __name__ == "__main__":

    """ 1. CONNECT TO EEG STREAM """

    # Search for active LSL streams
    print('Looking for an EEG stream...')
    streams = resolve_byprop('type', 'EEG', timeout=2)
    if len(streams) == 0:
        raise RuntimeError('Can\'t find EEG stream.')

    # Set active EEG stream to inlet and apply time correction
    print("Start acquiring data")
    inlet = StreamInlet(streams[0], max_chunklen=12)
    eeg_time_correction = inlet.time_correction()

    # Get the stream info and description
    info = inlet.info()
    description = info.desc()

    # Get the sampling frequency
    # This is an important value that represents how many EEG data points are
    # collected in a second. This influences our frequency band calculation.
    # for the Muse 2016, this should always be 256
    fs = int(info.nominal_srate())

    """ 2. INITIALIZE BUFFERS """

    # Initialize raw EEG data buffer
    eeg_buffer = np.zeros((int(fs * BUFFER_LENGTH), 1))
    filter_state = None  # for use with the notch filter

    # Compute the number of epochs in "buffer_length"
    n_win_test = int(np.floor((BUFFER_LENGTH - EPOCH_LENGTH) /
                              SHIFT_LENGTH + 1))

    # Initialize the band power buffer (for plotting)
    # bands will be ordered: [delta, theta, alpha, beta,gamma]
    band_buffer = np.zeros((n_win_test, 5))

    """ 3. GET DATA """

    # The try/except structure allows to quit the while loop by aborting the
    # script with <Ctrl-C>
    print('Press Ctrl-C in the console to break the while loop.')
    eegdf = pd.DataFrame(columns=['Alpha', 'Beta','Gamma','Delta','Theta']) #'time', 

    try:
        # The following loop acquires data, computes band powers, and calculates neurofeedback metrics based on those band powers
        while True:

            """ 3.1 ACQUIRE DATA """
            # Obtain EEG data from the LSL stream
            eeg_data, timestamp = inlet.pull_chunk(
                timeout=1, max_samples=int(SHIFT_LENGTH * fs))

            # Only keep the channel we're interested in
            ch_data = np.array(eeg_data)[:, INDEX_CHANNEL]

            # Update EEG buffer with the new data
            eeg_buffer, filter_state = update_buffer(
                eeg_buffer, ch_data, notch=True,
                filter_state=filter_state)

            """ 3.2 COMPUTE BAND POWERS """
            # Get newest samples from the buffer
            data_epoch = get_last_data(eeg_buffer,
                                             EPOCH_LENGTH * fs)

            # Compute band powers
            band_powers = compute_band_powers(data_epoch, fs)
            band_buffer, _ = update_buffer(band_buffer,
                                                 np.asarray([band_powers]))
            # Compute the average band powers for all epochs in buffer
            # This helps to smooth out noise
            smooth_band_powers = np.mean(band_buffer, axis=0)

            # print('Delta: ', band_powers[Band.Delta], ' Theta: ', band_powers[Band.Theta],
            #       ' Alpha: ', band_powers[Band.Alpha], ' Beta: ', band_powers[Band.Beta])

            """ 3.3 COMPUTE NEUROFEEDBACK METRICS """
            # These metrics could also be used to drive brain-computer interface

            #eegdf = eegdf.append({'Alpha':smooth_band_powers[Band.Alpha], 'Beta':smooth_band_powers[Band.Beta],'Gamma':smooth_band_powers[Band.Gamma],'Delta':smooth_band_powers[Band.Delta],'Theta':smooth_band_powers[Band.Theta]}, ignore_index=True)
            #print('Alpha'+str(smooth_band_powers[Band.Alpha])+" "+ 'Beta'+str(smooth_band_powers[Band.Beta])+" "+ 'Gamma'+str(smooth_band_powers[Band.Gamma])+" "+ 'Delta'+str(smooth_band_powers[Band.Delta])+" "+ 'Theta'+str(smooth_band_powers[Band.Theta]))
            #print('Delta'+str(smooth_band_powers[Band.Delta])+" "+ 'Theta'+str(smooth_band_powers[Band.Theta]))
            #if smooth_band_powers[Band.Theta]>0.7:
            if smooth_band_powers[Band.Delta]>0.95:
                print(1)
    
            else:
                print(0)
            #blink simul theta above .6-7
    except KeyboardInterrupt:
        #print(eegdf)
        print('Closing!')


