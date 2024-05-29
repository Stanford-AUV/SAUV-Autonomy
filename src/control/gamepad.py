#!/usr/bin/python
#
# Simple class for the Logitech F310 Gamepad.
# Needs libusb1 library.
#

import usb1
import struct

USB_VENDOR = 0x046D
USB_PRODUCT = 0xC21D
default_state = (0, 20, 0, 0, 0, 0, 123, 251, 128, 0, 128, 0, 128, 0, 0, 0, 0, 0, 0, 0)


class Gamepad(object):

    def __init__(self, serial=None):
        """Initialize the gamepad.

        Args:
            serial: Serial number of the gamepad. If None, the first gamepad found is used.
        """
        self.is_initialized = False
        self.context = usb1.USBContext()
        handle = None

        for device in self.context.getDeviceList():
            if (
                device.getVendorID() == USB_VENDOR
                and device.getProductID() == USB_PRODUCT
            ):
                if serial is not None and device.getSerialNumber() != serial:
                    continue
                handle = device.open()
                break

        if handle:
            self._handle = handle
            try:
                self._handle.detachKernelDriver(0)
            except usb1.USBError:
                print("Error detaching kernel driver (usually no problem)")
            except AttributeError:
                pass

            self._handle.claimInterface(0)

            # This value has to be sent to the gamepad, or it won't start working
            # Value was determined by sniffing the USB traffic with Wireshark
            # Getting other gamepads to work might be as simple as changing this
            self._handle.interruptWrite(0x02, struct.pack("<BBB", 0x01, 0x03, 0x04))
            self.changed = False
            self._state = default_state
            self._old_state = default_state
            self.is_initialized = True
            print("Gamepad initialized")
        else:
            if serial is not None:
                raise RuntimeError(f"Device with serial number '{serial}' not found")
            raise RuntimeError("Could not initialize Gamepad")

    def _getState(self, timeout):
        try:
            data = self._handle.interruptRead(0x81, 0x20, timeout=timeout)
            data = struct.unpack("<" + "B" * 20, data)
            return data
        except usb1.USBError as e:
            return None

    def read_gamepad(self, timeout=200):
        state = self._getState(timeout=timeout)
        self.changed = state is not None
        if self.changed:
            self._old_state = self._state
            self._state = state

    def X_was_released(self):
        return self.changed and (self._state[3] != 64) & (self._old_state[3] == 64)

    def Y_was_released(self):
        return self.changed and (self._state[3] != 128) & (self._old_state[3] == 128)

    def A_was_released(self):
        return self.changed and (self._state[3] != 16) & (self._old_state[3] == 16)

    def B_was_released(self):
        return self.changed and (self._state[3] != 32) & (self._old_state[3] == 32)

    def get_state(self):
        return self._state[:]

    def get_LB(self):
        return self._state[3] == 1

    def get_RB(self):
        return self._state[3] == 2

    def get_A(self):
        return self._state[3] == 16

    def get_B(self):
        return self._state[3] == 32

    def get_X(self):
        return self._state[3] == 64

    def get_Y(self):
        return self._state[3] == 128

    def get_analogR_x(self):
        return self._state[10]

    def get_analogR_y(self):
        return self._state[12]

    def get_analogL_x(self):
        return self._state[6]

    def get_analogL_y(self):
        return self._state[8]

    def get_dir_up(self):
        return self._state[2] in (1, 5, 9)

    def get_dir_down(self):
        return self._state[2] in (2, 6, 10)

    def get_dir_left(self):
        return self._state[2] in (4, 5, 6)

    def get_dir_right(self):
        return self._state[2] in (8, 9, 10)

    def __del__(self):
        if self.is_initialized:
            self._handle.releaseInterface(0)
            self._handle.reset()


# Unit test code
if __name__ == "__main__":
    pad = None

    pad = Gamepad()
    while True:
        pad.read_gamepad()
        if pad.changed:
            print(pad._state)
