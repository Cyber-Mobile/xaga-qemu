"""Host-side tooling for the MT6895 QEMU virtual UDC (fastboot/adb).

Modules
-------
vusb          framed chardev transport + USB enumeration/control/bulk helpers
mock_device   pure-Python fake mtu3 UDC device (fastboot and adb variants)
fastboot      fastboot protocol client built on vusb
adb           adb (USB ffs) protocol skeleton built on vusb

Everything here uses the Python standard library only.
"""
