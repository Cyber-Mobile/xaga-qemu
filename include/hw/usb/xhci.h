#ifndef HW_USB_XHCI_H
#define HW_USB_XHCI_H

#define TYPE_XHCI "base-xhci"
#define TYPE_NEC_XHCI "nec-usb-xhci"
#define TYPE_QEMU_XHCI "qemu-xhci"
#define TYPE_XHCI_SYSBUS "sysbus-xhci"

#define XHCI_MAXPORTS_2 15
#define XHCI_MAXPORTS_3 15

#define XHCI_MAXPORTS (XHCI_MAXPORTS_2 + XHCI_MAXPORTS_3)
#define XHCI_MAXSLOTS 64
#define XHCI_MAXINTRS 16

/*
 * MediaTek xHCI windows are 4 KiB (device tree reg = 0x1000) and the guest
 * ioremaps exactly that much, so the runtime and doorbell register sets have
 * to live inside the first page.  hcd-xhci.c is laid out for that (RTSOFF
 * 0x800, DBOFF 0xC00; see OFF_RUNTIME/OFF_DOORBELL there) and reports those
 * offsets to the guest through the capability registers.
 *
 * must be power of 2
 */
#define XHCI_LEN_REGS 0x1000

void xhci_sysbus_build_aml(Aml *scope, uint32_t mmio, unsigned int irq);

#endif
