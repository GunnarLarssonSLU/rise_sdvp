#pragma once

#include "hal.h"

extern const USBDescriptor *get_usb_descriptor(USBDriver *usbp,
                                               uint8_t dtype,
                                               uint8_t dindex,
                                               uint16_t lang);
