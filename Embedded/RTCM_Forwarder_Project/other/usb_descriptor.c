#include "hal.h"
#include "usb_descriptor.h"

// USB device descriptor
static const uint8_t device_descriptor_data[] = {
    USB_DESC_DEVICE(0x0200,    // bcdUSB
                    0xEF,      // bDeviceClass (Misc)
                    0x02,      // bDeviceSubClass (Common Class)
                    0x01,      // bDeviceProtocol (IAD)
                    64,        // bMaxPacketSize
                    0x0483,    // idVendor (STMicroelectronics)
                    0x5740,    // idProduct (Composite device)
                    0x0200,    // bcdDevice
                    1,         // iManufacturer
                    2,         // iProduct
                    3,         // iSerialNumber
                    1)         // bNumConfigurations
};

// USB configuration descriptor (with CDC-ACM + CDC-ECM)
static const uint8_t config_descriptor_data[] = {
    // Configuration descriptor
    USB_DESC_CONFIGURATION(98,  // wTotalLength (sum of all descriptors)
                           3,   // bNumInterfaces
                           1,   // bConfigurationValue
                           0,   // iConfiguration
                           0xC0,// bmAttributes (self-powered)
                           50), // bMaxPower (100 mA)

    // ----- Interface Association Descriptor (IAD) for CDC ACM -----
    USB_DESC_INTERFACE_ASSOCIATION(0, 2, 0x02, 0x02, 0x01, 0),

    // ----- CDC ACM -----
    USB_DESC_INTERFACE(0, 0, 1, 0x02, 0x02, 0x01, 0),
    USB_DESC_CS_INTERFACE(0x24, 0x00, 0x10, 0x01),
    USB_DESC_CS_INTERFACE(0x24, 0x02, 0x02, 0x01),
    USB_DESC_CS_INTERFACE(0x24, 0x06, 0, 1),
    USB_DESC_ENDPOINT(0x82, 0x03, 8, 0xFF),

    USB_DESC_INTERFACE(0, 1, 2, 0x0A, 0x00, 0x00, 0),
    USB_DESC_ENDPOINT(0x01, 0x02, 64, 0),
    USB_DESC_ENDPOINT(0x81, 0x02, 64, 0),

    // ----- CDC ECM (Ethernet) -----
    USB_DESC_INTERFACE(2, 0, 1, 0x02, 0x06, 0x00, 0),
    USB_DESC_CS_INTERFACE(0x24, 0x00, 0x10, 0x01),
    USB_DESC_CS_INTERFACE(0x24, 0x06, 0, 3),
    USB_DESC_CS_INTERFACE(0x24, 0x0F, 0x02, 0x83),
    USB_DESC_CS_INTERFACE(0x24, 0x0F, 0x02, 0x83),
    USB_DESC_ENDPOINT(0x83, 0x03, 8, 0xFF),

    USB_DESC_INTERFACE(2, 1, 2, 0x0A, 0x00, 0x00, 0),
    USB_DESC_ENDPOINT(0x02, 0x02, 64, 0),
    USB_DESC_ENDPOINT(0x84, 0x02, 64, 0)
};

// USB string descriptor
static const uint8_t langid_descriptor_data[] = {
    USB_DESC_BYTE(4),           // bLength
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
    USB_DESC_WORD(0x0409)       // US English
};

static const uint8_t manufacturer_string[] = {
    USB_DESC_BYTE(16), USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
    'C', 0, 'h', 0, 'i', 0, 'b', 0, 'i', 0, 'O', 0, 'S', 0
};

static const uint8_t product_string[] = {
    USB_DESC_BYTE(26), USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
    'R', 0, 'T', 0, 'C', 0, 'M', 0, ' ', 0,
    'F', 0, 'o', 0, 'r', 0, 'w', 0, 'a', 0, 'r', 0, 'd', 0
};

static const uint8_t serial_string[] = {
    USB_DESC_BYTE(8), USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
    '0', 0, '1', 0, '0', 0
};

const USBDescriptor device_descriptor = {
    sizeof device_descriptor_data, device_descriptor_data
};

const USBDescriptor config_descriptor = {
    sizeof config_descriptor_data, config_descriptor_data
};

const USBDescriptor string0_descriptor = {
    sizeof langid_descriptor_data, langid_descriptor_data
};

const USBDescriptor string1_descriptor = {
    sizeof manufacturer_string, manufacturer_string
};

const USBDescriptor string2_descriptor = {
    sizeof product_string, product_string
};

const USBDescriptor string3_descriptor = {
    sizeof serial_string, serial_string
};

// Main descriptor retrieval callback
const USBDescriptor *get_usb_descriptor(USBDriver *usbp,
                                        uint8_t dtype,
                                        uint8_t dindex,
                                        uint16_t lang) {
    (void)usbp;
    (void)lang;
    switch (dtype) {
    case USB_DESCRIPTOR_DEVICE:
        return &device_descriptor;
    case USB_DESCRIPTOR_CONFIGURATION:
        return &config_descriptor;
    case USB_DESCRIPTOR_STRING:
        switch (dindex) {
        case 0: return &string0_descriptor;
        case 1: return &string1_descriptor;
        case 2: return &string2_descriptor;
        case 3: return &string3_descriptor;
        default: return NULL;
        }
    default:
        return NULL;
    }
}
