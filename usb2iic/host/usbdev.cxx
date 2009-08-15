#include <iostream>
#include <sstream>
#include <stdint.h>
#include <usb.h>

class USBDev
{
  public:
    USBDev();
    ~USBDev();
    inline bool Failed() const { return fFail; }
    inline operator bool() const { return !fFail; }
    void SetOutput(uint8_t data);
    
  private:
    usb_dev_handle* fHandle;
    bool fFail, fOpen;
    
    static const int VENDOR_ID = 0x04b4;
    static const int PRODUCT_ID = 0x8613;
    // static const int VENDOR_ID = 0xfffe;
    // static const int PRODUCT_ID = 0x0002;
    static const int USB_TIMEOUT = 1000; // ms
};

// Open the connection to the USB device
USBDev::USBDev()
{
  struct usb_bus *bus;
  struct usb_bus *busses;
  struct usb_device *dev;
  struct usb_device *tDev = NULL;
  
  fFail = false;
  fOpen = false;
  
  // Init USB subsystem
  usb_init();
  usb_find_busses();
  usb_find_devices();
  
  /* Find the first USBMCA device */
  busses = usb_get_busses();
  
  for(bus = busses; bus; bus = bus->next) {
	for(dev = bus->devices; dev; dev = dev->next) {
	  if(dev->descriptor.idVendor == VENDOR_ID &&
		 dev->descriptor.idProduct == PRODUCT_ID) {
		tDev = dev;
	  }
	}	
  }
  
  if(!tDev) {
    fFail = true;
    return;
  }
  
  // Open the USB device
  fHandle = usb_open(tDev);
  // usb_claim_interface(fHandle, 0);
  // usb_set_altinterface(fHandle, 1);
  
  fOpen = true;
}

// Close the connection to the USB device
USBDev::~USBDev()
{
  if(!fOpen)
    return;
  
  if(usb_close(fHandle) < 0) {
    std::cerr << "Error while closing USB device." << std::endl;
    fFail = true;
  }
}

void USBDev::SetOutput(uint8_t data)
{
  if(fFail)
    return;
  
  if(usb_control_msg(fHandle, 0x40, 0x80, data, 0, NULL, 0, USB_TIMEOUT) != 0)
    fFail = true;
}

int main(int argc, char** argv)
{
  if(argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <value>" << std::endl;
    return -1;
  }
  
  std::istringstream args(argv[1]);
  int value;
  args >> value;
  if(!args) {
    std::cerr << "Error: Failed to parse value." << std::endl;
    return 1;
  }

  USBDev dev;
  if(!dev) {
    std::cerr << "Error: Failed to open USB device." << std::endl;
    return 2;
  }
  
  dev.SetOutput(value);
  if(!dev) {
    std::cerr << "Error: failed to write value." << std::endl;
    return 3;
  }
}

