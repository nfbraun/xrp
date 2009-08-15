#include <usb.h>
#include <iostream>

class USBDev
{
  public:
    USBDev();
    ~USBDev();
    inline bool Failed() const { return fFail; }
    
  private:
    usb_dev_handle* fHandle;
    bool fFail;
    
    static const int VENDOR_ID = 0x04b4;
    static const int PRODUCT_ID = 0x8613;
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
  
  // Open the USBMCA device
  fHandle = usb_open(tDev);
  usb_claim_interface(fHandle, 0);
  usb_set_altinterface(fHandle, 1);
}

// Close the connection to the USB device
USBDev::~USBDev()
{
  if(fFail)
    return;
  
  if(usb_close(fHandle) < 0) {
    std::cerr << "Error while closing USB device." << std::endl;
    fFail = true;
  }
}

int main()
{
  USBDev dev;
  if(dev.Failed()) {
    std::cerr << "Error: Failed to open USB device." << std::endl;
    return -1;
  }
}

