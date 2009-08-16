#include <fx2regs.h>
#include <delay.h>
#include <usb_common.h>
#include <isr.h>
#include <fx2utils.h>

// Read TRM p.15-115 for an explanation on this. 
// A single nop is sufficient for default setup but like that we're on 
// the safe side. 
#define	SYNCDELAY	_asm  nop; nop; nop; nop;  _endasm

#define bRequestType    SETUPDAT[0]
#define bRequest        SETUPDAT[1]
#define wValueL         SETUPDAT[2]
#define wValueH         SETUPDAT[3]
#define wIndexL         SETUPDAT[4]
#define wIndexH         SETUPDAT[5]
#define wLengthL        SETUPDAT[6]
#define wLengthH        SETUPDAT[7]

#define VRT_VENDOR_IN   0xC0
#define VRT_VENDOR_OUT  0x40
#define VRQ_TEST        0x80

void init_IO(void)
{
	OEA = 0xff;   // port A is output
	IOA = 0x00;
	
	OEB = 0x00;   // port B is input
}

static void isr_EP0OUT (void) interrupt
{
	IOA = EP0BUF[0];
	clear_usb_irq();
	EPIRQ = 0x01;
}

unsigned char app_vendor_cmd(void)
{
	if(bRequestType == VRT_VENDOR_OUT) {
		if(bRequest == VRQ_TEST) {
			// IOA = wValueL;
			EP0BCL = 0;
		} else {
			return 0;
		}
	} else {
		return 0;
	}
	return 1;
}

void main(void)
{
	CPUCS = bmCLKSPD1;
	init_IO();
	
	EA = 0;  // disable interrupts
	
	setup_autovectors();
	usb_install_handlers();
	
	hook_uv(UV_EP0OUT, (unsigned short) isr_EP0OUT);
	EPIE = 0x02;
	
	EA = 1;
	
	fx2_renumerate();
	
	while(1) {
		if(usb_setup_packet_avail())
			usb_handle_setup_packet();
	}
}
