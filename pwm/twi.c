/* This file is derived from twitest.c, which carries the license reproduced
 * below:
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <joerg@FreeBSD.ORG> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Joerg Wunsch
 * ----------------------------------------------------------------------------
 */

#include "twi.h"
#include "global.h"  // for F_CPU
#include <compat/twi.h>

/*
 * Saved TWI status register, for error messages only.  We need to
 * save it in a variable, since the datasheet only guarantees the TWSR
 * register to have valid contents while the TWINT bit in TWCR is set.
 */
uint8_t twst;

void twi_init(void)
{
    #if F_CPU < 3600000UL
        TWBR = 10;
    #else
        TWBR = (F_CPU / 100000UL - 16) / 2;
    #endif
}

/* Read len bytes from TWI address addr into buf. */
/* Returns number of bytes read, or -1 on error. */
int twi_read(uint8_t addr, uint8_t* buf, int len)
{
    uint8_t twcr;
    int rv = 0;  /* return value */
    
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);   /* send start condition */
    while(!(TWCR & _BV(TWINT)));  /* wait for transmission */
    switch((twst = TW_STATUS)) {
        case TW_START:
        case TW_REP_START:
            break;
            
        case TW_MT_ARB_LOST:
            goto arb_lost;
            
        default:
            goto error;
    }
    
    TWDR = (addr << 1) | TW_READ;
    TWCR = _BV(TWINT) | _BV(TWEN);   /* start transmission */
    while(!(TWCR & _BV(TWINT)));     /* wait for transmission */

    switch((twst = TW_STATUS)) {
        case TW_MR_SLA_ACK:
            break;
            
        case TW_MR_SLA_NACK:
            goto quit;
            
        case TW_MR_ARB_LOST:
            goto arb_lost;
            
        default:
            goto error;
    }
    
    twcr = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
    while(len > 0) {
        if(len == 1)
            twcr = _BV(TWINT) | _BV(TWEN);  /* send NAK this time */
        TWCR = twcr;
        while(!(TWCR & _BV(TWINT)));  /* wait for transmission */
        switch((twst = TW_STATUS)) {
            case TW_MR_DATA_NACK:
                len = 1;   /* premature end of loop */
                /* FALLTHROUGH */
            case TW_MR_DATA_ACK:
                *buf++ = TWDR;
                rv++;
                break;
            
            default:
                goto error;
        }
        --len;
    }
    
    quit:
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);  /* send stop condition */
    
    return rv;
    
    error:
    rv = -1;
    goto quit;
    
    arb_lost: /* arbitration lost: presently not handled */
    TWCR = _BV(TWINT);  /* release TWI bus */
    return -1;  /* do not send stop condition after arbitration lost */
}
