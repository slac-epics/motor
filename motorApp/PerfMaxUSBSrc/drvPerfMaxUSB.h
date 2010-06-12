/* File: drvpmaxUSB.h             */


/* Device Driver Support definitions for pmaxUSB MoCo dc motor controller. */
/*
 *      Original Author: Kurt Goetze
 *      Current Author: Kurt Goetze
 *      Date: 11/24/2003
 *
 * Modification Log:
 * -----------------
 * .00  11/24/2003  kag  initialized from drvMCB4B.h
 * .01  07/12/2004  rls  Converted from MPF to asyn.
 */

#ifndef	INCdrvpmaxUSBh
#define	INCdrvpmaxUSBh 1

#include "motor.h"
#include "motordrvCom.h"
#include <semaphore.h>
#include "usb.h"
#include "ArcusPerformaxDriver.h"

/* pmaxUSB default profile. */

#define PMAXUSB_NUM_CARDS	16
#define PMAXUSB_NUM_AXIS	16
#define	PMAXUSB_VENDORID	0x1589
#define PMAXUSB_PRODUCTID	0xa101
#define	PMAXUSB_EPIN		0x82
#define PMAXUSB_EPOUT		0x02
#define CTLA               1

enum pmaxusb_cb_type 
{

    PMAXUSB_CB_TX_NORMAL,
    PMAXUSB_CB_RX_NORMAL
};

struct pmaxusb_cbdata
{
    int type;
};

struct pmaxusb 
{
    AR_HANDLE handle;
    int found;
};

struct perfMaxUSBController
{
    char name[80];     	/* asyn port name */
    struct pmaxusb pmaxUSB;
};

/* Function prototypes. */
extern RTN_STATUS PerfMaxUSBSetup(int, int, int);
extern RTN_STATUS PerfMaxUSBConfig(int, const char *);
extern RTN_STATUS PerfMaxUSBShow(const char *);
extern RTN_STATUS PerfMaxUSBPolSet(int, int);

#endif	/* INCdrvpmaxUSBh */
