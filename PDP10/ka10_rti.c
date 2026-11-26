/* ka10_rti.c: BBN real-time clock for TENEX.

   TENEX 1.34 for BBN's own machines (PARAMS: RTICLK, RTISW) reads the date
   and time from a calendar clock on the I/O bus when it has not been told
   them, and has a JSYS that reads a microsecond interval timer:

        DATAI 600,B     ; GTAD: days since 17 Nov 1858,,seconds since
                        ; midnight GMT, the format of the GTAD JSYS
        DATAI 610,1     ; USEC: microsecond count

   No description of the hardware was available; this is those two
   instructions as the monitor uses them (LOGJS.MAC), nothing more.  The
   BACK register is a number of days subtracted from the date, so that a
   system can live in its own year:  DEPOSIT RTI BACK 18628  makes 2026
   into 1975 (51 years; the weekdays do not agree).
*/

#include "kx10_defs.h"
#include <time.h>

#ifndef NUM_DEVS_RTI
#define NUM_DEVS_RTI 0
#endif

#if (NUM_DEVS_RTI > 0)

#define RTI_DEVNUM 0600

t_stat         rti_devio(uint32 dev, uint64 *data);
const char    *rti_description (DEVICE *dptr);

static int32   rti_days = 0;

UNIT rti_unit[] = {
    {UDATA(NULL, UNIT_DISABLE, 0)},
};
DIB rti_dib = {RTI_DEVNUM, 3, &rti_devio, NULL};
REG rti_reg[] = {
    { DRDATAD (BACK, rti_days, 31, "days subtracted from the date"), PV_LEFT },
    { 0 }
};
DEVICE rti_dev = {
    "RTI", rti_unit, rti_reg, NULL, 1, 8, 0, 1, 8, 36, NULL, NULL, NULL, NULL, NULL, NULL,
    &rti_dib, DEV_DISABLE | DEV_DIS | DEV_DEBUG, 0, NULL, NULL, NULL, NULL, NULL, NULL, &rti_description
};

t_stat rti_devio(uint32 dev, uint64 *data)
{
    struct timespec now;
    uint64          days, secs;

    if ((dev & 3) != DATAI) {
        if ((dev & 3) == CONI)
            *data = 0;
        return SCPE_OK;
    }
    sim_rtcn_get_time (&now, 0);
    if ((dev & 0774) == RTI_DEVNUM + 010) {         /* 610: microseconds */
        *data = (((uint64)now.tv_sec * 1000000) + (uint64)(now.tv_nsec / 1000)) & FMASK;
        return SCPE_OK;
    }
    /* 600: calendar clock.  1 Jan 1970 is day 40587. */
    days = (uint64)((long long)(now.tv_sec / 86400) + 40587 - (long long)rti_days);
    secs = (uint64)(now.tv_sec % 86400);
    *data = ((days & RMASK) << 18) | secs;
    return SCPE_OK;
}

const char *rti_description (DEVICE *dptr)
{
    return "BBN real-time clock (calendar clock and microsecond timer)";
}
#endif
