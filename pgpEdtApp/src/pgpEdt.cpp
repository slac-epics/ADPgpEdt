/* pgpEdt.cpp
 *
 * This is a driver for the SLAC CameraLink frame grabber
 *
 * Author:  Dehong Zhang
 *          SLAC National Accelerator Laboratory
 *
 * Created: Sept 20, 2016
 */

#include <algorithm>
#include <unistd.h>
#include <iostream>
#include <iomanip>

#include <fcntl.h>

#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <poll.h>

#include "dbAccess.h"

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <iocsh.h>

#include "ADDriver.h"
#include <epicsExport.h>

#include "PgpCardG3_CLinkMod.h"
#include "PgpCardG3_CLinkWrap.h"

#include "pgpEdtSerial.h"

static const char *driverName = "pgpEdt";

typedef enum
{
    pgpEdtPack_24,
    pgpEdtPack_16
} pgpEdtPack_t;

typedef enum
{
    pgpEdtVOut_TD,
    pgpEdtVOut_TnB
} pgpEdtVOut_t;

typedef enum
{
    pgpEdtExpo_Full,
    pgpEdtExpo_ROI
} pgpEdtExpo_t;

typedef enum
{
    pgpEdtTrig_Free,
    pgpEdtTrig_EVR
} pgpEdtTrig_t;

/* Bit map of CSR */
typedef union
{
    epicsUInt32 All;
    struct
    {
        unsigned int TxReset     : 1; // MGT Tx reset
        unsigned int RxReset     : 1; // MGT Rx reset
        unsigned int CntReset    : 1; // counter reset
        unsigned int Pack16      : 1; // pack 16 or 24
        unsigned int TrgCC       : 2; // CC line to trigger
        unsigned int TrgPolarity : 1; // trigger polarity
        unsigned int Enable      : 1; // enable
        unsigned int NA          :21; // not used
        unsigned int CamLock     : 1; // camera clock locked or not
        unsigned int ExtLink     : 1; // fiber to RCX
        unsigned int EvrLink     : 1; // fiber from EVG
    } Bits;
} csr_word;



class pgpEdt : public ADDriver {
public:
    pgpEdt( const char *portName, int board, int chan,
            int maxSizeX, int maxSizeY, int numBits, NDDataType_t dataType,
            int maxBuffers, size_t maxMemory, int priority, int stackSize );

    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32  value );
    virtual asynStatus writeOctet( asynUser *pasynUser, const char *value,
                                   size_t maxChars, size_t *nActual );
    virtual void report( FILE *fp, int details );

    void acqTask();     // Should be private, but public so can be called from C

protected:

    int  baud;                                               // serial baud rate
    int  ssus;                                 // usleep between serial commands
    int  nbit;                                            // # of bits per pixel
    int  pack;                                     // MGT pack: 0 -> 24; 1 -> 16
    int  vOut;           // video output order: 0 -> Top Down; 1 -> Top & Bottom
    int  skipRow;                                                   // skip rows
    int  skipCol;                                                // skip columns
    int  fullRow;                                   // full frame number of rows
    int  fullCol;                                // full frame number of columns
    int  numTrains;                                      // number of MGT trains
    int  numCycles;                                      // number of MGT cycles
    int  cc;                            // cameralink trigger line (1 through 4)
    int  polarity;                                           // trigger polarity
    int  prescale;                             // EVR prescale: 0 -> 119; 1 -> 1
    int  code;                                               // EVR trigger code
    int  delay;                                       // invariant trigger delay
    int  width;                                           // trigger pulse width
    int  trg2frame;                        // cycles from trigger to frame valid
    int  frameRate;                                                // frame rate
    int  csta;                                                    // status word
    int  cmdInit;                                 // commands for initialization
    int  cmdFull;                                     // commands for full frame
    int  cmdROI;                                          // commands for HW ROI
    int  cmdEVR;                                     // commands for EVR trigger
    int  cmdFree;                                       // commands for free run
    int  cmdTPtn;                                   // commands for test pattern
    int  cmdAny;                                                  // any command
    int  resp;                                                       // response
    int  exposure;                                       // full frame or HW ROI
    int  tstPtn;                                        // test patternon or off
    int  reIni;                                                 // re-initialize

    #define FIRST_CLCAM_PARAM baud
    #define LAST_CLCAM_PARAM  reIni

private:
    /* These are the methods that are new to this class */
    long ser_send_recv   ( asynUser *pasynUser, const char *cmds, char *reply,
                                                                   int pass=0 );
    long set_register    ( asynUser *pasynUser, epicsUInt32 rAddr,
                                                epicsUInt32 rVal,  int pass=0 );
    long set_csr         ( asynUser *pasynUser, int acq,           int pass=0 );

    long update_nrow_ncol( asynUser *pasynUser,                    int pass=0 );
    long update_ntrn_ncyc( asynUser *pasynUser,                    int pass=0 );
    long init_camera     ();

    /* Our data */
    NDArray *pRaw;

    char     dev0[80];
    int      pdev0;
    int      pdev;

    int      channel;                                          // channel number
};

#define baudString       "baud"
#define ssusString       "ssus"
#define nbitString       "nbit"
#define packString       "pack"
#define vOutString       "vOut"
#define skipRowString    "skipRow"
#define skipColString    "skipCol"
#define fullRowString    "fullRow"
#define fullColString    "fullCol"
#define numTrainsString  "numTrains"
#define numCyclesString  "numCycles"
#define ccString         "cc"
#define polarityString   "polarity"
#define prescaleString   "prescale"
#define codeString       "code"
#define delayString      "delay"
#define widthString      "width"
#define trg2frameString  "trg2frame"
#define frameRateString  "frameRate"
#define cstaString       "csta"
#define cmdInitString    "cmdInit"
#define cmdFullString    "cmdFull"
#define cmdROIString     "cmdROI"
#define cmdEVRString     "cmdEVR"
#define cmdFreeString    "cmdFree"
#define cmdTPtnString    "cmdTPtn"
#define cmdAnyString     "cmdAny"
#define respString       "resp"
#define exposureString   "exposure"
#define tstPtnString     "tstPtn"
#define reIniString      "reIni"

#define NUM_CLCAM_PARAMS ((int)(&LAST_CLCAM_PARAM - &FIRST_CLCAM_PARAM + 1))


long pgpEdt::ser_send_recv( asynUser *pasynUser, const char *cmds,
                                                 char *reply,      int pass )
{
    int          ssusVal, ci, ti = 0, eosSize=0;
    char         sertfg[4], aval, eos[3], cmdsWithEos[90];
    epicsUInt32  rAddr;

    long         status = 1;

    if ( pass < 2 )
    {
        if ( (pass == 0) && (strlen(cmds) == 0) ) return( 0 );

        while ( 1 )
        {
            pdev0 = open( dev0, O_RDWR );
            if ( pdev0 != -1 ) break;

            if ( ti < 150 )
            {
                usleep( 200000 );
                ti++;
            }
            else
                break;
        }
    }
    getInputEosOctet(pasynUser, eos, 3, &eosSize);
    strcpy(cmdsWithEos, cmds);
    strncat(cmdsWithEos, eos, eosSize);

    if ( pdev0 == -1 ) return( -1 );

    // Check at 'cmds' as 'cmdsWithEos' now has more chars.
    if ( strlen(cmds) == 0 ) goto finished;

    getIntegerParam( ssus, &ssusVal );

    rAddr = SerSend_Addr + 4*channel;

    for ( ci=0; ci<=strlen(cmdsWithEos); ci++ )
    {
        if ( (ci < strlen(cmdsWithEos)) && (*(cmdsWithEos + ci) != 0x5F) )
            aval = *(cmdsWithEos + ci);
        else
            aval = 0x0D;

        status = pgpcard_setReg( pdev0, rAddr, aval );
        if ( status != 0 )
        {
            asynPrint( pasynUser, ASYN_TRACE_ERROR,
                       "%s:%s, ser_send_recv: failed to send \"%s\"\n",
                       driverName, portName, cmdsWithEos );
            break;
        }

        if ( (ci < strlen(cmdsWithEos)) && (*(cmdsWithEos + ci) == 0x5F) ) usleep( ssusVal );
    }

    if ( status == 0 )
    {
        usleep( 100000 );

        rAddr = SerReply_Addr + 4*channel;

        ti    = 0;
        ci    = 0;
        while ( ti < 200 )
        {
            status = pgpcard_readReg( pdev0, rAddr, (epicsUInt32 *)sertfg );

            if ( sertfg[0] != 0xffffffff )
            {
                if      ( sertfg[0] == 0x06 )
                {
                    sprintf( reply+ci, "ACK" );
                    ci += 3;
                }
                else if ( sertfg[0] == 0x15 )
                {
                    sprintf( reply+ci, "NAK" );
                    ci += 3;
                }
                else if ( sertfg[0] == 0x0d )
                {
                    *(reply+ci) = 0x5F;
                    ci++;
                }
                else
                {
                    *(reply+ci) = sertfg[0];
                    ci++;
                }
            }
            else if ( ci > 0 )
                break;

            usleep( 1000 );
            ti++;
        }

        if ( (ci > 0) && (*(reply+ci-1) == 0x5F) )
            *(reply+ci-1) = '\0';
        else
            *(reply+ci  ) = '\0';
    }

    finished:
    if ( (pass == 0) || (pass > 2) )
    {
        close( pdev0 );

        pdev0 = -1;
    }

    return( status );
}

long pgpEdt::set_register( asynUser *pasynUser, epicsUInt32 rAddr,
                                                epicsUInt32 rVal,  int pass )
{
    int ti = 0;

    if ( pass < 2 )
    {
        while ( 1 )
        {
            pdev0 = open( dev0, O_RDWR );
            if ( pdev0 != -1 ) break;

            if ( ti < 150 )
            {
                usleep( 200000 );
                ti++;
            }
            else
                break;
        }
    }

    if ( pdev0 == -1 ) return( -1 );

    pgpcard_setReg( pdev0, rAddr, rVal );

    if ( (pass == 0) || (pass > 2) )
    {
        close( pdev0 );

        pdev0 = -1;
    }

    return( 0 );
}

long pgpEdt::set_csr( asynUser *pasynUser, int acq, int pass )
{
    int       packVal, ccVal, tpolVal, ti = 0;
    csr_word  csr;

    if ( pass < 2 )
    {
        while ( 1 )
        {
            pdev0 = open( dev0, O_RDWR );
            if ( pdev0 != -1 ) break;

            if ( ti < 150 )
            {
                usleep( 200000 );
                ti++;
            }
            else
                break;
        }
    }

    if ( pdev0 == -1 ) return( -1 );

    getIntegerParam( polarity, &tpolVal );
    getIntegerParam( cc,       &ccVal   );
    getIntegerParam( pack,     &packVal );

    csr.All              = 0;

    csr.Bits.Enable      = acq;
    csr.Bits.TrgPolarity = tpolVal;
    csr.Bits.TrgCC       = ccVal;
    csr.Bits.Pack16      = packVal;

    pgpcard_setReg( pdev0, GrbCSR_Addr+4*channel, csr.All );

    if ( (pass == 0) || (pass > 2) )
    {
        close( pdev0 );

        pdev0 = -1;
    }

    return( 0 );
}

long pgpEdt::update_nrow_ncol( asynUser *pasynUser, int pass )
{
    int    expoVal, mrowVal, mcolVal, nrowVal, ncolVal;

    char   cmdFullStr[80], cmdROIStr[80], croi[80], cmds[80], respStr[80];

    char  *fmt, *aptr, *argp[8], *fidx1, *fidx2;
    short  argc, aval, clen;

    getIntegerParam( exposure, &expoVal );

    if ( expoVal == pgpEdtExpo_Full )                              // full frame
    {
        getIntegerParam( fullRow,     &nrowVal   );
        getIntegerParam( fullCol,     &ncolVal   );

        getStringParam ( cmdFull, 80, cmdFullStr );

        ser_send_recv( pasynUser, cmdFullStr, respStr, pass );
    }
    else                                                // HW region of interest
    {
        getIntegerParam( ADMinY,      &mrowVal   );
        getIntegerParam( ADMinX,      &mcolVal   );
        getIntegerParam( ADSizeY,     &nrowVal   );
        getIntegerParam( ADSizeX,     &ncolVal   );

        getStringParam ( cmdROI,  80, cmdROIStr  );

        if ( index(cmdROIStr, '%') == 0 )                       // no formatting
            ser_send_recv( pasynUser, cmdROIStr,  respStr, pass );
        else
        {
            strcpy( croi, cmdROIStr );
            fmt  = strtok( croi, ", " );

            argc = 0;
            while ( (aptr = strtok(0, ", ")) != 0 ) argp[argc++] = aptr;

            argc = 0;
            clen = 0;
            while ( 1 )
            {
                if      ( strcmp(argp[argc], "MinY" ) == 0 ) aval = mrowVal;
                else if ( strcmp(argp[argc], "MinX" ) == 0 ) aval = mcolVal;
                else if ( strcmp(argp[argc], "SizeY") == 0 ) aval = nrowVal;
                else if ( strcmp(argp[argc], "SizeX") == 0 ) aval = ncolVal;

                fidx1 = index( fmt,     '%' );
                fidx2 = index( fidx1+1, '%' );
                if ( fidx2 != 0 )
                {
                    *fidx2 = 0;
                    clen  += sprintf( cmds+clen, fmt, aval );

                    *fidx2 = '%';
                    fmt    = fidx2;
                }
                else
                {
                    clen  += sprintf( cmds+clen, fmt, aval );
                    break;
                }

                argc++;
            }

//          printf( "HW ROI Cmds: %s\n", cmds );
            ser_send_recv( pasynUser, cmds,       respStr, pass );
        }
    }

    setIntegerParam( NDArraySizeY, nrowVal );
    setIntegerParam( NDArraySizeX, ncolVal );

    return( 0 );
}

long pgpEdt::update_ntrn_ncyc( asynUser *pasynUser, int pass )
{
    epicsUInt32  rAddr;
    int          nbitVal, packVal, voutVal;
    int          skiprVal, skipcVal, nrowVal, ncolVal, ntrnVal, ncycVal;

    getIntegerParam( nbit,         &nbitVal  );
    getIntegerParam( pack,         &packVal  );
    getIntegerParam( vOut,         &voutVal  );

    getIntegerParam( skipRow,      &skiprVal );
    getIntegerParam( skipCol,      &skipcVal );
    getIntegerParam( NDArraySizeY, &nrowVal  );
    getIntegerParam( NDArraySizeX, &ncolVal  );

    nrowVal += skiprVal;
    ncolVal += skipcVal;

    if ( voutVal == pgpEdtVOut_TD )                      // readout line by line
    {
        ntrnVal = nrowVal;

        if      ( nbitVal > 12 )                       // one pixel per CL cycle
        {
            if ( packVal == pgpEdtPack_16 )           // one pixel per GTP cycle
                ncycVal = ncolVal;
            else                                 // one pixel per 1.5 GTP cycles
                ncycVal = ncolVal * 1.5;
        }
        else if ( nbitVal >  8 )                      // two pixels per CL cycle
        {
            if ( packVal == pgpEdtPack_16 )
                ncycVal = ncolVal;
            else                                // two pixels per 1.5 GTP cycles
                ncycVal = ncolVal * 1.5 / 2;
        }
        else                                          // two pixels per CL cycle
        {
            if ( packVal == pgpEdtPack_16 )          // two pixels per GTP cycle
                ncycVal = ncolVal       / 2;
            else {}                                               // unnecessary
        }
    }
    else                           // readout from top & bottom in the same time
    {
        ntrnVal = nrowVal / 2;

        if      ( nbitVal > 12 )                       // one pixel per CL cycle
        {
            if ( packVal == pgpEdtPack_16 )           // one pixel per GTP cycle
                ncycVal = ncolVal * 2;
            else                                 // one pixel per 1.5 GTP cycles
                ncycVal = ncolVal * 3;
        }
        else if ( nbitVal >  8 )                      // two pixels per CL cycle
        {
            if ( packVal == pgpEdtPack_16 ) {}                  // cannot happen
            else                                // two pixels per 1.5 GTP cycles
                ncycVal = ncolVal * 1.5;
        }
        else                                          // two pixels per CL cycle
        {
            if ( packVal == pgpEdtPack_16 )          // two pixels per GTP cycle
                ncycVal = ncolVal;
            else {}                                               // unnecessary
        }
    }

    rAddr = NumTrains_Addr + 4*channel;
    set_register( pasynUser, rAddr, ntrnVal, pass );

    rAddr = NumCycles_Addr + 4*channel;
    set_register( pasynUser, rAddr, ncycVal, pass );

    setIntegerParam( numTrains, ntrnVal );
    setIntegerParam( numCycles, ncycVal );

    return( 0 );
}

long pgpEdt::init_camera()
{
    epicsUInt32  rAddr;
    int          baudVal, nbitVal, psklVal, codeVal, tdlyVal, twisVal;
    int          trigVal, acqVal;

    char         cmdInitStr[80], cmdEVRStr[80], cmdFreeStr[80], respStr[80];

    long         status = 0;

    getIntegerParam( baud,              &baudVal   );
    getIntegerParam( nbit,              &nbitVal   );

    getIntegerParam( prescale,          &psklVal   );
    getIntegerParam( code,              &codeVal   );
    getIntegerParam( delay,             &tdlyVal   );
    getIntegerParam( width,             &twisVal   );

    getIntegerParam( ADTriggerMode,     &trigVal   );
    getIntegerParam( ADAcquire,         &acqVal    );

    getStringParam ( cmdInit,       80, cmdInitStr );
    getStringParam ( cmdEVR,        80, cmdEVRStr  );
    getStringParam ( cmdFree,       80, cmdFreeStr );

    set_csr         ( pasynUserSelf, 0,                   1 );        // disable

    rAddr = SerBaud_Addr   + 4*channel;
    set_register    ( pasynUserSelf, rAddr, baudVal,      2 );   // set the baud

    ser_send_recv   ( pasynUserSelf, cmdInitStr, respStr, 2 ); // send init cmds

    update_nrow_ncol( pasynUserSelf,                      2 );
    update_ntrn_ncyc( pasynUserSelf,                      2 );

    status = callParamCallbacks();

    rAddr = NumBits_Addr   + 4*channel;
    set_register    ( pasynUserSelf, rAddr, nbitVal,      2 );

    rAddr = PreScale_Addr  + 4*channel;
    set_register    ( pasynUserSelf, rAddr, psklVal,      2 );

    rAddr = TrgCode_Addr   + 4*channel;
    set_register    ( pasynUserSelf, rAddr, codeVal,      2 );

    rAddr = TrgDelay_Addr  + 4*channel;
    set_register    ( pasynUserSelf, rAddr, tdlyVal,      2 );

    rAddr = TrgWidth_Addr  + 4*channel;
    set_register    ( pasynUserSelf, rAddr, twisVal,      2 );

    if ( trigVal == pgpEdtTrig_EVR )
        ser_send_recv(pasynUserSelf, cmdEVRStr,  respStr, 2 );
    else
        ser_send_recv(pasynUserSelf, cmdFreeStr, respStr, 2 );

    set_csr         ( pasynUserSelf, acqVal,              3 );

    return( 0 );
}


/* Called when asyn clients call pasynInt32->write().
 * For all parameters it sets the value in the parameter library and
 * calls any registered callbacks.  For some parameters it performs actions.
 * \param[in] pasynUser pasynUser structure that encodes the reason and address
 * \param[in] value     Value to write */
asynStatus pgpEdt::writeInt32( asynUser *pasynUser, epicsInt32 value )
{
    int          param = pasynUser->reason;
    int          acqVal, expoVal;
    char         cmdTrigStr[80], cmdTPtnStr[80], cmdStr[80], respStr[80];

    epicsUInt32  rAddr;

    asynStatus   status = asynSuccess;

    /* Ensure that ADStatus is set correctly before we set ADAcquire */
    if ( param == ADAcquire )
    {
        getIntegerParam( ADAcquire, &acqVal );
        if ( value && !acqVal )
        {
            setIntegerParam( ADStatus,        ADStatusAcquire       );
            setStringParam ( ADStatusMessage, "Acquiring data"      );
        }

        if ( !value && acqVal )
        {
            setIntegerParam( ADStatus,        ADStatusIdle          );
            setStringParam ( ADStatusMessage, "Acquisition stopped" );
        }
    }

    /* Set the parameter in the library.
     * It may be overwritten later, but that's OK */
    status = setIntegerParam( param, value );

    callParamCallbacks();
 
    if ( ! interruptAccept ) return( status );// no action before initialization

    /* Action */
    if      ( param == ADAcquire      )
        set_csr     ( pasynUser,        value );
    else if ( param == baud           )
    {
        rAddr = SerBaud_Addr   + 4*channel;
        set_register( pasynUser, rAddr, value );
    }
    else if ( param == nbit           )                // this should not happen
    {
        rAddr = NumBits_Addr   + 4*channel;
        set_register( pasynUser, rAddr, value );

        update_ntrn_ncyc( pasynUser );
    }
    else if ( (param == pack   ) || (param == vOut   ) ||
              (param == skipRow) || (param == skipCol)    )
    {
        update_ntrn_ncyc( pasynUser );
    }
    else if ( (param == fullRow) || (param == fullCol) )           // full frame
    {
        getIntegerParam( exposure, &expoVal );

        if ( expoVal == pgpEdtExpo_Full )
        {
            update_nrow_ncol( pasynUser );
            update_ntrn_ncyc( pasynUser );
        }
    }
    else if ( (param == ADMinX ) || (param == ADMinY ) ||
              (param == ADSizeX) || (param == ADSizeY)    )            // HW ROI
    {
        getIntegerParam( exposure, &expoVal );

        if ( expoVal == pgpEdtExpo_ROI  )
        {
            update_nrow_ncol( pasynUser );
            update_ntrn_ncyc( pasynUser );
        }
    }
    else if ( param == exposure      )                   // change exposure mode
    {
        update_nrow_ncol( pasynUser );
        update_ntrn_ncyc( pasynUser );
    }
    else if ( param == prescale      )
    {
        rAddr = PreScale_Addr  + 4*channel;
        set_register( pasynUser, rAddr, value );
    }
    else if ( param == code          )
    {
        rAddr = TrgCode_Addr   + 4*channel;
        set_register( pasynUser, rAddr, value );
    }
    else if ( param == delay         )
    {
        rAddr = TrgDelay_Addr  + 4*channel;
        set_register( pasynUser, rAddr, value );
    }
    else if ( param == width         )
    {
        rAddr = TrgWidth_Addr  + 4*channel;
        set_register( pasynUser, rAddr, value );
    }
    else if ( param == ADTriggerMode )
    {
        if ( value == pgpEdtTrig_EVR )
            getStringParam ( cmdEVR,  80, cmdTrigStr );
        else
            getStringParam ( cmdFree, 80, cmdTrigStr );

        ser_send_recv( pasynUser, cmdTrigStr, respStr );
    }
    else if ( param == tstPtn        )
    {
        getStringParam ( cmdTPtn, 80, cmdTPtnStr );

        sprintf( cmdStr, "%s%d", cmdTPtnStr, value );
        ser_send_recv( pasynUser, cmdStr,     respStr );
    }
    else if ( (param == NDDataType) || (param == NDColorMode) )
    {
    }
    else if ( param < FIRST_CLCAM_PARAM )          /* belongs to a base class */
    {
        status = ADDriver::writeInt32( pasynUser, value );
    }

    if ( (param == pack) || (param == cc) || (param == polarity) )
    {
        getIntegerParam( ADAcquire, &acqVal );
        set_csr( pasynUser, acqVal );
    }

    callParamCallbacks();    /* Do callbacks so higher layers see any changes */

    if ( status )
        asynPrint( pasynUser, ASYN_TRACE_ERROR,
                   "%s:%s, writeInt32: parameter=%d, value=%d, failed (%d)\n",
                   driverName, portName, param, value, status );
    else
        asynPrint( pasynUser, ASYN_TRACEIO_DRIVER,
                   "%s:%s, writeInt32: parameter=%d, value=%d\n",
                   driverName, portName, param, value );

    return status;
}


/* Called when asyn clients call pasynOctet->write().
 * For all parameters it sets the value in the parameter library and
 * calls any registered callbacks.  For some parameters it performs actions.
 * \param[in ] pasynUser pasynUser structure that encodes the reason and address
 * \param[in ] value     Address of the string to write
 * \param[in ] nChars    Number of characters to write
 * \param[out] nActual   Number of characters actually written */
asynStatus pgpEdt::writeOctet( asynUser *pasynUser, const char *value, 
                               size_t nChars, size_t *nActual )
{
    int        param  = pasynUser->reason;
    char       respStr[80];
    asynStatus status = asynSuccess;

//  getStringParam( param, 80, oldStr );
//  printf( "writeOctet:  %s\n", oldStr );

    /* Set the parameter in the parameter library */
    status = setStringParam( param, (char *)value );

    if      ( param == cmdAny )
    {
        ser_send_recv( pasynUser, value, respStr );
        status = setStringParam( resp, respStr );
    }
    else if ( param < FIRST_CLCAM_PARAM )          /* belongs to a base class */
    {
        status = ADDriver::writeOctet( pasynUser, value, nChars, nActual );
    }

    /* Do callbacks so higher layers see any changes */
    status = callParamCallbacks();

    if ( status )
        asynPrint( pasynUser, ASYN_TRACE_ERROR,
                   "%s:%s, writeOctet: parameter=%d, value=%s, failed (%d)\n",
                   driverName, portName, param, value, status );
    else
        asynPrint( pasynUser, ASYN_TRACEIO_DRIVER,
                   "%s:%s, writeOctet: parameter=%d, value=%s\n",
                   driverName, portName, param, value );

    *nActual = nChars;
    return status;
}


/** Report status of the driver.
  * Prints details about the driver if details>0.  Then calls ADDriver::report()
  * \param[in] fp File pointer passed by caller where the output is written to
  * \param[in] details If >0 then driver details are printed */
void pgpEdt::report( FILE *fp, int details )
{
    fprintf( fp, "SLAC PgpEdt Frame Grabber %s\n", portName );
    if ( details > 0 )
    {
        int nx, ny, dataType;
        getIntegerParam(NDArraySizeX, &nx      );
        getIntegerParam(NDArraySizeY, &ny      );
        getIntegerParam(NDDataType,   &dataType);
        fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny  );
        fprintf(fp, "  Data type:         %d\n",     dataType);
    }

    ADDriver::report( fp, details );          /* Invoke the base class method */
}


static void acqTaskC( void *drvPvt )
{
    pgpEdt *pPvt = (pgpEdt *)drvPvt;

    pPvt->acqTask();
}

/* This thread acquires images, unpacks, and sends them to higher layers */
void pgpEdt::acqTask()
{
    struct pollfd   pfd[1];

    const char     *rbuf;
    epicsUInt32    *rdat, ir, icr, jcr;
    epicsUInt16    *idat, icol, irow;

    size_t          dims[2];
    int             nBit, pack16, vout;
    int             maxNrow, maxNcol, sRow, sCol, nRow, nCol, tCol;
    uint            maxSize, cstaVal, lane, eofe, fifoErr, lengthErr;
    epicsTimeStamp  timeNow;

    NDArray        *pNDArray = NULL;

    int             ret;

    pfd[0].fd     = pdev;
    pfd[0].events = POLLIN;

    while ( ! interruptAccept ) epicsThreadSleep( 1 );

    asynPrint( pasynUserSelf, ASYN_TRACE_FLOW,
               "%s:%s, acqTask: initializing ...\n", driverName, portName );

    init_camera();

    getIntegerParam( ADMaxSizeY, &maxNrow );
    getIntegerParam( ADMaxSizeX, &maxNcol );

    maxSize = maxNrow * maxNcol * 2 + 16;
    rbuf    = (char *)calloc( maxSize, sizeof(char) );

    while ( 1 )
    {
        if ( poll( pfd, 1, 2000 ) == 0 )    // no frame for 2 seconds, check CSR
        {
            asynPrint( pasynUserSelf, ASYN_TRACE_FLOW,
                       "%s:%s, acqTask: no image, check CSTA\n",
                       driverName, portName );

            pgpcard_readReg( pdev, GrbCSR_Addr+4*channel, &cstaVal );

            cstaVal = ((cstaVal >> 28) & 15) << 27 + (cstaVal & 255);

            setIntegerParam( csta,      cstaVal );
            setIntegerParam( frameRate, 0       );

//          printf( "No frame, post CSTA\n" );

            callParamCallbacks();

            continue;
        }

        ret = pgpcard_recv( pdev, (void *)rbuf, maxSize, &lane,
                            &eofe, &fifoErr, &lengthErr );

        if ( ret == 0 ) continue;

        asynPrint( pasynUserSelf, ASYN_TRACE_FLOW,
                   "%s:%s, acqTask: got image\n", driverName, portName );

//      getIntegerParam( NDArrayCallbacks, &arrayCallbacks );
//      if ( ! arrayCallbacks ) continue;

        getIntegerParam( nbit,         &nBit   );
        getIntegerParam( pack,         &pack16 );
        getIntegerParam( vOut,         &vout   );
        getIntegerParam( skipRow,      &sRow   );
        getIntegerParam( skipCol,      &sCol   );
        getIntegerParam( NDArraySizeY, &nRow   );
        getIntegerParam( NDArraySizeX, &nCol   );

        tCol = sCol + nCol;
        rdat = (epicsUInt32 *)rbuf;

        lock();

        if ( pNDArray ) pNDArray->release();            // free previous NDArray

        dims[0]  = nCol;
        dims[1]  = nRow;
        pNDArray = pNDArrayPool->alloc( 2, dims, NDInt16, 0, NULL );

        if ( (! pNDArray) || (! pNDArray->pData) )
        {
            asynPrint( pasynUserSelf, ASYN_TRACE_ERROR,
                       "%s:%s, acqTask: failed to allocate NDArray\n",
                       driverName, portName );

            pNDArray = NULL;

            unlock();
            continue;
        }

        epicsTimeGetCurrent( &timeNow );

        pNDArray->uniqueId             = *(rdat + 1);
        pNDArray->epicsTS.secPastEpoch = *(rdat + 2);
        pNDArray->epicsTS.nsec         = *(rdat + 3);
        pNDArray->timeStamp            = timeNow.secPastEpoch+timeNow.nsec/1.e9;

        pNDArray->ndims                = 2;
/*      pNDArray->bitsPerElement       = 12; */

        pNDArray->dims[0].size         = nCol;
        pNDArray->dims[0].offset       = 0;
        pNDArray->dims[0].binning      = 1;
        pNDArray->dims[1].size         = nRow;
        pNDArray->dims[1].offset       = 0;
        pNDArray->dims[1].binning      = 1;

        icol = 0;
        irow = 0;
        idat = (epicsUInt16 *)pNDArray->pData;

        if      ( (nBit == 24) && (pack16 == pgpEdtPack_24) ) {}
        else if ( (nBit >  12) && (pack16 == pgpEdtPack_16) ) {}
        else if ( (nBit <= 12) && (pack16 == pgpEdtPack_16) )
        {
            ir = 4 + sRow * tCol / 2;

            while ( irow < nRow )
            {
                icr = irow * nCol + icol - sCol ;

                if ( icol   >= sCol ) *(idat+icr  ) =  *(rdat+ir)       & 0xFFF;
                if ( icol+1 >= sCol ) *(idat+icr+1) = (*(rdat+ir) >>16) & 0xFFF;

                if ( icol < tCol-2 ) icol += 2;
                else
                {
                    irow += 1;
                    icol  = 0;
                }

                ir += 1;
            }
        }
        else if ( (nBit == 12) && (pack16 == pgpEdtPack_24) &&
                                  (vout   == pgpEdtVOut_TD)    )
        {
            ir = 4 + sRow * tCol * 3 / 8;

            while ( irow < nRow )
            {
                icr = irow * nCol + icol - sCol ;

                if ( icol   >= sCol )
                *(idat+icr  ) =   *(rdat+ir)       & 0xFFF;
                if ( icol+1 >= sCol )
                *(idat+icr+1) = ((*(rdat+ir) >> 4) & 0xF00) + ((*(rdat+ir) >> 16) & 0xFF);

                if ( icol+2 >= sCol )
                *(idat+icr+2) = ((*(rdat+ir+1) & 0xF) << 8) +  (*(rdat+ir) >> 24);
                if ( icol+3 >= sCol )
                *(idat+icr+3) = ((*(rdat+ir+1) <<  4) & 0xF00) + ((*(rdat+ir+1) >>  8) & 0xFF);

                if ( icol+4 >= sCol )
                *(idat+icr+4) =  (*(rdat+ir+1) >> 16) & 0xFFF;
                if ( icol+5 >= sCol )
                *(idat+icr+5) =  (*(rdat+ir+2) & 0xFF)         + ((*(rdat+ir+1) >> 28) << 8);

                if ( icol+6 >= sCol )
                *(idat+icr+6) =  (*(rdat+ir+2) >>  8) & 0xFFF;
                if ( icol+7 >= sCol )
                *(idat+icr+7) = ((*(rdat+ir+2) >> 24) & 0xFF) + ((*(rdat+ir+2) >> 12) & 0xF00);

                if ( icol < tCol-8 ) icol += 8;
                else
                {
                    irow += 1;
                    icol  = 0;
                }

                ir += 3;
            }
        }
        else if ( (nBit == 12) && (pack16 == pgpEdtPack_24 ) &&
                                  (vout   == pgpEdtVOut_TnB)    )
        {
            ir = 4 + sRow * tCol * 3 / 4;

            while ( irow < nRow/2 )                     // while ( ir < 393220 )
            {
                icr = irow          * nCol + icol - sCol;
                jcr = (nRow-1-irow) * nCol + icol - sCol;

                if ( icol   >= sCol ) {
                *(idat+icr  ) =   *(rdat+ir)       & 0xFFF;
                *(idat+jcr  ) = ((*(rdat+ir) >> 4) & 0xF00) + ((*(rdat+ir) >> 16) & 0xFF);
                }

                if ( icol+1 >= sCol ) {
                *(idat+icr+1) = ((*(rdat+ir+1) & 0xF) << 8) +  (*(rdat+ir) >> 24);
                *(idat+jcr+1) = ((*(rdat+ir+1) <<  4) & 0xF00) + ((*(rdat+ir+1) >>  8) & 0xFF);
                }

                if ( icol+2 >= sCol ) {
                *(idat+icr+2) =  (*(rdat+ir+1) >> 16) & 0xFFF;
                *(idat+jcr+2) =  (*(rdat+ir+2) & 0xFF) +         ((*(rdat+ir+1) >> 28) << 8);
                }

                if ( icol+3 >= sCol ) {
                *(idat+icr+3) =  (*(rdat+ir+2) >>  8) & 0xFFF;
                *(idat+jcr+3) = ((*(rdat+ir+2) >> 24) & 0xFF) + ((*(rdat+ir+2) >> 12) & 0xF00);
                }

                if ( icol < tCol-4 ) icol += 4;
                else
                {
                    irow += 1;
                    icol  = 0;
                }

                ir += 3;
            }
        }

        /* Get attributes that have been defined for this driver */
        getAttributes( pNDArray->pAttributeList );

        if ( pArrays[0] ) pArrays[0]->release();
        pNDArray->reserve();
        pArrays[0] = pNDArray;

        unlock();

//      printf( "New image:  %d,  %d  %d,  %04x %04x %04x %04x %04x %04x\n", *(rdat+1), *(rdat+2), *(rdat+3), *(rdat+4), *(rdat+5), *(rdat+6), *(rdat+7), *(rdat+8), *(rdat+9) );

        asynPrint( pasynUserSelf, ASYN_TRACE_FLOW,
                   "%s:%s, acqTask: calling imageData callbacks\n",
                   driverName, portName );

        doCallbacksGenericPointer( pNDArray, NDArrayData, 0 );

        setIntegerParam( NDArraySize,    nRow * nCol * sizeof(epicsInt16) );
        setIntegerParam( NDArrayCounter, *(rdat + 1)                      );

        setIntegerParam( trg2frame,       (*rdat)        & 0x7FFFF        );
        setIntegerParam( frameRate,      ((*rdat) >> 19) &   0x3FF        );

        setIntegerParam( csta,           0x78000000                       );

//      channel          = ((*rdat) >> 29) &       7;

        callParamCallbacks();

//      lock();
//      if (pArrays[0]) pArrays[0]->release();
//      pArrays[0] = pNDArray;
    }

    return;
}


/* Constructor for pgpEdt; It passes most parameters to ADDriver::ADDriver,
 * then creates a thread to communicate with the frame grabber channel.
 * \param[in] portName The name of the asyn port driver to be created.
 * \param[in] board Index of the board (0 - ).
 * \param[in] chan Channel number (0 - 7).
 * \param[in] maxSizeX The maximum X dimension of the images that this driver
 *            can create.
 * \param[in] maxSizeY The maximum Y dimension of the images that this driver
 *            can create.
 * \param[in] dataType The initial data type (NDDataType_t) of the images that
 *            this driver will create.
 * \param[in] maxBuffers The maximum number of NDArray buffers that the
 *            NDArrayPool for this driver is allowed to allocate. Set this to
 *            -1 to allow an unlimited number of buffers.
 * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for
 *            this driver is allowed to allocate.  Set this to -1 to allow an
 *            unlimited amount of memory.
 * \param[in] priority The thread priority for the asyn port driver thread
 *            if ASYN_CANBLOCK is set in asynFlags.
 * \param[in] stackSize The stack size for the asyn port driver thread
 *            if ASYN_CANBLOCK is set in asynFlags. */
pgpEdt::pgpEdt( const char *portName, int board, int chan,
                int maxSizeX, int maxSizeY, int numBits, NDDataType_t dataType,
                int maxBuffers, size_t maxMemory, int priority, int stackSize )
    : ADDriver(portName, 1, NUM_CLCAM_PARAMS, maxBuffers, maxMemory,
               0, 0,        /* No interfaces beyond those set in ADDriver.cpp */
               ASYN_CANBLOCK, 1,    /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=0, autoConnect=1 */
               priority, stackSize),
      pRaw(NULL), channel(chan)

{
    char  devi[80];
    int   status = asynSuccess;

    sprintf( dev0, "/dev/PgpCardG3_CLink_%d"  , board          );
    sprintf( devi, "/dev/PgpCardG3_CLink_%d%d", board, channel );

    pdev = open( devi, O_RDWR );

    createParam( baudString,       asynParamInt32,  &baud      );
    createParam( ssusString,       asynParamInt32,  &ssus      );
    createParam( nbitString,       asynParamInt32,  &nbit      );
    createParam( packString,       asynParamInt32,  &pack      );
    createParam( vOutString,       asynParamInt32,  &vOut      );
    createParam( skipRowString,    asynParamInt32,  &skipRow   );
    createParam( skipColString,    asynParamInt32,  &skipCol   );
    createParam( fullRowString,    asynParamInt32,  &fullRow   );
    createParam( fullColString,    asynParamInt32,  &fullCol   );
    createParam( numTrainsString,  asynParamInt32,  &numTrains );
    createParam( numCyclesString,  asynParamInt32,  &numCycles );
    createParam( ccString,         asynParamInt32,  &cc        );
    createParam( polarityString,   asynParamInt32,  &polarity  );
    createParam( prescaleString,   asynParamInt32,  &prescale  );
    createParam( codeString,       asynParamInt32,  &code      );
    createParam( delayString,      asynParamInt32,  &delay     );
    createParam( widthString,      asynParamInt32,  &width     );
    createParam( trg2frameString,  asynParamInt32,  &trg2frame );
    createParam( frameRateString,  asynParamInt32,  &frameRate );
    createParam( cstaString,       asynParamInt32,  &csta      );
    createParam( cmdInitString,    asynParamOctet,  &cmdInit   );
    createParam( cmdFullString,    asynParamOctet,  &cmdFull   );
    createParam( cmdROIString,     asynParamOctet,  &cmdROI    );
    createParam( cmdEVRString,     asynParamOctet,  &cmdEVR    );
    createParam( cmdFreeString,    asynParamOctet,  &cmdFree   );
    createParam( cmdTPtnString,    asynParamOctet,  &cmdTPtn   );
    createParam( cmdAnyString,     asynParamOctet,  &cmdAny    );
    createParam( respString,       asynParamOctet,  &resp      );
    createParam( exposureString,   asynParamInt32,  &exposure  );
    createParam( tstPtnString,     asynParamInt32,  &tstPtn    );
    createParam( reIniString,      asynParamInt32,  &reIni     );

    /* Set the fundamental parameters */
    status  = setStringParam ( ADManufacturer,  "SLAC PgpEdt, to be updated" );
    status |= setStringParam ( ADModel,         "Test Model, to be updated"  );
    status |= setIntegerParam( nbit,            numBits        );
    status |= setIntegerParam( NDDataType,      dataType       );
    status |= setIntegerParam( ADMaxSizeX,      maxSizeX       );
    status |= setIntegerParam( ADMaxSizeY,      maxSizeY       );

    if ( status )
    {
        printf( "%s:%s, pgpEdt: unable to set parameters, quit !!!\n",
                driverName, portName );
        return;
    }

    /* Create the thread that updates the images */
    status = (epicsThreadCreate(portName,
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)acqTaskC,
                                this) == NULL);
    if ( status )
    {
        printf( "%s:%s, pgpEdt: failed to create the acq thread, quit !!!\n",
                driverName, portName );
        return;
    }
}


/* Configuration command, called directly or from iocsh */
extern "C" int pgpEdtConfig( const char *portName, int board, int channel,
                             int maxSizeX, int maxSizeY,   int numBits,
                             int dataType, int maxBuffers, int maxMemory,
                             int priority, int stackSize )
{
    char  serPort[80];

    sprintf( serPort, "%s.SER", portName );

    new pgpEdt      ( portName, board, channel, maxSizeX, maxSizeY, numBits,
                      (NDDataType_t)dataType,
                      (maxBuffers < 0) ? 0 : maxBuffers,
                      (maxMemory  < 0) ? 0 : maxMemory, priority, stackSize );

    new pgpEdtSerial( serPort,  board, channel, ASYN_CANBLOCK, 1, 0, 0 );

    return( asynSuccess );
}

/* Code for iocsh registration */
static const iocshArg pgpEdtConfigArg0 = { "Port name",  iocshArgString };
static const iocshArg pgpEdtConfigArg1 = { "Board no",   iocshArgInt    };
static const iocshArg pgpEdtConfigArg2 = { "Channel no", iocshArgInt    };
static const iocshArg pgpEdtConfigArg3 = { "Max X size", iocshArgInt    };
static const iocshArg pgpEdtConfigArg4 = { "Max Y size", iocshArgInt    };
static const iocshArg pgpEdtConfigArg5 = { "No of bits", iocshArgInt    };
static const iocshArg pgpEdtConfigArg6 = { "Data type",  iocshArgInt    };
static const iocshArg pgpEdtConfigArg7 = { "maxBuffers", iocshArgInt    };
static const iocshArg pgpEdtConfigArg8 = { "maxMemory",  iocshArgInt    };
static const iocshArg pgpEdtConfigArg9 = { "priority",   iocshArgInt    };
static const iocshArg pgpEdtConfigArgA = { "stackSize",  iocshArgInt    };

static const iocshArg * const pgpEdtConfigArgs[] = { &pgpEdtConfigArg0,
                                                     &pgpEdtConfigArg1,
                                                     &pgpEdtConfigArg2,
                                                     &pgpEdtConfigArg3,
                                                     &pgpEdtConfigArg4,
                                                     &pgpEdtConfigArg5,
                                                     &pgpEdtConfigArg6,
                                                     &pgpEdtConfigArg7,
                                                     &pgpEdtConfigArg8,
                                                     &pgpEdtConfigArg9,
                                                     &pgpEdtConfigArgA };

static const iocshFuncDef configPgpEdt = { "pgpEdtConfig", 11, pgpEdtConfigArgs };

static void configPgpEdtCallFunc( const iocshArgBuf *args )
{
    pgpEdtConfig( args[0].sval, args[1].ival, args[ 2].ival, args[3].ival,
                  args[4].ival, args[5].ival, args[ 6].ival, args[7].ival,
                  args[8].ival, args[9].ival, args[10].ival               );
}

static void pgpEdtRegister( void )
{
    iocshRegister( &configPgpEdt, configPgpEdtCallFunc );
}

extern "C" {
epicsExportRegistrar( pgpEdtRegister );
}

