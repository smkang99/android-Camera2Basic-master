//
// Created by 1000308297 on 7/5/2017.
//


#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <android/log.h>
#include <android/bitmap.h>
#include <jni.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <time.h>
#include <math.h>
#include <sys/system_properties.h>
#include <CL/cl.h>

//------------------------------------------------------------------------------

#define  LOG_TAG    "Tp_NATIVE"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGW(...)  __android_log_print(ANDROID_LOG_WARN,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

// Set to 1 to enable debug log traces
#define DEBUG 1

//------------------------------------------------------------------------------
//  include h

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long u64;

typedef char s8;
typedef short s16;
typedef int s32;
typedef long s64;

// For setting USB property to accessory mode
char kstr_accessory[]="accessory";
char kstr_adb[]="mtp,adb";
char sysusb_now[] = "";
char kstr_sysusb[]="sys.usb.config";

char kstr_camera_optimization[]="persist.camera.isp.clock.optmz";
char kstr_zero[]="0";

// HASS sensor is now configured for imx290
//   these are used for raw_filhdr generation at get_raw_cfei()
#define  RSENSOR_BPP       10
// Bayer
#define  RSENSOR_MEDGE     8
// may not be right -- need to encode one of RGGB GRBG GBRG BGGR
#define  RSENSOR_PMOD      0
// default - processing stream may attach flags passing through
#define  RSENSOR_FLGS      0

// android keyboard constants
// from here, you can look up any value
// https://developer.android.com/reference/android/view/KeyEvent.html

#define  akb_KEYCODE_BACK     (0x00000004)

// crash codes
#define  CRASH_none       (0x0000)   // default - no crash
#define  CRASH_SELFCHK    (0x0001)   // test interface - shutdown on command
#define  CRASH_MEMCTRL    (0x0002)   // memctrl failure
#define  CRASH_CFEIFMT    (0x0003)   // get_raw_cfei() - MU_thread1
#define  CRASH_JCAMERA    (0x0004)   // Java side commanded crash
#define  CRASH_OPENCL     (0x0005)   // OpenCL major failure
#define  CRASH_UTPSYS     (0x0006)   // something in UTP process way out

typedef struct {
    void*    fwrd;   // 0x00
    void*    rvrs;  	// 0x08
} lnklst;

//
// there's not much call for this specific type
//  it's a useful construct to think of a linked list element as a chain
//   link carrying an unsigned parameter array
//    we can sort the link elements by a particular parameter index
//
// refer to routine lnklst_fsrchup()
//
typedef struct {
    lnklst        chain;       // 0x00..0x0F
    u64           p[16];       // 0x10..
} lnklst_upa;

//
// minimally, this must be set to 1 to prevent wrap
//  optionally, if you want to support "unget()" stupidity at this buffer
//   you have to pad it for the most unanswered unget's you expect to perform here
//
#define BPTRQUEUE_SAFE   0x10

typedef struct {
    int    push;
    int    pull;
    int    mask;
    int    spr0C;
    char*  body;
} bptrqueue;

typedef struct {
    int    push;
    int    pull;
    int    mask;
    int    tok;
    char*  body;
} bptrquetx;

// the lnklst takes 0x10 as two void*
//  then the remaining fields of an MBLK are home address (which is essentially a void*) -> size 8
//                                       and block size - which we may as well make a u64 -> size 8
#define MBLK_SIZE           (0x0020)
// this is the minimum unit our memctrl can deliver
//  generally we think that the ARM cache resolution is 32 bytes (0x20)
//   which in this case happens to be the same as MBLK_SIZE
// but is you try to malloc 1 byte -- anything less than MEMBLKUNIT
//  when the call succeeds, you will get minimally MEMBLKUNIT whether or not you asked for it
//   this is the granularity with which memctrl routines will issue memory
#define MEMBLKUNIT          (0x0100)                // power of 2 (0x80 is minimum)
// this must hold true, even if we change MEMBLKUNIT
#define MEMBLKMASK          (MEMBLKUNIT-1)
// so if we get the minimum allocation to a malloc(1), we'll get MEMBLKUNIT bytes
//  this represents the number of MBLK's in the minimal allocation of MEMBLKUNIT
#define MEMBLKMBLK          (MEMBLKUNIT/MBLK_SIZE)

typedef struct {
    lnklst        freepool;    // 0x00
    lnklst        mallpool;    // 0x10
    lnklst        mblkpool;    // 0x20
    volatile int  hwsema;      // 0x30
    u32           spr[3];      // 0x34..0x3F
} memctrl;

typedef struct {
    lnklst        chain;       // 0x00..0x0F
    u64           addr;        // 0x10
    u64           size;        // 0x18
} memblk;

#define MEMGBLKXFER         (0x0400)
#define GBLK64REFRESH       (MEMGBLKXFER>>6)

typedef struct {
    lnklst        chain;                 // 0x00..0x0F
    char*         scratch[48];           // 0x10..0x3F
} gblk_64;






typedef struct {
    lnklst           chain;    // 0x00..0x0F
    volatile int     sema;     // 0x10
    u32              spr;      // 0x14
} msg_queue;               // size 0x18

typedef int (*gblkfnc)(void* ioptr);

// the general concept is to call a function (fnc)
//  whose first argument is the dst* pointer for output
//     -- implicitly if dst==NULL, it's a function returning void
//  it can take up to 3 input arguments
//   we also need to specify which msg queue gets the message
//  the function returns an int that describes post disposal
//   for default retval=0, the message service routine automatically
//    releases the message as gblk_64
//     but if the message container (which may be some other object)
//      needs to be released differently, or gets passed on,
//       the execution of the function inherits the responsiblity to insure
//        the message is eventually releaded by other means
//         and returns non restval

typedef struct {
    lnklst           chain;    // 0x00..0x0F
    gblkfnc          fncptr;   // 0x10
    void*            dstptr;   // 0x18
    s64              arg0;     // 0x20
    s64              arg1;     // 0x28
    s64              arg2;     // 0x30
    volatile int*    doneptr;  // 0x38
} msg_gblkfnc;             // 0x40 size of gblk

#define J2JCMD_none       0x0000
#define J2JCMD_IMGVW      0x0001
//#define J2JCMD_FTUART     0x0002
#define J2JCMD_FTUART     0x0054

#define J2JST_IDLE        0x00
#define J2JST_WAIT        0x01

//
// typedef struct {
//    lnklst           chain;    // 0x00..0x0F
//    gblkfnc          fncptr;   // 0x10
//    void*            dstptr;   // 0x18
//    s64              arg0;     // 0x20
//    s64              arg1;     // 0x28
//    s64              arg2;     // 0x30
//    volatile int*    doneptr;  // 0x38
//    } msg_gblkfnc;             // 0x40 size of gblk
//
// based on msg_gblkfnc, the spec for a j2j communication
//  gets sent to thread 0
//   it always signals handoff at the thread1 queue
//    and the link gets rechained at jni2java_ctrl
//     so fncptr is always j2j_xchg_submit
//

typedef struct {
    lnklst           chn;      // 0x00..0x0F
    gblkfnc          fncptr;   // 0x10
    int              cmnd;     // 0x18   .
    int              p0;       // 0x1C    former dstptr
    int              p1;       // 0x20   .
    int              p2;       // 0x24    arg0
    int              p3;       // 0x28   .
    int              p4;       // 0x2C    arg1
    int              p5;       // 0x30   .
    int              p6;       // 0x34    arg2
    int*             retptr;   // 0x38
} j2j_xchg_spec;

typedef struct {
    lnklst           chn;        // 0x00..0x0F
    j2j_xchg_spec*   wrk;        // 0x10
    int              cmd;        // 0x18
    int              ret;        // 0x1C Java can provide retval
    int              j2jst;      // 0x20
    int              parm[7];    // 0x24..0x3F
                                 // parm[0-3] PC to Android
                                 // parm[4-7] Android to PC
} jni2java_ctrl;             // total size 0x40

#define PFL_SIGRES                0x8000
#define PFL_SIGRES_IO             0x80008000
#define PFL_SIG_HB                0x4000

// mask for actual signals (SIGRES and SIG_HB are reserved by the protocol)
#define PFL_SIG_USR               0x3FFF
// default .osig values on SIG_USR
#define PFL_OSIG_DEF              0x0000

// signals console line ready for pickup
#define PFL_OSIG_CNSL             0x0001
// signals ofile ready for pickup
#define PFL_OSIG_OFILE            0x0002

#define PFL_HDR_SIZE              0x40
#define PFL_TXSAFE                0x1F
// 16KB control buffers
#define PFL_BODY_SIZE             0x4000
// 256KB bulk buffers
#define PFL_BULK_SIZE             0x40000

// valid number of _RQS is 4,8,16
#define UTP_MAX_BLK_RQS           16
#define UTP_MAX_BLK_MSK           (UTP_MAX_BLK_RQS-1)
#define UTP_MAX_BLK_BND           (UTP_MAX_BLK_MSK-1)

#define UTP_MAX_BLK_BUF           0x40000
#define UTP_MAX_BLK_XFR           0x3FFFC
// this is used as an error code
//  if you read from a bulk buffer with error status
//   it returns this
#define UTP_BLK_ERROR             (UTP_MAX_BLK_BUF+1)

typedef struct {
    // direction pc2a
    int       sig_pc2a;    // 0x00
    int       pc2a_psh;    // 0x04
    int       a2pc_pll;    // 0x08
    int       pc2abpsh;    // 0x0C
    int       a2pcbpll;    // 0x10
    int       ctrlsize;    // 0x14
    int       bulksize;    // 0x18
    // spare tbd
    int       sprpc2a;     // 0x1C
    // direction a2pc
    int       sig_a2pc;    // 0x20
    int       a2pc_psh;    // 0x24
    int       pc2a_pll;    // 0x28
    int       a2pcbpsh;    // 0x2C
    int       pc2abpll;    // 0x30
    // spare tbd
    int       spra2pc[3];  // 0x34..0x3F
    // body pc2a
    //  char      pc2abody[PFL_BODY_SIZE];    // 0x0040..0x403F
    // body a2pc
    //  char      a2pcbody[PFL_BODY_SIZE];    // 0x4040..0x803F
    // bulk pc2a
    //  char      pc2abulk[PFL_BULK_SIZE];    // 0x8040..0x4803F
    // bulk a2pc
    //  char      a2pcbulk[PFL_BULK_SIZE];    // 0x48040..0x8803F
} pfl_hdr;

#define UTPST_DOWN                0
#define UTPST_RETRY               1
#define UTPST_RESET               2
#define UTPST_PINGS               3
#define UTPST_XCHG                4

// delay in milliseconds, between startup retry attempts
//  to get the device driver
#define UTP_RETRY_Z               1000
// startup heartbeat ping timeout ~1s
#define UTP_PING_ZTO              1000
// running ping timeout could be different
//  for now, it's the same ~1s
#define UTP_LOCK_ZTO              1000
// startup ping matches needed for lock
//  keep even number
#define UTP_PINGLOCK              4

// utp bulk output states
#define UTPBOST_IDLE              0
#define UTPBOST_PAYLD             1
// utp bulk input states
#define UTPBIST_IDLE              0
#define UTPBIST_PAYLD             1

typedef struct {
    int            fd;          //0x00

    u16            osig;        // 0x04
    u16            isig;        // 0x06
    u16            eosig;       // 0x08
    u16            zisig;       // 0x0A

    u16            pavail;      // 0x0C
    u16            pspace;      // 0x0E

    u32            bavail;      // 0x10
    u32            bspace;      // 0x14

    u32            spr[2];      // 0x18..0x1F

    pfl_hdr        utphdr;      // 0x20..0x5F

    u8             utpstate;    // 0x60
    u8             pings;       // 0x61
    u8             rmtres;      // 0x62 only meaningful during UTPST_XCHG
    u8             cm_state;    // 0x63 only meaningful during UTPST_XCHG

    int            timeref;     // 0x64

    int            gstcnt;      // 0x68 utp_getstatus() count
    int            pstcnt;      // 0x6C utp_putstatus() count

    u8             uhdr[4];     // 0x70 (rx u8 CMD u8 SEQ u16 SZ)
    u16            crscan;      // 0x74

    u8             bost;        // 0x76 bulk output state
    u8             bist;        // 0x77 bulk input state

    char*          datptr;      // 0x78
    u16            dlen;        // 0x80
    u8             ldvc;        // 0x81
    u8             lmempg;      // 0x82
    u32            ldva;        // 0x84

    u8             oflidx;      // 0x88  circulates 0..FF
    u8             iflidx;      // 0x89  circulates 0..FF
    u8             ocnslnum;    // 0x8A  number of console output lines
    u8             ocnslst;     // 0x8B  output console state
    u8             oflnum;      // 0x8C  number of output files on deck
    u8             oflst;       // 0x8D  output file state
    u8             iflnum;      // 0x8E  number of input files on deck
    u8             iflst;       // 0x8F  input file state <unused -- see .bist>

    lnklst         ocnslchn;    // 0x90 output console line chain
    lnklst         oflchn;      // 0xA0 output file chain
    lnklst         iflchn;      // 0xB0 input file chain

    lnklst         bochn;       // 0xC0 committed bulk output chain
    void*          oflwrk;      // 0xD0 (ofil_carrier*)
    char*          oflsrc;      // 0xD8
    u32            oflsiz;      // 0xE0

    u32            iflsiz;      // 0xE4
    void*          iflwrk;      // 0xE8 (ifil_carrier*)
    char*          ifldst;      // 0xF0

    int            tbd[66];     //0x0F8..0x1FF
} UTP_ctrl;


// the .flg field gets init 0x00
//  when the exchange gets dispatched on the link, "sent" flag gets set
//   and the exchange structure is transferred from the txchn to the
//    tachn to await ACK/response
#define FLG_uaiox_sent      0x01
// while we're processing the response,
//  first check is that response hdr is a match
#define FLG_uaiox_rhdr      0x02
// when we receive full ACK for the exchnage "resp" flag gets set
//  when the ACK is complete, the link just gets dropped from the tachn
//   so when the exchange is completed, whether or not it failed
//    all flags are set
#define FLG_uaiox_done      0x03

// every exchange gets init with .err=0
//  if the exchnage is aborted due to error, it is reflected in the .err byte
//   after the exchange (even if the exchange didn't really happen)
#define ERR_uaiox_none      0x00
#define ERR_uaiox_lnkdn     0x01
#define ERR_uaiox_unknwn    0x02
#define ERR_uaiox_to        0x03
#define ERR_uaiox_reterr    0x04
#define ERR_uaiox_setup     0x05

// the target only supports xchg layer
typedef struct {
    lnklst         chn;      // 0x00..0x0F
    u8             cmd;      // 0x10 | outbound header
    u8             seq;      // 0x11 |
    u16            len;      // 0x12 | <-- gets update for retval
    u8             flg;      // 0x14
    u8             err;      // 0x15 error code - link down, timeout
    u16            spr16;    // 0x16
    u32            spr32;    // 0x18
    u32            atrg;     // 0x1C - address target
    u64            ahst;     // 0x20 - address host, may be cast as ptr
    char           buf[16];  // 0x28..0x37 -- may be cast as anything
    int*           retptr;   // 0x38 | int pair [0] done - user inits 0
    //          - when set 1, [1] is return error code (0 means no error)
} uaio_xchg;             // total 0x40

// the comiox_ctrl structure from msc implementation is nested in FTU_ctrl (ftuptr)

// comiox needs access to some number of re-usable open-loop carriers
//  it can re-use the uaio_xchg if it's .flg is FLG_uaiox_done
//   then when it issues a transaction on it, it sets .flg=0
//    when the transaction I/O processing is done with it
//     .flg is restored to "done"

typedef struct {
    lnklst        ringchn;   // 0x00..0x0F
    uaio_xchg     carrier;   // 0x10..0x4F
} olxchg_t;              //  total 0x50

#define OLXCHG_NUM          (0x10)

// units msec between UART discovery attempts ~1s
#define FTDI_RETRY_Z              1000

// use 4KB bptrqueue
#define FTDI_BPTR_DEPTH           0x1000
#define FTDI_BPTR_MSK             (FTDI_BPTR_DEPTH-1)

// 256 byte bound on single read/write transfer
#define FTDI_XFER_MAX             0x100

// .ft_state values
#define FTUART_BOOT               0
#define FTUART_DSCVR              1
#define FTUART_PHYON              2

// comstate values
// COMDOWN is dummy state that only exists once at startup
#define COMDOWN         0
#define COMSYNC0        1
#define COMSYNC1        2
#define COMSYNC2        3
#define COMLOCK         4
#define COMXCHG         5

typedef struct {
    void*          D2xx_ptr;    // 0x00 - init 0 - java side sends ftD2xx
    u8             ft_state;    // 0x08
    u8             spr09;       // 0x09
    u16            rxvlen;      // 0x0A
    int            reftime;     // 0x0C
    bptrqueue*     rxqueue;     // 0x10
    bptrquetx*     txqueue;     // 0x18
    u8             comstate;    // 0x20
    u8             comtcnt;
    u8             comrcnt;
    u8             rxcollect;
    u32            tx_op_time;   // 0x24
    u32            rx_in_time;   // 0x28
    u8             txseq;        // 0x2C
    u8             txackseq;
    u8             rxseq;
    u8             rxackseq;
    u16            rfrshfe;      // 0x30
    u16            rfrshq0;      // 0x32
    u32            acktime;      // 0x34
    u64            spr64;        // 0x38
    //-- comiox_ctrl - start
    lnklst         cmtxchn;      // 0x40
    lnklst         cmtachn;      // 0x50
    bptrquetx*     cmtxque;      // 0x60
    bptrqueue*     cmrxque;      // 0x68
    u8             cmtseq;       // 0x70
    u8             cmtack;
    u8             cmrstate;
    u8             cmlmempg;
    u16            cmrscan;      // 0x74
    u16            cmdlen;
    u8             cmuhdr[4];    // 0x78
    u8             cmldvc;       // 0x7C
    u8             spr7D;
    u8             spr7E;
    u8             spr7F;
    char*          cmdatptr;     // 0x80
    u32            cmldva;       // 0x88
    //-- comiox_ctrl - end
    int            tbd[29];     // 0x8C..0xFF
} FTU_ctrl;

typedef struct {
    u16 red;        // channel red
    u16 green;      // channel green
    u16 blue;       // channel blue
    u16 clear;      // channel clear
    u16 infrared;   // channel infrared
    u16 spr[3];
} als;              // total 0x10

typedef struct {
    long time;          // time
    double latitude;    // latitude
    double longitude;   // longitude
    double altitude;    // altitude
    float speed;        // speed
    float bearing;      // bearing
    u32 spr[2];
} location;             // total 0x30

// use the memory variables - uartpktticks and uarttimeout
//  so these can be modified to config variables
//
// on the HP folio Windows 7, a thread timer tick is ~15.47 msec
//  this eventually may have to get tuned machine dependent
//   and be part of the .cfg file
// But for now, 15.47 msec means about 64.6 timer ticks/second
//
// so in the absence of anything else to do, our uart program
//  should pass some TX packet every second locked in COMXCHG state
//   ~1 second
//
#define UART_PKT_TICKS   (1000)
//
// every TXPKT should get an ACK within ~1.5 seconds
#define UART_ACK_TICKS   (1500)
//
// if the other side isn't talking back to us,
//  we probably have a physical cable disconnect, or the target is reset
//   OK, assume the target system gets reset,
//    it should reboot in under 5 seconds, if it doesn't, we can drop the link
//  actually, you can't die of UART_ACK_TICKS if nothing is arriving
//   a broken line doesn't fail due to UART_ACK_TICKS, rather UART_TIMEOUT
#define UART_TIMEOUT     (2000)

// units msec - this is the separation of SYNC packets when they trickle out
#define UART_SYNC_SEP    (50)

// one sub-queue is uniform size 4 KB
#define COM_SQBUFF_SIZE  0x00001000
// general guard band on all queue's
#define COM_BUFF_SAFEPAD 0x00000020
// by definition, considering the guard band
//  the maximum chunk a subque can accept in one shot
//   is the maximum number of tokens it ever issues
//    which is also the maximum payload any TXPKT can ever issue
#define COM_TXPKT_MAXPL  (COM_SQBUFF_SIZE-COM_BUFF_SAFEPAD)

// ACK packet is highest packet identifier index
#define UART_SYNC0       0x00
#define UART_SYNC1       0x01
#define UART_SYNC2       0x02
#define UART_TXPKT       0x03
#define UART_ACKPKT      0x04
// these bit fields are used to set the acceptance
//  compatability matrix based on the bit index
//   of the first header byte
//    refer to uart_rxhdr_mask[]
#define UART_SYNC0_BIT   (0x01)
#define UART_SYNC1_BIT   (0x02)
#define UART_SYNC2_BIT   (0x04)
#define UART_TXPKT_BIT   (0x08)
#define UART_ACKPKT_BIT  (0x10)

// protocol lock
//
// COMLINK heartbeat
//  while in COMXCHG state, even if nothing to report,
//   both peers should send a TXPKT at minimum rate of ~1/second
//   (in any lower state, sync packets should be generated faster than the minimum)
//
// COMLINK TIMEOUT
//  if the line is idle for ~5 seconds, drop the link
//   (On the PC side, this means the link is shut off - decayed to COMDOWN)
//     for an FPGA target board, there is no COMDOWN state,
//      and it just loops through COMSYNC0
//
// SYNC0,SYNC1,SYNC2 are all 4 byte sequences made of non-text ASCII characters
//  (so the target system could default to ASCII text, and auto-switch to
//    COMLINK on detection of SYNC0
//
// COMSYNC0 state -- trickle out SYNC0 packets at rate ~1 packet every 10-15 msec
//                    (even at 9600 BAUD, 4 byte packet fits in ~5msec)
//   wait for 2 (UART_SYNC0_RTRG) detections of SYNC0 from the other side
//    then blast 3 (UART_SYNC0_RTRG+1) SYNC0 packets and advance state to COMSYNC1
//
#define UART_SYNC0_RTRG   (2)
//
//   we may have traffic from the other side that keeps satisying TIMEOUT
//    but if we're not talking the same protocol, it's still a hopeless case
#define UART_SYNC0_DROP   (0xFA)
//      if we've sent that many SYNC0 packets, without hearing a SYNC0
//       the other side is not talking our language and we decay to COMDOWN
//
// COMSYNC1 state - "I can hear you" - and "you should be hearing me"
//                -- trickle out SYNC1 packets at rate ~1 packet every 10-15 msec
//   discard SYNC0 packets - we may have advanced state faster than other side
//                         -  but we left enough SYNC0 packets in queue
//                             that the other side should reliably detect
//   if the other side missed the SYNC0 burst, they'll never advance to SYNC1
//    while we're perpetually outputting SYNC1, so we put a cap on those
//     and decay back to SYNC0 if we hit UART_SYNC1_DROP
//
#define UART_SYNC1_DROP   (5)
//
//   it's expected, the other side will eventually join us in COMSYNC1 state
//    or we're joining the other side already in COMSYNC1 state
//     either way, we only need a single detection of a SYNC1 packet
//      then that means "I can hear you - and - you can hear me"
//   the response to that, is to advance state to COMSYNC3
//    and output a SYNC1->SYNC2 "rising edge" as two SYNC1 packets and
//                                               precisely one SYNC2
//
// COMSYNC2 state - waits on the SYNC1 -> SYNC2 rising edge
//                   so incoming SYNC0 packets are errors
//                   incoming SYNC1 packets are discarded
//                   precisely one incoming SYNC2 packet advances state to COMLOCK
//                    this is the sync one-shot
//                     at this point this side's TX(0) gets output
//                - (otherwise COMSYNC2 doesn't output anything)
//
// COMLOCK is waiting on two events and outputs directly from the poacket receiver
//   1) the arrival of the the other side's TX(0)
//       in response, it sets up the initial tokens on the front end and all subque's
//        to match the other side's TX(0) packet
//         it also issues the ACK(0) back to the other side
//   2) the arrival of ACK(0) from the other side, in response to this side's TX(0)
//
//   once those are satisfied in that order, switch state to COMXCHG
//    and the peers are connected - QCRLINK/COMLKINK is up.
//

// we won't generate any more TXPKT outputs on an excess of unanswered ACK's
//  while we're feeding 8 subque's with data, the more lead we can put on this
//   the better
//    realistically, the size of the front end buffer is usually smaller than
//     this number of TXPKT's -- so the token circulation
//      will probably limit flow rate before we hit this kind of backlog
#define UART_ACK_LIMIT    0xF0
// we won't generate a TXPKT unless there are at least ?? bytes of headway
//  on the front end output queue
//   then we'd rather preserve that buffer to allow a bigger chunk to be sent later
#define UART_TX_PKT_THR   0x40
// if the only purpose of the TXPKT is to refresh front end tokens
//  we put a bound on that
#define UART_TX_RFRSHFE   0x10

typedef struct {
    void*         shrptr;     // 0x00
    pthread_t     this_thr;   // 0x08
    int           thr_indx;   // 0X10
    volatile int  strt_flg;   // 0x14
    volatile int  stop_flg;   // 0x18
    volatile int  done_flg;   // 0x1C
    msg_queue     mqueue;     // 0x20..0x37
    // above is fixed by generic
    //------------------------------------
    // below is specific to thread n
    int           spr[18];    // 0x38..0x7F
} thread_ctrl;            // 0x80

// maximum number of threads - goes to memory allocation of thread_ctrl blocks
//  no all threads are necessarily use yet, but may be reserved for future functions
#define  MU_THREADNUM     (8)
#define  thr_ctrl         ((thread_ctrl*)(((u64)shrMEM_ptr)+0xFC00))
// actual number of working threads
//  minimum value is 1
#define  MU_THREADWRK     (5)

typedef void* (*thread_routine) (void *);

// from the generic that has to be sync'd with thread_ctrl
//  derive a specific variant for thread0 - timer thread
typedef struct {
    void*         shrptr;     // 0x00
    pthread_t     this_thr;   // 0x08
    int           thr_indx;   // 0X10
    volatile int  strt_flg;   // 0x14
    volatile int  stop_flg;   // 0x18
    volatile int  done_flg;   // 0x1C
    msg_queue     mqueue;     // 0x20..0x37
    // above is fixed by generic
    //------------------------------------
    // below is specific to thread n
    volatile int  frmtok;     // 0x38
    int           frmtref;    // 0x3C
    int           frmrate;    // 0x40 units msec
    int           rWDtim;     // 0x44
    u32           rWDset;     // 0x48
    int           spr[13];    // 0x4C..0x7F
} thread0_ctrl;            // 0x80


// from the generic that has to be sync'd with thread_ctrl
//  derive a specific variant for thread1 - camera raw receiver/dispatcher
typedef struct {
    void*         shrptr;     // 0x00
    pthread_t     this_thr;   // 0x08
    int           thr_indx;   // 0X10
    volatile int  strt_flg;   // 0x14
    volatile int  stop_flg;   // 0x18
    volatile int  done_flg;   // 0x1C
    msg_queue     mqueue;     // 0x20..0x37
    // above is fixed by generic
    //------------------------------------
    // below is specific to thread 1
    u32           snapreq;    // 0x38
    u32           snapcnt;    // 0x3C -- actually unused - can use for (SD/USB) file requests
    lnklst        cfepool;    // 0x40..0x4F
    u32           cfeicnt;    // 0x50
    u32           cfeidrop;   // 0x54
    u32           wt_texture; // 0x58 -- remnant from Tp01
    volatile int  cfedone;    // 0x5C
    lnklst        cfgpool;    // 0x60..0x6F
    u32           cfedim;     // 0x70
    // u16     cfexdim             // 0x70 lo
    // u16     cfeydim             // 0x72 hi
    u32           spr[3];     // 0x74..0x7F
} thread1_ctrl;           // 0x80

// from the generic that has to be sync'd with thread_ctrl
//  derive a specific variant for thread2 - oBitmap interface
typedef struct {
    void*         shrptr;     // 0x00
    pthread_t     this_thr;   // 0x08
    int           thr_indx;   // 0X10
    volatile int  strt_flg;   // 0x14
    volatile int  stop_flg;   // 0x18
    volatile int  done_flg;   // 0x1C
    msg_queue     mqueue;     // 0x20..0x37
    // above is fixed by generic
    //------------------------------------
    // below is specific to thread 2
    u32           imobytes;   // 0x38
    volatile int  hwsema_rdy; // 0x3C
    lnklst        imopool;    // 0x40..0x4F
    lnklst        imo_rdy;    // 0x50..0x5F
    u32           pixocnt;    // 0x60
    u32           pixodrop;   // 0x64
    u32           osnapreq;   // 0x68
    u32           ofilereq;   // 0x6C
    u32           tpflg;      // 0x70
    u32           tpmode;     // 0x74
    u32           tpxptr;     // 0x78
    u32           imodim;     // 0x7C
    // u16     imoxdim             // 0x7C lo
    // u16     imoydim             // 0x7E hi
} thread2_ctrl;            // 0x80

// from the generic that has to be sync'd with thread_ctrl
//  derive a specific variant for thread3 - HASS
typedef struct {
    void*         shrptr;     // 0x00
    pthread_t     this_thr;   // 0x08
    int           thr_indx;   // 0X10
    volatile int  strt_flg;   // 0x14
    volatile int  stop_flg;   // 0x18
    volatile int  done_flg;   // 0x1C
    msg_queue     mqueue;     // 0x20..0x37
    // above is fixed by generic
    //------------------------------------
    // below is specific to thread 3
    int           spr[18];    // 0x38..0x7F
} thread3_ctrl;            // 0x80

// from the generic that has to be sync'd with thread_ctrl
//  derive a specific variant for thread4 - pfN
typedef struct {
    void*         shrptr;     // 0x00
    pthread_t     this_thr;   // 0x08
    int           thr_indx;   // 0X10
    volatile int  strt_flg;   // 0x14
    volatile int  stop_flg;   // 0x18
    volatile int  done_flg;   // 0x1C
    msg_queue     mqueue;     // 0x20..0x37
    // above is fixed by generic
    //------------------------------------
    // below is specific to thread 4
    int           spr[18];    // 0x38..0x7F
} thread4_ctrl;            // 0x80

// from the generic that has to be sync'd with thread_ctrl
//  derive a specific variant for thread5 - worker/sequencer
typedef struct {
    void*         shrptr;     // 0x00
    pthread_t     this_thr;   // 0x08
    int           thr_indx;   // 0X10
    volatile int  strt_flg;   // 0x14
    volatile int  stop_flg;   // 0x18
    volatile int  done_flg;   // 0x1C
    msg_queue     mqueue;     // 0x20..0x37
    // above is fixed by generic
    //------------------------------------
    // below is specific to thread 5
    int           spr[18];    // 0x38..0x7F
} thread5_ctrl;            // 0x80

//------------------------------------------------------------------------------
// any setting of crash sets shutdown timer to max
#define  SHTDN_MAXTM      100
// after some delay - say 1 frame = 33 msec
//  none of the buffers should be busy any more
//   so we'll give it 1/5 frame delay to be safe
#define  SHTDN_SVM         50

// contains a justified and nested oconsole.h raw_filhdr
typedef struct {
    // a msg/gblk style header is at the start of the buffer
    lnklst       chain;       // 0x00..0x0F
    u32          spr[8];      // 0x10..0x2F
    // nested raw_filhdr (oconsole.h)
    u16          xdim;        // 0x30
    u16          ydim;        // 0x32
    u32          tstmp;       // 0x34
    u32          tdate;       // 0x38
    u8           pbpp;        // 0x3C
    u8           medge;       // 0x3D mosaic edge size - repsz
    u8           pmod;        // 0x3E pmod - could encode mosaic type
    u8           flgs;        // 0x3F flgs could be context dependent
    als          main;        // 0x40 main ALS
    als          slave;       // 0x50 slave ALS
    location     gps;         // 0x60 GPS location
    // end of header
    u16          pixels[16];  // size irrelevant, we just want the pixels pointer
    //  associated with the structure
} cfei_image;

typedef struct {
    // a msg/gblk style header is at the start of the buffer
    lnklst       chain;       // 0x00..0x0F
    u32          spr[8];      // 0x10..0x2F
    // nested raw_filhdr (oconsole.h)
    u16          xdim;        // 0x30
    u16          ydim;        // 0x32
    u32          tstmp;       // 0x34
    u32          tdate;       // 0x38
    u8           pbpp;        // 0x3C
    u8           medge;       // 0x3D mosaic edge size - repsz
    u8           pmod;        // 0x3E pmod - could encode mosaic type
    u8           flgs;        // 0x3F flgs could be context dependent
    // end of header
    void*        svmptr;   ;  // 0x40..0x47 pixels[] are indirected
} cfeg_image;

// thread 1 (control flags) at ACptr->thr1flgs
//
// substitute tst_inframe[] for sensor raw data
//  operates on get_raw_cfei()
#define  T1FLG_TSTIMG     0x00000001
// normally RAW data gets patched to display as RAW image - default 0
// when set, RAW data gets patched through the HASS GPU processing
//  of thread3
// when set, gets patched to display as RAW image
//  operates on cfe_fwrd_image()
#define  T1FLG_HASSEN     0x00000002
// this is the format of displayable output image
//  for example on ImageView - pixels are ARGB_8888
typedef struct {
    // a msg/gblk style header is at the start of the buffer
    lnklst       chain;       // 0x00..0x0F
    u32          spr[10];     // 0x10..0x37
    u32          ts_msec;     // 0x38
    // the last quad of the gblk is the dimensions
    //  so from here down, is looks like the RAW .bin file
    u16          xdim;        // 0x3C
    u16          ydim;        // 0x3E
    // end of header
    u32          pixels[16];  // size irreleval, we just want the pixels pointer
    //  associated with the structure
} pixo_image;

// image bitmap mask byte - what the thread 2 renderer is passing
// this passes nothing but fixed test patterns - say black screen
//  (they still may not be visible if we're outputting YUV)
//  (this is the startup default)
#define  IBM_MSK_NONE     0
// this passes only raw data frame
#define  IBM_MSK_RAW      1
// this passes only hass data frame
#define  IBM_MSK_HASS     2

// contains a justified and nested oconsole.h rgb_filhdr
typedef struct {
    // a msg/gblk style header is at the start of the buffer
    lnklst       chain;       // 0x00..0x0F
    u32          spr[8];      // 0x10..0x2F
    // nested rgb_filhdr (oconsole.h)
    u16          xdim;        // 0x30
    u16          ydim;        // 0x32
    u32          tstmp;       // 0x34
    u32          tdate;       // 0x38
    u8           pbpp;        // 0x3C
    u8           u8spr;       // 0x3D <tbd>
    u8           pmod;        // 0x3E pmod - could encode mosaic type
    u8           flgs;        // 0x3F flgs could be context dependent
    als          main;        // 0x40 main ALS
    als          slave;       // 0x50 slave ALS
    location     gps;         // 0x60 GPS location
    // end of header
    u32          pixels[8];   // size irrelevant, we just want the pixels pointer
    //  associated with the structure
} cofl_image;

// command states refer to cm_state
#define CMDST_HEADER     0x0000
#define CMDST_WTPAYLD    0x0001
// sanity?
#define CMDST_SANITY     0x0002
// this may appeal to another thread for completion
//  for example, it may only launch an I2C read...
#define CMDST_EXEC       0x0003
//  and this state waits on the I2C read data to arrive
//   before it can error out or dispatch it...
#define CMDST_EXEDONE    0x0004
// this is the state that actually deploys the full response...
#define CMDST_REPLY      0x0005
// ...and then loops back to the top to wait on next CMD header
// -- extensions for symmetric comiox handling
#define CMDST_WTRPAY     0x0006
#define CMDST_RZERO      0x0007

// exchange .cmd assignments
#define UAIOXCMD_WRBYTE     0x00
#define UAIOXCMD_RDBYTE     0x01
#define UAIOXCMD_WRWORD     0x02
#define UAIOXCMD_RDWORD     0x03
#define UAIOXCMD_WRQUAD     0x04
#define UAIOXCMD_RDQUAD     0x05
#define UAIOXCMD_WRLDVC     0x06
#define UAIOXCMD_RDLDVC     0x07
#define UAIOXCMD_HRTBT      0x08
#define UAIOXCMD_FNCKEY     0x09
#define UAIOXCMD_CONSOLE    0x0A
#define UAIOXCMD_OFILE      0x0B
// illegal value
#define UAIOXCMD_BOUND      0x0C
// illegal value - comiox doesn't support CONSOLE or OFILE
#define COMIOXCMD_BOUND     0x0A

// of course, this should be smaller than the subque

#define UAIOXCMD_MAXDATA    2048
#define UAIOXCMD_MAXBUFF    (UAIOXCMD_MAXDATA+16)

#define UAIOX_LDVC_BND      0x10
#define UAIOX_FK_BND        0x0020

// error list
// unused error code
#define UAERR_none          0x00000000
// unknown logical device
#define UAERR_LDVC_UNKN     0x00000001
// known device, but offline
#define UAERR_LDVC_ONLN     0x00000002
// unknown logical FNC#
#define UAERR_FNC_UNKN      0x00000003

// console line limit is 255 characters + asciiz
#define CNSL_LINE_LIM    0x100

typedef struct {
    // based on msg_gblkfnc
    lnklst       chn;                 // 0x00..0x0F
    gblkfnc      fncptr;              // 0x10
    void*        dstptr;              // 0x18
    s64          arg0;                // 0x20
    s64          arg1;                // 0x28
    s64          arg2;                // 0x30
    // doneptr used differently
    u32          spr38;               // 0x38
    u32          lenz;                // 0x3C  strlen()+1 includes asciiz
    // data field appended
    char         data[CNSL_LINE_LIM];
} cnsl_carrier;

#define CNSL_CARR_SIZE   (CNSL_LINE_LIM+64)
// won't queue past this console backlog (ocnslnum)
#define CNSL_BACKLOG     (0x80)

// ocnsl reset or idle state
#define CNSLST_IDLE      (0x00)
// ocnsl verify idle echo
#define CNSLST_IECHO     (0x01)
// ocnsl wait on message on deck to output
#define CNSLST_WTODCK    (0x02)
// ocnsl wait on message line pickup
#define CNSLST_WTLNPU    (0x03)

// won't queue past this console backlog (oflnum)
#define OFIL_BACKLOG     (0x10)

// .ofltype describes general type of transfer
// standard is block carrier like cfe_image or cofl_image
//  where the file data offset is 0x30 from refptr
#define OFLTYP_CFE       (0x0000)
#define OFLTYP_COFL      (0x0001)
// not implemented and no rules yet
//  transferring direct from the file system - no carrier buffer involved
#define OFLTYP_FS        (0x0002)

typedef struct {
    // based on msg_gblkfnc
    lnklst       chn;                 // 0x00..0x0F
    gblkfnc      fncptr;              // 0x10
    void*        dstptr;              // 0x18
    // different use model arg0...
    u16          ofltype;             // 0x20
    u8           p2;                  // 0x22
    u8           p3;                  // 0x23
    u8           p4;                  // 0x24
    u8           p5;                  // 0x25
    u8           p6;                  // 0x26
    u8           p7;                  // 0x27
    void*        refptr;              // 0x28
    u32          filsize;             // 0x30
    char         filname[12];         // 0x34..0x3F  11 char + asciiz
} ofil_carrier;

// ofl reset or idle state
#define OFILST_IDLE      (0x00)
// ofl verify idle echo
#define OFILST_IECHO     (0x01)
// ofl wait on file on deck to output
#define OFILST_WTODCK    (0x02)
// ofl wait on message file pickup
#define OFILST_WTFLPU    (0x03)

// won't queue past this console backlog (iflnum)
#define IFIL_BACKLOG     (0x10)

// .ifltype describes general type of transfer
// standard is block carrier like cfe_image or cofl_image
//  (nothing about the transfer mechanism cares what happens to the data
//    after it arrives)
// most obvious use - fill the (raw) test image buffer with a new test image
#define IFLTYP_RWTST     (0x0000)
// not implemented and no rules yet
//  transferring direct to the file system - no carrier buffer involved
#define IFLTYP_FS        (0x0001)

// ifil return status
#define IFLSTATUS_OK     0x00
#define IFLSTATUS_ERR    0x01

typedef struct {
    // based on msg_gblkfnc
    lnklst       chn;                 // 0x00..0x0F
    gblkfnc      fncptr;              // 0x10
    void*        dstptr;              // 0x18
    // different use model arg0...
    u16          ifltype;             // 0x20
    u8           p2;                  // 0x22
    u8           p3;                  // 0x23
    u8           p4;                  // 0x24
    u8           p5;                  // 0x25
    u8           p6;                  // 0x26
    u8           p7;                  // 0x27
    void*        refptr;              // 0x28
    u32          filsize;             // 0x30
    char         filname[12];         // 0x34..0x3F  11 char + asciiz
} ifil_carrier;

// rs0. ofile header for raw files (snapshot)
// pmod could encode mosaic type as RGGB, GBRG, etc...
// reserve pmod=0 as unspecified/unknown (default)
typedef struct {
    u16    xdim;    //
    u16    ydim;    //
    u32    tstmp;   // (0xFFFFFFFF invalid)
    u32    tdate;   // 0 invalid
    u8     pbpp;    // pixel - bits per pixel
    u8     medge;   // mosaic edge size - repsz
    u8     pmod;    // pmod - could encode mosaic type
    u8     flgs;    // flgs could be context dependent
    als    main;    // main ALS
    als    slave;   // slave ALS
} raw_filhdr;     // total 0x10 - 0x08 aligned

// raw size header
typedef struct {
    u16    xdim;    //
    u16    ydim;    //
} raw_sizehdr;      // total 0x04 - 0x02 aligned

// cs0. ofile header for rgb files (snapshot)
//  msbits are implicit 0's
//   if target system needs to modify transparency in ARGB context
//    that's a target adaptation operation
// each pixel occupies 3x(.pbpp) bits - msbits are 0
typedef struct {
    u16    xdim;    //
    u16    ydim;    //
    u32    tstmp;   // (0xFFFFFFFF invalid)
    u32    tdate;   // 0 invalid
    u8     pbpp;    // pixel - bits per pixel - we standardize 8 for now
    //   bit 10 bit R G B components possible down the road
    u8     u8spr;   // <tbd>
    u8     pmod;    // pmod - could encode mosaic type RGB vs BGR for example
    //   standard default format 0 is RGB
    u8     flgs;    // flgs could be context dependent - default 0
    als    main;    // main ALS
    als    slave;   // slave ALS
} rgb_filhdr;       // total 0x30 - 0x08 aligned



// error codes
// error code (-1) means wrong number of command line arguments
#define  HERROR_none      (0x0000)   // default - all OK
#define  HERROR_TOPIO     (0x0001)   // reserved - if top level has explicit files that don't open
#define  HERROR_CRASH     (0x0002)   // nothing specificc here, but crash code
//  has been set for further description
// the rest of the errors are informative, but non-fatal to the application
#define  HERROR_FILE_OP   (0x0003)   // general file IO
#define  HERROR_HIFRAME   (0x0004)   // HASS invalid input frame
#define  HERROR_HOFRAME   (0x0005)   // HASS invalid output buffer pointer
#define  HERROR_HCONFIG   (0x0006)   // HASS invalid configuration
#define  HERROR_TILEMOD   (0x0007)   // HASS invalid X,Y given tile size
#define  HERROR_FNCUNKN   (0x0008)   // HASS - not really - unknown fk() syntax
#define  HERROR_ACCEPT    (0x0009)   // HASS accept input frame to SVM

// maximum number of components
#define maxHassComp       16
#define numOfHassCh       16
#define repUnitSize       8
#define bitsPerPix        10
#define gammaTblLen       (2<<bitsPerPix)

#define H_def_WIDTH       1920
#define H_def_HEIGHT      1080

#define META_DATA_SIZE 86

typedef enum {
    LAMBDA_350,            // 0x00
    LAMBDA_360,            // 0x01
    LAMBDA_370,            // 0x02
    LAMBDA_380,            // 0x03
    LAMBDA_390,            // 0x04
    LAMBDA_400,            // 0x05
    LAMBDA_410,            // 0x06
    LAMBDA_420,            // 0x07
    LAMBDA_430,            // 0x08
    LAMBDA_440,            // 0x09
    LAMBDA_450,            // 0x0A
    LAMBDA_460,            // 0x0B
    LAMBDA_470,            // 0x0C
    LAMBDA_480,            // 0x0D
    LAMBDA_490,            // 0x0E
    LAMBDA_500,            // 0x0F
    LAMBDA_510,            // 0x10
    LAMBDA_520,            // 0x11
    LAMBDA_530,            // 0x12
    LAMBDA_540,            // 0x13
    LAMBDA_550,            // 0x14
    LAMBDA_560,            // 0x15
    LAMBDA_570,            // 0x16
    LAMBDA_580,            // 0x17
    LAMBDA_590,            // 0x18
    LAMBDA_600,            // 0x19
    LAMBDA_610,            // 0x1A
    LAMBDA_620,            // 0x1B
    LAMBDA_630,            // 0x1C
    LAMBDA_640,            // 0x1D
    LAMBDA_650,            // 0x1E
    LAMBDA_660,            // 0x1F
    LAMBDA_670,            // 0x20
    LAMBDA_680,            // 0x21
    LAMBDA_690,            // 0x22
    LAMBDA_700,            // 0x23
    LAMBDA_710,            // 0x24
    LAMBDA_720,            // 0x25
    LAMBDA_730,            // 0x26
    LAMBDA_740,            // 0x27
    LAMBDA_750,            // 0x28
    LAMBDA_760,            // 0x29
    LAMBDA_770,            // 0x2A
    LAMBDA_780,            // 0x2B
    LAMBDA_790,            // 0x2C
    LAMBDA_800,            // 0x2D
    LAMBDA_810,            // 0x2E
    LAMBDA_820,            // 0x2F
    LAMBDA_830,            // 0x30
    LAMBDA_840,            // 0x31
    LAMBDA_850,            // 0x32
    LAMBDA_860,            // 0x33
    LAMBDA_870,            // 0x34
    LAMBDA_880,            // 0x35
    LAMBDA_890,            // 0x36
    LAMBDA_900,            // 0x37
    LAMBDA_910,            // 0x38
    LAMBDA_920,            // 0x39
    LAMBDA_930,            // 0x3A
    LAMBDA_940,            // 0x3B
    LAMBDA_950,            // 0x3C
    LAMBDA_960,            // 0x3D
    LAMBDA_970,            // 0x3E
    LAMBDA_980,            // 0x3F
    LAMBDA_990,            // 0x40
    LAMBDA_1000            // 0x41
} LAMBDA_index;

//
// HASS kernel execution options
//
// ACptr->hasscaptr is low side of hassopts
#define  HSOPT_CAP_COE    (0x0001)
#define  HSOPT_CAP_GDE    (0x0002)
#define  HSOPT_CAP_SHR    (0x0004)
#define  HSOPT_CAP_NR     (0x0008)
#define  HSOPT_CAP_RCN    (0x0010)
#define  HSOPT_CAP_APP    (0x0020)
#define  HSOPT_CAP_CMB    (0x0040)
#define  HSOPT_CAP_PRC    (0x0080)
#define  HSOPT_CAP_OUT    (0x0100)
#define  HSOPT_CAP_RGB    (0x0200)
//--
#define  HSOPT_CAP_ALL    (0x03FF)
//--
// ACptr->hasstmren is high side of hassopts
#define  HSOPT_TMEAS      (0xFFFF0000)

// the number of times in the record
#define  HASS_TMR_NUM     10
//
// conceptually, this timing structure has a built in
//  msg_gblkfnc structure
//   coincidently, the structure is the size of
//
typedef struct {
    lnklst           chn;      // 0x00..0x0F
    gblkfnc          fncptr;   // 0x10
    // msg_gblkfnc emulator to this point
    //  the rest is custom starting at the .dstptr position
    //   all times are unsigned millisec
    //
    u32              tstrt;    // 0x18
    u32              tcoeptr;  // 0x1C
    u32              tguide;   // 0x20
    u32              tshrink;  // 0x24
    u32              tNR;      // 0x28
    u32              trecon;   // 0x2C
    u32              tapp;     // 0x30
    u32              tcomb;    // 0x34
    u32              tproc;    // 0x38
    u32              tout;     // 0x3C
} hass_timer;

// the thread0 version is slightly different
//  it uses it as a running total over N frames
//   so the fncptr is replaced as two integers
typedef struct {
    lnklst           chn;      // 0x00..0x0F <unused>
    u32              numfr;    // 0x10 number of frames to accumulate
    u32              frnum;    // 0x14 frame number being worked
    //---
    u32              tstrt;    // 0x18
    u32              tcoeptr;  // 0x1C
    u32              tguide;   // 0x20
    u32              tshrink;  // 0x24
    u32              tNR;      // 0x28
    u32              trecon;   // 0x2C
    u32              tapp;     // 0x30
    u32              tcomb;    // 0x34
    u32              tproc;    // 0x38
    u32              tout;     // 0x3C
} hass_tmracc;


// this is the contents of const_16colors_WN13_C11-0943_lambda500
// float lutWaveLength [66][16];

typedef float lutWaveConfig [66][numOfHassCh];

//
// lutWaveConfig lutWaveLength = {
//
//    // 350
//     {
//      (float) 8.35E-05      ,
//      (float) -0.00011294   ,
//      (float) -1.32E-05     ,
//      (float) -2.80E-05     ,
//      (float) -1.92E-05     ,
//      (float) -4.42E-05     ,
//      (float) 1.09E-05      ,
//      (float) -1.31E-05     ,
//      (float) 7.16E-05      ,
//      (float) -1.85E-05     ,
//      (float) -7.98E-05     ,
//      (float) -9.10E-05     ,
//      (float) -9.13E-05     ,
//      (float) -1.50E-05     ,
//      (float) 0.00012351    ,
//      (float) 0.00025871
//     },
//
//    // 360
//     {
//      (float) 0.00017416    ,
//      (float) -0.00021931   ,
//      (float) -1.03E-05     ,
//      (float) -5.71E-05     ,
//      (float) -2.92E-05     ,
//      (float) -6.93E-05     ,
//      (float) 4.33E-06      ,
//      (float) -2.40E-05     ,
//      (float) 0.00011817    ,
//      (float) -1.58E-06     ,
//      (float) -0.00014499   ,
//      (float) -0.00016722   ,
//      (float) -0.00017184   ,
//      (float) -3.15E-05     ,
//      (float) 0.00021349    ,
//      (float) 0.0004671
//     },
//
//    // 370
//     {
//      (float) 0.00026421    ,
//      (float) -0.00030891   ,
//      (float) 9.49E-06      ,
//      (float) -8.93E-05     ,
//      (float) -2.74E-05     ,
//      (float) -7.88E-05     ,
//      (float) -8.75E-06     ,
//      (float) -3.34E-05     ,
//      (float) 0.00013505    ,
//      (float) 3.44E-05      ,
//      (float) -0.00017238   ,
//      (float) -0.00023338   ,
//      (float) -0.00023093   ,
//      (float) -4.15E-05     ,
//      (float) 0.00026339    ,
//      (float) 0.00060385
//     },
//
//    // 380
//     {
//      (float) 0.00034447    ,
//      (float) -0.00037287   ,
//      (float) 3.62E-05      ,
//      (float) -0.00011936   ,
//      (float) -1.76E-05     ,
//      (float) -8.16E-05     ,
//      (float) -1.50E-05     ,
//      (float) -4.31E-05     ,
//      (float) 0.00013376    ,
//      (float) 5.70E-05      ,
//      (float) -0.00015984   ,
//      (float) -0.00026578   ,
//      (float) -0.00027134   ,
//      (float) -5.95E-05     ,
//      (float) 0.00027823    ,
//      (float) 0.00067934
//     },
//
//    // 390
//     {
//      (float) 0.00042117    ,
//      (float) -0.00041052   ,
//      (float) 6.17E-05      ,
//      (float) -0.00013122   ,
//      (float) -1.26E-05     ,
//      (float) -7.21E-05     ,
//      (float) -9.20E-06     ,
//      (float) -5.03E-05     ,
//      (float) 0.00010816    ,
//      (float) 6.15E-05      ,
//      (float) -0.00012242   ,
//      (float) -0.00023529   ,
//      (float) -0.00028587   ,
//      (float) -9.65E-05     ,
//      (float) 0.00026149    ,
//      (float) 0.00067812
//     },
//
//    // 400
//     {
//      (float) 0.00049358    ,
//      (float) -0.00042605   ,
//      (float) 8.76E-05      ,
//      (float) -0.000121     ,
//      (float) -1.75E-05     ,
//      (float) -5.19E-05     ,
//      (float) 2.06E-06      ,
//      (float) -4.93E-05     ,
//      (float) 8.07E-05      ,
//      (float) 4.43E-05      ,
//      (float) -9.22E-05     ,
//      (float) -0.00016961   ,
//      (float) -0.00024199   ,
//      (float) -0.00012271   ,
//      (float) 0.00020902    ,
//      (float) 0.00058815
//     },
//
//    // 410
//     {
//      (float) 0.00056882    ,
//      (float) -0.00045043   ,
//      (float) 0.00015032    ,
//      (float) -0.0001177    ,
//      (float) -1.94E-05     ,
//      (float) -2.70E-05     ,
//      (float) 1.32E-05      ,
//      (float) -4.17E-05     ,
//      (float) 4.74E-05      ,
//      (float) 2.28E-05      ,
//      (float) -5.75E-05     ,
//      (float) -8.98E-05     ,
//      (float) -0.00016344   ,
//      (float) -0.00013318   ,
//      (float) 0.00012661    ,
//      (float) 0.00043308
//     },
//
//    // 420
//     {
//      (float) 0.00063723    ,
//      (float) -0.00047388   ,
//      (float) 0.00024064    ,
//      (float) -0.00011615   ,
//      (float) -2.59E-05     ,
//      (float) 1.31E-05      ,
//      (float) 1.36E-05      ,
//      (float) -3.31E-05     ,
//      (float) 3.19E-05      ,
//      (float) -4.65E-06     ,
//      (float) -2.60E-05     ,
//      (float) -2.60E-05     ,
//      (float) -5.38E-05     ,
//      (float) -9.02E-05     ,
//      (float) 2.17E-05      ,
//      (float) 0.00020098
//     },
//
//    // 430
//     {
//      (float) 0.00065882    ,
//      (float) -0.00042457   ,
//      (float) 0.00024841    ,
//      (float) -6.08E-05     ,
//      (float) -3.59E-05     ,
//      (float) 3.86E-05      ,
//      (float) 1.61E-05      ,
//      (float) -2.66E-05     ,
//      (float) 8.43E-06      ,
//      (float) -1.06E-05     ,
//      (float) 8.40E-06      ,
//      (float) 2.90E-05      ,
//      (float) 2.81E-05      ,
//      (float) -3.78E-05     ,
//      (float) -5.29E-05     ,
//      (float) -4.33E-05
//     },
//
//    // 440
//     {
//      (float) 0.00059767    ,
//      (float) -0.00025785   ,
//      (float) 0.00011307    ,
//      (float) 7.06E-05      ,
//      (float) -5.19E-05     ,
//      (float) 2.75E-05      ,
//      (float) 2.68E-05      ,
//      (float) -1.86E-05     ,
//      (float) -2.46E-06     ,
//      (float) -1.11E-05     ,
//      (float) 2.84E-05      ,
//      (float) 5.32E-05      ,
//      (float) 8.49E-05      ,
//      (float) 1.83E-05      ,
//      (float) -9.83E-05     ,
//      (float) -0.00022887
//     },
//
//    // 450
//     {
//      (float) 0.0004586     ,
//      (float) -4.08E-06     ,
//      (float) -0.00012388   ,
//      (float) 0.00022386    ,
//      (float) -5.70E-05     ,
//      (float) -1.55E-05     ,
//      (float) 4.19E-05      ,
//      (float) -7.76E-06     ,
//      (float) -2.31E-05     ,
//      (float) 1.85E-06      ,
//      (float) 4.64E-05      ,
//      (float) 6.04E-05      ,
//      (float) 8.87E-05      ,
//      (float) 3.53E-05      ,
//      (float) -0.00010183   ,
//      (float) -0.00029473
//     },
//
//    // 460
//     {
//      (float) 0.00026866    ,
//      (float) 0.00026264    ,
//      (float) -0.00031659   ,
//      (float) 0.00027451    ,
//      (float) -9.59E-06     ,
//      (float) -6.54E-05     ,
//      (float) 3.93E-05      ,
//      (float) 8.10E-06      ,
//      (float) -2.05E-05     ,
//      (float) 2.05E-06      ,
//      (float) 5.47E-05      ,
//      (float) 4.41E-05      ,
//      (float) 8.44E-05      ,
//      (float) 4.37E-05      ,
//      (float) -9.83E-05     ,
//      (float) -0.00028939
//     },
//
//    // 470
//     {
//      (float) 6.86E-05      ,
//      (float) 0.00047258    ,
//      (float) -0.00037985   ,
//      (float) 0.00016359    ,
//      (float) 0.00010486    ,
//      (float) -0.00010601   ,
//      (float) 1.78E-05      ,
//      (float) 2.68E-05      ,
//      (float) -1.43E-05     ,
//      (float) -9.97E-07     ,
//      (float) 5.62E-05      ,
//      (float) 2.06E-05      ,
//      (float) 7.14E-05      ,
//      (float) 4.18E-05      ,
//      (float) -8.69E-05     ,
//      (float) -0.0002355
//     },
//
//    // 480
//     {
//      (float) -0.00012052   ,
//      (float) 0.00060666    ,
//      (float) -0.00033082   ,
//      (float) -4.02E-05     ,
//      (float) 0.00021445    ,
//      (float) -9.30E-05     ,
//      (float) -3.47E-05     ,
//      (float) 4.16E-05      ,
//      (float) 6.03E-06      ,
//      (float) -1.76E-05     ,
//      (float) 4.69E-05      ,
//      (float) 1.43E-05      ,
//      (float) 4.00E-05      ,
//      (float) 2.01E-05      ,
//      (float) -5.68E-05     ,
//      (float) -0.00014606
//     },
//
//    // 490
//     {
//      (float) -0.00027821   ,
//      (float) 0.00065588    ,
//      (float) -0.00020061   ,
//      (float) -0.00022881   ,
//      (float) 0.00023055    ,
//      (float) -7.42E-06     ,
//      (float) -8.80E-05     ,
//      (float) 3.10E-05      ,
//      (float) 4.30E-05      ,
//      (float) -3.30E-05     ,
//      (float) 1.69E-05      ,
//      (float) -2.60E-06     ,
//      (float) 3.55E-05      ,
//      (float) 1.64E-05      ,
//      (float) -3.44E-05     ,
//      (float) -7.50E-05
//     },
//
//    // 500
//     {
//      (float) -0.00037618   ,
//      (float) 0.00059427    ,
//      (float) -2.65E-05     ,
//      (float) -0.00032275   ,
//      (float) 0.0001173     ,
//      (float) 0.00012076    ,
//      (float) -0.00011724   ,
//      (float) -8.69E-07     ,
//      (float) 6.40E-05      ,
//      (float) -2.97E-05     ,
//      (float) -7.16E-06     ,
//      (float) -2.79E-05     ,
//      (float) 1.19E-05      ,
//      (float) 1.41E-05      ,
//      (float) 3.41E-06      ,
//      (float) 2.42E-06
//     },
//
//    // 510
//     {
//      (float) -0.00040126   ,
//      (float) 0.00043195    ,
//      (float) 0.00016375    ,
//      (float) -0.00031532   ,
//      (float) -7.19E-05     ,
//      (float) 0.00023543    ,
//      (float) -0.00010928   ,
//      (float) -4.32E-05     ,
//      (float) 5.79E-05      ,
//      (float) -2.57E-06     ,
//      (float) -2.49E-05     ,
//      (float) -4.21E-05     ,
//      (float) -2.67E-05     ,
//      (float) -2.87E-06     ,
//      (float) 5.23E-05      ,
//      (float) 7.26E-05
//     },
//
//    // 520
//     {
//      (float) -0.00036641   ,
//      (float) 0.00022287    ,
//      (float) 0.00032229    ,
//      (float) -0.00022255   ,
//      (float) -0.00025152   ,
//      (float) 0.00026195    ,
//      (float) -3.94E-05     ,
//      (float) -8.28E-05     ,
//      (float) 2.23E-05      ,
//      (float) 3.53E-05      ,
//      (float) -3.09E-05     ,
//      (float) -5.10E-05     ,
//      (float) -5.36E-05     ,
//      (float) -8.81E-06     ,
//      (float) 7.18E-05      ,
//      (float) 0.00011806
//     },
//
//    // 530
//     {
//      (float) -0.00029432   ,
//      (float) 1.77E-05      ,
//      (float) 0.00042084    ,
//      (float) -8.82E-05     ,
//      (float) -0.00034768   ,
//      (float) 0.00016529    ,
//      (float) 7.74E-05      ,
//      (float) -9.41E-05     ,
//      (float) -4.17E-05     ,
//      (float) 6.59E-05      ,
//      (float) -1.83E-05     ,
//      (float) -3.04E-05     ,
//      (float) -8.21E-05     ,
//      (float) -3.04E-05     ,
//      (float) 8.03E-05      ,
//      (float) 0.00013866
//     },
//
//    // 540
//     {
//      (float) -0.00019977   ,
//      (float) -0.00015819   ,
//      (float) 0.00044871    ,
//      (float) 6.72E-05      ,
//      (float) -0.00035346   ,
//      (float) -1.42E-05     ,
//      (float) 0.00018627    ,
//      (float) -5.77E-05     ,
//      (float) -0.00011321   ,
//      (float) 7.39E-05      ,
//      (float) 6.14E-06      ,
//      (float) 1.06E-05      ,
//      (float) -0.00010474   ,
//      (float) -5.64E-05     ,
//      (float) 7.13E-05      ,
//      (float) 0.00013819
//     },
//
//    // 550
//     {
//      (float) -0.00010004   ,
//      (float) -0.00027681   ,
//      (float) 0.00040446    ,
//      (float) 0.0002216     ,
//      (float) -0.00028926   ,
//      (float) -0.00019085   ,
//      (float) 0.00022834    ,
//      (float) 2.14E-05      ,
//      (float) -0.00016211   ,
//      (float) 5.27E-05      ,
//      (float) 2.42E-05      ,
//      (float) 4.80E-05      ,
//      (float) -8.52E-05     ,
//      (float) -5.46E-05     ,
//      (float) 2.95E-05      ,
//      (float) 9.18E-05
//     },
//
//    // 560
//     {
//      (float) -1.04E-05     ,
//      (float) -0.00033435   ,
//      (float) 0.00030974    ,
//      (float) 0.00033321    ,
//      (float) -0.00016645   ,
//      (float) -0.00030324   ,
//      (float) 0.0001732     ,
//      (float) 0.00011267    ,
//      (float) -0.00016791   ,
//      (float) 1.34E-05      ,
//      (float) 4.29E-05      ,
//      (float) 7.07E-05      ,
//      (float) -4.78E-05     ,
//      (float) -3.65E-05     ,
//      (float) -2.70E-05     ,
//      (float) 2.68E-05
//     },
//
//    // 570
//     {
//      (float) 5.65E-05      ,
//      (float) -0.00033865   ,
//      (float) 0.00017949    ,
//      (float) 0.00038221    ,
//      (float) -6.32E-06     ,
//      (float) -0.00033871   ,
//      (float) 4.41E-05      ,
//      (float) 0.00017592    ,
//      (float) -0.00011333   ,
//      (float) -3.96E-05     ,
//      (float) 5.32E-05      ,
//      (float) 8.50E-05      ,
//      (float) -2.08E-05     ,
//      (float) -3.81E-05     ,
//      (float) -5.16E-05     ,
//      (float) -1.47E-05
//     },
//
//    // 580
//     {
//      (float) 9.54E-05      ,
//      (float) -0.00030233   ,
//      (float) 4.63E-05      ,
//      (float) 0.00035996    ,
//      (float) 0.00016289    ,
//      (float) -0.00030875   ,
//      (float) -0.00010602   ,
//      (float) 0.00019062    ,
//      (float) -7.01E-06     ,
//      (float) -9.09E-05     ,
//      (float) 3.06E-05      ,
//      (float) 8.48E-05      ,
//      (float) 6.52E-06      ,
//      (float) -3.54E-05     ,
//      (float) -6.27E-05     ,
//      (float) -2.77E-05
//     },
//
//    // 590
//     {
//      (float) 0.00011107    ,
//      (float) -0.0002441    ,
//      (float) -6.53E-05     ,
//      (float) 0.00028938    ,
//      (float) 0.00030158    ,
//      (float) -0.00022922   ,
//      (float) -0.00023127   ,
//      (float) 0.00015709    ,
//      (float) 0.0001228     ,
//      (float) -0.00012972   ,
//      (float) -2.00E-05     ,
//      (float) 5.36E-05      ,
//      (float) 5.15E-05      ,
//      (float) -8.58E-06     ,
//      (float) -6.80E-05     ,
//      (float) -3.75E-05
//     },
//
//    // 600
//     {
//      (float) 0.000112      ,
//      (float) -0.00017614   ,
//      (float) -0.0001524    ,
//      (float) 0.00020266    ,
//      (float) 0.000382      ,
//      (float) -0.00010924   ,
//      (float) -0.00029963   ,
//      (float) 6.73E-05      ,
//      (float) 0.00023228    ,
//      (float) -0.00012471   ,
//      (float) -7.21E-05     ,
//      (float) -4.85E-06     ,
//      (float) 8.90E-05      ,
//      (float) 3.74E-05      ,
//      (float) -5.88E-05     ,
//      (float) -5.67E-05
//     },
//
//    // 610
//     {
//      (float) 0.00010255    ,
//      (float) -0.00010401   ,
//      (float) -0.00022155   ,
//      (float) 0.00011003    ,
//      (float) 0.0004095     ,
//      (float) 2.68E-05      ,
//      (float) -0.00030487   ,
//      (float) -5.29E-05     ,
//      (float) 0.00026563    ,
//      (float) -5.75E-05     ,
//      (float) -9.59E-05     ,
//      (float) -6.86E-05     ,
//      (float) 9.17E-05      ,
//      (float) 4.86E-05      ,
//      (float) -1.91E-05     ,
//      (float) -5.19E-05
//     },
//
//    // 620
//     {
//      (float) 8.90E-05      ,
//      (float) -3.01E-05     ,
//      (float) -0.00026119   ,
//      (float) 9.68E-06      ,
//      (float) 0.0003936     ,
//      (float) 0.00016693    ,
//      (float) -0.00024827   ,
//      (float) -0.0001742    ,
//      (float) 0.00020451    ,
//      (float) 5.15E-05      ,
//      (float) -8.41E-05     ,
//      (float) -0.00010378   ,
//      (float) 6.68E-05      ,
//      (float) 2.76E-05      ,
//      (float) 2.06E-05      ,
//      (float) -4.06E-05
//     },
//
//    // 630
//     {
//      (float) 7.23E-05      ,
//      (float) 4.11E-05      ,
//      (float) -0.00026775   ,
//      (float) -8.59E-05     ,
//      (float) 0.00032816    ,
//      (float) 0.00029992    ,
//      (float) -0.00013694   ,
//      (float) -0.00027039   ,
//      (float) 7.08E-05      ,
//      (float) 0.00016053    ,
//      (float) -4.43E-05     ,
//      (float) -0.00010957   ,
//      (float) 3.13E-05      ,
//      (float) 9.00E-06      ,
//      (float) 3.51E-05      ,
//      (float) -3.72E-05
//     },
//
//    // 640
//     {
//      (float) 5.13E-05      ,
//      (float) 0.00010245    ,
//      (float) -0.00024074   ,
//      (float) -0.00016263   ,
//      (float) 0.00021675    ,
//      (float) 0.00040276    ,
//      (float) 6.05E-06      ,
//      (float) -0.00030797   ,
//      (float) -8.27E-05     ,
//      (float) 0.00020528    ,
//      (float) 1.08E-05      ,
//      (float) -8.19E-05     ,
//      (float) 1.52E-06      ,
//      (float) 4.35E-06      ,
//      (float) 3.29E-05      ,
//      (float) -5.74E-05
//     },
//
//    // 650
//     {
//      (float) 2.49E-05      ,
//      (float) 0.00014988    ,
//      (float) -0.00018869   ,
//      (float) -0.00021089   ,
//      (float) 8.30E-05      ,
//      (float) 0.00044546    ,
//      (float) 0.00016469    ,
//      (float) -0.00028489   ,
//      (float) -0.00020895   ,
//      (float) 0.00016503    ,
//      (float) 7.32E-05      ,
//      (float) -2.50E-05     ,
//      (float) -2.58E-05     ,
//      (float) 1.64E-05      ,
//      (float) 3.92E-06      ,
//      (float) -8.14E-05
//     },
//
//    // 660
//     {
//      (float) -1.08E-05     ,
//      (float) 0.00018028    ,
//      (float) -0.00012607   ,
//      (float) -0.00022984   ,
//      (float) -5.39E-05     ,
//      (float) 0.00041633    ,
//      (float) 0.00030935    ,
//      (float) -0.00019767   ,
//      (float) -0.00027551   ,
//      (float) 4.47E-05      ,
//      (float) 0.00011608    ,
//      (float) 3.88E-05      ,
//      (float) -4.03E-05     ,
//      (float) 2.27E-05      ,
//      (float) -1.32E-05     ,
//      (float) -9.00E-05
//     },
//
//    // 670
//     {
//      (float) -5.44E-05     ,
//      (float) 0.00019696    ,
//      (float) -6.59E-05     ,
//      (float) -0.00021939   ,
//      (float) -0.00017514   ,
//      (float) 0.00031866    ,
//      (float) 0.00041779    ,
//      (float) -5.46E-05     ,
//      (float) -0.0002879    ,
//      (float) -0.00010143   ,
//      (float) 0.00011433    ,
//      (float) 8.63E-05      ,
//      (float) -3.13E-05     ,
//      (float) 1.66E-05      ,
//      (float) -1.74E-05     ,
//      (float) -7.23E-05
//     },
//
//    // 680
//     {
//      (float) -0.00010099   ,
//      (float) 0.00020525    ,
//      (float) -1.28E-05     ,
//      (float) -0.00018574   ,
//      (float) -0.00026246   ,
//      (float) 0.00017404    ,
//      (float) 0.00046593    ,
//      (float) 0.00012887    ,
//      (float) -0.00025351   ,
//      (float) -0.00022591   ,
//      (float) 6.10E-05      ,
//      (float) 0.00010539    ,
//      (float) -4.14E-06     ,
//      (float) 1.73E-05      ,
//      (float) -1.90E-05     ,
//      (float) -4.89E-05
//     },
//
//    // 690
//     {
//      (float) -0.00014473   ,
//      (float) 0.00020722    ,
//      (float) 3.35E-05      ,
//      (float) -0.00013683   ,
//      (float) -0.00031093   ,
//      (float) 2.32E-05      ,
//      (float) 0.00044215    ,
//      (float) 0.00031108    ,
//      (float) -0.00018055   ,
//      (float) -0.00029103   ,
//      (float) -1.77E-05     ,
//      (float) 8.59E-05      ,
//      (float) 2.82E-05      ,
//      (float) 2.48E-05      ,
//      (float) -2.46E-05     ,
//      (float) -3.30E-05
//     },
//
//    // 700
//     {
//      (float) -0.00018147   ,
//      (float) 0.00020381    ,
//      (float) 7.53E-05      ,
//      (float) -8.24E-05     ,
//      (float) -0.00032061   ,
//      (float) -0.00010361   ,
//      (float) 0.00035458    ,
//      (float) 0.00045004    ,
//      (float) -7.25E-05     ,
//      (float) -0.00028525   ,
//      (float) -9.58E-05     ,
//      (float) 3.66E-05      ,
//      (float) 5.26E-05      ,
//      (float) 3.01E-05      ,
//      (float) -3.48E-05     ,
//      (float) -3.39E-05
//     },
//
//    // 710
//     {
//      (float) -0.00020884   ,
//      (float) 0.0001943     ,
//      (float) 0.00011546    ,
//      (float) -3.26E-05     ,
//      (float) -0.00029467   ,
//      (float) -0.0001912    ,
//      (float) 0.00021894    ,
//      (float) 0.00052404    ,
//      (float) 6.55E-05      ,
//      (float) -0.00022186   ,
//      (float) -0.00015422   ,
//      (float) -2.36E-05     ,
//      (float) 5.84E-05      ,
//      (float) 2.33E-05      ,
//      (float) -4.53E-05     ,
//      (float) -5.34E-05
//     },
//
//    // 720
//     {
//      (float) -0.00022693   ,
//      (float) 0.00017761    ,
//      (float) 0.00015005    ,
//      (float) 9.84E-06      ,
//      (float) -0.00024492   ,
//      (float) -0.00023498   ,
//      (float) 6.12E-05      ,
//      (float) 0.00052175    ,
//      (float) 0.00021945    ,
//      (float) -0.00011796   ,
//      (float) -0.00017953   ,
//      (float) -8.46E-05     ,
//      (float) 4.12E-05      ,
//      (float) -1.30E-07     ,
//      (float) -5.92E-05     ,
//      (float) -7.05E-05
//     },
//
//    // 730
//     {
//      (float) -0.00023349   ,
//      (float) 0.00015485    ,
//      (float) 0.0001752     ,
//      (float) 4.47E-05      ,
//      (float) -0.00018038   ,
//      (float) -0.00024193   ,
//      (float) -8.20E-05     ,
//      (float) 0.00044097    ,
//      (float) 0.00034984    ,
//      (float) 2.74E-05      ,
//      (float) -0.00015867   ,
//      (float) -0.0001361    ,
//      (float) -4.56E-06     ,
//      (float) -4.20E-05     ,
//      (float) -7.30E-05     ,
//      (float) -8.33E-05
//     },
//
//    // 740
//     {
//      (float) -0.00022753   ,
//      (float) 0.00012927    ,
//      (float) 0.00018578    ,
//      (float) 7.37E-05      ,
//      (float) -0.0001115    ,
//      (float) -0.00022232   ,
//      (float) -0.0001872    ,
//      (float) 0.00030915    ,
//      (float) 0.00042597    ,
//      (float) 0.00017746    ,
//      (float) -7.99E-05     ,
//      (float) -0.0001565    ,
//      (float) -7.58E-05     ,
//      (float) -0.0001011    ,
//      (float) -8.27E-05     ,
//      (float) -9.66E-05
//     },
//
//    // 750
//     {
//      (float) -0.00021082   ,
//      (float) 0.00010412    ,
//      (float) 0.00018338    ,
//      (float) 9.65E-05      ,
//      (float) -4.57E-05     ,
//      (float) -0.00018689   ,
//      (float) -0.00024998   ,
//      (float) 0.00015939    ,
//      (float) 0.00043308    ,
//      (float) 0.00032855    ,
//      (float) 2.55E-05      ,
//      (float) -0.00014868   ,
//      (float) -0.00014451   ,
//      (float) -0.00016539   ,
//      (float) -9.09E-05     ,
//      (float) -0.0001192
//     },
//
//    // 760
//     {
//      (float) -0.00018696   ,
//      (float) 7.98E-05      ,
//      (float) 0.0001724     ,
//      (float) 0.00010987    ,
//      (float) 1.50E-05      ,
//      (float) -0.00014212   ,
//      (float) -0.00027988   ,
//      (float) 1.45E-05      ,
//      (float) 0.00039022    ,
//      (float) 0.00045335    ,
//      (float) 0.00014322    ,
//      (float) -0.00011548   ,
//      (float) -0.00020194   ,
//      (float) -0.00021956   ,
//      (float) -9.92E-05     ,
//      (float) -0.00015327
//     },
//
//    // 770
//     {
//      (float) -0.00016111   ,
//      (float) 5.57E-05      ,
//      (float) 0.00015189    ,
//      (float) 0.00011466    ,
//      (float) 6.59E-05      ,
//      (float) -9.45E-05     ,
//      (float) -0.00028486   ,
//      (float) -0.00011497   ,
//      (float) 0.00032522    ,
//      (float) 0.00053491    ,
//      (float) 0.00024616    ,
//      (float) -5.68E-05     ,
//      (float) -0.00024238   ,
//      (float) -0.00025973   ,
//      (float) -0.0001169    ,
//      (float) -0.00017188
//     },
//
//    // 780
//     {
//      (float) -0.00013719   ,
//      (float) 3.08E-05      ,
//      (float) 0.00012244    ,
//      (float) 0.0001122     ,
//      (float) 0.00010311    ,
//      (float) -5.07E-05     ,
//      (float) -0.0002723    ,
//      (float) -0.00021559   ,
//      (float) 0.00025227    ,
//      (float) 0.00056439    ,
//      (float) 0.00032177    ,
//      (float) 6.53E-06      ,
//      (float) -0.00025144   ,
//      (float) -0.0002825    ,
//      (float) -0.00012933   ,
//      (float) -0.0001749
//     },
//
//    // 790
//     {
//      (float) -0.0001174    ,
//      (float) 4.36E-06      ,
//      (float) 8.35E-05      ,
//      (float) 0.00010468    ,
//      (float) 0.00012565    ,
//      (float) -1.91E-05     ,
//      (float) -0.00023888   ,
//      (float) -0.00028608   ,
//      (float) 0.00017774    ,
//      (float) 0.00053588    ,
//      (float) 0.00037568    ,
//      (float) 6.71E-05      ,
//      (float) -0.00023436   ,
//      (float) -0.00029123   ,
//      (float) -0.00012781   ,
//      (float) -0.00015634
//     },
//
//    // 800
//     {
//      (float) -0.00010331   ,
//      (float) -2.28E-05     ,
//      (float) 4.03E-05      ,
//      (float) 9.25E-05      ,
//      (float) 0.00013653    ,
//      (float) -7.85E-07     ,
//      (float) -0.0001938    ,
//      (float) -0.00033367   ,
//      (float) 0.00011872    ,
//      (float) 0.0004713     ,
//      (float) 0.00039017    ,
//      (float) 0.0001206     ,
//      (float) -0.0001959    ,
//      (float) -0.00028526   ,
//      (float) -0.00011785   ,
//      (float) -0.00011448
//     },
//
//    // 810
//     {
//      (float) -9.54E-05     ,
//      (float) -5.17E-05     ,
//      (float) -5.30E-06     ,
//      (float) 7.43E-05      ,
//      (float) 0.00013357    ,
//      (float) 7.74E-06      ,
//      (float) -0.00015365   ,
//      (float) -0.00033853   ,
//      (float) 7.58E-05      ,
//      (float) 0.00037175    ,
//      (float) 0.00034756    ,
//      (float) 0.00016037    ,
//      (float) -0.00013155   ,
//      (float) -0.00025106   ,
//      (float) -8.91E-05     ,
//      (float) -6.02E-05
//     },
//
//    // 820
//     {
//      (float) -8.83E-05     ,
//      (float) -7.86E-05     ,
//      (float) -5.17E-05     ,
//      (float) 5.57E-05      ,
//      (float) 0.00012418    ,
//      (float) 5.79E-06      ,
//      (float) -0.0001038    ,
//      (float) -0.00032146   ,
//      (float) 2.75E-05      ,
//      (float) 0.0002567     ,
//      (float) 0.00028849    ,
//      (float) 0.00019938    ,
//      (float) -5.91E-05     ,
//      (float) -0.0002018    ,
//      (float) -6.57E-05     ,
//      (float) -1.41E-06
//     },
//
//    // 830
//     {
//      (float) -7.89E-05     ,
//      (float) -0.00010037   ,
//      (float) -8.78E-05     ,
//      (float) 3.83E-05      ,
//      (float) 0.00011681    ,
//      (float) 1.25E-05      ,
//      (float) -6.83E-05     ,
//      (float) -0.00030253   ,
//      (float) -1.80E-05     ,
//      (float) 0.00016624    ,
//      (float) 0.00023401    ,
//      (float) 0.00022636    ,
//      (float) 3.12E-07      ,
//      (float) -0.00013033   ,
//      (float) -5.71E-05     ,
//      (float) 2.93E-05
//     },
//
//    // 840
//     {
//      (float) -7.32E-05     ,
//      (float) -0.00012121   ,
//      (float) -0.00012296   ,
//      (float) 1.94E-05      ,
//      (float) 0.00010085    ,
//      (float) 1.27E-05      ,
//      (float) -3.66E-05     ,
//      (float) -0.00025774   ,
//      (float) -5.19E-05     ,
//      (float) 6.03E-05      ,
//      (float) 0.00015006    ,
//      (float) 0.0002392     ,
//      (float) 6.14E-05      ,
//      (float) -2.56E-05     ,
//      (float) -3.36E-05     ,
//      (float) 5.07E-05
//     },
//
//    // 850
//     {
//      (float) -5.82E-05     ,
//      (float) -0.0001325    ,
//      (float) -0.00014597   ,
//      (float) 3.95E-06      ,
//      (float) 9.43E-05      ,
//      (float) 2.46E-05      ,
//      (float) -2.73E-06     ,
//      (float) -0.00022544   ,
//      (float) -0.00010659   ,
//      (float) -3.00E-05     ,
//      (float) 0.00011427    ,
//      (float) 0.0002768     ,
//      (float) 0.00012171    ,
//      (float) 5.24E-05      ,
//      (float) -4.74E-05     ,
//      (float) 3.35E-05
//     },
//
//    // 860
//     {
//      (float) -4.90E-05     ,
//      (float) -0.00014545   ,
//      (float) -0.00016856   ,
//      (float) -1.09E-05     ,
//      (float) 8.09E-05      ,
//      (float) 3.43E-05      ,
//      (float) 1.19E-05      ,
//      (float) -0.00017778   ,
//      (float) -0.00014303   ,
//      (float) -0.00010971   ,
//      (float) 6.05E-05      ,
//      (float) 0.00027807    ,
//      (float) 0.00016791    ,
//      (float) 0.00014425    ,
//      (float) -3.49E-05     ,
//      (float) 3.06E-05
//     },
//
//    // 870
//     {
//      (float) -3.75E-05     ,
//      (float) -0.00015322   ,
//      (float) -0.00018643   ,
//      (float) -2.37E-05     ,
//      (float) 6.85E-05      ,
//      (float) 3.74E-05      ,
//      (float) 4.21E-05      ,
//      (float) -0.00013164   ,
//      (float) -0.00018803   ,
//      (float) -0.00019107   ,
//      (float) 1.72E-05      ,
//      (float) 0.000287      ,
//      (float) 0.00023087    ,
//      (float) 0.00021679    ,
//      (float) -2.76E-05     ,
//      (float) 7.81E-06
//     },
//
//    // 880
//     {
//      (float) -3.09E-05     ,
//      (float) -0.00016222   ,
//      (float) -0.00020071   ,
//      (float) -3.50E-05     ,
//      (float) 5.25E-05      ,
//      (float) 4.44E-05      ,
//      (float) 4.30E-05      ,
//      (float) -7.61E-05     ,
//      (float) -0.00021367   ,
//      (float) -0.00024868   ,
//      (float) -3.04E-05     ,
//      (float) 0.00025351    ,
//      (float) 0.00027473    ,
//      (float) 0.00029394    ,
//      (float) 1.56E-06      ,
//      (float) -1.61E-06
//     },
//
//    // 890
//     {
//      (float) -2.64E-05     ,
//      (float) -0.00016895   ,
//      (float) -0.00021624   ,
//      (float) -4.40E-05     ,
//      (float) 3.29E-05      ,
//      (float) 3.50E-05      ,
//      (float) 6.69E-05      ,
//      (float) -2.05E-05     ,
//      (float) -0.00024509   ,
//      (float) -0.00030765   ,
//      (float) -8.33E-05     ,
//      (float) 0.00021396    ,
//      (float) 0.00033142    ,
//      (float) 0.0003614     ,
//      (float) 3.91E-05      ,
//      (float) -9.66E-06
//     },
//
//    // 900
//     {
//      (float) -2.42E-05     ,
//      (float) -0.00017557   ,
//      (float) -0.00022736   ,
//      (float) -5.01E-05     ,
//      (float) 1.48E-05      ,
//      (float) 3.45E-05      ,
//      (float) 5.72E-05      ,
//      (float) 3.66E-05      ,
//      (float) -0.00026276   ,
//      (float) -0.00033563   ,
//      (float) -0.00012087   ,
//      (float) 0.00014795    ,
//      (float) 0.00035653    ,
//      (float) 0.00041109    ,
//      (float) 9.00E-05      ,
//      (float) 1.80E-07
//     },
//
//    // 910
//     {
//      (float) -2.36E-05     ,
//      (float) -0.00017873   ,
//      (float) -0.00023846   ,
//      (float) -5.54E-05     ,
//      (float) -5.26E-06     ,
//      (float) 1.91E-05      ,
//      (float) 7.27E-05      ,
//      (float) 9.00E-05      ,
//      (float) -0.00028249   ,
//      (float) -0.00036914   ,
//      (float) -0.0001687    ,
//      (float) 9.07E-05      ,
//      (float) 0.00039333    ,
//      (float) 0.00045169    ,
//      (float) 0.00014194    ,
//      (float) 7.90E-06
//     },
//
//    // 920
//     {
//      (float) -2.52E-05     ,
//      (float) -0.00018209   ,
//      (float) -0.00024679   ,
//      (float) -5.68E-05     ,
//      (float) -2.31E-05     ,
//      (float) 1.20E-05      ,
//      (float) 5.88E-05      ,
//      (float) 0.00013828    ,
//      (float) -0.00028852   ,
//      (float) -0.00037475   ,
//      (float) -0.00019222   ,
//      (float) 1.45E-05      ,
//      (float) 0.00038608    ,
//      (float) 0.00047528    ,
//      (float) 0.00019965    ,
//      (float) 4.28E-05
//     },
//
//    // 930
//     {
//      (float) -2.57E-05     ,
//      (float) -0.00017957   ,
//      (float) -0.00024945   ,
//      (float) -5.79E-05     ,
//      (float) -4.09E-05     ,
//      (float) -2.78E-06     ,
//      (float) 6.94E-05      ,
//      (float) 0.00017537    ,
//      (float) -0.00029191   ,
//      (float) -0.00038642   ,
//      (float) -0.00022623   ,
//      (float) -3.42E-05     ,
//      (float) 0.00039807    ,
//      (float) 0.00048282    ,
//      (float) 0.00024262    ,
//      (float) 6.01E-05
//     },
//
//    // 940
//     {
//      (float) -3.03E-05     ,
//      (float) -0.0001785    ,
//      (float) -0.00025305   ,
//      (float) -5.57E-05     ,
//      (float) -5.86E-05     ,
//      (float) -1.46E-05     ,
//      (float) 5.67E-05      ,
//      (float) 0.00020873    ,
//      (float) -0.00027734   ,
//      (float) -0.0003827    ,
//      (float) -0.00024439   ,
//      (float) -9.51E-05     ,
//      (float) 0.00036299    ,
//      (float) 0.00047463    ,
//      (float) 0.0003012     ,
//      (float) 0.00011175
//     },
//
//    // 950
//     {
//      (float) -2.77E-05     ,
//      (float) -0.00016608   ,
//      (float) -0.00023866   ,
//      (float) -5.29E-05     ,
//      (float) -6.57E-05     ,
//      (float) -2.13E-05     ,
//      (float) 6.07E-05      ,
//      (float) 0.00021603    ,
//      (float) -0.00025811   ,
//      (float) -0.0003758    ,
//      (float) -0.000247     ,
//      (float) -0.00010581   ,
//      (float) 0.00034645    ,
//      (float) 0.00044054    ,
//      (float) 0.00031294    ,
//      (float) 0.00011081
//     },
//
//    // 960
//     {
//      (float) -3.17E-05     ,
//      (float) -0.00015555   ,
//      (float) -0.00022589   ,
//      (float) -4.79E-05     ,
//      (float) -7.49E-05     ,
//      (float) -3.40E-05     ,
//      (float) 5.35E-05      ,
//      (float) 0.00021988    ,
//      (float) -0.00021702   ,
//      (float) -0.00035622   ,
//      (float) -0.00025337   ,
//      (float) -0.00013527   ,
//      (float) 0.00029386    ,
//      (float) 0.00040672    ,
//      (float) 0.0003419     ,
//      (float) 0.00014253
//     },
//
//    // 970
//     {
//      (float) -2.42E-05     ,
//      (float) -0.00013134   ,
//      (float) -0.00019104   ,
//      (float) -4.19E-05     ,
//      (float) -6.70E-05     ,
//      (float) -2.59E-05     ,
//      (float) 4.36E-05      ,
//      (float) 0.00019674    ,
//      (float) -0.00018208   ,
//      (float) -0.00031773   ,
//      (float) -0.00021956   ,
//      (float) -0.00010997   ,
//      (float) 0.00025291    ,
//      (float) 0.00034089    ,
//      (float) 0.00029964    ,
//      (float) 0.00011537
//     },
//
//    // 980
//     {
//      (float) -2.23E-05     ,
//      (float) -0.00010657   ,
//      (float) -0.00015557   ,
//      (float) -3.37E-05     ,
//      (float) -6.08E-05     ,
//      (float) -3.04E-05     ,
//      (float) 3.90E-05      ,
//      (float) 0.00016758    ,
//      (float) -0.0001367    ,
//      (float) -0.00025982   ,
//      (float) -0.0001957    ,
//      (float) -0.00010466   ,
//      (float) 0.00019676    ,
//      (float) 0.00027984    ,
//      (float) 0.00026437    ,
//      (float) 0.00010588
//     },
//
//    // 990
//     {
//      (float) -1.68E-05     ,
//      (float) -7.74E-05     ,
//      (float) -0.00011301   ,
//      (float) -2.41E-05     ,
//      (float) -4.67E-05     ,
//      (float) -2.10E-05     ,
//      (float) 1.75E-05      ,
//      (float) 0.00012937    ,
//      (float) -9.17E-05     ,
//      (float) -0.00018756   ,
//      (float) -0.00014492   ,
//      (float) -8.22E-05     ,
//      (float) 0.00012836    ,
//      (float) 0.0001961     ,
//      (float) 0.00020082    ,
//      (float) 9.36E-05
//     },
//
//    // 1000
//     {
//      (float) -7.85E-06     ,
//      (float) -3.94E-05     ,
//      (float) -5.73E-05     ,
//      (float) -1.32E-05     ,
//      (float) -2.50E-05     ,
//      (float) -1.02E-05     ,
//      (float) 1.01E-05      ,
//      (float) 7.00E-05      ,
//      (float) -4.81E-05     ,
//      (float) -9.99E-05     ,
//      (float) -8.00E-05     ,
//      (float) -4.04E-05     ,
//      (float) 7.29E-05      ,
//      (float) 0.00010326    ,
//      (float) 0.00010377    ,
//      (float) 4.12E-05
//     }
//    };
//



lutWaveConfig lutWaveLength = {

        // 350
        {
                (float) 0.536333632   ,
                (float) -0.145395237  ,
                (float) -0.036944691  ,
                (float) 0.013983367   ,
                (float) 9.53E-06      ,
                (float) 0.006554703   ,
                (float) 0.027847558   ,
                (float) -0.028205087  ,
                (float) 0.027092774   ,
                (float) -0.002534485  ,
                (float) 0.015492935   ,
                (float) -0.016843601  ,
                (float) -0.005680743  ,
                (float) -0.007269762  ,
                (float) -0.054821155  ,
                (float) -0.006872507
        },

        // 360
        {
                (float) 1.072587814   ,
                (float) -2.90E-01     ,
                (float) -7.39E-02     ,
                (float) 2.80E-02      ,
                (float) 2.07E-05      ,
                (float) 1.31E-02      ,
                (float) 5.56E-02      ,
                (float) -5.64E-02     ,
                (float) 5.40E-02      ,
                (float) -5.08E-03     ,
                (float) 3.10E-02      ,
                (float) -3.36E-02     ,
                (float) -1.14E-02     ,
                (float) -1.46E-02     ,
                (float) -1.09E-01     ,
                (float) -1.37E-02
        },

        // 370
        {
                (float) 1.608683093   ,
                (float) -4.36E-01     ,
                (float) -1.11E-01     ,
                (float) 4.21E-02      ,
                (float) 4.69E-05      ,
                (float) 1.96E-02      ,
                (float) 8.34E-02      ,
                (float) -8.46E-02     ,
                (float) 8.10E-02      ,
                (float) -7.55E-03     ,
                (float) 4.65E-02      ,
                (float) -5.05E-02     ,
                (float) -1.70E-02     ,
                (float) -2.20E-02     ,
                (float) -1.64E-01     ,
                (float) -2.05E-02
        },

        // 380
        {
                (float) 2.142116766   ,
                (float) -0.579594674  ,
                (float) -1.48E-01     ,
                (float) 5.60E-02      ,
                (float) 2.65E-04      ,
                (float) 2.59E-02      ,
                (float) 1.11E-01      ,
                (float) -1.12E-01     ,
                (float) 1.06E-01      ,
                (float) -1.05E-02     ,
                (float) 6.32E-02      ,
                (float) -6.55E-02     ,
                (float) -2.19E-02     ,
                (float) -3.01E-02     ,
                (float) -2.19E-01     ,
                (float) -2.78E-02
        },

        // 390
        {
                (float) 2.663831424   ,
                (float) -0.720063952  ,
                (float) -1.84E-01     ,
                (float) 7.23E-02      ,
                (float) 3.42E-04      ,
                (float) 3.07E-02      ,
                (float) 1.41E-01      ,
                (float) -1.39E-01     ,
                (float) 1.22E-01      ,
                (float) -2.05E-02     ,
                (float) 7.95E-02      ,
                (float) -6.91E-02     ,
                (float) -1.60E-02     ,
                (float) -3.66E-02     ,
                (float) -2.74E-01     ,
                (float) -4.29E-02
        },

        // 400
        {
                (float) 3.158691661   ,
                (float) -0.848893666  ,
                (float) -2.28E-01     ,
                (float) 9.85E-02      ,
                (float) -7.27E-05     ,
                (float) 2.87E-02      ,
                (float) 1.70E-01      ,
                (float) -1.57E-01     ,
                (float) 1.24E-01      ,
                (float) -4.49E-02     ,
                (float) 8.26E-02      ,
                (float) -5.88E-02     ,
                (float) 9.85E-03      ,
                (float) -3.10E-02     ,
                (float) -3.20E-01     ,
                (float) -7.43E-02
        },

        // 410
        {
                (float) 3.622486576   ,
                (float) -0.979709654  ,
                (float) -2.63E-01     ,
                (float) 1.31E-01      ,
                (float) -2.49E-03     ,
                (float) 1.79E-02      ,
                (float) 1.92E-01      ,
                (float) -1.61E-01     ,
                (float) 1.15E-01      ,
                (float) -8.06E-02     ,
                (float) 7.03E-02      ,
                (float) -4.81E-02     ,
                (float) 5.08E-02      ,
                (float) -8.06E-03     ,
                (float) -3.45E-01     ,
                (float) -1.20E-01
        },

        // 420
        {
                (float) 4.037697237   ,
                (float) -1.097416235  ,
                (float) -2.94E-01     ,
                (float) 1.78E-01      ,
                (float) -8.02E-03     ,
                (float) -2.18E-03     ,
                (float) 2.05E-01      ,
                (float) -1.52E-01     ,
                (float) 1.06E-01      ,
                (float) -1.20E-01     ,
                (float) 4.09E-02      ,
                (float) -4.85E-02     ,
                (float) 9.26E-02      ,
                (float) 2.51E-02      ,
                (float) -3.40E-01     ,
                (float) -1.72E-01
        },

        // 430
        {
                (float) 4.344775155   ,
                (float) -1.114736542  ,
                (float) -3.76E-01     ,
                (float) 2.40E-01      ,
                (float) 4.33E-03      ,
                (float) -3.69E-02     ,
                (float) 2.06E-01      ,
                (float) -1.26E-01     ,
                (float) 1.05E-01      ,
                (float) -1.41E-01     ,
                (float) -1.42E-03     ,
                (float) -6.20E-02     ,
                (float) 1.06E-01      ,
                (float) 5.44E-02      ,
                (float) -3.14E-01     ,
                (float) -2.22E-01
        },

        // 440
        {
                (float) 4.447664134   ,
                (float) -0.933628104  ,
                (float) -5.58E-01     ,
                (float) 2.99E-01      ,
                (float) 6.28E-02      ,
                (float) -8.74E-02     ,
                (float) 1.84E-01      ,
                (float) -8.70E-02     ,
                (float) 1.10E-01      ,
                (float) -1.45E-01     ,
                (float) -5.32E-02     ,
                (float) -9.26E-02     ,
                (float) 8.94E-02      ,
                (float) 7.03E-02      ,
                (float) -2.62E-01     ,
                (float) -2.37E-01
        },

        // 450
        {
                (float) 4.280419886   ,
                (float) -0.527435126  ,
                (float) -0.805433997  ,
                (float) 3.02E-01      ,
                (float) 1.64E-01      ,
                (float) -1.24E-01     ,
                (float) 1.24E-01      ,
                (float) -4.85E-02     ,
                (float) 1.12E-01      ,
                (float) -1.39E-01     ,
                (float) -1.06E-01     ,
                (float) -1.35E-01     ,
                (float) 4.57E-02      ,
                (float) 7.39E-02      ,
                (float) -1.80E-01     ,
                (float) -1.95E-01
        },

        // 460
        {
                (float) 3.808004543   ,
                (float) 0.070711345   ,
                (float) -1.000446352  ,
                (float) 1.64E-01      ,
                (float) 2.87E-01      ,
                (float) -1.06E-01     ,
                (float) 3.29E-02      ,
                (float) -2.46E-02     ,
                (float) 1.08E-01      ,
                (float) -1.25E-01     ,
                (float) -1.45E-01     ,
                (float) -1.72E-01     ,
                (float) -9.69E-03     ,
                (float) 6.55E-02      ,
                (float) -8.82E-02     ,
                (float) -1.08E-01
        },

        // 470
        {
                (float) 3.068395657   ,
                (float) 7.54E-01      ,
                (float) -1.033855476  ,
                (float) -1.46E-01     ,
                (float) 3.87E-01      ,
                (float) -1.49E-02     ,
                (float) -6.55E-02     ,
                (float) -3.35E-02     ,
                (float) 8.02E-02      ,
                (float) -1.18E-01     ,
                (float) -1.74E-01     ,
                (float) -1.91E-01     ,
                (float) -5.76E-02     ,
                (float) 5.68E-02      ,
                (float) 1.73E-02      ,
                (float) 1.79E-02
        },

        // 480
        {
                (float) 2.125233439   ,
                (float) 1.444179903   ,
                (float) -0.895332746  ,
                (float) -5.40E-01     ,
                (float) 3.96E-01      ,
                (float) 1.37E-01      ,
                (float) -1.35E-01     ,
                (float) -7.47E-02     ,
                (float) 2.23E-02      ,
                (float) -1.28E-01     ,
                (float) -1.94E-01     ,
                (float) -1.82E-01     ,
                (float) -7.71E-02     ,
                (float) 6.20E-02      ,
                (float) 1.26E-01      ,
                (float) 1.48E-01
        },

        // 490
        {
                (float) 1.04462108    ,
                (float) 2.077801223   ,
                (float) -0.621902305  ,
                (float) -0.874874127  ,
                (float) 2.42E-01      ,
                (float) 3.08E-01      ,
                (float) -1.42E-01     ,
                (float) -1.33E-01     ,
                (float) -6.48E-02     ,
                (float) -1.55E-01     ,
                (float) -2.07E-01     ,
                (float) -1.55E-01     ,
                (float) -6.71E-02     ,
                (float) 8.50E-02      ,
                (float) 2.30E-01      ,
                (float) 2.66E-01
        },

        // 500
        {
                (float) -0.042506258  ,
                (float) 2.521932029   ,
                (float) -0.203791685  ,
                (float) -1.050540175  ,
                (float) -9.10E-02     ,
                (float) 4.34E-01      ,
                (float) -7.11E-02     ,
                (float) -1.82E-01     ,
                (float) -1.74E-01     ,
                (float) -1.90E-01     ,
                (float) -2.02E-01     ,
                (float) -1.20E-01     ,
                (float) -5.12E-02     ,
                (float) 1.11E-01      ,
                (float) 3.25E-01      ,
                (float) 3.68E-01
        },

        // 510
        {
                (float) -9.75E-01     ,
                (float) 2.63753316    ,
                (float) 3.82E-01      ,
                (float) -1.061305779  ,
                (float) -5.11E-01     ,
                (float) 0.45120194    ,
                (float) 5.28E-02      ,
                (float) -1.88E-01     ,
                (float) -2.86E-01     ,
                (float) -2.25E-01     ,
                (float) -1.78E-01     ,
                (float) -8.02E-02     ,
                (float) -3.18E-02     ,
                (float) 1.33E-01      ,
                (float) 3.87E-01      ,
                (float) 4.30E-01
        },

        // 520
        {
                (float) -1.652261939  ,
                (float) 2.413878738   ,
                (float) 1.05E+00      ,
                (float) -0.907210663  ,
                (float) -0.882263065  ,
                (float) 0.307872428   ,
                (float) 1.85E-01      ,
                (float) -1.28E-01     ,
                (float) -3.66E-01     ,
                (float) -2.50E-01     ,
                (float) -1.33E-01     ,
                (float) -3.29E-02     ,
                (float) -2.00E-02     ,
                (float) 1.28E-01      ,
                (float) 3.92E-01      ,
                (float) 0.437496652
        },

        // 530
        {
                (float) -2.043438687  ,
                (float) 1.916793874   ,
                (float) 1.692066864   ,
                (float) -0.602079293  ,
                (float) -1.113822856  ,
                (float) 1.24E-02      ,
                (float) 2.78E-01      ,
                (float) -1.04E-03     ,
                (float) -3.78E-01     ,
                (float) -2.52E-01     ,
                (float) -8.02E-02     ,
                (float) 1.72E-02      ,
                (float) -2.11E-02     ,
                (float) 8.42E-02      ,
                (float) 3.36E-01      ,
                (float) 0.381761811
        },

        // 540
        {
                (float) -2.157411074  ,
                (float) 1.232919829   ,
                (float) 2.191694158   ,
                (float) -0.13308034   ,
                (float) -1.19657102   ,
                (float) -3.63E-01     ,
                (float) 2.84E-01      ,
                (float) 1.66E-01      ,
                (float) -3.06E-01     ,
                (float) -2.28E-01     ,
                (float) -4.09E-02     ,
                (float) 5.64E-02      ,
                (float) -2.49E-02     ,
                (float) 2.57E-02      ,
                (float) 2.27E-01      ,
                (float) 2.66E-01
        },

        // 550
        {
                (float) -2.045623588  ,
                (float) 0.495336943   ,
                (float) 2.430126457   ,
                (float) 4.68E-01      ,
                (float) -1.120655638  ,
                (float) -7.12E-01     ,
                (float) 1.63E-01      ,
                (float) 3.27E-01      ,
                (float) -1.55E-01     ,
                (float) -1.72E-01     ,
                (float) -1.72E-02     ,
                (float) 7.55E-02      ,
                (float) -2.86E-02     ,
                (float) -3.36E-02     ,
                (float) 8.46E-02      ,
                (float) 1.09E-01
        },

        // 560
        {
                (float) -1.783634083  ,
                (float) -0.179559145  ,
                (float) 2.363427385   ,
                (float) 1.099640862   ,
                (float) -0.869550913  ,
                (float) -0.955596291  ,
                (float) -8.14E-02     ,
                (float) 4.25E-01      ,
                (float) 4.69E-02      ,
                (float) -8.70E-02     ,
                (float) -5.76E-03     ,
                (float) 6.32E-02      ,
                (float) -3.35E-02     ,
                (float) -8.22E-02     ,
                (float) -5.68E-02     ,
                (float) -4.81E-02
        },

        // 570
        {
                (float) -1.449423666  ,
                (float) -7.07E-01     ,
                (float) 2.00891725    ,
                (float) 1.638159395   ,
                (float) -0.422837951  ,
                (float) -1.077632949  ,
                (float) -3.87E-01     ,
                (float) 0.418587326   ,
                (float) 2.54E-01      ,
                (float) 1.43E-02      ,
                (float) -1.04E-02     ,
                (float) 2.72E-02      ,
                (float) -3.53E-02     ,
                (float) -1.04E-01     ,
                (float) -1.67E-01     ,
                (float) -1.75E-01
        },

        // 580
        {
                (float) -1.105401056  ,
                (float) -1.061226328  ,
                (float) 1.473378127   ,
                (float) 1.966053463   ,
                (float) 0.17240856    ,
                (float) -1.060550995  ,
                (float) -6.65E-01     ,
                (float) 0.284831653   ,
                (float) 4.14E-01      ,
                (float) 1.26E-01      ,
                (float) -2.63E-02     ,
                (float) -2.22E-02     ,
                (float) -3.75E-02     ,
                (float) -1.02E-01     ,
                (float) -2.28E-01     ,
                (float) -2.50E-01
        },

        // 590
        {
                (float) -0.779969968  ,
                (float) -1.266170051  ,
                (float) 0.870265972   ,
                (float) 2.060600092   ,
                (float) 8.04E-01      ,
                (float) -0.890605415  ,
                (float) -0.847582726  ,
                (float) 4.41E-02      ,
                (float) 0.484253535   ,
                (float) 2.36E-01      ,
                (float) -3.72E-02     ,
                (float) -7.39E-02     ,
                (float) -4.53E-02     ,
                (float) -9.18E-02     ,
                (float) -2.36E-01     ,
                (float) -2.72E-01
        },

        // 600
        {
                (float) -0.486239809  ,
                (float) -1.345700452  ,
                (float) 0.263379896   ,
                (float) 1.9565988     ,
                (float) 1.374700048   ,
                (float) -0.573437226  ,
                (float) -0.912613328  ,
                (float) -2.50E-01     ,
                (float) 0.435748731   ,
                (float) 3.30E-01      ,
                (float) -2.74E-02     ,
                (float) -1.14E-01     ,
                (float) -5.68E-02     ,
                (float) -8.42E-02     ,
                (float) -2.06E-01     ,
                (float) -2.48E-01
        },

        // 610
        {
                (float) -0.235969319  ,
                (float) -1.312370778  ,
                (float) -3.05E-01     ,
                (float) 1.682731378   ,
                (float) 1.832814221   ,
                (float) -0.130299557  ,
                (float) -0.871100207  ,
                (float) -5.22E-01     ,
                (float) 0.261790878   ,
                (float) 3.80E-01      ,
                (float) 1.03E-02      ,
                (float) -1.32E-01     ,
                (float) -6.99E-02     ,
                (float) -7.91E-02     ,
                (float) -1.57E-01     ,
                (float) -1.94E-01
        },

        // 620
        {
                (float) -4.01E-02     ,
                (float) -1.179727419  ,
                (float) -7.86E-01     ,
                (float) 1.256198957   ,
                (float) 2.133416887   ,
                (float) 4.18E-01      ,
                (float) -0.724711833  ,
                (float) -0.722884461  ,
                (float) -7.59E-03     ,
                (float) 3.60E-01      ,
                (float) 6.28E-02      ,
                (float) -1.22E-01     ,
                (float) -7.87E-02     ,
                (float) -7.51E-02     ,
                (float) -1.06E-01     ,
                (float) -1.23E-01
        },

        // 630
        {
                (float) 9.89E-02      ,
                (float) -0.976809695  ,
                (float) -1.129951399  ,
                (float) 0.728644655   ,
                (float) 2.214456855   ,
                (float) 1.023963833   ,
                (float) -0.446792413  ,
                (float) -0.834433594  ,
                (float) -3.16E-01     ,
                (float) 2.61E-01      ,
                (float) 1.10E-01      ,
                (float) -8.98E-02     ,
                (float) -8.78E-02     ,
                (float) -7.31E-02     ,
                (float) -5.92E-02     ,
                (float) -6.04E-02
        },

        // 640
        {
                (float) 1.86E-01      ,
                (float) -0.737106181  ,
                (float) -1.315985797  ,
                (float) 0.17161405    ,
                (float) 2.062069935   ,
                (float) 1.594660001   ,
                (float) -0.024987324  ,
                (float) -0.841941708  ,
                (float) -5.90E-01     ,
                (float) 7.79E-02      ,
                (float) 1.32E-01      ,
                (float) -3.94E-02     ,
                (float) -9.93E-02     ,
                (float) -8.94E-02     ,
                (float) -2.49E-02     ,
                (float) -1.12E-02
        },

        // 650
        {
                (float) 2.26E-01      ,
                (float) -0.488464435  ,
                (float) -1.350149705  ,
                (float) -3.52E-01     ,
                (float) 1.706368035   ,
                (float) 2.042207197   ,
                (float) 5.17E-01      ,
                (float) -0.735278809  ,
                (float) -0.779493262  ,
                (float) -1.63E-01     ,
                (float) 1.04E-01      ,
                (float) 1.16E-02      ,
                (float) -1.06E-01     ,
                (float) -1.15E-01     ,
                (float) -1.29E-02     ,
                (float) 1.66E-02
        },

        // 660
        {
                (float) 2.28E-01      ,
                (float) -0.253448528  ,
                (float) -1.258185231  ,
                (float) -7.83E-01     ,
                (float) 1.191009454   ,
                (float) 2.304792584   ,
                (float) 1.124906264   ,
                (float) -0.49176165   ,
                (float) -0.869471462  ,
                (float) -4.28E-01     ,
                (float) 1.69E-02      ,
                (float) 3.91E-02      ,
                (float) -9.77E-02     ,
                (float) -1.44E-01     ,
                (float) -1.70E-02     ,
                (float) 1.25E-02
        },

        // 670
        {
                (float) 2.03E-01      ,
                (float) -5.05E-02     ,
                (float) -1.080532909  ,
                (float) -1.081526046  ,
                (float) 0.587738396   ,
                (float) 2.32950183    ,
                (float) 1.729567712   ,
                (float) -0.094943884  ,
                (float) -0.860294878  ,
                (float) -0.675690597  ,
                (float) -1.35E-01     ,
                (float) 1.77E-02      ,
                (float) -8.18E-02     ,
                (float) -1.64E-01     ,
                (float) -3.29E-02     ,
                (float) -1.52E-03
        },

        // 680
        {
                (float) 1.60E-01      ,
                (float) 1.14E-01      ,
                (float) -0.862320877  ,
                (float) -1.237766337  ,
                (float) -0.007825918  ,
                (float) 2.110296661   ,
                (float) 2.231538809   ,
                (float) 4.49E-01      ,
                (float) -0.741118454  ,
                (float) -0.864545503  ,
                (float) -3.34E-01     ,
                (float) -6.16E-02     ,
                (float) -7.79E-02     ,
                (float) -1.82E-01     ,
                (float) -4.53E-02     ,
                (float) -2.64E-02
        },

        // 690
        {
                (float) 1.08E-01      ,
                (float) 2.40E-01      ,
                (float) -0.639540415  ,
                (float) -1.275863067  ,
                (float) -5.22E-01     ,
                (float) 1.711889876   ,
                (float) 2.539292061   ,
                (float) 1.074097382   ,
                (float) -0.490490435  ,
                (float) -0.960244172  ,
                (float) -5.48E-01     ,
                (float) -2.03E-01     ,
                (float) -1.07E-01     ,
                (float) -2.09E-01     ,
                (float) -5.36E-02     ,
                (float) -4.09E-02
        },

        // 700
        {
                (float) 4.69E-02      ,
                (float) 3.31E-01      ,
                (float) -0.433007673  ,
                (float) -1.229344536  ,
                (float) -0.918929678  ,
                (float) 1.219055639   ,
                (float) 2.624542929   ,
                (float) 1.687061455   ,
                (float) -0.110039565  ,
                (float) -0.945386844  ,
                (float) -0.7386952    ,
                (float) -3.92E-01     ,
                (float) -1.76E-01     ,
                (float) -2.40E-01     ,
                (float) -5.48E-02     ,
                (float) -5.84E-02
        },

        // 710
        {
                (float) -2.21E-02     ,
                (float) 3.90E-01      ,
                (float) -0.254640292  ,
                (float) -1.127647322  ,
                (float) -1.189221807  ,
                (float) 0.699327255   ,
                (float) 2.500798076   ,
                (float) 2.210682935   ,
                (float) 3.74E-01      ,
                (float) -0.82199952   ,
                (float) -0.876065891  ,
                (float) -5.98E-01     ,
                (float) -2.83E-01     ,
                (float) -2.77E-01     ,
                (float) -5.60E-02     ,
                (float) -8.02E-02
        },

        // 720
        {
                (float) -1.02E-01     ,
                (float) 4.17E-01      ,
                (float) -1.12E-01     ,
                (float) -0.996275177  ,
                (float) -1.342323786  ,
                (float) 0.208161487   ,
                (float) 2.201983056   ,
                (float) 2.585453062   ,
                (float) 9.14E-01      ,
                (float) -0.60060945   ,
                (float) -0.953053861  ,
                (float) -0.795899883  ,
                (float) -4.09E-01     ,
                (float) -3.06E-01     ,
                (float) -5.20E-02     ,
                (float) -9.85E-02
        },

        // 730
        {
                (float) -1.95E-01     ,
                (float) 0.409371016   ,
                (float) -7.47E-03     ,
                (float) -0.853144292  ,
                (float) -1.395953177  ,
                (float) -2.19E-01     ,
                (float) 1.777436909   ,
                (float) 2.768110795   ,
                (float) 1.44894696    ,
                (float) -0.28959871   ,
                (float) -0.962826327  ,
                (float) -0.961237309  ,
                (float) -0.542371904  ,
                (float) -3.23E-01     ,
                (float) -4.01E-02     ,
                (float) -9.57E-02
        },

        // 740
        {
                (float) -3.00E-01     ,
                (float) 0.367063385   ,
                (float) 5.80E-02      ,
                (float) -0.712277759  ,
                (float) -1.37406444   ,
                (float) -5.63E-01     ,
                (float) 1.289330003   ,
                (float) 2.747850802   ,
                (float) 1.902810507   ,
                (float) 8.94E-02      ,
                (float) -0.897597098  ,
                (float) -1.068098835  ,
                (float) -0.665957856  ,
                (float) -3.32E-01     ,
                (float) -1.24E-02     ,
                (float) -5.92E-02
        },

        // 750
        {
                (float) -4.15E-01     ,
                (float) 2.95E-01      ,
                (float) 9.06E-02      ,
                (float) -0.582296006  ,
                (float) -1.300413411  ,
                (float) -0.82410497   ,
                (float) 0.795701256   ,
                (float) 2.550852174   ,
                (float) 2.225222459   ,
                (float) 5.01E-01      ,
                (float) -0.753274449  ,
                (float) -1.09825047   ,
                (float) -0.760623662  ,
                (float) -3.34E-01     ,
                (float) 2.40E-02      ,
                (float) -1.51E-03
        },

        // 760
        {
                (float) -0.536571985  ,
                (float) 1.94E-01      ,
                (float) 9.41E-02      ,
                (float) -0.46788664   ,
                (float) -1.192836825  ,
                (float) -1.009305133  ,
                (float) 0.32813242    ,
                (float) 2.211556896   ,
                (float) 2.38813663    ,
                (float) 0.903277841   ,
                (float) -0.538359632  ,
                (float) -1.047362137  ,
                (float) -0.809247643  ,
                (float) -3.17E-01     ,
                (float) 7.03E-02      ,
                (float) 7.91E-02
        },

        // 770
        {
                (float) -0.663097622  ,
                (float) 6.67E-02      ,
                (float) 7.07E-02      ,
                (float) -0.369844168  ,
                (float) -1.064881072  ,
                (float) -1.12748842   ,
                (float) -1.01E-01     ,
                (float) 1.777834164   ,
                (float) 2.388374983   ,
                (float) 1.253656527   ,
                (float) -0.268544208  ,
                (float) -0.918929678  ,
                (float) -0.80110392   ,
                (float) -2.77E-01     ,
                (float) 1.28E-01      ,
                (float) 1.87E-01
        },

        // 780
        {
                (float) -0.786842475  ,
                (float) -7.51E-02     ,
                (float) 3.06E-02      ,
                (float) -2.86E-01     ,
                (float) -0.927311753  ,
                (float) -1.195736785  ,
                (float) -4.80E-01     ,
                (float) 1.302916115   ,
                (float) 2.259426092   ,
                (float) 1.528596537   ,
                (float) 3.47E-02      ,
                (float) -0.716011954  ,
                (float) -0.735278809  ,
                (float) -2.21E-01     ,
                (float) 1.86E-01      ,
                (float) 2.92E-01
        },

        // 790
        {
                (float) -0.908918858  ,
                (float) -2.33E-01     ,
                (float) -2.56E-02     ,
                (float) -2.18E-01     ,
                (float) -0.788669847  ,
                (float) -1.22588842   ,
                (float) -0.796773844  ,
                (float) 0.810638034   ,
                (float) 2.00224337    ,
                (float) 1.694172315   ,
                (float) 3.53E-01      ,
                (float) -0.437893906  ,
                (float) -0.602913528  ,
                (float) -1.29E-01     ,
                (float) 2.44E-01      ,
                (float) 3.97E-01
        },

        // 800
        {
                (float) -1.022652892  ,
                (float) -3.96E-01     ,
                (float) -9.34E-02     ,
                (float) -1.60E-01     ,
                (float) -0.652451195  ,
                (float) -1.223624068  ,
                (float) -1.058604447  ,
                (float) 0.324557127   ,
                (float) 1.650593469   ,
                (float) 1.771438363   ,
                (float) 6.72E-01      ,
                (float) -0.111231329  ,
                (float) -0.417276385  ,
                (float) -1.77E-02     ,
                (float) 2.99E-01      ,
                (float) 4.97E-01
        },

        // 810
        {
                (float) -1.119424148  ,
                (float) -5.50E-01     ,
                (float) -1.58E-01     ,
                (float) -1.10E-01     ,
                (float) -0.521714658  ,
                (float) -1.201894234  ,
                (float) -1.277849341  ,
                (float) -1.20E-01     ,
                (float) 1.284125966   ,
                (float) 1.78875867    ,
                (float) 0.964534523   ,
                (float) 2.34E-01      ,
                (float) -0.19584659   ,
                (float) 9.73E-02      ,
                (float) 3.29E-01      ,
                (float) 0.552263548
        },

        // 820
        {
                (float) -1.206502389  ,
                (float) -0.703498429  ,
                (float) -2.28E-01     ,
                (float) -6.99E-02     ,
                (float) -0.399995804  ,
                (float) -1.164989268  ,
                (float) -1.45017845   ,
                (float) -5.25E-01     ,
                (float) 0.879283654   ,
                (float) 1.706169408   ,
                (float) 1.21035576    ,
                (float) 6.07E-01      ,
                (float) 7.51E-02      ,
                (float) 2.42E-01      ,
                (float) 3.64E-01      ,
                (float) 0.59473008
        },

        // 830
        {
                (float) -1.269864521  ,
                (float) -0.832645947  ,
                (float) -2.87E-01     ,
                (float) -3.26E-02     ,
                (float) -0.284434398  ,
                (float) -1.110247564  ,
                (float) -1.583338241  ,
                (float) -0.88599726   ,
                (float) 0.487948004   ,
                (float) 1.618376109   ,
                (float) 1.450416803   ,
                (float) 0.973711108   ,
                (float) 3.47E-01      ,
                (float) 3.57E-01      ,
                (float) 3.58E-01      ,
                (float) 0.570537266
        },

        // 840
        {
                (float) -1.315032385  ,
                (float) -0.944393707  ,
                (float) -3.38E-01     ,
                (float) -1.30E-03     ,
                (float) -1.80E-01     ,
                (float) -1.049745666  ,
                (float) -1.683009456  ,
                (float) -1.194386119  ,
                (float) 0.134669359   ,
                (float) 1.501225685   ,
                (float) 1.629419791   ,
                (float) 1.307762623   ,
                (float) 6.29E-01      ,
                (float) 4.81E-01      ,
                (float) 3.42E-01      ,
                (float) 0.512458622
        },

        // 850
        {
                (float) -1.346574412  ,
                (float) -1.045177236  ,
                (float) -3.88E-01     ,
                (float) 2.50E-02      ,
                (float) -8.42E-02     ,
                (float) -0.977405577  ,
                (float) -1.744623667  ,
                (float) -1.462215268  ,
                (float) -2.28E-01     ,
                (float) 1.339145748   ,
                (float) 1.781250555   ,
                (float) 1.637364886   ,
                (float) 0.930370615   ,
                (float) 0.608236741   ,
                (float) 3.21E-01      ,
                (float) 0.424228343
        },

        // 860
        {
                (float) -1.353168841  ,
                (float) -1.114617366  ,
                (float) -4.21E-01     ,
                (float) 4.93E-02      ,
                (float) 1.69E-03      ,
                (float) -0.901450469  ,
                (float) -1.778112242  ,
                (float) -1.677130086  ,
                (float) -5.25E-01     ,
                (float) 1.207098271   ,
                (float) 1.905789918   ,
                (float) 1.906425525   ,
                (float) 1.192956002   ,
                (float) 0.705206625   ,
                (float) 2.73E-01      ,
                (float) 0.291584983
        },

        // 870
        {
                (float) -1.355353742  ,
                (float) -1.187593063  ,
                (float) -0.460656603  ,
                (float) 6.48E-02      ,
                (float) 7.47E-02      ,
                (float) -0.822913206  ,
                (float) -1.781766986  ,
                (float) -1.853471468  ,
                (float) -0.834234966  ,
                (float) 1.003465488   ,
                (float) 1.955605663   ,
                (float) 2.150260488   ,
                (float) 1.479853379   ,
                (float) 0.843292374   ,
                (float) 2.69E-01      ,
                (float) 1.89E-01
        },

        // 880
        {
                (float) -1.331717084  ,
                (float) -1.224776107  ,
                (float) -0.482505614  ,
                (float) 7.98E-02      ,
                (float) 1.38E-01      ,
                (float) -0.74203214   ,
                (float) -1.75987825   ,
                (float) -1.978447811  ,
                (float) -1.07227001   ,
                (float) 0.85000598    ,
                (float) 1.997436588   ,
                (float) 2.327674458   ,
                (float) 1.70342835    ,
                (float) 0.933191123   ,
                (float) 2.32E-01      ,
                (float) 4.81E-02
        },

        // 890
        {
                (float) -1.308874937  ,
                (float) -1.270817932  ,
                (float) -0.515676386  ,
                (float) 8.58E-02      ,
                (float) 1.87E-01      ,
                (float) -0.663772955  ,
                (float) -1.714948738  ,
                (float) -2.069180795  ,
                (float) -1.318289874  ,
                (float) 0.623689951   ,
                (float) 1.947064686   ,
                (float) 2.460834249   ,
                (float) 1.942416805   ,
                (float) 1.079698674   ,
                (float) 2.64E-01      ,
                (float) -2.92E-02
        },

        // 900
        {
                (float) -1.259933152  ,
                (float) -1.278286321  ,
                (float) -0.52862689   ,
                (float) 9.18E-02      ,
                (float) 2.26E-01      ,
                (float) -0.585235692  ,
                (float) -1.646779823  ,
                (float) -2.107555603  ,
                (float) -1.488235454  ,
                (float) 0.456405977   ,
                (float) 1.903406389   ,
                (float) 2.530314104   ,
                (float) 2.094605098   ,
                (float) 1.157759231   ,
                (float) 2.61E-01      ,
                (float) -1.32E-01
        },

        // 910
        {
                (float) -1.213136543  ,
                (float) -1.293342276  ,
                (float) -0.551548489  ,
                (float) 8.90E-02      ,
                (float) 2.49E-01      ,
                (float) -0.512458622  ,
                (float) -1.560337191  ,
                (float) -2.114110306  ,
                (float) -1.654844095  ,
                (float) 0.228024224   ,
                (float) 1.769134285   ,
                (float) 2.549700135   ,
                (float) 2.24663449    ,
                (float) 1.284483495   ,
                (float) 3.27E-01      ,
                (float) -1.58E-01
        },

        // 920
        {
                (float) -1.140677277  ,
                (float) -1.268196051  ,
                (float) -0.551945744  ,
                (float) 8.66E-02      ,
                (float) 2.63E-01      ,
                (float) -0.441747277  ,
                (float) -1.454905781  ,
                (float) -2.068465736  ,
                (float) -1.736042965  ,
                (float) 6.67E-02      ,
                (float) 1.651229077   ,
                (float) 2.503737761   ,
                (float) 2.293470824   ,
                (float) 1.327863713   ,
                (float) 3.53E-01      ,
                (float) -1.94E-01
        },

        // 930
        {
                (float) -1.068654992  ,
                (float) -1.244082688  ,
                (float) -0.556911428  ,
                (float) 7.71E-02      ,
                (float) 2.62E-01      ,
                (float) -0.377789263  ,
                (float) -1.336364965  ,
                (float) -1.989849022  ,
                (float) -1.798213333  ,
                (float) -1.31E-01     ,
                (float) 1.455541389   ,
                (float) 2.404145996   ,
                (float) 2.325569008   ,
                (float) 1.402150351   ,
                (float) 4.31E-01      ,
                (float) -1.63E-01
        },

        // 940
        {
                (float) -0.974823421  ,
                (float) -1.182428751  ,
                (float) -0.540067827  ,
                (float) 6.79E-02      ,
                (float) 2.53E-01      ,
                (float) -3.18E-01     ,
                (float) -1.203959958  ,
                (float) -1.864197346  ,
                (float) -1.775132832  ,
                (float) -2.62E-01     ,
                (float) 1.277968517   ,
                (float) 2.240000335   ,
                (float) 2.248144058   ,
                (float) 1.396707961   ,
                (float) 0.476030362   ,
                (float) -1.32E-01
        },

        // 950
        {
                (float) -0.873761813  ,
                (float) -1.103057253  ,
                (float) -0.514643523  ,
                (float) 5.56E-02      ,
                (float) 2.33E-01      ,
                (float) -2.63E-01     ,
                (float) -1.059398956  ,
                (float) -1.699177724  ,
                (float) -1.707798152  ,
                (float) -3.81E-01     ,
                (float) 1.062894798   ,
                (float) 2.032395005   ,
                (float) 2.128927908   ,
                (float) 1.374263068   ,
                (float) 0.519450306   ,
                (float) -8.02E-02
        },

        // 960
        {
                (float) -0.758001781  ,
                (float) -0.994924511  ,
                (float) -0.473090677  ,
                (float) 4.33E-02      ,
                (float) 2.06E-01      ,
                (float) -2.13E-01     ,
                (float) -0.906177801  ,
                (float) -1.497888745  ,
                (float) -1.570069932  ,
                (float) -4.47E-01     ,
                (float) 0.852945665   ,
                (float) 1.768101423   ,
                (float) 1.923865009   ,
                (float) 1.29489157    ,
                (float) 0.541815748   ,
                (float) -1.91E-02
        },

        // 970
        {
                (float) -0.62571595   ,
                (float) -0.846192334  ,
                (float) -0.407583369  ,
                (float) 3.32E-02      ,
                (float) 1.74E-01      ,
                (float) -1.64E-01     ,
                (float) -0.738655474  ,
                (float) -1.251074371  ,
                (float) -1.361193387  ,
                (float) -0.450407431  ,
                (float) 0.660118211   ,
                (float) 1.4816013     ,
                (float) 1.656949545   ,
                (float) 1.136823906   ,
                (float) 0.494979413   ,
                (float) -3.51E-03
        },

        // 980
        {
                (float) -0.484690515  ,
                (float) -0.67672346   ,
                (float) -0.330913203  ,
                (float) 2.25E-02      ,
                (float) 1.35E-01      ,
                (float) -1.20E-01     ,
                (float) -0.565492131  ,
                (float) -0.977167224  ,
                (float) -1.100395646  ,
                (float) -0.413423014  ,
                (float) 0.463278485   ,
                (float) 1.145563511   ,
                (float) 1.328181517   ,
                (float) 0.943638923   ,
                (float) 0.436662417   ,
                (float) 2.53E-02
        },

        // 990
        {
                (float) -0.330118694  ,
                (float) -0.473368755  ,
                (float) -2.33E-01     ,
                (float) 1.41E-02      ,
                (float) 9.38E-02      ,
                (float) -7.71E-02     ,
                (float) -0.382953575  ,
                (float) -0.673863225  ,
                (float) -0.77766589   ,
                (float) -0.315817523  ,
                (float) 0.297543805   ,
                (float) 0.790894473   ,
                (float) 0.935376025   ,
                (float) 0.672552285   ,
                (float) 0.319392816   ,
                (float) 2.35E-02
        },

        // 1000
        {
                (float) -1.69E-01     ,
                (float) -0.247092452  ,
                (float) -1.23E-01     ,
                (float) 6.40E-03      ,
                (float) 4.77E-02      ,
                (float) -3.79E-02     ,
                (float) -1.94E-01     ,
                (float) -0.344817119  ,
                (float) -0.408377879  ,
                (float) -1.78E-01     ,
                (float) 1.36E-01      ,
                (float) 0.402419058   ,
                (float) 0.492754787   ,
                (float) 0.362296328   ,
                (float) 1.76E-01      ,
                (float) 1.70E-02
        }
};

// wavelengths and gains that get configured at startup
int hass_dflt_plns=3;
int hass_dflt_lmbda[3]={LAMBDA_600,LAMBDA_530,LAMBDA_460};

// original table
//float hass_dflt_gains[3]={(float)8192.0,(float)8192.0,(float)8192.0};
// table adjusted for Hassptr->okbpp handling
float hass_dflt_gains[3]={(float)1.0,(float)1.0,(float)1.0};

// float reconCoeffs[3][16];
typedef float reconCoeTbl[maxHassComp][numOfHassCh];
reconCoeTbl reconCoeffs;

// double *power= new double[colors]; // <---- [16]
// double power[16];
float hass_power[numOfHassCh];

// double *coef= new double[period*period*colors];  // <-- [8x8x16] = 1K
// double coef[8x8x16];
float hass_coef[repUnitSize*repUnitSize*numOfHassCh];

float shrink_coef[repUnitSize];
float zoom_coef[repUnitSize];

float shr_coef_sum[numOfHassCh];
float shr_retval[numOfHassCh];

// from gen_planeComb -- given we know repUnitSize is 8
// int combine_tap_center = repUnitSize/2;                    <-- 4
// int combine_tap_side   = repUnitSize/4;                    <-- 2
// int combine_tap = combine_tap_center + combine_tap_side;   <-- 6
// then this has 6 components
//  double*combine_coef = new double[combine_tap];
float combine_coef[6];

// int filterConfig[8][8];  // [y][x]
typedef int hassMosaic[repUnitSize][repUnitSize];

// Tom - my original guess was like this
//  arbitrary, I made it up
// hassMosaic filterConfig = {
//    {0x0,0x1,0x2,0x3,0x0,0x1,0x2,0x3},
//    {0x4,0x5,0x6,0x7,0x4,0x5,0x6,0x7},
//    {0x8,0x9,0xA,0xB,0x8,0x9,0xA,0xB},
//    {0xC,0xD,0xE,0xF,0xC,0xD,0xE,0xF},
//    {0x0,0x1,0x2,0x3,0x0,0x1,0x2,0x3},
//    {0x4,0x5,0x6,0x7,0x4,0x5,0x6,0x7},
//    {0x8,0x9,0xA,0xB,0x8,0x9,0xA,0xB},
//    {0xC,0xD,0xE,0xF,0xC,0xD,0xE,0xF}
//    };

// re-aligned for the test images we have
///
/// // from the gcc model provided by Atsugi,
/// //  the actual mosaic is more like this
/// hassMosaic filterConfig = {
///    {0xA,0x5,0x4,0x4,0x8,0xC,0x6,0x6},
///    {0xD,0x5,0xC,0x4,0xB,0xC,0xE,0x6},
///    {0x1,0x3,0xF,0x2,0x3,0x1,0xD,0x0},
///    {0x4,0x3,0xA,0x2,0x8,0x1,0x6,0x0},
///    {0x9,0xC,0x7,0x6,0xB,0x5,0x5,0x4},
///    {0xB,0xC,0xE,0x6,0xD,0x5,0xC,0x4},
///    {0x2,0x1,0xC,0x0,0x0,0x3,0xE,0x2},
///    {0x8,0x1,0x6,0x0,0x4,0x3,0xA,0x2}
///    };
///

hassMosaic filterConfig = {
        {0xC,0x6,0x6,  0xA,0x5,0x4,0x4,0x8},
        {0xC,0xE,0x6,  0xD,0x5,0xC,0x4,0xB},
        {0x1,0xD,0x0,  0x1,0x3,0xF,0x2,0x3},
        {0x1,0x6,0x0,  0x4,0x3,0xA,0x2,0x8},
        {0x5,0x5,0x4,  0x9,0xC,0x7,0x6,0xB},
        {0x5,0xC,0x4,  0xB,0xC,0xE,0x6,0xD},
        {0x3,0xE,0x2,  0x2,0x1,0xC,0x0,0x0},
        {0x3,0xA,0x2,  0x8,0x1,0x6,0x0,0x4}
};


short gammaLut[gammaTblLen] = {
        0,   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14,  15,  16,  17,  18,  19,
        20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,
        40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,
        60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,
        80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  98,  99,
        100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
        120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139,
        140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
        160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179,
        180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199,
        200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219,
        220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239,
        240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259,
        260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279,
        280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299,
        300, 301, 302, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 314, 315, 316, 317, 318, 319,
        320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 335, 336, 337, 338, 339,
        340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359,
        360, 361, 362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376, 377, 378, 379,
        380, 381, 382, 383, 384, 385, 386, 387, 388, 389, 390, 391, 392, 393, 394, 395, 396, 397, 398, 399,
        400, 401, 402, 403, 404, 405, 406, 407, 408, 409, 410, 411, 412, 413, 414, 415, 416, 417, 418, 419,
        420, 421, 422, 423, 424, 425, 426, 427, 428, 429, 430, 431, 432, 433, 434, 435, 436, 437, 438, 439,
        440, 441, 442, 443, 444, 445, 446, 447, 448, 449, 450, 451, 452, 453, 454, 455, 456, 457, 458, 459,
        460, 461, 462, 463, 464, 465, 466, 467, 468, 469, 470, 471, 472, 473, 474, 475, 476, 477, 478, 479,
        480, 481, 482, 483, 484, 485, 486, 487, 488, 489, 490, 491, 492, 493, 494, 495, 496, 497, 498, 499,
        500, 501, 502, 503, 504, 505, 506, 507, 508, 509, 510, 511, 512, 513, 514, 515, 516, 517, 518, 519,
        520, 521, 522, 523, 524, 525, 526, 527, 528, 529, 530, 531, 532, 533, 534, 535, 536, 537, 538, 539,
        540, 541, 542, 543, 544, 545, 546, 547, 548, 549, 550, 551, 552, 553, 554, 555, 556, 557, 558, 559,
        560, 561, 562, 563, 564, 565, 566, 567, 568, 569, 570, 571, 572, 573, 574, 575, 576, 577, 578, 579,
        580, 581, 582, 583, 584, 585, 586, 587, 588, 589, 590, 591, 592, 593, 594, 595, 596, 597, 598, 599,
        600, 601, 602, 603, 604, 605, 606, 607, 608, 609, 610, 611, 612, 613, 614, 615, 616, 617, 618, 619,
        620, 621, 622, 623, 624, 625, 626, 627, 628, 629, 630, 631, 632, 633, 634, 635, 636, 637, 638, 639,
        640, 641, 642, 643, 644, 645, 646, 647, 648, 649, 650, 651, 652, 653, 654, 655, 656, 657, 658, 659,
        660, 661, 662, 663, 664, 665, 666, 667, 668, 669, 670, 671, 672, 673, 674, 675, 676, 677, 678, 679,
        680, 681, 682, 683, 684, 685, 686, 687, 688, 689, 690, 691, 692, 693, 694, 695, 696, 697, 698, 699,
        700, 701, 702, 703, 704, 705, 706, 707, 708, 709, 710, 711, 712, 713, 714, 715, 716, 717, 718, 719,
        720, 721, 722, 723, 724, 725, 726, 727, 728, 729, 730, 731, 732, 733, 734, 735, 736, 737, 738, 739,
        740, 741, 742, 743, 744, 745, 746, 747, 748, 749, 750, 751, 752, 753, 754, 755, 756, 757, 758, 759,
        760, 761, 762, 763, 764, 765, 766, 767, 768, 769, 770, 771, 772, 773, 774, 775, 776, 777, 778, 779,
        780, 781, 782, 783, 784, 785, 786, 787, 788, 789, 790, 791, 792, 793, 794, 795, 796, 797, 798, 799,
        800, 801, 802, 803, 804, 805, 806, 807, 808, 809, 810, 811, 812, 813, 814, 815, 816, 817, 818, 819,
        820, 821, 822, 823, 824, 825, 826, 827, 828, 829, 830, 831, 832, 833, 834, 835, 836, 837, 838, 839,
        840, 841, 842, 843, 844, 845, 846, 847, 848, 849, 850, 851, 852, 853, 854, 855, 856, 857, 858, 859,
        860, 861, 862, 863, 864, 865, 866, 867, 868, 869, 870, 871, 872, 873, 874, 875, 876, 877, 878, 879,
        880, 881, 882, 883, 884, 885, 886, 887, 888, 889, 890, 891, 892, 893, 894, 895, 896, 897, 898, 899,
        900, 901, 902, 903, 904, 905, 906, 907, 908, 909, 910, 911, 912, 913, 914, 915, 916, 917, 918, 919,
        920, 921, 922, 923, 924, 925, 926, 927, 928, 929, 930, 931, 932, 933, 934, 935, 936, 937, 938, 939,
        940, 941, 942, 943, 944, 945, 946, 947, 948, 949, 950, 951, 952, 953, 954, 955, 956, 957, 958, 959,
        960, 961, 962, 963, 964, 965, 966, 967, 968, 969, 970, 971, 972, 973, 974, 975, 976, 977, 978, 979,
        980, 981, 982, 983, 984, 985, 986, 987, 988, 989, 990, 991, 992, 993, 994, 995, 996, 997, 998, 999,
        1000,1001,1002,1003,1004,1005,1006,1007,1008,1009,1010,1011,1012,1013,1014,1015,1016,1017,1018,1019,
        1020,1021,1022,1023};

FILE* f_rawin;
char rawin_fname[]="/storage/emulated/0/Dropbox/raw001.bin";

char testrec_key='A';
float tol_reconCoeffs= ((float)0.0000001);    // raw001
float tol_plnguide=    ((float)0.000020);     // raw001
float tol_plnshrink=   ((float)0.000001);     // raw001
float tol_planeNR=     ((float)0.000020);     // raw001
float tol_plnrecon=    ((float)0.000200);     // raw001
float tol_plncomb=     ((float)0.000020);     // raw001
float tol_plnproc=     ((float)0.000200);     // raw001
short tol_plnoutput=   1;                     // raw001


short tst_inframe[H_def_HEIGHT*H_def_WIDTH];

FILE* f_hasscoef;
char hasscoef_fname[]="/storage/emulated/0/DCIM/__hass_coef.dat";

FILE* f_reconcoe;
char reconcoe_fname[]="/storage/emulated/0/DCIM/__reconcoe.dat";

FILE* f_plnguide;
char plnguide_fname[]="/storage/emulated/0/DCIM/__plnguide.dat";

FILE* f_plnshrink;
char plnshrink_fname[]="/storage/emulated/0/DCIM/__plnshrink.dat";

FILE* f_plnnrfnc;
char plnnrfnc_fname[]="/storage/emulated/0/DCIM/__plnnrfnc.dat";

FILE* f_plnrecon;
char plnrecon_fname[]="/storage/emulated/0/DCIM/__plnrecon.dat";

FILE* f_plncomb;
char plncomb_fname[]="/storage/emulated/0/DCIM/__plncomb.dat";

FILE* f_plnproc;
char plnproc_fname[]="/storage/emulated/0/DCIM/__plnproc.dat";

FILE* f_plnoutput;
char plnoutput_fname[]="/storage/emulated/0/DCIM/__plnoutput.dat";

FILE* f_rgboutput;
char rgboutput_fname[]="/storage/emulated/0/DCIM/__rgboutput.dat";

// thread3 has two of these to issue reports to thread0
//  it will never deploy more than one
//   but it has to autoadvance to the next when report is suued
hass_timer hasstmr_pool[2];

// the thread0 measurement system is initialized
//  to start idle
hass_tmracc hasstmracc={
        {(void*)0,(void*)0},       // chn
        0,                         // numfr
        0,                         // frnum
        0,                         // tstrt
        0,                         // tcoeptr
        0,                         // tguide
        0,                         // tshrink
        0,                         // tNR
        0,                         // trecon
        0,                         // tapp
        0,                         // tcomb
        0,                         // tproc
        0                          // tout
};




// 1   MB 0x00100000
// 256 MB 0x10000000
#define  OCL_SVM_SIZE     0x10000000

//
// 2Mpix                                         N=3   N=16
//
// reconCoeff     [N][16] sizeof(float)          192  1024
//
// input frame    [x][y] sizeof(short)            4M    4M
// planeGuide     [x][y] sizeof((float)           8M    8M      * ----------| independent of N
// planeShrink    [16][x/8][y/8] sizeof(float)    2M    2M                  |
// planeNR        [16][x/8][y/8] sizeof(float)    2M    2M                  |
// planeRecon     [N][x/8][y/8] sizeof(float)   .375M   2M                  |
//      App       [N][x/8][y/8] sizeof(float)   .375M   2M - extends Recon  |
//                                                                          |
// planeComb      [x][y] sizeof((float)           8M    8M      * ----------|
//
// planeProc      [N][x][y] sizeof(float)        24M  128M
//
// planesOutput   [N][x][y] sizeof(short)        12M   64M
//

// allow for 240 integer parameters - should never need that many
#define  SVMOFF_PRMS      0x00000000
#define  SVMMAX_PRMS      0x000003C0
//
// [0] width
// [1] height
// [2] width-1
// [3] height-1
// [4] width/8
// [5] height/8
// [6] (width/8)-1
// [7] (height/8)-1
// [8] numOfPlanes N
// [9] nrTapSize/2
// [10] gamma_mode   1 enabled / 0 disabled
// [11] gammalen-1
//

// plain gains    [N] sizeof(float) - max N=16 -> 64 bytes
#define  SVMOFF_PLGAIN    0x000003C0
#define  SVMMAX_PLGAIN    0x00000040
//#define  SVMOFF_PLGAIN    0x00090000
//#define  SVMMAX_PLGAIN    0x00010000

// reconCoeff     [N][16] sizeof(float) - max N=16 -> 1024 bytes
#define  SVMOFF_RECON     0x00000400
#define  SVMMAX_RECON     0x00000400
//#define  SVMOFF_RECON     0x000A0000
//#define  SVMMAX_RECON     0x00010000

// filterConfig[][] -- it's currently 8x8 = 64 int's -> 256 bytes
// we'll allocate SVM memory space to allow 16x16 = 256 int's -> 1024 bytes
#define  SVMOFF_FCONFIG   0x00000800
#define  SVMMAX_FCONFIG   0x00000400
//#define  SVMOFF_FCONFIG   0x000B0000
//#define  SVMMAX_FCONFIG   0x00010000

// shrink_coef[8]
//  it's a vector used in 2 dimension
//   if we want to precalculate 8x8, allocate enough room for 64 floats's
//    well hey, if we expand to 16x16, it's 256 int's again
// but the current direct implementation only uses
#define  SVMOFF_SHRCOE    0x00000C00
#define  SVMMAX_SHRCOE    0x00000400
//#define  SVMOFF_SHRCOE    0x000C0000
//#define  SVMMAX_SHRCOE    0x00010000

// combine_coef[6]
//  it's a vector used in 2 dimension
//   if we want to precalculate 8x8, allocate enough room for 64 floats's
//    well hey, if we expand to 16x16, it's 256 int's again
// but the current direct implementation only uses
#define  SVMOFF_CMBCOE    0x00001000
#define  SVMMAX_CMBCOE    0x00000400
//#define  SVMOFF_CMBCOE    0x000D0000
//#define  SVMMAX_CMBCOE    0x00010000

// combine_coef[6]
//  it's a vector used in 2 dimension
//   if we want to precalculate 8x8, allocate enough room for 64 floats's
//    well hey, if we expand to 16x16, it's 256 int's again
// but the current direct implementation only uses
#define  SVMOFF_ZMCOE     0x00001400
#define  SVMMAX_ZMCOE     0x00000400
//#define  SVMOFF_ZMCOE     0x000E0000
//#define  SVMMAX_ZMCOE     0x00010000
// next offset 0x00001800

// unallocated 0x00001800 .. 0x00003FFF

// float hass_coef[repUnitSize*repUnitSize*numOfHassCh];
//  8x8x16 = 1K floats -> 4KB
//   we'll allocate SVM for 16x16x16 = 4K floats -> 16 KB
#define  SVMOFF_HCOEF     0x00004000
#define  SVMMAX_HCOEF     0x00004000
//#define  SVMOFF_HCOEF     0x000F0000
//#define  SVMMAX_HCOEF     0x00010000
// next offset 0x00008000

// unallocated 0x00008000 .. 0x000FFFFF

// input frame is 2 Mpix of short - 4MB
//  start on a 1 MB  boundary
#define  SVMOFF_IFRM      0x00100000
#define  SVMMAX_IFRM      0x00400000

// planeGuide is 2 Mpix of float - 8MB
#define  SVMOFF_PLNGD     0x00500000
#define  SVMMAX_PLNGD     0x00800000

// a note on h/8 w/8 planes
// we'll use (h/8)+1 and (w/8)+1
// so 241 x 136 = 32776
//  x 4 bytes per float -> 131104
//  x 16 planes (color or N) -> 2097664 = 0x200200
// round that up to 64K boundary = use 0x210000

// planeShrink    [16][x/8][y/8] sizeof(float)    2MB
#define  SVMOFF_PLNSHR    0x00D00000
#define  SVMMAX_PLNSHR    0x00210000

// planeNR        [16][x/8][y/8] sizeof(float)    2MB
#define  SVMOFF_PLNNR     0x00F10000
#define  SVMMAX_PLNNR     0x00210000
// next offset 0x01100000

// planeRecon     [N][x/8][y/8] sizeof(float)     2M (N=16)
#define  SVMOFF_PLNRCN    0x01120000
#define  SVMMAX_PLNRCN    0x00210000

// planeApp       [N][x/8][y/8] sizeof(float)     2M (N=16)
#define  SVMOFF_PLNAPP    0x01330000
#define  SVMMAX_PLNAPP    0x00210000

// planeComb      [x][y] sizeof((float)           8M (N=16)
#define  SVMOFF_PLNCMB    0x01540000
#define  SVMMAX_PLNCMB    0x00800000

//
// gamma table(s) -- current implementation is 1024 (10 bit) shorts, for 2KB
//  now suppose - we want 12,14,16 bit lookups
//    then worst case would need 128 KB for a 16 bit short translation table
//     then really way out there, suppose all 16 Nplns use different tables
//      then worst case is 16 x 128 KB -> 2MB
//
// I really don't think there is much merit in that
//  eventually the display is like ARGB_8888 - so where's the benefit in
//   huge gamma tables and high resolution pixels?
//    having said that, we have the memory available to allocate 2MB
//
#define  SVMOFF_GAMMA     0x01D40000
#define  SVMMAX_GAMMA     0x00200000

// a second input frame, so we can use them as ping-pong buffers
//  we could really provide more for queuing, but we want unavailable buffers
//   to drop frames to stay current... not have a backlog of stale frames
//    they are all the same size allocation
#define  SVMOFF_IFRM2     0x01F40000
// #define  SVMMAX_IFRM      0x00400000
// next offset 0x02340000

// unallocated 0x02340000 .. 0x03FFFFFF

// planesOutput   [N][x][y] sizeof(short)        64M (N=16)
#define  SVMOFF_PLNOUT    0x04000000
#define  SVMMAX_PLNOUT    0x04000000

// planeProc      [N][x][y] sizeof(float)       128M (N=16)
#define  SVMOFF_PLNPRC    0x08000000
#define  SVMMAX_PLNPRC    0x08000000
// next offset 0x10000000


typedef struct {
    // each kernel should either have a filename (which gets loaded to ksrc/klen)
    //  or the kernel source text file has already been included in a resident memory
    //   array (ksrc) of size klen
    char*           kfilename;
    char*           ksrc;
    u32             klen;
    u32             spr;
    char*           kname;
    cl_program      program;
    cl_kernel       kernel;
    void(*fnc)(void);
} CL_proc_ctrl;

typedef enum {
    HASS_KINDX_PLANEGUID0=0,
    HASS_KINDX_PLANEGUID1,
    HASS_KINDX_SHR0,
    HASS_KINDX_SHR1,
    HASS_KINDX_SHR2,
    HASS_KINDX_SHR3,
    HASS_KINDX_SHR4,
    HASS_KINDX_SHR5,
    HASS_KINDX_NRF0,
    HASS_KINDX_NRF1,
    HASS_KINDX_PLANERECON,
    HASS_KINDX_PLANECMB0,
    HASS_KINDX_PLANECMB1,
    HASS_KINDX_ZOOM,
    HASS_KINDX_POST
} HASS_KINDX;

typedef struct {
    int              width;                      // 0x00
    int              height;                     // 0x04
    int              numPlane;                   // 0x08
    int              nrTapSize;                  // 0x0C
    int              gamma_en;                   // 0x10
    int              okbpp;                      // 0x14  (always >= 8)
    u32              spr[6];                     // 0x18..0x2F
    hassMosaic*      mosaicptr;                  // 0x30
    lutWaveConfig*   waveMix;                    // 0x38
    short*           gammaptr;                   // 0x40
    short*           planeHassRaw;               // 0x48
    short*           planesOutput;               // 0x50
    //--------------------------------------------------
    reconCoeTbl*     reconCoeptr;                // 0x58
    float*           planeGuide;                 // 0x60
    float*           planeShrink;                // 0x68
    float*           planeNR;                    // 0x70
    float*           planeRecon;                 // 0x78
    float*           planeComb;                  // 0x80
    float*           planeProc;                  // 0x88
    //--------------------------------------------------
    void*            svmbase;                    // 0x0090
    float            postGainPln[maxHassComp];   // 0x98..0xD7
    int              waveLngthTbl[maxHassComp];  // 0xD8..0x0117
    u32              hrsrv[30];                  // 0x0118..0x018F
    //--------------------------------------------------
} Hass_ctrl;                                  // 0x190 final size

//   1 MB =   0x100000
// 256 MB = 0x10000000
#define  shrMEM_size      (0x10000000)

// shrMEM_ptr as a void* always points here
//  the Application Control Block is always at the start of the shared memeory
//   and the rest is used for app_malloc() memctrl
#define  ACB_FIXED_SIZE   (0x10000)

// alc_state is Java Activity life cycle state
//  all threads can observe the os state of the android interface

typedef struct {
    memctrl*       memc;         // 0x0000
    volatile int   hwsema_crsh;  // 0x0008
    volatile u32   crsh_code;    // 0x000C

    int            vwdim;        // 0x0010 dimensions of texture/ImgVw y||x
    int            vwpix;        // 0x0014 pixels of texture/ImgVw    (y*x)

    u32            thr1flgs;     // 0x0018
    u32            hassoblk;     // 0x001C
    u16            hasscaptr;    // 0x0020
    u16            hasstmren;    // 0x0022
    u32            hasstmridx;   // 0x0024

    u8             ibm_msk;      // 0x0028

    // circular logical index for output raw sanpshots
    //  this variable owned by thread1. cfe_file_image()
    u8             snpraw;       // 0x0029

    // circular logical index for output rgb sanpshots
    //  this variable owned by thread2. cofl_file_image()
    u8             snprgb;       // 0x002A

    u8             tmp_tbd;      // 0x002B

    volatile u32   alc_state;    // 0x002C

    int            tmr_zoff;     // 0x0030
    volatile int   tmr_cnt;      // 0x0034
    volatile int   thr0cnt;      // 0x0038

    volatile int   hwsema_cam;   // 0x003C

    lnklst         gblkhead;     // 0x0040..0x004F
    volatile int   hwsema_gblk;  // 0x0050

    volatile int   shtdn_tstmp;  // 0x54
    volatile int   shtdn_timer;  // 0x58

    // <tbd>
    u32            cifc_tbd;     // 0x005C

    lnklst         coflpool;     // 0x0060..0x006F  color output file carrier pool

    // Hass_ctrl
    Hass_ctrl      hass_base;    // 0x0070..0x01FF

    // general scratch variable area
    u32            uvar[128];    // 0x0200..0x03FF
    u32            uvar_x[128];  // 0x0400..0x05FF
    u32            uvar_y[128];  // 0x0600..0x07FF

    // J2J (Jni2Java) command flow
    jni2java_ctrl  j2jctrl;      // 0x0800..0x083F
    lnklst*        olxchg;       // 0x0840..0x0847 next element in open loop xchg ring

    u32            spr848;       // <unused>
    u32            spr84c;       // <unused>

    volatile int   hwsema_olx;   // 0x0850..0x0853
    u32            olx_spr[43];  // 0x0854..0x08FF


    UTP_ctrl       utpbase;      // 0x0900..0x0AFF

    FTU_ctrl       ftubase;      // 0x0B00..0x0BFF

    // Reserved for read/write for USB2
    u32            usb2[0x500];  // 0x0C00..0x1FFF

    // ALS data sent through USB3
    als            usbmainals;   // 0x2000 main ALS in USB3 data
    als            usbslaveals;  // 0x2010 slave ALS in USB3 data

    als            slaveals;     // 0x2020 slave ALS

    // Location data sent through USB3
    location       gps;          // 0x2030 GPS location

    u32            tbd[0x36E8];  // 0x2060..0xFBFF
    //
    // at 0x80 per thread and threads.h MUTHREADNUM = 8
    //  we have 0x400 bytes      // 0xFC00..0xFFFF
    //   if you move this boundary, it has to be co-ordinated
    //    with threads.h
    // #define  MU_THREADNUM     (8)
    // #define  thr_ctrl         ((thread_ctrl*)(((u64)shrMEM_ptr)+0xFC00))
    //                                                               ^^^^
} AppCtrlBlk;                // preserve ACB_FIXED_SIZE

#define  ACptr            ((AppCtrlBlk*)shrMEM_ptr)
#define  Hassptr          ((Hass_ctrl*)(&(ACptr->hass_base)))
#define  utpptr           ((UTP_ctrl*)(&(ACptr->utpbase)))
#define  ftuptr           ((FTU_ctrl*)(&(ACptr->ftubase)))

//------------------------------------------------------------------------------
//  ansi's


void lnklst_init(lnklst* head);
void lnklst_fins(lnklst* pred,lnklst* ins);
void lnklst_rins(lnklst* succ,lnklst* ins);
void lnklst_drop(lnklst* dropitem);
lnklst* lnklst_fpop(lnklst* head);
lnklst* lnklst_fprvw(lnklst* ref,lnklst* head);
u32 lnklst_fsrchu64(lnklst* head,u64 mtchval,u32 idx,lnklst** pred,lnklst** succ);

void hsp_lnklst_fins(int* hwsema,lnklst* pred,lnklst* ins);
lnklst* hsp_lnklst_fpop(int* hwsema,lnklst* head);

void bptrqueue_init(bptrqueue* bq,char* buff,int buffsize);
void bptrquetx_init(bptrquetx* bq,char* buff,int buffsize);
int bptrqueue_avail(bptrqueue* bq);
int bptrquetx_avail(bptrquetx* bq);
int bptrqueue_space(bptrqueue* bq);
void bptrqueue_rd(bptrqueue* bq,char* dst,int num);
void bptrqueue_wr(bptrqueue* bq,char* src,int num);
void bptr_que_to_que(bptrqueue* bqdst,bptrqueue* bqsrc,int num);
char bptrqueue_prvw(bptrqueue* bq,int index);
void bptrqueue_drop(bptrqueue* bq,int num);

memctrl* memctrl_init(u64 aptr,u64 blksize);
u64 memctrl_mblkget(memctrl* mc);
u64 memctrl_mblkret(memctrl* mc,u64 aret);
u64 memctrl_fblkret(memctrl* mc,memblk* mret);
u64 memctrl_mallret(memctrl* mc,u64 aret);
u64 memctrl_mallrec(memctrl* mc,memblk* mall);
u64 memctrl_freeget(memctrl* mc,u64 reqsize,u64* res);

void memcCapt(void);
void memcRlse(void);
void* appmalloc(u32 size);
void appfree(void* addr);

void init_gblkchain(void);
void gblkCapt(void);
void gblkRlse(void);
void* gblkalloc(void);
void gblkfree(void* dropitem);

void msgqueue_submit(msg_queue* queue,msg_gblkfnc* msg);
u32 msgexec_gblkvoid(msg_queue* queue,gblkfnc fnc,s64 a0,s64 a1,s64 a2);
u32 msgexec_gblkvack(msg_queue* queue,gblkfnc fnc,s64 a0,s64 a1,s64 a2);
u32 msgexec_gblkfnc(msg_queue* queue,gblkfnc fnc,void* dst,s64 a0,s64 a1,s64 a2);
u32 msgexec_gblkpost(msg_queue* queue,gblkfnc fnc,void* dst,s64 a0,s64 a1,s64 a2,volatile int* done);
void msgqueue_import(lnklst* dst,msg_queue* queue);
void msgqueue_service(msg_queue* queue);

u32 msgexec_gblkvoid_a2(msg_queue* queue,gblkfnc fnc,s64 a0,s64 a1);
void msgexec_gblkv_carr(msg_queue* queue,gblkfnc fnc,msg_gblkfnc* msg);

void camCapt(void);
void camRlse(void);

// these are message routines to thread4

u32 utp_cnsl_FSM(void);

void ofile_release(ofil_carrier* ofil);
u32 utp_ofil_promote(ofil_carrier* ofil);
u32 utp_ofil_body(ofil_carrier* ofil,u32* done);
u32 utp_blko_FSM(void);

u32 utp_ofile_FSM(void);

void ifile_release(ifil_carrier* ifil);
u32 utp_ifil_promote(ifil_carrier* ifil);
u32 utp_ifil_body(ifil_carrier* ifil,u32* done);
void utp_ifil_deploy(ifil_carrier* ifil,u32 status);
u32 utp_blki_FSM(void);
int64_t now_ms_i64(void);

int msg_hass_tmeasr(void* ioptr);
int set_hass_tmeasr(u32 Nfrms);
int msg_hass_tmeasu(void* ioptr);

int j2j_xchg_submit(void* ioptr);
void init_j2j_ctrl(jni2java_ctrl* j2j);
void j2j_xchg_void(j2j_xchg_spec* j2jspec);
int j2j_xchg_wrsp(j2j_xchg_spec* j2jspec);
void j2j_FSM(jni2java_ctrl* j2j);

void j2j_set_imgvw(int viewsel);
int j2j_get_FTDI_uart(void);

u32 open_utp(void);
void close_utp(void);

u32 utp_trafblk(void);
void utp_dbgoutput(char* src);

void utp_shutdown(void);
void utp_startup(void);
u32 utp_getstatus(void);
u32 utp_putstatus(void);
void utp_reset_ostatus(void);
u32 utp_first_status(void);

u32 utp_pfifo_rd(char* dst,int num);
u32 utp_pfifo_wr(char* src,int num);
u32 utp_bfifo_rd(char* dst,int num);
u32 utp_bfifo_wr(char* src,int num);

char* uaiox_lmembase(u32 lmempg);

void uuiox_wrbyte(void);
void uuiox_wrword(void);
void uuiox_wrquad(void);
void uuiox_rdbyte(void);
void uuiox_rdword(void);
void uuiox_rdquad(void);

void uuiox_wrldvc(void);
void uuiox_rdldvc(void);

void utpcnsl_flush(void);
int send_utpcnsl(void* ioptr);

void ucnsl_fl_fnckey(u32 fnum,u32 arglo,u32 arghi);
void ucnsl_ldvc_fl_acc(u32 cmdnum,u32 dvc,u32 addr);

void utpofile_flush(void);
int msg_snd_uofile(void* ioptr);
void send_utpofile(ofil_carrier* ofil);

void utpifile_flush(void);
int msg_snd_uifile(void* ioptr);
void send_utpifile(ifil_carrier* ifil);

void uuiox_pll_ocnsl(void);
void uuiox_pll_ofile(void);

void uuiox_fnckey(void);

u32 uuiox_lenchk(u32 cmd,u32 len);
u32 uuiox_sanechk(void);
u32 uuiox_cmdexec(void);
void utp_dmp_cfstat(void);
u32 utp_cmd_uuiox(void);

void utp_softreset(void);
void utp_restart(void);
void utp_isig_srv(u16 msk);
u32 utpxchgpass(void);

void utp_FSM(void);

void olxchgring_init(void);
uaio_xchg* try_olx_carrier(void);
uaio_xchg* get_ol_carrier(void);

void init_comiox_ctrl(void);
void uaiox_SYSresponse(uaio_xchg* xchg);
void uaiox_quedrop(lnklst* head,u8 errval);
void comiox_reset(void);
u32 comiox_lenchk(u32 cmd,u32 len);
u32 comiox_rsphdrchk(void);
void comiox_rspnullpay(void);
u32 comiox_rspdata(void);
u32 comiox_sanechk(void);

void comiox_wrbyte(void);
void comiox_rdbyte(void);
void comiox_wrword(void);
void comiox_rdword(void);
void comiox_wrquad(void);
void comiox_rdquad(void);
void comiox_wrldvc(void);
void comiox_rdldvc(void);
void comiox_fnckey(void);

u32 comiox_cmdexec(void);
u32 comiox_rpath(u32* rhdwy,u32* thdwy);
u32 comiox_send(uaio_xchg* xchg,bptrquetx* que);
u32 comiox_tpath(u32* thdwy);
void comiox_prc(void);

void comiox_msg_void(uaio_xchg* xchg);
u32 comiox_msg_xchg(uaio_xchg* xchg);

u32 caio_rbbuf(u64 haddr,u32 taddr,u32 numbytes);
void caio_rbbuf_ol(u64 haddr,u32 taddr,u32 numbytes);
u32 caio_rbbuf_lrg(u64 haddr,u32 taddr,u32 numbytes);
u32 caio_rwbuf(u64 haddr,u32 taddr,u32 numbytes);
void caio_rwbuf_ol(u64 haddr,u32 taddr,u32 numbytes);
u32 caio_rwbuf_lrg(u64 haddr,u32 taddr,u32 numbytes);
u32 caio_rqbuf(u64 haddr,u32 taddr,u32 numbytes);
void caio_rqbuf_ol(u64 haddr,u32 taddr,u32 numbytes);
u32 caio_rqbuf_lrg(u64 haddr,u32 taddr,u32 numbytes);

u32 caio_wbbuf(u32 taddr,u64 haddr,u32 numbytes);
void caio_wbbuf_ol(u32 taddr,u64 haddr,u32 numbytes);
u32 caio_wbbuf_lrg(u32 taddr,u64 haddr,u32 numbytes);
u32 caio_wwbuf(u32 taddr,u64 haddr,u32 numbytes);
void caio_wwbuf_ol(u32 taddr,u64 haddr,u32 numbytes);
u32 caio_wwbuf_lrg(u32 taddr,u64 haddr,u32 numbytes);
u32 caio_wqbuf(u32 taddr,u64 haddr,u32 numbytes);
void caio_wqbuf_ol(u32 taddr,u64 haddr,u32 numbytes);
u32 caio_wqbuf_lrg(u32 taddr,u64 haddr,u32 numbytes);

u32 caio_rbyte(u32 addr,u32* val);
u32 caio_rword(u32 addr,u32* val);
u32 caio_rquad(u32 addr,u32* val);
void caio_wbyte(u32 addr,u32 val);
void caio_wword(u32 addr,u32 val);
void caio_wquad(u32 addr,u32 val);

u32 caio_rldvcbuf(u32 ldvc,u64 haddr,u32 taddr,u32 numbytes);
void caio_rldvcbol(u32 ldvc,u64 haddr,u32 taddr,u32 numbytes);
u32 caio_rldvc_lrg(u32 ldvc,u64 haddr,u32 taddr,u32 numbytes);
u32 caio_wldvcbuf(u32 ldvc,u32 taddr,u64 haddr,u32 numbytes);
void caio_wldvcbol(u32 ldvc,u32 taddr,u64 haddr,u32 numbytes);
u32 caio_wldvc_lrg(u32 ldvc,u32 taddr,u64 haddr,u32 numbytes);

u32 caio_rldvc_reg(u32 ldvc,u32 addr,u32* val,u32 vsz);
void caio_wldvc_reg(u32 ldvc,u32 addr,u32 val,u32 vsz);

u32 caio_fnckey(u32 findx,u32 arglo,u32 arghi,s64* val);

int msg_ftu_show_D2xx(void* ioptr);
void ftu_show_D2xx(void);
int msg_set_D2xx(void* ioptr);

int fttx_avail(void);
int fttx_commit(void);
void ftrx_commit(int val);

void ft_fe_loopback(void);

void com_offline(void);
void com_online(void);

void comstate_COMSYNC0(void);
void comstate_COMSYNC1(void);
void comstate_COMSYNC2(void);
void comstate_COMLOCK(void);
void comstate_COMXCHG(void);
void com_send_SYNCx(char* src);
void com_send_TX_0(void);
void com_send_ACK_0(void);
void comrx_txpkt_onlyfet(void);
u32 com_rxhdr_deploy(u8 hdrtype);
void com_send_ACK(void);

u32 com_TX0hdr_sane(u8* src);
u32 com_TXhdr_sane(u8* src);
u32 com_rxhdr_sane(char* src);
void comrx_txpkt_all(void);
void com_fesend_XCHG(void);

void com_fe_extract(void);
void com_fe_send(void);

void ft_bpque_init(void);
void ft_uart_restart(void);
void ft_uart_FSM(void);

// fnc keys 0x0000..0x000F are reserved for temporary test features
//  these should always be named tst_fXXXXky() and defined in zztst.c

u32 fnc_0010_ky(char* arg,char* dst);
u32 fnc_0011_ky(char* arg,char* dst);


void* MU_thread0(void* ptr);
void* MU_thread1(void* ptr);
void* MU_thread2(void* ptr);
void* MU_thread3(void* ptr);
void* MU_thread4(void* ptr);
void* MU_thread5(void* ptr);
void threads_startup(void);
void threads_shutdown(void);
void thread_set_crash(u32 crashcode);
void clr_wt_texture(void);
void set_wt_texture(void);
void wait_on_texture(void);

void* cfe_getbuffer(thread1_ctrl* thr_ptr);
int init_cfepool(thread1_ctrl* thr_ptr);
int msg_snapshot(void* ioptr);
int req_snapshot(int n);
int msg_cfe_refill(void* ioptr);
int cfe_refill(cfei_image* cfeiptr);
int msg_cfeg_refill(void* ioptr);
int cfeg_refill(cfeg_image* cfegptr);

int cfe_file_image(cfei_image* cfeiptr);
void cfeiptr_rawhdr(cfei_image* cfeiptr);
int get_raw_cfei(u32 ix,u32 iy,s64* pix,s64* dst);
int get_raw2x_cfei(u32 ix,u32 iy,s64* pix,s64* dst0,s64* dst1);
int msg_fe_frame(void* ioptr);

int msg_thr1flgs(void* ioptr);
int thr1flgs_setclr(u32 setmsk,u32 clrmsk);

void* imo_getbuffer(thread2_ctrl* thr_ptr);
void imo_freebuffer(void* buff);
void imo_cleanup(thread2_ctrl* thr_ptr);

int msg_imo_refill(void* ioptr);
int imo_refill(pixo_image* pixoptr);

int msg_cofl_refill(void* ioptr);
int cofl_refill(cofl_image* coflptr);

int msg_ibm_msk(void* ioptr);
int set_ibm_msk(u32 mskval);

int msg_osnapshot(void* ioptr);
int req_osnapshot(int n);

void tp_monitor(void);

int msg_raw2pixo_frame(void* ioptr);
int raw2pixo_fwrd(cfei_image* cfeiptr);

int cofl_file_image(cofl_image* coflptr);

int msg_hass2pixo_frame(void* ioptr);
int hass2pixo_fwrd(u32 hassspec,s64 hdr0,s64 hdr1);

int set_hass_gains(int N,float* tbl);
void set_hass_mode(int Npln,int gammaEnab,int nrTaps);
int set_hass_lambda(int N,int* tbl);


void* alloc_SVM(int bytesize);
void free_SVM(void* ptr);

int hass_fConfig_SVM(hassMosaic* fConfig);
int hass_coef_SVM(float* coef);
int shrnk_coef_SVM(float* coef);
int hass_parms_SVM(void);
int hass_reconcoe_SVM(void);
int comb_coef_SVM(float* coef);
int zoom_coef_SVM(float* coef);
int hass_gains_SVM(void);
int hass_gamma_SVM(char* src,int bytesize);

int set_hass_imgdim(int x,int y);
void reset_hass_imgdim(void);
int resize_hass_imgdim(int x,int y);

void gen_hass_coef(void);
void gen_shrink_coef(void);
void gen_zoom_coef(void);
void gen_combine_coef(void);

// you have to call Hass_init somewhere at startup
void Hass_init(void);

int gen_tst_inframe(void);
int dump_hass_coef(void);
int dump_reconCoeffs(float* src);
int dmp_SVM_reconCoeffs(void);
int dump_planeGuide(float* src);
int dmp_SVM_planeGuide(void);
int dump_planeShrink(float* src);
int dmp_SVM_planeShrink(void);
int dump_planeNRfnc(float* src);
int dmp_SVM_planeNRfnc(void);
int dump_planeRecon(float* src);
int dmp_SVM_planeRecon(void);
int dump_planeComb(float* src);
int dmp_SVM_planeComb(void);
int dump_planeProc(float* src);
int dmp_SVM_planeProc(void);
int dump_planesOutput(short* src);
int dmp_SVM_planesOutput(void);
void range_planesOutput(short* src);
int rng_SVM_planesOutput(void);
int dump_RGBfile(short * src);
int dmp_SVM_RGBfile(void);
void hass_tmr_report(hass_tmracc* tmracc);






int msg_hass_captr(void* ioptr);
int set_hass_capture(u32 msk);
int msg_hass_tmren(void* ioptr);
int set_hass_tmren(u32 N);

void hass_set_kparms(void);
void hass_rcnCoe_setup(reconCoeTbl* rcncoeptr);
int hiframe_accept(void);

int hass_input_frame(cfeg_image* cfegptr);
int hass_input_frame_cap(cfeg_image* cfegptr,u32 hassopts);
int hass_input_frame_tmr(cfeg_image* cfegptr,hass_timer* tmrptr);

void vfvdummy(void);
void CLCreateContext(cl_device_type type,cl_context* context, cl_device_id* device);
char* CLReadFileAsArray(char* filename,u32* size);
void CLDisplayBuildErrorsAndWarnings(cl_program program, cl_device_id device);
int CLBuildProgram_dbg(cl_program* program,cl_device_id device,cl_context context,
                       char* src,u32 srclen,char* compilerOptions);
int CLBuildProgram(cl_program* program,cl_device_id device,cl_context context,
                   char* src,u32 srclen,char* compilerOptions);

void CL_kernel_build_dbg(CL_proc_ctrl* kctrl);
void CL_kernel_build(CL_proc_ctrl* kctrl);
void CL_kernel_setup_dbg(int kindx);
void CL_kernel_setup(int kindx);
void CL_kernel_build_suite(void);
void CL_startup(void);

// void CL_kernel_run(int kindx);

void CL_kernel_release(int kindx);
void CL_shutdown(void);

void kgen_planeGuide(void);
void khass_shrink(void);
void khass_NRfnc(void);
void kgen_planeRecon(void);
void kgen_planeComb(void);
void khass_zoom(void);
void khass_post(void);

int init_cfegptr(cfeg_image* cfegptr,void* svmhome);
int create_cfegptr(void* svmhome);
void init_cfegpool(void);
void drop_cfegpool(void);

int msg_raw2hass_frame(void* ioptr);
int raw2hass_fwrd(cfeg_image* cfegptr);

int msg_utp_fhdr_state(void* ioptr);
void utp_fhdr_state(void);

void utp_frc_lseek(int fd,off_t offset, int whence);
void utp_frc_read(int fd,void *buf,size_t n,off_t offset);
void utp_frc_write(int fd,const void *buf,size_t n,off_t offset);
ssize_t utp_blk_read(int fd,void *buf,size_t n,off_t offset);
ssize_t utp_blk_write(int fd,const void *buf,size_t n,off_t offset);
void setUSBAccssry(void);
void setUSBAdb(void);
void setCameraOptimizationOff(void);
void setScalingGovernor(int mode);

int msg_com_xchg(void* ioptr);
void issue_com_xchg(uaio_xchg* xchg);

int msg_comio_tst(void* ioptr);
int comio_tst_N(u32 N);

int msg_comv_fk_mode(void* ioptr);
int comv_fk_mode(u32 findx,u32 arglo,u32 arghi);

u32 tst_f0001ky(char* arg,char* dst);

void tst_comio_00(void);
void tst_comio_01(void);
void tst_comio_02(void);
void tst_comio_03(void);
u32 tst_f0002ky(char* arg,char* dst);
int msg_tst_comrx_stat(void* ioptr);
void tst_comrx_stat(void);

int msg_rgb2pixo_frame(void* ioptr);
int msg_raw2cfe_frame(void* ioptr);

int msg_ofil_refill(void* ioptr);
int ofil_refill(ofil_carrier* ofilptr);

void ofilCapt();
void ofilRlse();

int commanderExecute(int command, int arg1, int arg2);
void usbTransferSuccess();
void usbTransferFail();

//------------------------------------------------------------------------------
//  global data


char scratch[256];

void* shrMEM_ptr=MAP_FAILED;


// table of thread entry points for thread index [0..((MU_THREADNUM-1))]
thread_routine MU_threads[]={
        MU_thread0,
        MU_thread1,
        MU_thread2,
        MU_thread3,
        MU_thread4,
        MU_thread5,
        (thread_routine)NULL,
        (thread_routine)NULL,
        (thread_routine)NULL
};


char hexASCII[]="0123456789ABCDEF";
char snprobin_fname[]="rs0.";
char snprgbot_fname[]="cs0.";

// QQQQQ naming variable
//char utpdvcname[]="/dev/usb_accessory";
char utpdvcname[]="/dev/uSOMutp";

char utpdbgln[1024];
char utppb_ln[1024];

// declare u32 to force alignment
u32 uuioxbuff[UAIOXCMD_MAXBUFF>>2];

// declare u32 to force alignment
u32 comioxbuff[UAIOXCMD_MAXBUFF>>2];

// the set of open loop uaio_xchg carriers
olxchg_t olxchg_set[OLXCHG_NUM];

// char com_connect_fail0[]="Tp COM: ERROR-> port is already active\n";
char com_disconnect[]="Tp COM: disconnected\n";
char com_lnkstatup[]="Tp COM: link layer connected\n";
char com_ack_fault[]="Tp COM: link reset - ACK failure\n";

char fttxbody[FTDI_BPTR_DEPTH];
char ftrxbody[FTDI_BPTR_DEPTH];

bptrqueue ftrxqueue={
        0,0,FTDI_BPTR_MSK,0,ftrxbody};

bptrquetx fttxqueue={
        0,0,FTDI_BPTR_MSK,0,fttxbody};

u32 uartpktticks=UART_PKT_TICKS;
u32 uarttimeout=UART_TIMEOUT;

// define the separate branch queue's
//  in this version, there is only one branch queue
char comtx0body[COM_SQBUFF_SIZE];
char comrx0body[COM_SQBUFF_SIZE];
// define the queue control structures
bptrquetx comtx0que={0,0,(COM_SQBUFF_SIZE-1),0,comtx0body};
bptrqueue comrx0que={0,0,(COM_SQBUFF_SIZE-1),0,comrx0body};

// byte representation
//
// char kpkt_com_SYNC0[]={(char)0x00,(char)0xFF,(char)0x96,(char)0xC3};
// char kpkt_com_SYNC1[]={(char)0x01,(char)0xA5,(char)0x82,(char)0x84};
// char kpkt_com_SYNC2[]={(char)0x02,(char)0xF0,(char)0x88,(char)0xEE};
//
// but used aligned integer representation
int kpkt_com_SYNC0=0xC396FF00;
int kpkt_com_SYNC1=0x8482A501;
int kpkt_com_SYNC2=0xEE88F002;

// the overlap of fields is basically centered on the type of packet we expect
//  but a lower packet may be visible if we advanced state before our peer connection
//   and a higher packet may be visible if the peer advanced state before we did
// the table is indexed by .comstate
//
u8 uart_rxhdr_mask[]={
        (0),                                              // COMDOWN shouldn't be looking at input
        (UART_SYNC0_BIT),                                 // COMSYNC0 expects to see these
        (UART_SYNC0_BIT|UART_SYNC1_BIT),                  // COMSYNC1 expects to see these
        (UART_SYNC1_BIT|UART_SYNC2_BIT),                  // COMSYNC2 expects to see these
        (UART_TXPKT_BIT|UART_ACKPKT_BIT),                 // COMLOCK  expects to see these
        (UART_TXPKT_BIT|UART_ACKPKT_BIT)                  // COMXCHG  expects to see these
};

// this is really (size-1) when we've identified the
//  the packet type
u8 uart_rxhdr_minacc[]={
        3,  // SYNC0 (4)
        3,  // SYNC1 (4)
        3,  // SYNC2 (4)
        7,  // TX    (8)
        1   // ACK   (2)
};


// force integer aligned
int pkt_com_TXhdr[(COM_TXPKT_MAXPL+8)>>2];
// char pkt_com_TXhdr[COM_TXPKT_MAXPL+8];
//    ={
//     0x03,                     // 0x00 identifier
//     0x00,                     // 0x01 sequence
//     0x00,0x00,                // 0x02 payload count
//     0x00,0x00,                // 0x04 front end tokens
//     0x00,0x00,                // 0x06 sub-que0 tokens
//     // maximum data payload COM_TXPKT_MAXPL
//     };

// force integer aligned
int pkt_com_RXhdr[(COM_TXPKT_MAXPL+8)>>2];
// char pkt_com_RXhdr[COM_TXPKT_MAXPL+8];
//    ={
//     0x00,                     // 0x00 identifier
//     0x00,                     // 0x01 sequence
//     0x00,0x00,                // 0x02 payload count
//     0x00,0x00,                // 0x04 front end tokens
//     0x00,0x00,                // 0x06 sub-que0 tokens
//     // maximum data payload COM_TXPKT_MAXPL
//     };

char tstrobin_fname[]="/mnt/tpifc/rt00";
raw_filhdr tstrobin_hdr={
        H_def_WIDTH,     // xdim
        H_def_HEIGHT,    // ydim
        0xFFFFFFFF,      // tstmp invalid
        0x00000000,      // tdate
        0x08,            // pbpp
        0x08,            // medge
        0,               // pmod
        0                // flgs
};

// OpenCL.glb

cl_context     CL_context;
cl_device_id   CL_device;
cl_device_type CL_type = CL_DEVICE_TYPE_ALL;
// static char const* CL_PLATFORM_STRING = NULL;

char CL_COMPILER_OPTIONS[]="-w";

// there will be multiples of these
// cl_program     program;

cl_command_queue CL_queue;

char kname_planeguid0[]="PLANEGUID0";
char kname_planeguid1[]="PLANEGUID1";
char kname_shr0[]="HASSSHR0";
char kname_shr1[]="HASSSHR1";
char kname_shr2[]="HASSSHR2";
char kname_shr3[]="HASSSHR3";
char kname_shr4[]="HASSSHR4";
char kname_shr5[]="HASSSHR5";
char kname_nrf0[]="HASSNRF0";
char kname_nrf1[]="HASSNRF1";
char kname_planerecon[]="PLANERECON";
char kname_planecmb0[]="PLANECMB0";
char kname_planecmb1[]="PLANECMB1";
char kname_zoom[]="HASSZOOM";
char kname_post[]="HASSPOST";

char kfile_planeguid0[]="/storage/emulated/0/Dropbox/k_planeguid0.cl";
char kfile_planeguid1[]="/storage/emulated/0/Dropbox/k_planeguid1.cl";
char kfile_shr0[]="/storage/emulated/0/Dropbox/k_hassshr0.cl";
char kfile_shr1[]="/storage/emulated/0/Dropbox/k_hassshr1.cl";
char kfile_shr2[]="/storage/emulated/0/Dropbox/k_hassshr2.cl";
char kfile_shr3[]="/storage/emulated/0/Dropbox/k_hassshr3.cl";
char kfile_shr4[]="/storage/emulated/0/Dropbox/k_hassshr4.cl";
char kfile_shr5[]="/storage/emulated/0/Dropbox/k_hassshr5.cl";
char kfile_nrf0[]="/storage/emulated/0/Dropbox/k_hassnrf0.cl";
char kfile_nrf1[]="/storage/emulated/0/Dropbox/k_hassnrf1.cl";
char kfile_planerecon[]="/storage/emulated/0/Dropbox/k_planerecon.cl";
char kfile_planecmb0[]="/storage/emulated/0/Dropbox/k_planecmb0.cl";
char kfile_planecmb1[]="/storage/emulated/0/Dropbox/k_planecmb1.cl";
char kfile_zoom[]="/storage/emulated/0/Dropbox/k_hasszoom.cl";
char kfile_post[]="/storage/emulated/0/Dropbox/k_hasspost.cl";

char kfile_bld_info[]="/storage/emulated/0/Dropbox/kbld_info.txt";

CL_proc_ctrl kern_ctrl[]={
        // HASS_KINDX_PLANEGUID0
        {
                kfile_planeguid0,
                (char*)0,
                0,
                0,
                kname_planeguid0,
                (cl_program)0,
                (cl_kernel)0,
                kgen_planeGuide
        },
        // HASS_KINDX_PLANEGUID1
        {
                kfile_planeguid1,
                (char*)0,
                0,
                0,
                kname_planeguid1,
                (cl_program)0,
                (cl_kernel)0,
                kgen_planeGuide
        },
        // HASS_KINDX_SHR0
        {
                kfile_shr0,
                (char*)0,
                0,
                0,
                kname_shr0,
                (cl_program)0,
                (cl_kernel)0,
                khass_shrink
        },
        // HASS_KINDX_SHR1
        {
                kfile_shr1,
                (char*)0,
                0,
                0,
                kname_shr1,
                (cl_program)0,
                (cl_kernel)0,
                khass_shrink
        },
        // HASS_KINDX_SHR2
        {
                kfile_shr2,
                (char*)0,
                0,
                0,
                kname_shr2,
                (cl_program)0,
                (cl_kernel)0,
                khass_shrink
        },
        // HASS_KINDX_SHR3
        {
                kfile_shr3,
                (char*)0,
                0,
                0,
                kname_shr3,
                (cl_program)0,
                (cl_kernel)0,
                khass_shrink
        },
        // HASS_KINDX_SHR4
        {
                kfile_shr4,
                (char*)0,
                0,
                0,
                kname_shr4,
                (cl_program)0,
                (cl_kernel)0,
                khass_shrink
        },
        // HASS_KINDX_SHR5
        {
                kfile_shr5,
                (char*)0,
                0,
                0,
                kname_shr5,
                (cl_program)0,
                (cl_kernel)0,
                khass_shrink
        },
        // HASS_KINDX_NRF0
        {
                kfile_nrf0,
                (char*)0,
                0,
                0,
                kname_nrf0,
                (cl_program)0,
                (cl_kernel)0,
                khass_NRfnc
        },
        // HASS_KINDX_NRF1
        {
                kfile_nrf1,
                (char*)0,
                0,
                0,
                kname_nrf1,
                (cl_program)0,
                (cl_kernel)0,
                khass_NRfnc
        },
        // HASS_KINDX_PLANERECON
        {
                kfile_planerecon,
                (char*)0,
                0,
                0,
                kname_planerecon,
                (cl_program)0,
                (cl_kernel)0,
                kgen_planeRecon
        },
        // HASS_KINDX_PLANECMB0
        {
                kfile_planecmb0,
                (char*)0,
                0,
                0,
                kname_planecmb0,
                (cl_program)0,
                (cl_kernel)0,
                kgen_planeComb
        },
        // HASS_KINDX_PLANECMB1
        {
                kfile_planecmb1,
                (char*)0,
                0,
                0,
                kname_planecmb1,
                (cl_program)0,
                (cl_kernel)0,
                kgen_planeComb
        },
        // HASS_KINDX_ZOOM
        {
                kfile_zoom,
                (char*)0,
                0,
                0,
                kname_zoom,
                (cl_program)0,
                (cl_kernel)0,
                khass_zoom
        },
        // HASS_KINDX_POST
        {
                kfile_post,
                (char*)0,
                0,
                0,
                kname_post,
                (cl_program)0,
                (cl_kernel)0,
                khass_post
        }
};


//------------------------------------------------------------------------------
//  code

void wait_thread(void){
    //sleep(1);
    usleep(1000);
    //sched_yield();
}


void lnklst_init(lnklst* head) {
    // an empty list points to itself
    head->fwrd=(void*)head;
    head->rvrs=(void*)head;
}

void lnklst_fins(lnklst* pred,lnklst* ins) {
    // insert element forward of predecessor
    lnklst* succ;
    succ=(lnklst*)(pred->fwrd);
    ins->fwrd=(void*)succ;
    ins->rvrs=(void*)pred;
    pred->fwrd=(void*)ins;
    succ->rvrs=(void*)ins;
}

void lnklst_rins(lnklst* succ,lnklst* ins) {
    // insert element reverse of successor
    lnklst* pred;
    pred=(lnklst*)(succ->rvrs);
    ins->fwrd=(void*)succ;
    ins->rvrs=(void*)pred;
    pred->fwrd=(void*)ins;
    succ->rvrs=(void*)ins;
}

void lnklst_drop(lnklst* dropitem) {
    // drop an item from its place in the list
    lnklst* succ;
    lnklst* pred;
    succ=(lnklst*)(dropitem->fwrd);
    pred=(lnklst*)(dropitem->rvrs);
    succ->rvrs=(void*)pred;
    pred->fwrd=(void*)succ;
}

lnklst* lnklst_fpop(lnklst* head) {
    // returns 0 if the list is empty
    //  otherwise returns most forward element (by pulling from head.rvrs)
    //   and unchains it
    lnklst* retval;
    retval=(lnklst*)(head->rvrs);
    if (retval==head) {
        // empty list
        retval=(lnklst*)0;
    }
    else {
        // unchain the item
        lnklst_drop(retval);
    }
    return(retval);
}

lnklst* lnklst_fprvw(lnklst* ref,lnklst* head) {
    // returns 0 if the list is empty
    //  otherwise returns most forward element (by pulling from ref.rvrs)
    //   but doesn't unchain it
    lnklst* retval;
    retval=(lnklst*)(ref->rvrs);
    if (retval==head) {
        // end of list
        retval=(lnklst*)0;
    }
    // else return retval as is
    return(retval);
}

u32 lnklst_fsrchup(lnklst* head,u64 mtchval,u32 idx,lnklst** pred,lnklst** succ) {
    //
    // this scans for the existence and/or insert position
    //  of an unsigned parmater p[idx] value in the list
    //   it searches in the forward direction (increasing p[idx])
    //    (assumes the list is sorted sequentially on this parameter)
    //
    // returns 0 if it finds a hit,
    //            then succ points to a lnklst element whose p[idx]==mtchval
    // returns !0 if no hit
    //            then pred.p[idx] < mtchval < succ.p[idx]
    //             in this case, either or both of the end elements
    //              could be the lnklst head (which is simultaneously
    //               smaller and larger than any real mtchval
    //
    u64 tstval;
    u32 retval;
    u32 done=0;
    // seed pred as the last failing element checked
    *pred=head;
    while (!done) {
        // succ is forward element from last failure
        *succ=(lnklst*)((*pred)->fwrd);
        if (!((*succ)==head)) {
            tstval=((lnklst_upa*)(*succ))->p[idx];
            if (!(tstval==mtchval)) {
                // no hit at this element
                if (mtchval>tstval) {
                    // a mtchval element would have to be past this successor
                    //  jump over it to continue scan
                    //   last round successor become predecessor
                    *pred=*succ;
                }
                else {
                    // can't jump over this one
                    // *pred and *succ stand as no hit bounds
                    retval=1;
                    done=1;
                }
            }
            else {
                // got a hit, *succ matches search criteria
                retval=0;
                done=1;
            }
        }
        else {
            // went full circle to find the head again
            // *pred and *succ stand as no hit bounds
            retval=1;
            done=1;
        }
    }
    return(retval);
}

//------------------------------------------------------------------------------

void hsp_lnklst_fins(int* hwsema,lnklst* pred,lnklst* ins) {
    // hwsema protected insert element forward of predecessor
    int tst=1;
    // capture the semaphore
    while (tst) {
        tst=__sync_fetch_and_or(hwsema,1);
        if (tst) {
            // someone else already set semaphore
            //  try again next pass
            sched_yield();
        }
    }
    // queue the message
    lnklst_fins(pred,ins);
    // release the semaphore
    tst=__sync_fetch_and_and(hwsema,0);
}

lnklst* hsp_lnklst_fpop(int* hwsema,lnklst* head) {
    // hwsema protected lnklst_fpop
    //  returns 0 if the list is empty
    //   otherwise returns most forward element (by pulling from head.rvrs)
    //    and unchains it
    lnklst* retval;
    int tst=1;
    // capture the semaphore
    while (tst) {
        tst=__sync_fetch_and_or(hwsema,1);
        if (tst) {
            // someone else already set semaphore
            //  try again next pass
            sched_yield();
        }
    }
    retval=(lnklst*)(head->rvrs);
    if (retval==head) {
        // empty list
        retval=(lnklst*)0;
    }
    else {
        // unchain the item
        lnklst_drop(retval);
    }
    // release the semaphore
    tst=__sync_fetch_and_and(hwsema,0);
    return(retval);
}

void bptrqueue_init(bptrqueue* bq,char* buff,int buffsize) {
    // buffsize must be power of 2
    bq->spr0C=0;
    bq->push=0;
    bq->pull=0;
    bq->mask=buffsize-1;
    bq->body=buff;
}

void bptrquetx_init(bptrquetx* bq,char* buff,int buffsize) {
    // buffsize must be power of 2
    bq->tok=0;
    bq->push=0;
    bq->pull=0;
    bq->mask=buffsize-1;
    bq->body=buff;
}

int bptrqueue_avail(bptrqueue* bq) {
    // returns number of bytes that can be read
    int retval;
    retval=((bq->push)-(bq->pull))&(bq->mask);
    return(retval);
}

int bptrquetx_avail(bptrquetx* bq) {
    // returns number of bytes that can be read
    int retval;
    retval=bptrqueue_avail((bptrqueue*) bq);
    if ((bq->tok)<retval) {
        retval=(bq->tok);
    }
    return(retval);
}

int bptrqueue_space(bptrqueue* bq) {
    // returns room to write to the queue
    //  (intentionally underestimates by BPTRQUEUE_SAFE)
    int retval;
    // total space
    retval=((bq->mask)+1);
    // total minus read availability
    retval=retval-(((bq->push)-(bq->pull))&(bq->mask));
    // wrap safety margin
    retval=retval-BPTRQUEUE_SAFE;
    if (retval<0) {
        retval=0;
    }
    return(retval);
}

void bptrqueue_rd(bptrqueue* bq,char* dst,int num) {
    // assumes the read is available will fit - no checks
    char* src;
    int offset;
    offset=bq->pull;
    while (num) {
        src=((bq->body)+offset);
        *dst=*src;
        dst++;
        offset=(offset+1)&(bq->mask);
        num--;
    }
    bq->pull=offset;
}

void bptrqueue_wr(bptrqueue* bq,char* src,int num) {
    // assumes the write will fit - no checks
    char* dst;
    int offset;
    offset=bq->push;
    while (num) {
        dst=((bq->body)+offset);
        *dst=*src;
        src++;
        offset=(offset+1)&(bq->mask);
        num--;
    }
    bq->push=offset;
}

void bptr_que_to_que(bptrqueue* bqdst,bptrqueue* bqsrc,int num) {
    // read from one queue, write directly to another
    char* dst;
    int doff;
    char* src;
    int soff;
    doff=bqdst->push;
    soff=bqsrc->pull;
    while (num) {
        dst=((bqdst->body)+doff);
        src=((bqsrc->body)+soff);
        *dst=*src;
        doff=(doff+1)&(bqdst->mask);
        soff=(soff+1)&(bqsrc->mask);
        num--;
    }
    bqdst->push=doff;
    bqsrc->pull=soff;
}

char bptrqueue_prvw(bptrqueue* bq,int index) {
    // assumes index is available, preview the character without changing push/pull pointers
    char* src;
    int offset;
    char retval;
    offset=((bq->pull)+index)&(bq->mask);
    src=((bq->body)+offset);
    retval=*src;
    return(retval);
}

void bptrqueue_drop(bptrqueue* bq,int num) {
    // assumes num characters available - just drop them
    bq->pull=((bq->pull)+num)&(bq->mask);
}

//-----------------------------------------------------------------------------

memctrl* memctrl_init(u64 aptr,u64 blksize) {
    //
    // aptr is MEMBLKUNIT aligned
    // blksize is multiple of MEMBLKUNIT (but bigger)
    //
    memctrl* retval = (memctrl*)aptr;
    // freepool starts empty
    lnklst_init((lnklst*)aptr);
    aptr+=sizeof(lnklst);
    // mallpool starts empty
    lnklst_init((lnklst*)aptr);
    aptr+=sizeof(lnklst);
    // mblkpool starts empty
    lnklst_init((lnklst*)aptr);
    aptr+=sizeof(lnklst);
    // hwsema and spr[3] init empty
    *((u64*)(aptr))=0;
    *(((u64*)(aptr))+1)=0;
    aptr+=(2*sizeof(u64));
    // treat aptr as first memblk element in freepool
    (((memblk*)aptr)->addr)=((u64)(retval))+MEMBLKUNIT;
    (((memblk*)aptr)->size)=blksize-MEMBLKUNIT;
    // and chain it into freepool
    lnklst_fins((lnklst*)retval,(lnklst*)aptr);
    aptr+=sizeof(memblk);
    // what's left of the first modulo-MEMBLKUNIT page is
    //  an mblk with MEMBLKMBLK-3 elements, whose start address is itself
    (((memblk*)aptr)->addr)=((u64)aptr);
    (((memblk*)aptr)->size)=(((u64)MEMBLKUNIT)-(aptr-((u64)retval)));
    // and chain it into mblkpool
    lnklst_fins((lnklst*)(&(retval->mblkpool)),(lnklst*)aptr);
    // done
    return(retval);
}

u64 memctrl_mblkget(memctrl* mc) {
    //
    // given we need an mblk from the mblkpool
    //
    // returns - retval !0 is unchained memblk (just an address)
    //         - retval =0 (none avaliable)
    //
    lnklst* mbpool;
    u64 mblk;
    u64 msz;
    u64 retval=0;
    mbpool=&(mc->mblkpool);
    // point first element
    mblk=(u64)(mbpool->fwrd);
    if (!(mblk==((u64)(mbpool)))) {
        // first mblk contains something
        // reduce size and write it back
        msz=(((memblk*)(mblk))->size);
        msz-=MBLK_SIZE;
        (((memblk*)(mblk))->size)=msz;
        // return value is address of mblk stripped out
        retval=mblk+msz;
        // optionally disconnect empty element from mblkpool
        if (!msz) {
            lnklst_drop((lnklst*)mblk);
        }
    }
    else {
        // mbpool is empty, need to appeal to freepool
        mbpool=&(mc->freepool);
        // point first element
        mblk=(u64)(mbpool->fwrd);
        if (!(mblk==((u64)(mbpool)))) {
            // first mblk contains something
            //  we're converting the mblk.address as a colection of mblk's
            // check if it's bigger than MEMBLKUNIT base block
            msz=(((memblk*)(mblk))->size);
            retval=(((memblk*)(mblk))->addr);
            if (msz>MEMBLKUNIT) {
                // the block survives as a freepool element
                //  reduce its size, and raise its address
                (((memblk*)(mblk))->size)=msz-MEMBLKUNIT;
                (((memblk*)(mblk))->addr)=retval+MEMBLKUNIT;
            }
            else {
                // implicitly the size of this block is exactly MEMBLKUNIT
                //  disconnect it from the freepool
                lnklst_drop((lnklst*)mblk);
            }
            // retval is a blk in its own right
            //  its address is itself, and its size is (MEMBLKMBLK-1) mblks
            //   after we pull one off the end
            msz=MEMBLKUNIT-MBLK_SIZE;
            (((memblk*)(retval))->size)=msz;
            (((memblk*)(retval))->addr)=retval;
            // chain it into the mblkpool
            mbpool=&(mc->mblkpool);
            lnklst_fins(mbpool,(lnklst*)retval);
            // what we return is the last mblk in the group of MEMBLKMBLK
            retval+=msz;
        }
        // else freepool is also empty
        //  return default retval=0 failed attempt
    }
    return(retval);
}

u64 memctrl_mblkret(memctrl* mc,u64 aret) {
    //
    // given we want to return an mblk to the mblkpool
    //  aret is the mblk being returned, but it's just an address
    //   not yet configured as an mblk
    //
    // normally returns 0 on success
    // returns 1 for error (severe)
    //
    lnklst* head;
    memblk* pred;
    memblk* succ;
    u64 tstval;
    u64 tstmerge;
    u64 retval=0;
    head=&(mc->mblkpool);
    // we don't expect to find a hit
    tstval=lnklst_fsrchup(head,aret,0,(lnklst**)(&pred),(lnklst**)(&succ));
    if (tstval) {
        // aret fits between pred and succ
        // check for merge to downward link - assume not possible
        tstmerge=0;
        if (aret&((u64)MEMBLKMASK)) {
            if (!(((u64)pred)==((u64)head))) {
                if (aret==((pred->addr)+(pred->size))) {
                    // all feasability checks pass
                    tstmerge=1;
                }
                // else not contiguous
            }
            // else you can't merge with the head
        }
        // else page boundary can't merge downward
        if (tstmerge) {
            // raise the size of the downward block
            pred->size=(pred->size)+MBLK_SIZE;
        }
        else {
            // aret is disjoint from predecessor
            // create an mblk from this address, whose address is itself
            ((memblk*)(aret))->addr=aret;
            ((memblk*)(aret))->size=MBLK_SIZE;
            // link it into chain
            lnklst_fins((lnklst*)pred,(lnklst*)aret);
            // and make it the new predecessor as focus object
            pred=(memblk*)aret;
        }
        // check for merge with upward link - assume not possible
        tstmerge=0;
        if ((succ->addr)&MEMBLKMASK) {
            if (!(((u64)succ)==((u64)head))) {
                if (((pred->addr)+(pred->size))==((u64)(succ))) {
                    // all feasability checks pass
                    tstmerge=1;
                }
                // else not contiguous
            }
            // else you can't merge with the head
        }
        // else can't upward merge to a page boundary
        if (tstmerge) {
            // the predecessor absorbs the successor
            // predecessor size increases
            pred->size=(pred->size)+(succ->size);
            // successor gets dropped
            lnklst_drop((lnklst*)succ);
        }
        //------------------------------------------------------------------
        // the landscape has changed, now hunt for a full mblk we can free
        // tstmerge is total mblks not full
        tstmerge=0;
        // succ is a detected full mblk
        succ=(memblk*)0;
        // scan the list
        pred=(memblk*)(((lnklst*)(head))->fwrd);
        while (!(((u64)pred)==((u64)head))) {
            if ((pred->size)==MEMBLKUNIT) {
                // succ detects full mblk
                succ=pred;
            }
            else {
                // otherwise tstmerge includes it in sum of non-full mblks
                tstmerge+=(pred->size);
            }
            // continue scan
            pred=(memblk*)(((lnklst*)pred)->fwrd);
        }
        if (succ) {
            // we want to have at least 2 mblk's available beyond the full set
            if (tstmerge>=(2*MBLK_SIZE)) {
                // disconnect the full block
                lnklst_drop((lnklst*)succ);
                // get an mblk to represent it
                pred=(memblk*)(memctrl_mblkget(mc));
                // configure it to represent full mblk area
                pred->addr=(u64)succ;
                pred->size=MEMBLKUNIT;
                // send that memory to the freepool
                // pass error check value to retval
                retval=memctrl_fblkret(mc,pred);
            }
        }
        // else no full mblk detected
        //------------------------------------------------------------------
    }
    else {
        // if we got a hit, there's something really wrong
        //  we're returning a block that's already in the pool!
        //   flag the error on exit
        retval=1;
    }
    return(retval);
}

u64 memctrl_fblkret(memctrl* mc,memblk* mret) {
    //
    // given we want to return an mblk to the freepool
    //  mret is the mblk being returned, containing address and size information
    //
    // normally returns 0 on success
    // returns 1 for error (severe)
    //
    lnklst* head;
    memblk* pred;
    memblk* succ;
    u64 tstval;
    u64 tstmerge;
    u64 retval=0;
    head=&(mc->freepool);
    // we don't expect to find a hit
    tstval=lnklst_fsrchup(head,(mret->addr),0,(lnklst**)(&pred),(lnklst**)(&succ));
    if (tstval) {
        // mret fits between pred and succ
        // check for merge to downward link - assume not possible
        tstmerge=0;
        if (!(((u64)pred)==((u64)head))) {
            if ((mret->addr)==((pred->addr)+(pred->size))) {
                // all feasability checks pass
                tstmerge=1;
                // pred absorbs new block
                pred->size=(pred->size)+(mret->size);
                // return that mblk, pass error check value to retval
                retval=memctrl_mblkret(mc,(u64)mret);
            }
            // else not contiguous
        }
        // else you can't merge with the head
        if (!tstmerge) {
            // new block joins the list
            lnklst_fins((lnklst*)pred,(lnklst*)mret);
            // and new block becomes pred
            pred=mret;
        }
        if (!retval) {
            // check for merge to upward link
            if (!(((u64)succ)==((u64)head))) {
                if (((pred->addr)+(pred->size))==(succ->addr)) {
                    // we can now merge pred to succ
                    // successor survives with reduced address and increased size
                    succ->addr=(pred->addr);
                    succ->size=(succ->size)+(pred->size);
                    // disconnect the pred
                    lnklst_drop((lnklst*)pred);
                    // and return its mblk (checking for error)
                    retval=memctrl_mblkret(mc,(u64)pred);
                }
                // else not contiguous
            }
            // else you can't merge with the head
        }
    }
    else {
        // if we got a hit, there's something really wrong
        //  we're returning a block that's already in the pool!
        //   flag the error on exit
        retval=1;
    }
    return(retval);
}

u64 memctrl_mallret(memctrl* mc,u64 aret) {
    //
    // given we want to free an allocated block from mallpool to the freepool
    //  this is the essense of free()
    //
    // normally returns 0 on success
    // returns 1 for error (severe)
    //
    lnklst* head;
    memblk* pred;
    memblk* succ;
    u64 tstval;
    u64 retval=0;
    head=&(mc->mallpool);
    // we expect to find a hit at successor
    tstval=lnklst_fsrchup(head,aret,0,(lnklst**)(&pred),(lnklst**)(&succ));
    if (!tstval) {
        // disconnect it from the mall chain
        lnklst_drop((lnklst*)succ);
        // and return the memblk to the freepool
        //  passing errors to retval
        retval=memctrl_fblkret(mc,succ);
    }
    else {
        // if we didn't get a hit, there's something really wrong
        //  we're returning a block that's not in the allocation pool!
        //   flag the error on exit
        retval=1;
    }
    return(retval);
}

u64 memctrl_mallrec(memctrl* mc,memblk* mall) {
    //
    // given we want to record a memblk that has been granted, in the mallpool
    //  mall is the memblk being recorded, containing address and size information
    //
    // normally returns 0 on success
    // returns 1 for error (severe)
    //
    lnklst* head;
    memblk* pred;
    memblk* succ;
    u64 tstval;
    u64 retval=0;
    head=&(mc->mallpool);
    // we don't expect to find a hit
    tstval=lnklst_fsrchup(head,(mall->addr),0,(lnklst**)(&pred),(lnklst**)(&succ));
    if (tstval) {
        // connect the link between the pred and succ
        lnklst_fins((lnklst*)pred,(lnklst*)mall);
    }
    else {
        // if we got a hit, there's something really wrong
        //  we're recording an alloction that's already issued!
        //   flag the error on exit
        retval=1;
    }
    return(retval);
}

u64 memctrl_freeget(memctrl* mc,u64 reqsize,u64* res) {
    //
    // size must be modulo-MEMBLKUNIT
    //
    // given we want to issue free memory to an application
    //  and keep track of it in the mallpool
    //   this is the essence of malloc()
    //
    // normally returns 0 on success
    //                     res =  0 no memory granted
    //                         != 0 address of memory region granted
    // returns 1 for error (severe)
    //
    lnklst* head;
    memblk* scan;
    u64 mblk;
    u64 hit;
    u64 retval=0;
    //
    // it is rare to find a free piece of memory exactly the right size
    //  more often than not, you find a big chunk of memory
    //   and lop off the piece you need
    //    that involves splitting the bickchunk represented by an mblk
    //     into two smaller chunks - this issue piece,
    //      and the piece retained in the freepool
    // so sometimes we get lucky and find an exact fit...
    //  then the effort to ask for an mblk first
    //   is less optimal
    //    but first get an available mblk to handle any splitting of chunks
    //     if we can't get an mblk, we know the freepool is empty too
    //
    // assume no grant
    *res=0;
    mblk=memctrl_mblkget(mc);
    if (mblk) {
        head=&(mc->freepool);
        // seed the scan
        scan=(memblk*)(((lnklst*)(head))->rvrs);
        hit=0;
        // exit on done or end of list
        while ( (!hit) && (!(((u64)scan)==((u64)head))) ) {
            if ((scan->size)>=reqsize) {
                // scan link can supply request
                if ((scan->size)==reqsize) {
                    // scan block is right sized...
                    // we don't need the mblk
                    retval=memctrl_mblkret(mc,mblk);
                    if (!retval) {
                        // we'll use the scan element as the allocated mblk
                        mblk=(u64)scan;
                        // just disconnect it from freepool
                        lnklst_drop((lnklst*)scan);
                    }
                }
                else {
                    // scan block is oversized
                    //  use mblk to cut off what we need from the end of scan
                    scan->size=(scan->size)-reqsize;
                    ((memblk*)(mblk))->addr=(scan->addr)+(scan->size);
                    ((memblk*)(mblk))->size=reqsize;
                }
                hit=1;
            }
            else {
                // scan link too small, continue scan
                scan=(memblk*)(((lnklst*)scan)->rvrs);
            }
        }
        if ((hit) && (!retval)) {
            // mblk gets linked into mallpool
            // return res=address granted
            *res=((memblk*)(mblk))->addr;
            retval=memctrl_mallrec(mc,((memblk*)mblk));
        }
    }
    // else no starting mblk available
    return(retval);
}

//-----------------------------------------------------------------------------

void memcCapt(void) {
    // capture the memc lock semaphore
    int* sema;
    int tst=1;
    sema=(int*)(&((ACptr->memc)->hwsema));
    while (tst) {
        tst=__sync_fetch_and_or(sema,1);
        if (tst) {
            // someone else already set semaphore
            //  try again next pass
            sched_yield();
        }
    }
}

void memcRlse(void) {
    // release the memc lock semaphore
    int* sema;
    int tst;
    sema=(int*)(&((ACptr->memc)->hwsema));
    tst=__sync_fetch_and_and(sema,0);
}

void* appmalloc(u32 size) {
    // returns memory allocated
    //  (may set a crash code and return NULL ptr
    //    in fact this version sets crash code if it returns NULL pointer for any reason)
    u64 tstval;
    u32 crashret=0;
    u64 addr=0;
    if (size) {
        // round up size to modulo-MEMBLKUNIT
        if (size&MEMBLKMASK) {
            size=size&(~MEMBLKMASK);
            size=size+MEMBLKUNIT;
        }
        // try to get memory
        memcCapt();
        tstval=memctrl_freeget(ACptr->memc,(u64)size,&addr);
        memcRlse();
        if (!tstval) {
            // no memctrl internal errors
            if (!addr) {
                // current memory can't handle it
                //  too much input to handle or, MEMCTRL_SIZE build parameter too small
                crashret=CRASH_MEMCTRL;
            }
            // else addr is valid retval
        }
        else {
            // local memctrl system failure
            crashret=CRASH_MEMCTRL;
        }
    }
    else {
        // STUPID USER trying to malloc(0)
        crashret=CRASH_MEMCTRL;
    }
    if (crashret) {
        thread_set_crash(crashret);
        addr=0;
    }
    return ((void*)addr);
}

void appfree(void* addr) {
    //  (may set a crash code)
    u64 tstval;
    memcCapt();
    tstval=memctrl_mallret(ACptr->memc,(u64)addr);
    memcRlse();
    if (tstval) {
        thread_set_crash(CRASH_MEMCTRL);
    }
}

//-----------------------------------------------------------------------------

void init_gblkchain(void) {
    // initialize gblk chain empty
    lnklst_init((lnklst*)(&(ACptr->gblkhead)));
}

void gblkCapt(void) {
    // capture the gblk lock semaphore
    int* sema;
    int tst=1;
    sema=(int*)(&(ACptr->hwsema_gblk));
    while (tst) {
        tst=__sync_fetch_and_or(sema,1);
        if (tst) {
            // someone else already set semaphore
            //  try again next pass
            sched_yield();
        }
    }
}

void gblkRlse(void) {
    // release the gblk lock semaphore
    int* sema;
    int tst;
    sema=(int*)(&(ACptr->hwsema_gblk));
    tst=__sync_fetch_and_and(sema,0);
}

void* gblkalloc(void) {
    // returns memory allocated
    //  (may set a crash code and return NULL ptr
    //    in fact this version sets crash code if it returns NULL pointer for any reason)
    lnklst* head;
    lnklst* ptr;
    int i;
    u32 crashret=0;
    // is there anything in the list presently
    head=(lnklst*)(&(ACptr->gblkhead));
    gblkCapt();
    ptr=(lnklst*)(head->rvrs);
    if (ptr==head) {
        // when pool is empty, need to replenish it
        //  before we can draw from it
        ptr=(lnklst*)appmalloc(MEMGBLKXFER);
        if (ptr) {
            // convert allocated memory into more gblk_64's
            for (i=0;i<GBLK64REFRESH;i++) {
                lnklst_fins(head,ptr);
                ptr=(lnklst*)( ((char*)ptr)+64 );
            }
            // deliver result
            ptr=lnklst_fpop(head);
        }
        // else return ptr=0 crash recorded
    }
    else {
        // ptr is block we can return
        //  just disconnect it from the chain
        lnklst_drop(ptr);
    }
    gblkRlse();
    return ((void*)ptr);
}

void gblkfree(void* dropitem) {
    gblkCapt();
    lnklst_rins(((lnklst*)(&(ACptr->gblkhead))),((lnklst*)dropitem));
    gblkRlse();
}

//-----------------------------------------------------------------------------



void msgqueue_submit(msg_queue* queue,msg_gblkfnc* msg) {
    int* hwsema;
    int tst=1;
    // point semaphore
    hwsema=(int*)(&(queue->sema));
    // capture the semaphore
    while (tst) {
        tst=__sync_fetch_and_or(hwsema,1);
        if (tst) {
            // someone else already set semaphore
            //  try again next pass
            sched_yield();
        }
    }
    // queue the message
    lnklst_fins((lnklst*)queue,(lnklst*)msg);
    // release the semaphore
    tst=__sync_fetch_and_and(hwsema,0);
}

//-----------------------------------------------------------------------------
// msg send variants

u32 msgexec_gblkvoid(msg_queue* queue,gblkfnc fnc,s64 a0,s64 a1,s64 a2) {
    // returns 1 on crash abort, else 0
    //
    // for a void function call, the sender assumes the action is completed when
    //  the message is dispatched
    //   all necessary information to complete the task travels in the body of the message
    //
    msg_gblkfnc* msg;
    volatile int done=0;
    u32 error=0;
    // need a mesage carrier
    msg=(msg_gblkfnc*)gblkalloc();
    if (msg) {
        // have message carrier
        msg->fncptr=fnc;
        msg->dstptr=(void*)0;
        msg->arg0=a0;
        msg->arg1=a1;
        msg->arg2=a2;
        msg->doneptr=(int*)0;
        msgqueue_submit(queue,msg);
    }
    else {
        // output crash detected
        error=1;
        utp_dbgoutput("no msg available\n");
    }
    return(error);
}

u32 msgexec_gblkvack(msg_queue* queue,gblkfnc fnc,s64 a0,s64 a1,s64 a2) {
    // returns 1 on crash abort, else 0
    //
    // although the function may return void, if we pass pointers to data
    //  that must be held by the sender until used/consumed/copied at execution
    //   process, then we have to wait on a "done" marker to signal it
    //    is safe to proceed (and the input data structures can be modified or
    //     destroyed)
    //
    msg_gblkfnc* msg;
    volatile int done=0;
    u32 error=0;
    // need a mesage carrier
    msg=(msg_gblkfnc*)gblkalloc();
    if (msg) {
        // have message carrier
        msg->fncptr=fnc;
        msg->dstptr=(void*)0;
        msg->arg0=a0;
        msg->arg1=a1;
        msg->arg2=a2;
        msg->doneptr=&done;
        msgqueue_submit(queue,msg);
        // wait for "done" indication
        while (!done) {
            sched_yield();
        }
    }
    else {
        // output crash detected
        error=1;
    }
    return(error);
}

u32 msgexec_gblkfnc(msg_queue* queue,gblkfnc fnc,void* dst,s64 a0,s64 a1,s64 a2) {
    // returns 1 on crash abort, else 0
    //
    // when the function provides a return value, the dst pointer indicates
    //  the destination for the return data (which may be a structure)
    //   (you could pass additional args for destination structure(s)
    //     in the calling args as well)
    //
    //  in this case, the "done" marker indicates both
    //   - that all input data may be destroyed
    //   - and that all return data has been dispatched
    //
    //  this variant waits on internal "done"
    //
    msg_gblkfnc* msg;
    volatile int done=0;
    u32 error=0;
    // need a mesage carrier
    msg=(msg_gblkfnc*)gblkalloc();
    if (msg) {
        // have message carrier
        msg->fncptr=fnc;
        msg->dstptr=dst;
        msg->arg0=a0;
        msg->arg1=a1;
        msg->arg2=a2;
        msg->doneptr=&done;
        msgqueue_submit(queue,msg);
        // wait for "done" indication
        while (!done) {
            sched_yield();
        }
    }
    else {
        // output crash detected
        error=1;
    }
    return(error);
}

u32 msgexec_gblkpost(msg_queue* queue,gblkfnc fnc,void* dst,s64 a0,s64 a1,s64 a2,volatile int* done) {
    // returns 1 on crash abort, else 0
    //
    // when the function provides a return value, the dst pointer indicates
    //  the destination for the return data (which may be a structure)
    //   (you could pass additional args for destination structure(s)
    //     in the calling args as well)
    //
    //  in this case, the "done" marker indicates both
    //   - that all input data may be destroyed
    //   - and that all return data has been dispatched
    //
    //  this variant provides a done flag externally
    //   it posts the message and doesn't wait on the result
    //    useful for when the sender just wants to post the message
    //     and do other things while waiting
    //      and check periodically or at some later time, for how to
    //       proced after done
    //
    msg_gblkfnc* msg;
    u32 error=0;
    // need a mesage carrier
    msg=(msg_gblkfnc*)gblkalloc();
    if (msg) {
        // have message carrier
        msg->fncptr=fnc;
        msg->dstptr=dst;
        msg->arg0=a0;
        msg->arg1=a1;
        msg->arg2=a2;
        msg->doneptr=done;
        msgqueue_submit(queue,msg);
    }
    else {
        // output crash detected
        error=1;
    }
    return(error);
}

//-----------------------------------------------------------------------------

void msgqueue_import(lnklst* dst,msg_queue* queue) {
    // import contents of a public sema protected message queue
    //  to a private queue for servicing
    //   retval=1 (true) means something was imported
    //         =0 (false) means msgqueue and dst are both empty
    lnklst* fptr;
    lnklst* rptr;
    int* hwsema;
    int tst=1;
    // insure dst chain starts empty
    lnklst_init(dst);
    // point semaphore
    hwsema=(int*)(&(queue->sema));
    // capture the semaphore
    while (tst) {
        tst=__sync_fetch_and_or(hwsema,1);
        if (tst) {
            // someone else already set semaphore
            //  try again next pass
            sched_yield();
        }
    }
    fptr=(lnklst*)(((lnklst*)queue)->fwrd);
    if  (!(fptr==((lnklst*)queue))) {
        // public queue contains something
        rptr=(lnklst*)(((lnklst*)queue)->rvrs);
        // empty the queue
        lnklst_init((lnklst*)queue);
        // release the semaphore
        tst=__sync_fetch_and_and(hwsema,0);
        // dst que inherits links
        dst->fwrd=(void*)fptr;
        dst->rvrs=(void*)rptr;
        // and links connect to new head
        fptr->rvrs=(void*)dst;
        rptr->fwrd=(void*)dst;
        // done with content signalling
        // retval=1;
    }
    else {
        // public queue is empty
        // release the semaphore
        tst=__sync_fetch_and_and(hwsema,0);
        // retval=0;
    }
    // return (retval);
}

void msgqueue_service(msg_queue* queue) {
    lnklst head;
    msg_gblkfnc* msg;
    gblkfnc fnc;
    void* ioptr;
    int done;
    int tst;
    int handoff;
    // import public msg queue to private local queue
    msgqueue_import(&head,queue);
    done=0;
    while (!done) {
        msg=(msg_gblkfnc*)lnklst_fpop(&head);
        if (msg) {
            // popped next message to execute
            fnc=msg->fncptr;
            ioptr=(void*)(&(msg->dstptr));
            // run the function
            handoff=(*fnc)(ioptr);
            // the service loop is responsible
            //  to dispose of the gblk carrier unless that responsibility
            //   got handed off through the function call
            if (!handoff) {
                gblkfree((void*)msg);
            }
        }
        else {
            // empty list, no more messages
            done=1;
        }
    }
}

//------------------------------------------------------------------------------
// customizations for efficiency

u32 msgexec_gblkvoid_a2(msg_queue* queue,gblkfnc fnc,s64 a0,s64 a1) {
    // returns 1 on crash abort, else 0
    //  a customization of msgexec_gblkvoid() that only passes 2 calling args
    //   .arg2 and .doneptr are not overwritten
    //    and the called .fncptr should be aware of how to deal that
    //     custom implementation
    //
    // for a void function call, the sender assumes the action is completed when
    //  the message is dispatched
    //   all necessary information to complete the task travels in the body of the message
    //
    msg_gblkfnc* msg;
    u32 error=0;
    // need a mesage carrier
    msg=(msg_gblkfnc*)gblkalloc();
    if (msg) {
        // have message carrier
        msg->fncptr=fnc;
        msg->dstptr=(void*)0;
        msg->arg0=a0;
        msg->arg1=a1;
        // msg->arg2=a2;
        // msg->doneptr=(int*)0;
        msgqueue_submit(queue,msg);
    }
    else {
        // output crash detected
        error=1;
    }
    return(error);
}

void msgexec_gblkv_carr(msg_queue* queue,gblkfnc fnc,msg_gblkfnc* msg) {
    //
    //  a customization of msgexec_gblkvoid() that passes no calling args
    //   .dstptr ... .doneptr are not overwritten
    //    and the called .fncptr should be aware of how to deal that
    //     caller is responsible for all that setup
    //      fncptr handler is aware of the custom implementation
    //  this implementation doesn't even allocate a carrier
    //
    // for a void function call, the sender assumes the action is completed when
    //  the message is dispatched
    //   all necessary information to complete the task travels in the body of the message
    //
    msg->fncptr=fnc;
    msgqueue_submit(queue,msg);
}

void camCapt(void) {
    // capture the camera lock semaphore
    int* hwsema;
    int tst=1;
    hwsema=(int*)(&(ACptr->hwsema_cam));
    while (tst) {
        tst=__sync_fetch_and_or(hwsema,1);
        if (tst) {
            // someone else already set semaphore
            //  try again next pass
            sched_yield();
        }
    }
}

void camRlse(void) {
    // release the camera lock semaphore
    int* hwsema;
    int tst;
    hwsema=(int*)(&(ACptr->hwsema_cam));
    tst=__sync_fetch_and_and(hwsema,0);
}

u32 utp_cnsl_FSM(void) {
    // signalling ocnsl messages on deck
    //  only gets called when UTP is "up" and not reset
    u32 st;
    u32 error;
    u32 done;
    error=0;
    done=0;
    while ((!done) && (!error)) {
        st=(u32)(utpptr->ocnslst);
        switch (st) {
            case CNSLST_IDLE:
                if (!((utpptr->osig)&PFL_OSIG_CNSL)) {
                    // output signal already commanded low,
                    //  proceed to next state, not done
                    utpptr->ocnslst=CNSLST_IECHO;
                }
                else {
                    // output signal not commanded low yet,
                    //  do it now, and update sig_a2pc that is already on deck
                    utpptr->osig=(utpptr->osig)&(~PFL_OSIG_CNSL);
                    utpptr->utphdr.sig_a2pc=(utpptr->utphdr.sig_a2pc)&(~PFL_OSIG_CNSL);
                    // need another status cycle to get that out
                    //  and have the chance to echo...
                    //   so advance state and we're done this pass
                    utpptr->ocnslst=CNSLST_IECHO;
                    done=1;
                }
                break;
            case CNSLST_IECHO:
                // waiting on echo of our output to show CNSL signal low
                if ((utpptr->eosig)&PFL_OSIG_CNSL) {
                    // echo signal still high, try again next pass
                    done=1;
                }
                else {
                    // echo signal is showing low
                    //  advance state, not done yet
                    utpptr->ocnslst=CNSLST_WTODCK;
                }
                break;
            case CNSLST_WTODCK:
                // don't raise the CNSL output signal until there is
                //  an output console line ready to send
                if (utpptr->ocnslnum) {
                    // confirmed that a console line output is ready
                    //  it can't get deployed unitl polled for by uaiox command
                    //   but PC side should see a rising CNSL signal now
                    utpptr->osig=(utpptr->osig)|PFL_OSIG_CNSL;
                    utpptr->utphdr.sig_a2pc=(utpptr->utphdr.sig_a2pc)|PFL_OSIG_CNSL;
                    utpptr->ocnslst=CNSLST_WTLNPU;
                    done=1;
                }
                else {
                    // try again next pass
                    done=1;
                }
                break;
            case CNSLST_WTLNPU:
                // do nothing, we don't return to CNSLST_IDLE
                //  until we get a uuiox message to pick up an
                //   output console line
                //    (or the queue gets flushed by utp teardown failure)
                done=1;
                break;
            default:
                // unknown state resets/realigns
                error=1;
                break;
        }
    }
    return(error);
}

//------------------------------------------------------------------------------

void ofile_release(ofil_carrier* ofil) {
    // generic release for any kind of ofil_carrier
    switch (ofil->ofltype) {
        case OFLTYP_CFE:
            // release cfe_image target of carrier - call as void
            // error=cfe_refill((cfei_image*)(ofil->refptr));
            cfe_refill((cfei_image*)(ofil->refptr));
            break;
        case OFLTYP_COFL:
            // release cofl_image target of carrier - call as void
            // error=cofl_refill((cofl_image*)(ofil->refptr));
            cofl_refill((cofl_image*)(ofil->refptr));
            break;

            // add other types as needed

        default:
            // this shouldn't happen - crash the program
            thread_set_crash(CRASH_UTPSYS);
            break;
    }
    // when the carrier target is released, we're done with this carrier
    gblkfree((void*)ofil);
    //ofilRlse();
    usbTransferSuccess();
}

u32 utp_ofil_promote(ofil_carrier* ofil) {
    // returns crash error true
    u32 error;
    char* src;
    // for verbosity debug
    // cfei_image* croot;
    int siz;
    error=0;
    // depending on type of ofile, prepare for data transmission
    //   if it has to output from a file system file, then it may fail open()
    //    and return error
    switch (ofil->ofltype) {
        case OFLTYP_CFE:
        case OFLTYP_COFL:
            src=((char*)(ofil->refptr))+0x30;
            siz=ofil->filsize;
            // set auto-transfer controls
            utpptr->oflsrc=src;
            utpptr->oflsiz=siz;
            // VERBOSITY debug
            // croot=(cfei_image*)(ofil->refptr);
            // utp_dbgoutput("CFEI/COFL Android send header:\n");
            // sprintf(utpdbgln,"   xdim %04X ydim %04X tstmp %08X tdat %08X\n",
            //                  croot->xdim,croot->ydim,croot->tstmp,croot->tdate);
            // utp_dbgoutput(utpdbgln);
            // sprintf(utpdbgln,"   pbpp %02X medge %02X pmod %02X flgs %02X\n",
            //                  croot->pbpp,croot->medge,croot->pmod,croot->flgs);
            // utp_dbgoutput(utpdbgln);
            // no header to send, set next state
            utpptr->bost=UTPBOST_PAYLD;
            break;

            // add other types as needed

        default:
            // this shouldn't happen - crash the program
            error=1;
            break;
    }
    return(error);
}

u32 utp_ofil_body(ofil_carrier* ofil,u32* done) {
    // returns crash error true
    // set *done if stalled or incomplete
    u32 error;
    u32 osz;
    error=0;
    // depending on type of ofile, it can tranfer output differently
    switch (ofil->ofltype) {
        case OFLTYP_CFE:
        case OFLTYP_COFL:
            // seed osz with output data
            osz=utpptr->oflsiz;
            // only the last chunk can be short
            //  all start and middle chunks are full size...
            if (osz>UTP_MAX_BLK_XFR) {
                osz=UTP_MAX_BLK_XFR;
            }
            error=utp_bfifo_wr((utpptr->oflsrc),osz);
            if (!error) {
                // psh pointer is auto-advanced
                // post advance read indicators
                utpptr->oflsiz=(utpptr->oflsiz)-osz;
                utpptr->oflsrc=(utpptr->oflsrc)+osz;
            }
            break;

            // add other types as needed

        default:
            // this shouldn't happen - crash the program
            error=1;
            break;
    }
    return(error);
}

u32 utp_blko_FSM(void) {
    // returns crash error true
    u32 error;
    u32 done;
    u8 st;
    u32 msk;
    u32 val;
    ofil_carrier* ofil;
    error=0;
    done=0;
    while ((!done) && (!error)) {
        st=utpptr->bost;
        switch (st) {
            case UTPBOST_IDLE:
                // .oflwrk is empty - objective promote an output file
                ofil=(ofil_carrier*)lnklst_fpop(&(utpptr->bochn));
                if (ofil) {
                    // VERBOSITY debug -- you could print details as well
                    // utp_dbgoutput("UTPBOST_IDLE has ofil\n");
                    // promote this ofil to working unload
                    utpptr->oflwrk=(void*)ofil;
                    error=utp_ofil_promote(ofil);
                }
                else {
                    // try again next pass
                    done=1;
                }
                break;
            case UTPBOST_PAYLD:
                // .oflwrk is carrier
                // calculate headway
                // VERBOSITY debug
                // sprintf(utpdbgln,"UTPBOST_PAYLD sees psh %08X pll %08X\n",
                //                  (utpptr->utphdr.a2pcbpsh),(utpptr->utphdr.a2pcbpll));
                // utp_dbgoutput(utpdbgln);
                // space=msk-((push-pull)&msk) -- though we always leave some headway
                ofil=(ofil_carrier*)(utpptr->oflwrk);
                val=((utpptr->utphdr.a2pcbpsh)-(utpptr->utphdr.a2pcbpll))&UTP_MAX_BLK_MSK;
                val=UTP_MAX_BLK_MSK-val;
                if (val>UTP_MAX_BLK_BND) {
                    val=UTP_MAX_BLK_BND;
                }
                // implicitly utpptr->oflsiz is non-0 here
                while ((!done) && (!error)) {
                    // if we get stuck, done gets set and we return here next pass
                    if (val) {
                        // there's at least 1 req carrier available
                        // try to extract the data
                        val=val-1;
                        error=utp_ofil_body(ofil,&done);
                        if (!error) {
                            // despite still having val!=0
                            // we may have completed the file transfer
                            if (!(utpptr->oflsiz)) {
                                done=1;
                            }
                        }
                    }
                    else {
                        // no more reqs to service this pass
                        done=1;
                    }
                }
                if (done && (!error)) {
                    // the last of the data may have gotten sent
                    if (!(utpptr->oflsiz)) {
                        // when the whole ofil_carrier is done,
                        // we're done with the working ofil
                        ofile_release(ofil);
                        utpptr->oflwrk=(void*)0;
                        utpptr->bost=UTPBOST_IDLE;
                    }
                    // else we'll just continue next pass
                }
                break;
            default:
                // unknown state - try to repair it
                if (utpptr->oflwrk) {
                    ofil=(ofil_carrier*)(utpptr->oflwrk);
                    ofile_release(ofil);
                    utpptr->oflwrk=(void*)0;
                }
                utpptr->bost=UTPBOST_IDLE;
                error=1;
                break;
        }
    }
    return(error);
}

//------------------------------------------------------------------------------

u32 utp_ofile_FSM(void) {
    // signalling ofile messages on deck
    //  only gets called when UTP is "up" and not reset
    u32 st;
    u32 error;
    u32 done;
    error=0;
    done=0;
    while ((!done) && (!error)) {
        st=(u32)(utpptr->oflst);
        switch (st) {
            case OFILST_IDLE:
                if (!((utpptr->osig)&PFL_OSIG_OFILE)) {
                    // output signal already commanded low,
                    //  proceed to next state, not done
                    utpptr->oflst=OFILST_IECHO;
                }
                else {
                    // output signal not commanded low yet,
                    //  do it now, and update sig_a2pc that is already on deck
                    utpptr->osig=(utpptr->osig)&(~PFL_OSIG_OFILE);
                    utpptr->utphdr.sig_a2pc=(utpptr->utphdr.sig_a2pc)&(~PFL_OSIG_OFILE);
                    // need another status cycle to get that out
                    //  and have the chance to echo...
                    //   so advance state and we're done this pass
                    utpptr->oflst=OFILST_IECHO;
                    done=1;
                }
                break;
            case OFILST_IECHO:
                // waiting on echo of our output to show OFIL signal low
                if ((utpptr->eosig)&PFL_OSIG_OFILE) {
                    // echo signal still high, try again next pass
                    done=1;
                }
                else {
                    // echo signal is showing low
                    //  advance state, not done yet
                    utpptr->oflst=OFILST_WTODCK;
                }
                break;
            case OFILST_WTODCK:
                // don't raise the OFIL output signal until there is
                //  an output console line ready to send
                if (utpptr->oflnum) {
                    // confirmed that an ofile output is ready
                    //  it can't get deployed unitl polled for by uaiox command
                    //   but PC side should see a rising OFIL signal now
                    utpptr->osig=(utpptr->osig)|PFL_OSIG_OFILE;
                    utpptr->utphdr.sig_a2pc=(utpptr->utphdr.sig_a2pc)|PFL_OSIG_OFILE;
                    utpptr->oflst=OFILST_WTFLPU;
                    done=1;
                }
                else {
                    // try again next pass
                    done=1;
                }
                break;
            case OFILST_WTFLPU:
                // do nothing, we don't return to OFILST_IDLE
                //  until we get a uuiox message to pick up an
                //   output console file
                //    (or the queue gets flushed by utp teardown failure)
                done=1;
                break;
            default:
                // unknown state resets/realigns
                error=1;
                break;
        }
    }
    return(error);
}

//------------------------------------------------------------------------------

void ifile_release(ifil_carrier* ifil) {
    // generic release for any kind of ifil_carrier
    switch (ifil->ifltype) {
        case IFLTYP_RWTST:
            // the target is raw test buffer tst_inframe[]
            //  so the object at .refptr is really a dummy without a buffer
            //   because the buffer is already resident
            // the .refptr dummy is just a gblk set up for this purpose
            gblkfree(ifil->refptr);
            break;

            // add other types as needed

        default:
            // this shouldn't happen - crash the program
            thread_set_crash(CRASH_UTPSYS);
            break;
    }
    // when the carrier target is released, we're done with this carrier
    gblkfree((void*)ifil);
}

u32 utp_ifil_promote(ifil_carrier* ifil) {
    // returns crash error true
    u32 error;
    char* dst;
    int siz;
    error=0;
    // depending on type of ifile, prepare for data reception
    //   if it has to output to a file system file, then it may fail open()
    //    and return error
    switch (ifil->ifltype) {
        case IFLTYP_RWTST:
            // Recall from hass_test.glb
            // short tst_inframe[H_def_HEIGHT*H_def_WIDTH];
            // although we use the actual commanded size, not always the default maximum
            // hardwired, basically ignores ifil->refptr
            dst=(char*)tst_inframe;
            siz=ifil->filsize;
            // set auto-transfer controls
            utpptr->ifldst=dst;
            utpptr->iflsiz=siz;
            utpptr->bist=UTPBIST_PAYLD;
            break;

            // add other types as needed

        default:
            // this shouldn't happen - crash the program
            error=1;
            break;
    }
    return(error);
}

u32 utp_ifil_body(ifil_carrier* ifil,u32* done) {
    // returns crash error true
    // set *done if stalled or incomplete
    u32 error;
    u32 isz;
    error=0;
    // depending on type of ofile, it can tranfer output differently
    switch (ifil->ifltype) {
        case IFLTYP_RWTST:
            // seed isz with input expectation left - should be non-zero
            isz=utpptr->iflsiz;
            // only the last chunk can be short
            //  all start and middle chunks are full size...
            if (isz>UTP_MAX_BLK_XFR) {
                isz=UTP_MAX_BLK_XFR;
            }
            // then when we request reading the next on deck req,
            //  we expect isz return bytes
            error=utp_bfifo_rd((utpptr->ifldst),isz);
            if (!error) {
                // pll pointer is auto-advanced
                // post advance read indicators
                utpptr->iflsiz=(utpptr->iflsiz)-isz;
                utpptr->ifldst=(utpptr->ifldst)+isz;
            }
            break;

            // add other types as needed

        default:
            // this shouldn't happen - crash the program
            error=1;
            break;
    }
    return(error);
}

void utp_ifil_deploy(ifil_carrier* ifil,u32 status) {
    // returns crash error true
    //  this gets called whether the ifile transfer completes successfully
    //   or gets torn down due to some UTP error
    //    so it must return void as part of error cleanup (status != IFLSTATUS_OK)
    //
    // it's basically automated, keyed off the filename
    u32 tchar;
    tchar=(u32)((u8)(ifil->filname[0]));
    switch (tchar) {
        // for now, let's say ifltype IFLTYP_RWTST
        //  is always encoded filename tf.. (test frame xx)
        case 't':
            // there's nothing else to do -
            //  the test frame may have been overwritten with partial data
            //   (no corrective action)
            //    or it may have completed successfully
            //    either way - do nothing
            break;
        default:
            // we don't want to message this to oconsole... we may be terminating
            //  an ifile error due to UTP failure
            //   so send these to the driver dbg console
            sprintf(utpdbgln,"utp_ifil_deploy() has no handler for %s status %d\n",
                    (char*)(&(ifil->filname[0])),status);
            utp_dbgoutput(utpdbgln);
            break;
    }
}

u32 utp_blki_FSM(void) {
    // returns crash error true
    u32 error;
    u32 done;
    int val;
    u8 st;
    ifil_carrier* ifil;
    error=0;
    done=0;
    while ((!done) && (!error)) {
        st=utpptr->bist;
        switch (st) {
            case UTPBIST_IDLE:
                // .iflwrk is empty - objective promote an input file
                ifil=(ifil_carrier*)lnklst_fpop(&(utpptr->iflchn));
                if (ifil) {
                    // VERBOSITY debug -- you could print details as well
                    // utp_dbgoutput("UTPBIST_IDLE has ifil\n");
                    // promote this ifil to working unload
                    utpptr->iflwrk=(void*)ifil;
                    utpptr->iflnum=(utpptr->iflnum)-1;
                    error=utp_ifil_promote(ifil);
                }
                else {
                    // try again next pass
                    done=1;
                }
                break;
            case UTPBIST_PAYLD:
                // .iflwrk is carrier
                // calculate headway
                // VERBOSITY debug
                // sprintf(utpdbgln,"UTPBIST_PAYLD sees psh %08X pll %08X\n",
                //                  (utpptr->utphdr.pc2abpsh),(utpptr->utphdr.pc2abpll));
                // utp_dbgoutput(utpdbgln);
                // val is in units of number req's queue'd up and filled
                //  (push-pull)
                ifil=(ifil_carrier*)(utpptr->iflwrk);
                val=((utpptr->utphdr.pc2abpsh)-(utpptr->utphdr.pc2abpll))&UTP_MAX_BLK_MSK;
                // implicitly utpptr->iflsiz is non-0 here
                while ((!done) && (!error)) {
                    // if we get stuck, done gets set and we return here next pass
                    if (val) {
                        // there's at least 1 req carrier available
                        // try to extract the data
                        val=val-1;
                        error=utp_ifil_body(ifil,&done);
                        if (!error) {
                            // despite still having val!=0
                            // we may have completed the file transfer
                            if (!(utpptr->iflsiz)) {
                                done=1;
                            }
                        }
                    }
                    else {
                        // no more reqs to service this pass
                        done=1;
                    }
                }
                if (done && (!error)) {
                    // the last of the data may have gotten picked up
                    if (!(utpptr->iflsiz)) {
                        // when the whole ifil_carrier is done,
                        //  deploy the package
                        utp_ifil_deploy(ifil,IFLSTATUS_OK);
                        // we're done with the working ifil
                        ifile_release(ifil);
                        utpptr->iflwrk=(void*)0;
                        utpptr->bist=UTPBIST_IDLE;
                    }
                    // else we'll just continue next pass
                }
                break;
            default:
                // unknown state - try to repair it
                if (utpptr->iflwrk) {
                    ifil=(ifil_carrier*)(utpptr->iflwrk);
                    ifile_release(ifil);
                    utpptr->iflwrk=(void*)0;
                }
                utpptr->bist=UTPBIST_IDLE;
                error=1;
                break;
        }
    }
    return(error);
}

//------------------------------------------------------------------------------

// Return current time in milliseconds as i64
int64_t now_ms_i64(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return ((((int64_t)(now.tv_sec))*1000000000LL + (now.tv_nsec))/1000000);
}

//------------------------------------------------------------------------------

int msg_hass_tmeasr(void* ioptr) {
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    //  only calling arg is number of frames at arg0
    //
    u32 N;
    hass_tmracc* tmracc;
    char* dst;
    N=(u32)(*(((s64*)ioptr)+1));
    tmracc=&hasstmracc;
    // no checks if another measurement in place...
    //  if there is, the previous measurement gets whacked
    //   with the overwrite of new measurement
    tmracc->numfr=N;
    tmracc->frnum=N;
    // reset the running totals to 0
    dst=(char*)(&(tmracc->tstrt));
    memset(dst,0,(HASS_TMR_NUM<<2));
    // VERBOSITY - debug (but timing measurement should use minimal overhead)
    // LOGI("msg_hass_tmeasr() start %d frame measurement\n",N);
    // issue the first collection flag to thread3
    //  (if you set up N=0, that's a mechanism to "kill" a measurement in progress
    //    should you decide something's wrong and you no longer want to wait on
    //     the results)
    // you just pass N to thread 3, if it fails, it just won't do the measurement
    // error=set_hass_tmren(N);
    set_hass_tmren(N);
    // default return - auto-destroy the carrier
    return(0);
}

int set_hass_tmeasr(u32 Nfrms) {
    //
    // returns error true, 0 OK
    //
    // start a measurement cycle over N frames
    //  thread0 will issue a report of some sort when done
    //
    msg_queue* queue;
    int retval;
    // the message gets passed to MU_thread0
    queue=&((thr_ctrl)->mqueue);
    retval=msgexec_gblkvoid(queue,msg_hass_tmeasr,((s64)Nfrms),0,0);
    return(retval);
}

int msg_hass_tmeasu(void* ioptr) {
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    //  implicit calling arg is msg the carrier
    //
    hass_timer* tmrptr;
    hass_tmracc* tmracc;
    tmrptr=(hass_timer*)(((char*)ioptr)-0x18);
    tmracc=&hasstmracc;
    if (tmracc->numfr) {
        // a measurement is in effect
        if (tmracc->frnum) {
            // VERBOSITY - debug (but timing measurement should use minimal overhead)
            // LOGI("msg_hass_tmeasu() at frame %d\n",tmracc->frnum);
            // and we're still collecting
            // accumlate results
            // strt is unique
            if ((tmracc->frnum)==(tmracc->numfr)) {
                // only the first update records overall tstrt
                tmracc->tstrt=(tmrptr->tstrt);
            }
            // end is unique
            tmracc->frnum=(tmracc->frnum)-1;
            if (!(tmracc->frnum)) {
                // only the last update replaces overall .tstrt as total time
                tmracc->tstrt=(tmrptr->tout)-(tmracc->tstrt);
            }
            // others just accumulate
            tmracc->tcoeptr=(tmracc->tcoeptr)+((tmrptr->tcoeptr)-(tmrptr->tstrt));
            tmracc->tguide=(tmracc->tguide)+((tmrptr->tguide)-(tmrptr->tcoeptr));
            tmracc->tshrink=(tmracc->tshrink)+((tmrptr->tshrink)-(tmrptr->tguide));
            tmracc->tNR=(tmracc->tNR)+((tmrptr->tNR)-(tmrptr->tshrink));
            tmracc->trecon=(tmracc->trecon)+((tmrptr->trecon)-(tmrptr->tNR));
            tmracc->tapp=(tmracc->tapp)+((tmrptr->tapp)-(tmrptr->trecon));
            tmracc->tcomb=(tmracc->tcomb)+((tmrptr->tcomb)-(tmrptr->tapp));
            tmracc->tproc=(tmracc->tproc)+((tmrptr->tproc)-(tmrptr->tcomb));
            tmracc->tout=(tmracc->tout)+((tmrptr->tout)-(tmrptr->tproc));
            //
            // generate a report ... optional
            if (!(tmracc->frnum)) {
                hass_tmr_report(tmracc);
            }
        }
        // else {
        // redundant report - maybe measurement was cancelled...
        //  basically do nothing
    }
    // else {
    // redundant report - maybe measurement was cancelled...
    //  basically do nothing
    // return handoff true - don't destroy carrier
    return(1);
}

//------------------------------------------------------------------------------


// if the issuer is awaiting the response
//  retptr points at a pair of integers {done,retval} in that order
//   then the issuer initializes done=0
//    and at thread0, when java side provides .ret
//     that gets transferred to retval, and done gets set

int j2j_xchg_submit(void* ioptr) {
    jni2java_ctrl* j2j;
    j2j_xchg_spec* carr;
    // always handoff
    int retval=1;
    // point back to the carrier lnklst
    carr=(j2j_xchg_spec*)(((char*)ioptr)-0x18);
    // and attach it at the j2j chain
    j2j=&(ACptr->j2jctrl);
    lnklst_fins((lnklst*)j2j,(lnklst*)carr);
    return(retval);
}

void init_j2j_ctrl(jni2java_ctrl* j2j) {
    // initial setup (all contents should be 0)
    lnklst_init((lnklst*)j2j);
}

void j2j_xchg_void(j2j_xchg_spec* j2jspec) {
    // automatic destructor for carrier
    msg_queue* queue;
    // message it to thread0
    queue=&(thr_ctrl->mqueue);
    j2jspec->fncptr=j2j_xchg_submit;
    // cmd and p0..p6 should already be set
    // not expecting retval - this is what makes it void
    j2jspec->retptr=(int*)0;
    msgqueue_submit(queue,(msg_gblkfnc*)j2jspec);
}

int j2j_xchg_wrsp(j2j_xchg_spec* j2jspec) {
    // automatic destructor for carrier
    int retval;
    volatile int reply[2];
    msg_queue* queue;
    // message it to thread0
    queue=&(thr_ctrl->mqueue);
    j2jspec->fncptr=j2j_xchg_submit;
    // cmd and p0..p6 should already be set
    // expecting retval - caller is responsible to set retptr
    j2jspec->retptr=(int*)(&(reply[0]));
    // must set "done" to 0
    reply[0]=0;
    msgqueue_submit(queue,(msg_gblkfnc*)j2jspec);
    // now we spin, waiting on response
    //  not ideal, but functional
    //   (as part of an FSM, it could exit and poll again later)
    while (!reply[0]) {
        sched_yield();
    }
    // when thread 0 sets "done"
    //  then "retval" should be valid
    retval=(int)(reply[1]);
    return(retval);
}

void j2j_FSM(jni2java_ctrl* j2j) {
    j2j_xchg_spec* j2jspec;
    int* rptr;
    int st;
    int done=0;
    while (!done) {
        // next action based on state
        st=j2j->j2jst;
        switch (st) {
            case J2JST_IDLE:
                // check for something in the queue
                j2jspec=(j2j_xchg_spec*)lnklst_fpop(&(j2j->chn));
                if (j2jspec) {
                    // this becomes the working command
                    j2j->wrk=j2jspec;
                    // we don't really need to copy over the parameters
                    //  and could just leave them in the .wrk carrier
                    //   but for diagnostics and debug visibility
                    //    the stuff gets transferred to the jni2java_ctrl so
                    //     for monitoring we can consitently read them there
                    //      and not dynamically figure out where they are
                    j2j->parm[0]=j2jspec->p0;
                    j2j->parm[1]=j2jspec->p1;
                    j2j->parm[2]=j2jspec->p2;
                    j2j->parm[3]=j2jspec->p3;
                    j2j->parm[4]=j2jspec->p4;
                    j2j->parm[5]=j2jspec->p5;
                    j2j->parm[6]=j2jspec->p6;
                    // then set the (non-0) cmd
                    j2j->cmd=j2jspec->cmnd;
                    // VERBOSITY - debug
                    // LOGI("J2JST_IDLE next cmd %08X\n",j2j->cmd);
                    // LOGI("  args %08X %08X %08X %08X %08X %08X %08X\n",
                    //          j2j->parm[0],j2j->parm[1],j2j->parm[2],j2j->parm[3],j2j->parm[4],j2j->parm[5],j2j->parm[6]);
                    // LOGI("  rptr %08X\n",(((s64)(j2jspec->retptr))&0xFFFFFFFF));
                    // and advance state
                    j2j->j2jst=J2JST_WAIT;
                }
                else {
                    // nothing to do this pass
                    done=1;
                }
                break;
            case J2JST_WAIT:
                // recall target transaction
                j2jspec=j2j->wrk;
                // check for java side response
                //  when .cmd resets to 0, command has been executed by Java side
                //   and potentially there is return value at .ret
                if (!(j2j->cmd)) {
                    // Java side completed command
                    // VEBOSITY - debug
                    // LOGI("J2JST_WAIT detected command serviced\n");
                    // is there a retptr?
                    rptr=j2jspec->retptr;
                    if (rptr) {
                        // issuer of carrier expects to see return value
                        // VERBOSITY - debug
                        // LOGI("J2JST_WAIT set retval %08X at rptr\n",j2j->ret);
                        // set retval first
                        *(rptr+1)=j2j->ret;
                        // mark done
                        *rptr=1;
                    }
                    // else no deployment of result necessary
                    // dispose of carrier and reset .wrk to 0
                    gblkfree((void*)(j2j->wrk));
                    j2j->wrk=(j2j_xchg_spec*)0;
                    // reset state
                    j2j->j2jst=J2JST_IDLE;
                }
                else {
                    // nothing to do this pass
                    done=1;
                }
                break;
            default:
                // shouldn't happen, but can't really fix it - crash?
                //  opt to quietly reset the state machine after error warning
                LOGI("j2j_FSM() in unknown state %d\n",st);
                // quiet reset
                j2j->ret=0;
                j2j->cmd=0;
                j2j->wrk=(j2j_xchg_spec*)0;
                j2j->j2jst=J2JST_IDLE;
                break;
        }
    }
}

//------------------------------------------------------------------------------

void j2j_set_imgvw(int viewsel) {
    //
    // selects between texture and ImgVw (bitmap) display
    //   viewsel = 0   => TextureView
    //             1+  => ImageView
    //
    j2j_xchg_spec* j2jspec;
    // need a mesage carrier
    j2jspec=(j2j_xchg_spec*)gblkalloc();
    j2jspec->cmnd=J2JCMD_IMGVW;
    // one calling arg
    j2jspec->p0=viewsel;
    // void call
    j2j_xchg_void(j2jspec);
}

//------------------------------------------------------------------------------

void setUSBAccssry(void) {
    int len;
    len=__system_property_get(kstr_sysusb,sysusb_now);
    if (strcmp(kstr_accessory,sysusb_now)) {
        // if needed, set accessory
        len=__system_property_set(kstr_sysusb,kstr_accessory);
    }
    // else - do nothing, mode is already accessory
}

void setUSBAdb(void) {
    int len;
    len=__system_property_get(kstr_sysusb,sysusb_now);
    if (strcmp(kstr_adb,sysusb_now)) {
        // if needed, set accessory
        len=__system_property_set(kstr_sysusb,kstr_adb);
    }
    // else - do nothing, mode is already adb
}

void setCameraOptimizationOff() {
    __system_property_set(kstr_camera_optimization,kstr_zero);
}

//------------------------------------------------------------------------------

int j2j_get_FTDI_uart(void) {
    // no calling args - FTDI channel allocation is implicit
    // returns port number assigned, which is (-1) on failure
    //                                          0 or higher on success
    int retval = -1;
//    j2j_xchg_spec* j2jspec;
//    // need a mesage carrier
//    j2jspec=(j2j_xchg_spec*)gblkalloc();
//    j2jspec->cmnd=J2JCMD_FTUART;
//    // no calling args need to be set up
//    //  j2jspec-> p0..p6 are don't care
//    // call, waiting on response
//    retval=j2j_xchg_wrsp(j2jspec);

    return(retval);
}
//------------------------------------------------------------------------------

u32 open_utp(void) {
    // returns error true reported
    u32 retval;
    if ((utpptr->fd)==(-1)) {
        // when device was never opened, we're still trying to open it
        //  for the first time - then do a real "open"
        utpptr->fd=open(utpdvcname,(O_RDWR | O_DSYNC | O_SYNC));
        if ((utpptr->fd)!=(-1)) {
            // device opened
            // do whatever utp inits are necessary
            //  must have SIGRES=1 SIG_HB=0
            utpptr->osig=PFL_SIGRES|PFL_OSIG_DEF;
            utpptr->isig=0;
            utpptr->eosig=0;
            utpptr->zisig=0;
            // force USB accessory mode
            //setUSBAccssry();
            // return success
            retval=0;
        }
        else {
            // do whatever utp shutdown activities are necessary
            // return error true
            retval=1;
        }
    }
    else {
        // when the device has been opened initially, it really stays open forever
        //  and we can keep using the non-USB interfaces (debug console access)
        //   then it's already open and conceptually the open action has succeeded
        //    but then we stioll should do the USB related open inits
        //  must have SIGRES=1 SIG_HB=0
        utpptr->osig=PFL_SIGRES|PFL_OSIG_DEF;
        utpptr->isig=0;
        utpptr->eosig=0;
        utpptr->zisig=0;

        // return success
        retval=0;
    }
   return(retval);
}

void close_utp(void) {
    // device may not even be open
    //   this just insures it's closed whether it needs it or not
    if ((utpptr->fd)!=(-1)) {
        close ((utpptr->fd));
        utpptr->fd=(-1);
    }
}

//------------------------------------------------------------------------------

u32 utp_trafblk(void) {
    // returns error true if traffic is blocked
    u32 error;
    error=1;
    // we have to be in exchange state with neither side in reset
    //  to allow traffic data exchanges
    if ( ((utpptr->utpstate)==UTPST_XCHG) &&
         (!((*((u32*)(&(utpptr->osig))))&PFL_SIGRES_IO)) ) {
        error=0;
    }
    return(error);
}

void utp_dbgoutput(char* src) {
    // we can't deal with error contingencies... this has to output a useful debug messgae
    //  to the console/debug terminal, or drop it on the floor without complaints
    //
    int siz;
    // the device has to be on line
    if ((utpptr->fd)!=(-1)) {
        // the device is available to "print"
        siz=strlen(src)+1;
        utp_frc_write(utpptr->fd,(void*)src,siz,
                      (PFL_HDR_SIZE+((PFL_BODY_SIZE+PFL_BULK_SIZE)<<1)) );
    }
}

//------------------------------------------------------------------------------

void utp_shutdown(void) {
    // utp cleanup on program exit
    if ((utpptr->fd)!=(-1)) {
        // if we're exiting program with UTP open
        // set .osig with RES bit
        utpptr->osig=PFL_SIGRES|PFL_OSIG_DEF;
        // dummy (reset) default all other output signals
        utp_reset_ostatus();
        // do a void status update
        // error=utp_putstatus();
        utp_putstatus();
        // and close the device
        close_utp();
    }
}

void utp_startup(void) {
    // the file descriptor must be invalid
    utpptr->fd=(-1);
    // must start in lowest level gound state
    utpptr->utpstate=UTPST_DOWN;
    // ground state on command processing
    utpptr->cm_state=CMDST_HEADER;
    // the rest is run time adaptive/reactive
    //
    // init diagnostic counters
    utpptr->gstcnt=0;
    utpptr->pstcnt=0;
}

u32 utp_getstatus(void) {
    // return error true
    //  otherwise do a UTP status read cycle
    u32 val;
    u32 error;
    error=0;
    // ONLY for debugging - the counter is reserved for that purpose, but really unused
    // utpptr->gstcnt=(utpptr->gstcnt)+1;
    // get a copy of the current UTP header
    //  host may have gone offline and come to life
    //   don't assume anything about this file is static
    utp_frc_read(utpptr->fd,(void*)(&(utpptr->utphdr)),PFL_HDR_SIZE,0);
    // read appears succesful
    // reformat input signals
    val=(u32)(utpptr->utphdr.sig_pc2a);
    utpptr->zisig=utpptr->isig;
    utpptr->isig=(u16)val;
    utpptr->eosig=(u16)(val>>16);
    return(error);
}

u32 utp_putstatus(void) {
    // return error true
    //  otherwise do a UTP status write cycle
    //   this requiress, device fd is valid, and
    //    .sig_a2pv through .pc2abpll have all been set up
    //     start offset 0x20, and write 0x14 bytes
    u32 error;
    error=0;
    // ONLY for debugging - the counter is reserved for that purpose, but really unused
    // utpptr->pstcnt=(utpptr->pstcnt)+1;
    utp_frc_write(utpptr->fd,(void*)(&(utpptr->utphdr.sig_a2pc)),0x14,0x20);
    return(error);
}

void utp_reset_ostatus(void) {
    // return error true
    //  before getting to UTPST_XCHG, the only information output to the device
    //   is .osig SIGRES (=1) and SIG_HB (1/0)
    // so basically, once .osig is set up, and known to signal a reset from this side
    //  all the push/pull pointers sourced by this side are fixed 0's
    // .sig_a2pc carries .osig in the low bits,
    //  and echo back .isig in the high bits
    utpptr->utphdr.sig_a2pc=(((int)(utpptr->isig))<<16)|((int)(utpptr->osig));
    // all push/pull pointers get cleared
    utpptr->utphdr.a2pc_psh=0;
    utpptr->utphdr.pc2a_pll=0;
    utpptr->utphdr.a2pcbpsh=0;
    utpptr->utphdr.pc2abpll=0;
}

u32 utp_first_status(void) {
    // return error true
    u32 error;
    // we just opened the device...
    // first read of status
    error=utp_getstatus();
    if (!error) {
        // then it's known .osig is set to command SIGRES=1, SIG_HB=0
        //  so just loop back .isig, and set push/pointers to 0
        utp_reset_ostatus();
        error=utp_putstatus();
    }
    // else error set at utp_getstatus()
    return(error);
}

//------------------------------------------------------------------------------

u32 utp_pfifo_rd(char* dst,int num) {
    // return error true
    //  otherwise do a command fifo read, num is !0
    int msk;
    u32 error;
    error=0;
    // set device fpos to command space
    utp_frc_read(utpptr->fd,(void*)dst,num,PFL_HDR_SIZE);
    // reads auto-advance pull
    msk=(utpptr->utphdr.ctrlsize)-1;
    utpptr->utphdr.pc2a_pll=((utpptr->utphdr.pc2a_pll)+num)&msk;
    return(error);
}

u32 utp_pfifo_wr(char* src,int num) {
    // return error true
    //  otherwise do a command fifo write, num is !0
    int msk;
    u32 error=0;
    utp_frc_write(utpptr->fd,(void*)src,num,PFL_HDR_SIZE);
    // reads auto-advance push
    msk=(utpptr->utphdr.ctrlsize)-1;
    utpptr->utphdr.a2pc_psh=((utpptr->utphdr.a2pc_psh)+num)&msk;
    return(error);
}

u32 utp_bfifo_rd(char* dst,int num) {
    // return error true
    //  otherwise do a bulk fifo read, num is !0
    u32 error;
    int retN;
    error=0;
    retN=(int)utp_blk_read(utpptr->fd,(void*)dst,num,(PFL_HDR_SIZE+(PFL_BODY_SIZE<<1)));
    if (retN!=num) {
        error=1;
    }
    // reads auto-advance pull (success or fail)
    utpptr->utphdr.pc2abpll=((utpptr->utphdr.pc2abpll)+1)&UTP_MAX_BLK_MSK;
    return(error);
}

u32 utp_bfifo_wr(char* src,int num) {
    // return error true
    //  otherwise do a bulk fifo write, num is !0
    u32 error;
    int retN;
    error=0;
    retN=(int)utp_blk_write(utpptr->fd,(void*)src,num,(PFL_HDR_SIZE+(PFL_BODY_SIZE<<1)));
    if (retN!=num) {
        error=1;
    }
    // writes auto-advance psh
    utpptr->utphdr.a2pcbpsh=((utpptr->utphdr.a2pcbpsh)+1)&UTP_MAX_BLK_MSK;
    return(error);
}

//------------------------------------------------------------------------------

char* uaiox_lmembase(u32 lmempg) {
    // returns 0 pointer on error, otherwise remaps logical memory page to
    //  to memory pointer
    char* retval=(char*)0;
    switch (lmempg) {
        case 0:
            // page 0 sees ACptr (all of it)
            retval=(char*)(shrMEM_ptr);
            break;
        default:
            // returns invalid 0 pointer
            break;
    }
    return(retval);
}

//------------------------------------------------------------------------------

void uuiox_wrbyte(void) {
    char* src;
    char* dst;
    u32 n;
    src=((char*)uuioxbuff)+4;
    dst=(char*)(utpptr->datptr);
    n=(u32)(utpptr->dlen);
    while (n) {
        *dst=*src;
        dst++;
        src++;
        n--;
    }
    // set response payload length
    *((u16*)(&(utpptr->uhdr[2])))=0;
    // ready to deploy
    utpptr->cm_state=CMDST_REPLY;
}

void uuiox_wrword(void) {
    u16* src;
    u16* dst;
    u32 n;
    src=(u16*)(((char*)uuioxbuff)+4);
    dst=(u16*)(utpptr->datptr);
    n=((u32)(utpptr->dlen))>>1;
    while (n) {
        *dst=*src;
        dst++;
        src++;
        n--;
    }
    // set response payload length
    *((u16*)(&(utpptr->uhdr[2])))=0;
    // ready to deploy
    utpptr->cm_state=CMDST_REPLY;
}

void uuiox_wrquad(void) {
    u32* src;
    u32* dst;
    u32 n;
    src=(u32*)(((char*)uuioxbuff)+4);
    dst=(u32*)(utpptr->datptr);
    n=((u32)(utpptr->dlen))>>2;
    while (n) {
        *dst=*src;
        dst++;
        src++;
        n--;
    }
    // set response payload length
    *((u16*)(&(utpptr->uhdr[2])))=0;
    // ready to deploy
    utpptr->cm_state=CMDST_REPLY;
}

void uuiox_rdbyte(void) {
    char* src;
    char* dst;
    u32 n;
    u32 s;
    dst=(char*)uuioxbuff;
    src=(char*)(utpptr->datptr);
    n=(u32)(utpptr->dlen);
    s=n;
    while (s) {
        *dst=*src;
        dst++;
        src++;
        s--;
    }
    // set response payload length
    *((u16*)(&(utpptr->uhdr[2])))=(u16)n;
    // ready to deploy
    utpptr->cm_state=CMDST_REPLY;
}

void uuiox_rdword(void) {
    u16* src;
    u16* dst;
    u32 n;
    u32 s;
    dst=(u16*)uuioxbuff;
    src=(u16*)(utpptr->datptr);
    n=(u32)(utpptr->dlen);
    s=n>>1;
    while (s) {
        *dst=*src;
        dst++;
        src++;
        s--;
    }
    // set response payload length
    *((u16*)(&(utpptr->uhdr[2])))=(u16)n;
    // ready to deploy
    utpptr->cm_state=CMDST_REPLY;
}

void uuiox_rdquad(void) {
    u32* src;
    u32* dst;
    u32 n;
    u32 s;
    dst=(u32*)uuioxbuff;
    src=(u32*)(utpptr->datptr);
    n=(u32)(utpptr->dlen);
    s=n>>2;
    while (s) {
        *dst=*src;
        dst++;
        src++;
        s--;
    }
    // set response payload length
    *((u16*)(&(utpptr->uhdr[2])))=(u16)n;
    // ready to deploy
    utpptr->cm_state=CMDST_REPLY;
}

//------------------------------------------------------------------------------

void uuiox_wrldvc(void) {
    char* src;
    u32 n;
    u32 dvc;
    u32 dva;
    src=((char*)uuioxbuff)+4;
    n=(u32)(utpptr->dlen);
    dvc=(u32)(utpptr->ldvc);
    dva=utpptr->ldva;
    //
    // set response payload length - default (-1), waiting to get filled in
    *((u16*)(&(utpptr->uhdr[2])))=0xFFFF;
    // await response
    utpptr->cm_state=CMDST_EXEDONE;
    //
    // given this information, the general approach is to deplay a message
    //  to the appropriate device driver to execute the write
    //   (pfctrl may be included in the argument list)
    //
    // the device driver should one of the follwing, either directly
    //  or by messaging back to the sender thread
    //
    // return OK
    //    -- set u16 at utpptr->uhdr[2] = 0x0000
    //
    // return ERRNO
    //    -- set quad at utpptr->uuioxbuff = ERRNO
    //    -- set u16 at utpptr->uhdr[2] = 0x0004
    //
    // so here deploy what we know to logical device handler
    switch (dvc) {

        // QQQQQ
        // known logical device dispatches go here ...

        default:
            // VERBOSE - issue console message (optional)
            ucnsl_ldvc_fl_acc(
                    (u32)(utpptr->uhdr[0]),    // cmdnum
                    (u32)(utpptr->ldvc),       // dvc
                    (u32)(utpptr->ldva));      // addr
            // unknown device must fail - send ERRNO
            *((u32*)uuioxbuff)=UAERR_LDVC_UNKN;
            *((u16*)(&(utpptr->uhdr[2])))=4;
            // so from the CMDST_EXEDONE service -- this is already done
            break;
    }
}

void uuiox_rdldvc(void) {
    char* dst;
    u32 n;
    u32 dvc;
    u32 dva;
    dst=(char*)uuioxbuff;
    n=(u32)(utpptr->dlen);
    dvc=(u32)(utpptr->ldvc);
    dva=utpptr->ldva;
    //
    // set response payload length - default (-1), waiting to get filled in
    *((u16*)(&(utpptr->uhdr[2])))=0xFFFF;
    // await response
    utpptr->cm_state=CMDST_EXEDONE;
    //
    // given this information, the general approach is to deplay a message
    //  to the appropriate device driver to execute the write
    //   (pfctrl may be included in the argument list)
    //
    // the device driver should one of the follwing, either directly
    //  or by messaging back to the sender thread
    //
    // return ERRNO
    //    -- set quad at dst==uuioxbuff = ERRNO
    //    -- set u16 at utpptr->uhdr[2] = 0x0004
    //
    // return all read data
    //    -- set quad at dst==uuioxbuff = 0x00000000 (OK) followed by n bytes
    //    -- set u16 at utpptr->uhdr[2] = (4+n)
    //
    // so here deploy what we know to logical device handler
    switch (dvc) {

        // QQQQQ
        // known logical device dispatches go here ...

        default:
            // VERBOSE - issue console message (optional) - obsolete implementation
            ucnsl_ldvc_fl_acc(
                    (u32)(utpptr->uhdr[0]),    // cmdnum
                    (u32)(utpptr->ldvc),       // dvc
                    (u32)(utpptr->ldva));      // addr
            // unknown device must fail - send ERRNO
            *((u32*)uuioxbuff)=UAERR_LDVC_UNKN;
            *((u16*)(&(utpptr->uhdr[2])))=4;
            // so from the CMDST_EXEDONE service -- this is already done
            break;
    }
}

//------------------------------------------------------------------------------

void utpcnsl_flush(void) {
    // quietly destroy all undelivered console messages
    lnklst* carr;
    u32 done;
    done=0;
    while (!done) {
        carr=lnklst_fpop(&(utpptr->ocnslchn));
        if (carr) {
            appfree((void*)carr);
        }
        else {
            // when all outstanding messages destroyed
            //  reset count and state
            utpptr->ocnslnum=0;
            utpptr->ocnslst=0;   // CNSLST_IDLE
            done=1;
        }
    }
}

int send_utpcnsl(void* ioptr) {
    lnklst* carr;
    int handoff=1;
    // msg service doesn't destroy this carrier,
    //  always return handoff==true
    //   enter with ioptr pointing at dstptr
    //
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    //               - arg0 <unused... utp delivery implicit>
    // point back to the carrier lnklst
    carr=(lnklst*)(((char*)ioptr)-0x18);
    // this gets chained into the ocnslchn of the utpptr
    //  if it's still on line and not backlogged
    if ( ((utpptr->utpstate)==UTPST_XCHG) &&
         (!((*((u32*)(&(utpptr->osig))))&PFL_SIGRES_IO)) &&
         ((utpptr->ocnslnum)<CNSL_BACKLOG) ) {
        // submit the console line for output
        lnklst_fins((&(utpptr->ocnslchn)),carr);
        // raise the number of console lines
        utpptr->ocnslnum=(utpptr->ocnslnum)+1;
        // from here the ocnsl state machine should
        //  release the message when polled
    }
    else {
        // this utp is no longer accepting console lines
        //  then drop it here and now
        appfree((void*)carr);
    }
    return(handoff);
}

//------------------------------------------------------------------------------
// any thread can commit a console output line
//  the function and format strings are mutually defined pairs

char kstr_ucnsl_tstfk[]="CNSL: unknown fk=0x%04X called with arglo=0x%08X arghi=0x%08X";

void ucnsl_fl_fnckey(u32 fnum,u32 arglo,u32 arghi) {
    // check if UTP is up and not reset
    msg_queue* queue;
    cnsl_carrier* cptr;
    int len;
    if (utpptr->utpstate==UTPST_XCHG) {
        // link is up on this channel
        //  fast bypass if channel reset
        if (!((*((u32*)(&(utpptr->osig))))&PFL_SIGRES_IO)) {
            // then snag a carrier
            cptr=(cnsl_carrier*)appmalloc(CNSL_CARR_SIZE);
            if (cptr) {
                // sprintf with all the same args...
                len=sprintf((char*)(&(cptr->data)),kstr_ucnsl_tstfk,fnum,arglo,arghi);
                // the stored length includes the asciiz
                cptr->lenz=(len+1);
                // then just send the carrier to utpptr
                //  that's always managed at thread4
                cptr->fncptr=send_utpcnsl;
                // dstptr don't care
                // arg0 is unused
                // arg1,arg2 are don't care
                // doneptr unused for that purpose
                // thread4 always manages pfN interfaces
                queue=&((thr_ctrl+4)->mqueue);
                msgqueue_submit(queue,(msg_gblkfnc*)cptr);
            }
        }
        // else unissued console messages get dropped
    }
    // else unissued console messages get dropped
}

char kstr_ucnsl_flldvc[]="CNSL: cmd %d can't access ldvc=0x%02X address=0x%08X";

void ucnsl_ldvc_fl_acc(u32 cmdnum,u32 dvc,u32 addr) {
    msg_queue* queue;
    int len;
    cnsl_carrier* cptr;
    // check if the pfN is up and not reset
    if (utpptr->utpstate==UTPST_XCHG) {
        // link is up on this channel
        //  fast bypass if channel reset
        if (!((*((u32*)(&(utpptr->osig))))&PFL_SIGRES_IO)) {
            // then snag a carrier
            cptr=(cnsl_carrier*)appmalloc(CNSL_CARR_SIZE);
            if (cptr) {
                // sprintf with all the same args...
                len=sprintf((char*)(&(cptr->data)),kstr_ucnsl_flldvc,cmdnum,dvc,addr);
                // the stored length includes the asciiz
                cptr->lenz=(len+1);
                // then just send the carrier to pfctrl
                //  that's always managed at thread4
                cptr->fncptr=send_utpcnsl;
                // dstptr don't care
                // arg0 is unused
                // arg1,arg2 are don't care
                // doneptr unused for that purpose
                // thread4 always manages pfN interfaces
                queue=&((thr_ctrl+4)->mqueue);
                msgqueue_submit(queue,(msg_gblkfnc*)cptr);
            }
        }
        // else unissued console messages get dropped
    }
    // else unissued console messages get dropped
}

//------------------------------------------------------------------------------

void utpofile_flush(void) {
    // quietly destroy all undelivered output files messages
    lnklst* ofil;
    u32 done;
    // release working output file if there is one
    if (utpptr->oflwrk) {
        ofile_release((ofil_carrier*)(utpptr->oflwrk));
        utpptr->oflwrk=(void*)0;
    }
    done=0;
    while (!done) {
        ofil=lnklst_fpop(&(utpptr->bochn));
        if (ofil) {
            ofile_release((ofil_carrier*)ofil);
        }
        else {
            // when all outstanding messages destroyed
            //  reset bulk output state
            utpptr->bost=UTPBOST_IDLE;
            done=1;
        }
    }
    done=0;
    while (!done) {
        ofil=lnklst_fpop(&(utpptr->oflchn));
        if (ofil) {
            ofile_release((ofil_carrier*)ofil);
        }
        else {
            // when all outstanding messages destroyed
            //  reset count and state
            utpptr->oflnum=0;
            utpptr->oflst=0;     // OFILST_IDLE
            done=1;
        }
    }
}

int msg_snd_uofile(void* ioptr) {
    lnklst* ofil;
    int handoff=1;
    // msg service doesn't destroy this carrier,
    //  always return handoff==true
    //   enter with ioptr pointing at dstptr
    //
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    //               - arg0 <unused>
    //
    // point back to the carrier lnklst
    ofil=(lnklst*)(((char*)ioptr)-0x18);
    // this gets chained into the oflchn of the utpptr
    //  if it's still on line and not backlogged
    if ( ((utpptr->utpstate)==UTPST_XCHG) &&
         (!((*((u32*)(&(utpptr->osig))))&PFL_SIGRES_IO)) &&
         ((utpptr->oflnum)<OFIL_BACKLOG) ) {
        // submit the file for output
        lnklst_fins((&(utpptr->oflchn)),ofil);
        // raise the number of output files
        utpptr->oflnum=(utpptr->oflnum)+1;
        // from here the ofile state machine should
        //  release the file when properly polled
    }
    else {
        // this utpptr is no longer accepting output files
        //  then drop it here and now
        // VERBOSE
        //  we could LOGI a message
        ofile_release((ofil_carrier*)ofil);
    }
    return(handoff);
}

void send_utpofile(ofil_carrier* ofil) {
    // returns 1 on crash abort, else 0
    //  normal submit of utp ofile (from any thread)
    //   ofil has been comepletely set up
    //    there are no checks that utp traffic is flowing here
    //     assumes caller did that check before going to the
    //      trouble of creating an ofil_carrier
    msg_queue* queue;
    // the message gets passed to MU_thread4
    queue=&((thr_ctrl+4)->mqueue);
    msgexec_gblkv_carr(queue,msg_snd_uofile,(msg_gblkfnc*)ofil);
}

//------------------------------------------------------------------------------

void utpifile_flush(void) {
    // quietly destroy all undelivered input files messages
    lnklst* ifil;
    u32 done;
    // release working input file if there is one
    if (utpptr->iflwrk) {
        // clean up err shutdown of ifil collection
        utp_ifil_deploy((ifil_carrier*)(utpptr->iflwrk),IFLSTATUS_ERR);
        // release it
        ifile_release((ifil_carrier*)(utpptr->iflwrk));
        utpptr->iflwrk=(void*)0;
    }
    done=0;
    while (!done) {
        ifil=lnklst_fpop(&(utpptr->iflchn));
        if (ifil) {
            utp_ifil_deploy((ifil_carrier*)ifil,IFLSTATUS_ERR);
            ifile_release((ifil_carrier*)ifil);
        }
        else {
            // when all outstanding messages destroyed
            //  reset bulk input state
            utpptr->iflnum=0;
            utpptr->bist=UTPBIST_IDLE;
            done=1;
        }
    }
}

int msg_snd_uifile(void* ioptr) {
    lnklst* ifil;
    int handoff=1;
    // msg service doesn't destroy this carrier,
    //  always return handoff==true
    //   enter with ioptr pointing at dstptr
    //
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    //               - arg0 <unused>
    //
    // point back to the carrier lnklst
    ifil=(lnklst*)(((char*)ioptr)-0x18);
    // this gets chained into the iflchn of the utpptr
    //  if it's still on line and not backlogged
    if ( ((utpptr->utpstate)==UTPST_XCHG) &&
         (!((*((u32*)(&(utpptr->osig))))&PFL_SIGRES_IO)) &&
         ((utpptr->iflnum)<IFIL_BACKLOG) ) {
        // submit the file for input collection
        lnklst_fins((&(utpptr->iflchn)),ifil);
        // raise the number of input files
        utpptr->iflnum=(utpptr->iflnum)+1;
        // from here the ifile state machine should
        //  release the file when properly filled
    }
    else {
        // this utpptr is no longer accepting input files
        //  then drop it here and now
        // VERBOSE
        //  we could LOGI a message
        ifile_release((ifil_carrier*)ifil);
    }
    return(handoff);
}

void send_utpifile(ifil_carrier* ifil) {
    // returns 1 on crash abort, else 0
    //  normal submit of utp ifile (from any thread)
    //   ifil has been comepletely set up
    //    there are no checks that utp traffic is flowing here
    //     assumes caller did that check before going to the
    //      trouble of creating an ifil_carrier
    //
    msg_queue* queue;
    // the message gets passed to MU_thread4
    queue=&((thr_ctrl+4)->mqueue);
    msgexec_gblkv_carr(queue,msg_snd_uifile,(msg_gblkfnc*)ifil);
}

//------------------------------------------------------------------------------

void uuiox_pll_ocnsl(void) {
    u32 paylen;
    cnsl_carrier* carr;
    // we know header arrived with .len==0
    if (utpptr->ocnslnum) {
        // there are console lines on deck
        //  then this pop should succeed
        carr=(cnsl_carrier*)lnklst_fpop(&(utpptr->ocnslchn));
        // now there is one less line in the output queue
        utpptr->ocnslnum=(utpptr->ocnslnum)-1;
        // the output payload length includes the asciiz terminator
        paylen=(carr->lenz);
        *((u16*)(&(utpptr->uhdr[2])))=(u16)paylen;
        // so transfer the payload to uuioxbuff
        strcpy(((char*)uuioxbuff),(char*)(&(carr->data)));
        // now have to release the console carrier
        appfree((void*)carr);
        // reset the signalling state so that next console message
        //  issues another osig rising edge
        utpptr->ocnslst=CNSLST_IDLE;
        // send the reply
        utpptr->cm_state=CMDST_REPLY;
    }
    else {
        // when no console lines on deck, just return the header
        //  with .len==0
        // all we have to do is send the reply
        utpptr->cm_state=CMDST_REPLY;
    }
}

void uuiox_pll_ofile(void) {
    int paylen;
    ofil_carrier* ofil;
    // we know header arrived with .len==0
    if (utpptr->oflnum) {
        // there are output files on deck
        //  then this pop should succeed
        ofil=(ofil_carrier*)lnklst_fpop(&(utpptr->oflchn));
        // now there is one less line in the output queue
        utpptr->oflnum=(utpptr->oflnum)-1;
        // the output payload length includes the
        //  file size + file name + asciiz terminator
        paylen=4+strlen(&(ofil->filname[0]))+1;
        *((u16*)(&(utpptr->uhdr[2])))=(u16)paylen;
        // so transfer the payload to uuioxbuff
        memcpy(uuioxbuff,(char*)(&(ofil->filsize)),paylen);
        // this ofil_carrier gets promoted to bulk output queue
        lnklst_fins(&(utpptr->bochn),(lnklst*)ofil);
        // reset the signalling state so that next ofile message
        //  issues another osig rising edge
        utpptr->oflst=OFILST_IDLE;
        // send the reply
        utpptr->cm_state=CMDST_REPLY;
    }
    else {
        // when no output files on deck, just return the header
        //  with .len==0
        // all we have to do is send the reply
        utpptr->cm_state=CMDST_REPLY;
    }
}

//------------------------------------------------------------------------------

void uuiox_fnckey(void) {
    u32 fnc;
    char* arg;
    u32 err;
    u32 trm;
    s64 retval;
    char* dst;
    fnc=utpptr->dlen;
    // conceptually function key takes s64 arg and returns s64
    // s64 FNC#(s64* arg)
    //  though arg may also be interpretted as (u8*)arg as pointer to 8 bytes
    arg=(char*)(&(utpptr->datptr));
    dst=(char*)uuioxbuff;
    //
    // set response payload length - default (-1), waiting to get filled in
    *((u16*)(&(utpptr->uhdr[2])))=0xFFFF;
    // await response
    utpptr->cm_state=CMDST_EXEDONE;
    //
    // given this information, the general approach is to deplay a message
    //  to the appropriate driver to execute the function, or do it directly here
    //   (pfctrl may be included in the argument list)
    //
    // the handler should one of the follwing, either directly
    //  or by messaging back to the sender thread
    //
    // return ERRNO
    //    -- set quad at dst==uuioxbuff = ERRNO
    //    -- set u16 at utpptr->uhdr[2] = 0x0004
    //
    // return s64 return arg
    //    -- set quad at dst==uuioxbuff = 0x00000000 (OK) followed by 8 bytes
    //    -- set u16 at utpptr->uhdr[2] = 0x000C
    //
    // so here deploy what we know to messaged thread handler, or complete locally
    err=0;
    trm=0;
    switch (fnc) {
        // any FNC# may hand off to another thread, and leave err/trm=0
        //  or may set err (no data return)
        //  or may set trm asnd supply retval locally

        // known function key dispatches go here ...

        case 0x0001:
            *((u16*)(&(utpptr->uhdr[2])))=tst_f0001ky(arg,dst);
            break;

        case 0x0010:
            // image mode at HDMI
// QQQQQ DAISY
            if ((ftuptr->comstate)==COMXCHG) {
                // if comiox link up, daisy chain the command - but call as void
                // error=comv_fk_mode(u32 findx,u32 arglo,u32 arghi);
                comv_fk_mode(
                        fnc,                      // fnum
                        *((u32*)arg),             // arglo
                        *((u32*)(arg+4)));        // arghi
            }
            // local execution of fnckey()
            *((u16*)(&(utpptr->uhdr[2])))=fnc_0010_ky(arg,dst);
            break;

        case 0x0011:
            // Goes to the Commander
            *((u16*)(&(utpptr->uhdr[2])))=fnc_0011_ky(arg, dst);
            break;

        default:
            // VERBOSE - issue console message (optional)
            ucnsl_fl_fnckey(
                    fnc,                      // fnum
                    *((u32*)arg),             // arglo
                    *((u32*)(arg+4)));        // arghi
            // unknown function key must fail - send ERRNO
            err=UAERR_FNC_UNKN;
            break;

    }

    if (err) {
        *((u32*)dst)=err;
        *((u16*)(&(utpptr->uhdr[2])))=4;
        // so from the CMDST_EXEDONE service -- this is already done
    }
    if (trm) {
        *((u32*)dst)=0;
        dst+=4;
        *((s64*)dst)=retval;
        *((u16*)(&(utpptr->uhdr[2])))=12;
        // so from the CMDST_EXEDONE service -- this is already done
    }
}

//------------------------------------------------------------------------------

u32 uuiox_lenchk(u32 cmd,u32 len) {
    // returns error true (unreported) when header .len check fails
    //  assume fail
    u32 retval=1;
    switch (cmd) {
        case UAIOXCMD_WRBYTE:
            // 0x00
            // minimum 4 addr + 1 byte
            if (len>=5) {
                // maximum 4 addr + UAIOXCMD_MAXDATA bytes
                if (len<=(UAIOXCMD_MAXDATA+4)) {
                    // no alignment requirements
                    retval=0;
                }
            }
            break;
        case UAIOXCMD_WRWORD:
            // 0x02
            // minimum 4 addr + 1 word
            if (len>=6) {
                // maximum 4 addr + UAIOXCMD_MAXDATA bytes
                if (len<=(UAIOXCMD_MAXDATA+4)) {
                    // alignment requirements
                    if (!(len&0x01)) {
                        // size aligned
                        retval=0;
                    }
                }
            }
            break;
        case UAIOXCMD_WRQUAD:
            // 0x04
            // minimum 4 addr + 1 quad
            if (len>=8) {
                // maximum 4 addr + UAIOXCMD_MAXDATA bytes
                if (len<=(UAIOXCMD_MAXDATA+4)) {
                    // alignment requirements
                    if (!(len&0x03)) {
                        // size aligned
                        retval=0;
                    }
                }
            }
            break;
        case UAIOXCMD_RDBYTE:
            // 0x01
        case UAIOXCMD_RDWORD:
            // 0x03
        case UAIOXCMD_RDQUAD:
            // 0x05
            // fixed -- needs 4 addr + u16 numbytes
            if (len==6) {
                retval=0;
            }
            break;
        case UAIOXCMD_WRLDVC:
            // 0x06
            // minimum 4 addr + 1 byte + 1 LDVC
            if (len>=6) {
                // maximum 4 addr + UAIOXCMD_MAXDATA bytes + 1 LDVC
                if (len<=(UAIOXCMD_MAXDATA+5)) {
                    // no alignment requirements
                    retval=0;
                }
            }
            break;
        case UAIOXCMD_RDLDVC:
            // 0x07
            // fixed -- needs 4 addr + u16 numbytes + 1 LDVC
            if (len==7) {
                retval=0;
            }
            break;
        case UAIOXCMD_CONSOLE:
            // 0x0A
        case UAIOXCMD_OFILE:
            // 0x0B
            // fixed -- 0 header only
            if (len==0) {
                retval=0;
            }
            break;
        case UAIOXCMD_FNCKEY:
            // 0x09
            // fixed -- 10
            if (len==10) {
                retval=0;
            }
            break;
        default:
            // unknown - not implemented defaults error
            // case UAIOXCMD_HRTBT:
            // 0x08
            retval=1;
            break;
    }
    return(retval);
}

u32 uuiox_sanechk(void) {
    // returns error true (unreported) when command sanity check fails
    //  assume fail
    u32 cmd;
    u32 aoff;
    char* dptr;
    u32 retval=1;
    cmd=(u32)(utpptr->uhdr[0]);
    dptr=(char*)uuioxbuff;
    switch (cmd) {
        case UAIOXCMD_WRBYTE:
            // 0x00
            // the address does not need to be aligned
            aoff=*((u32*)dptr);
            // logical memory page is high byte of address
            utpptr->lmempg=aoff>>24;
            aoff=aoff&0x00FFFFFF;
            // actual data length
            utpptr->dlen=(utpptr->crscan)-4;
            // get actual data pointer
            utpptr->datptr=uaiox_lmembase(utpptr->lmempg);
            if (utpptr->datptr) {
                // data pointer is valid, adjust for offset
                utpptr->datptr=(utpptr->datptr)+aoff;
                // all OK
                retval=0;
            }
            // else return default error
            break;
        case UAIOXCMD_WRWORD:
            // 0x02
            // the address needs to be word aligned
            aoff=*((u32*)dptr);
            if (!(aoff&0x01)) {
                // address is aligned
                // logical memory page is high byte of address
                utpptr->lmempg=aoff>>24;
                aoff=aoff&0x00FFFFFF;
                // actual data length
                utpptr->dlen=(utpptr->crscan)-4;
                // get actual data pointer
                utpptr->datptr=uaiox_lmembase(utpptr->lmempg);
                if (utpptr->datptr) {
                    // data pointer is valid, adjust for offset
                    utpptr->datptr=(utpptr->datptr)+aoff;
                    // all OK
                    retval=0;
                }
                // else return default error
            }
            // else default fail on non-aligned
            break;
        case UAIOXCMD_WRQUAD:
            // 0x04
            // the address needs to be quad aligned
            aoff=*((u32*)dptr);
            if (!(aoff&0x03)) {
                // address is aligned
                // logical memory page is high byte of address
                utpptr->lmempg=aoff>>24;
                aoff=aoff&0x00FFFFFF;
                // actual data length
                utpptr->dlen=(utpptr->crscan)-4;
                // get actual data pointer
                utpptr->datptr=uaiox_lmembase(utpptr->lmempg);
                if (utpptr->datptr) {
                    // data pointer is valid, adjust for offset
                    utpptr->datptr=(utpptr->datptr)+aoff;
                    // all OK
                    retval=0;
                }
                // else return default error
            }
            // else default fail on non-aligned
            break;
        case UAIOXCMD_RDBYTE:
            // 0x01
            // the address does not need to be aligned
            aoff=*((u32*)dptr);
            // actual data length
            dptr+=4;
            utpptr->dlen=*((u16*)dptr);
            // logical memory page is high byte of address
            utpptr->lmempg=aoff>>24;
            aoff=aoff&0x00FFFFFF;
            // get actual data pointer
            utpptr->datptr=uaiox_lmembase(utpptr->lmempg);
            if (utpptr->datptr) {
                // data pointer is valid, adjust for offset
                utpptr->datptr=(utpptr->datptr)+aoff;
                // all OK
                retval=0;
            }
            // else return default error
            break;
        case UAIOXCMD_RDWORD:
            // 0x03
            // the address needs to be word aligned
            aoff=*((u32*)dptr);
            if (!(aoff&0x01)) {
                // address is aligned
                // actual data length
                dptr+=4;
                utpptr->dlen=*((u16*)dptr);
                // logical memory page is high byte of address
                utpptr->lmempg=aoff>>24;
                aoff=aoff&0x00FFFFFF;
                // get actual data pointer
                utpptr->datptr=uaiox_lmembase(utpptr->lmempg);
                if (utpptr->datptr) {
                    // data pointer is valid, adjust for offset
                    utpptr->datptr=(utpptr->datptr)+aoff;
                    // all OK
                    retval=0;
                }
                // else return default error
            }
            // else default fail on non-aligned
            break;
        case UAIOXCMD_RDQUAD:
            // 0x05
            // the address needs to be quad aligned
            aoff=*((u32*)dptr);
            if (!(aoff&0x03)) {
                // address is aligned
                // actual data length
                dptr+=4;
                utpptr->dlen=*((u16*)dptr);
                // logical memory page is high byte of address
                utpptr->lmempg=aoff>>24;
                aoff=aoff&0x00FFFFFF;
                // get actual data pointer
                utpptr->datptr=uaiox_lmembase(utpptr->lmempg);
                if (utpptr->datptr) {
                    // data pointer is valid, adjust for offset
                    utpptr->datptr=(utpptr->datptr)+aoff;
                    // all OK
                    retval=0;
                }
                // else return default error
            }
            // else default fail on non-aligned
            break;
        case UAIOXCMD_WRLDVC:
            // 0x06 - implicit byte action
            // logical device is last data byte
            utpptr->ldvc=*((u8*)(dptr+(((u32)(utpptr->crscan))-1)));
            if ((utpptr->ldvc)<UAIOX_LDVC_BND) {
                // logical device is legal
                // subsequent logical device handling may make use of
                //  utpptr->lmempg and utpptr->datptr
                // the address has no alignment checks
                utpptr->ldva=*((u32*)dptr);
                // actual data length
                utpptr->dlen=(utpptr->crscan)-5;
                // all OK
                retval=0;
            }
            // else return default error
            //
            // in this system, no logical devices are supported yet
            //  #define UAIOX_LDVC_BND      0x00
            // }
            break;
        case UAIOXCMD_RDLDVC:
            // 0x07 - implicit byte action
            // logical device is last data byte
            utpptr->ldvc=*((u8*)(dptr+(((u32)(utpptr->crscan))-1)));
            if ((utpptr->ldvc)<UAIOX_LDVC_BND) {
                // logical device is legal
                // subsequent logical device handling may make use of
                //  utpptr->lmempg and utpptr->datptr
                // the address has no alignment checks
                utpptr->ldva=*((u32*)dptr);
                // actual data length
                dptr+=4;
                utpptr->dlen=*((u16*)dptr);
                // all OK
                retval=0;
            }
            // else return default error
            //
            // in this system, no logical devices are supported yet
            //  #define UAIOX_LDVC_BND      0x00
            // }
            break;
        case UAIOXCMD_FNCKEY:
            // 0x09
            // we know the command is the right length with 0x0A bytes
            // just get the data off the I/O buffer
            //  and transfer to
            //    char*   datptr;
            //    u16     dlen;
            memcpy((void*)(&(utpptr->datptr)),(void*)dptr,0x0A);
            // that leaves the 8 bytes datptr up to 64 bit s64 FNC# argument(s)
            //  and dlen=#FNC
//            sprintf(utpdbgln,"length = %d\n", utpptr->dlen);
//            utp_dbgoutput(utpdbgln);
            if ((utpptr->dlen)<UAIOX_FK_BND) {
                // generically, we don't have overall rules to check argument values
                //  against FNC# operations
                //   execution of the FNC# has to make that determination
                // all OK
                retval=0;
            }
            // else return default error
            //
            // in this system, no function keys supported yet
            //  #define UAIOX_FK_BND        0x0000
            // }
            break;
            //
            // this is header only command that should never get here
            //  because it bypasses CMDST_WTPAYLD and CMDST_SANITY
            // case UAIOXCMD_HRTBT:
            //    // 0x08
        default:
            // unknown - not implemented defaults error
            retval=1;
            break;
    }
    return(retval);
}

u32 uuiox_cmdexec(void) {
    // returns error true (unreported) if insurmountable error detected
    //  but execution can "pass" and report error across the link in some cases
    u32 cmd;
    u32 retval=1;
    cmd=(u32)(utpptr->uhdr[0]);
    switch (cmd) {

        case (UAIOXCMD_WRBYTE):
            // 0x00
            uuiox_wrbyte();
            retval=0;
            break;

        case (UAIOXCMD_RDBYTE):
            // 0x01
            uuiox_rdbyte();
            retval=0;
            break;

        case (UAIOXCMD_WRWORD):
            // 0x02
            uuiox_wrword();
            retval=0;
            break;

        case (UAIOXCMD_RDWORD):
            // 0x03
            uuiox_rdword();
            retval=0;
            break;

        case (UAIOXCMD_WRQUAD):
            // 0x04
            uuiox_wrquad();
            retval=0;
            break;

        case (UAIOXCMD_RDQUAD):
            // 0x05
            uuiox_rdquad();
            retval=0;
            break;

        case (UAIOXCMD_WRLDVC):
            // 0x06
            uuiox_wrldvc();
            retval=0;
            break;

        case (UAIOXCMD_RDLDVC):
            // 0x07
            uuiox_rdldvc();
            retval=0;
            break;

        case (UAIOXCMD_FNCKEY):
            // 0x09
            uuiox_fnckey();
            retval=0;
            break;

        case (UAIOXCMD_CONSOLE):
            // 0x0A
            uuiox_pll_ocnsl();
            retval=0;
            break;

        case (UAIOXCMD_OFILE):
            // 0x0B
            uuiox_pll_ofile();
            retval=0;
            break;

        default:
            // unknown command should never get here
            // default exit error true
            // case (UAIOXCMD_HRTBT):
            // 0x08
            // this is header only command whose data length is already 0
            //  not supported in this version
            break;

    }
    return(retval);
}

void utp_dmp_cfstat(void) {
    // dmp status of what android currentl assumes is dvc fifo state
    utp_dbgoutput("APP cfifo state\n");
    sprintf(utpdbgln,"  ctrl rxfifo psh %08X pll %08X avail %08X\n",
            (utpptr->utphdr.pc2a_psh),(utpptr->utphdr.pc2a_pll),
            (utpptr->pavail));
    utp_dbgoutput(utpdbgln);
    sprintf(utpdbgln,"  ctrl txfifo psh %08X pll %08X space %08X\n",
            (utpptr->utphdr.a2pc_psh),(utpptr->utphdr.a2pc_pll),
            (utpptr->pspace));
    utp_dbgoutput(utpdbgln);
}

u32 utp_cmd_uuiox(void) {
    // return error true on major protocol violations
    //  that completely reset the UTP/device interface
    //   this traffic model is q820usb uartaiox protocol
    //
    // this traffic handler gets called from utpxchgpass()
    //  so if we exit here with error, utpxchgpass() error terminates
    //   that returns to void utp_FSM() at state UTPST_XCHG
    //    which takes care of any utp_softreset() requirements while it
    //     reboots UTP all the way to a device close() after a retry delay
    //  since utp_FSM() is a void call, the error can't propagate any further
    //
    // it shouldn't touch the RES or HB signals
    //  but any user .osig changes should be communicated to both
    //   .osig and .sig_a2pc
    // this is responsible for setting setting outputs
    //    utpptr->utphdr.a2pc_psh=0;
    //    utpptr->utphdr.pc2a_pll=0;
    //
    //    utpptr->utphdr.a2pcbpsh | these are untouched as they belong to bulk transfer
    //    utpptr->utphdr.pc2abpll |  state machine processes
    //
    u32 msk;
    u32 val;
    u32 done;
    u8 cmst;
    u32 error;
    error=0;
    // we are in state UTPST_XCHG, non resetting from either side
    //  we can trust the status from the device driver
    //   establish the bounds as read from the interface device driver
    //
    // avail=(push-pull)&msk
    msk=((u32)(utpptr->utphdr.ctrlsize))-1;
    val=((utpptr->utphdr.pc2a_psh)-(utpptr->utphdr.pc2a_pll))&msk;
    utpptr->pavail=(u16)val;
    // space=msk-((push-pull)&msk) -- though we always leave some headway
    val=((utpptr->utphdr.a2pc_psh)-(utpptr->utphdr.a2pc_pll))&msk;
    val=msk-val;
    if (val>PFL_TXSAFE) {
        val=val-PFL_TXSAFE;
    }
    else {
        val=0;
    }
    utpptr->pspace=(u16)val;
    // we may be able to execute multiple commands in a single pass
    done=0;
    while ((!done) && (!error)) {
        cmst=utpptr->cm_state;
        switch (cmst) {

            case CMDST_HEADER:
                // implicitly crscan=0, waiting for header
                if ((utpptr->pavail)>=4) {
                    // we can pull a command header, and reduce avail accordingly
                    // VERBOSITY debug
                    // utp_dbgoutput("CMDST_HEADER engaging\n");
                    // utp_dmp_cfstat();
                    error=utp_pfifo_rd((char*)(&(utpptr->uhdr[0])),4);
                    if (!error) {
                        // VERBOSITY debug
                        // sprintf(utpdbgln,"utp_cmd_uuiox() CMDST_HEADER pickup %08X (avail %04X)\n",
                        //                  *((u32*)(&(utpptr->uhdr[0]))),(utpptr->pavail));
                        // utp_dbgoutput(utpdbgln);
                        utpptr->pavail=(utpptr->pavail)-4;
                        // is known command?
                        if ((utpptr->uhdr[0])<UAIOXCMD_BOUND) {
                            // looks like legal command
                            // we don't care about sequence number - only host checks,
                            //  and we'll echo same value no matter what it is
                            // assume the .len field is valid, no matter what it is
                            //  we'll use .hdr for response generation, so pull out
                            //   the length
                            utpptr->crscan=*((u16*)(&(utpptr->uhdr[2])));
                            // that's how much we need to wait on
                            if (!(uuiox_lenchk((u32)(utpptr->uhdr[0]),(u32)(utpptr->crscan)))) {
                                // we're OK with the length as reported
                                if (utpptr->crscan) {
                                    // we have to wait on non-zero length payload to fully assess
                                    //  (not done yet)
                                    utpptr->cm_state=CMDST_WTPAYLD;
                                }
                                else {
                                    // zero length means this command needs header only
                                    //  advance to processing state, but not done
                                    //   (a header only command is generally assumed sane
                                    //     and we can simultaneously bypass CMDST_SANITY)
                                    utpptr->cm_state=CMDST_EXEC;
                                }
                            }
                            else {
                                // some problem with the data length
                                sprintf(utpdbgln,"utp_cmd_uuiox() command 0x%02X length 0x%04X error\n",
                                        (utpptr->uhdr[0]),(utpptr->crscan));
                                utp_dbgoutput(utpdbgln);
                                error=1;
                            }
                        }
                        else {
                            // unknown command
                            sprintf(utpdbgln,"utp_cmd_uuiox() unknown command 0x%02X\n",
                                    (utpptr->uhdr[0]));
                            utp_dbgoutput(utpdbgln);
                            error=1;
                        }
                    }
                    // else exit on utp_pfifo_rd() header read error
                }
                else {
                    // insufficient data to pull a header
                    // let it slide to next pass
                    done=1;
                }
                break;

            case CMDST_WTPAYLD:
                // implicitly crscan is the size of payload we're waiting on
                if ((utpptr->pavail)>=(utpptr->crscan)) {
                    // VERBOSITY debug
                    // sprintf(utpdbgln,"utp_cmd_uuiox() CMDST_WTPAYLD avail %04X needs %04X)\n",
                    //                  (utpptr->pavail),(utpptr->crscan));
                    // utp_dbgoutput(utpdbgln);
                    // all data is available to complete command
                    // we can pull the payload and reduce avail
                    error=utp_pfifo_rd((char*)uuioxbuff,(utpptr->crscan));
                    if (!error) {
                        utpptr->pavail=(utpptr->pavail)-(utpptr->crscan);
                        // now do sanity checks on entire data package
                        utpptr->cm_state=CMDST_SANITY;
                    }
                    // else exit on  utp_pfifo_rd() data read error
                    // VERBOSITY debug
                    // else {
                    // utp_dbgoutput("CMDST_WTPAYLD read error");
                    // }
                }
                else {
                    // we'll have to try again next pass
                    done=1;
                }
                break;

            case CMDST_SANITY:
                if (!(uuiox_sanechk())) {
                    // everything about the payload appears properly formatted
                    //  proceed to the execution phase, though we're not done yet
                    utpptr->cm_state=CMDST_EXEC;
                }
                else {
                    // detected some offending error in command data payload
                    //  (add status LOGI/LOGE in uaiox_sanechk() to debug
                    sprintf(utpdbgln,"utp_cmd_uuiox() format error cmd 0x%02X\n",
                            (utpptr->uhdr[0]));
                    utp_dbgoutput(utpdbgln);
                    error=1;
                }
                break;

            case CMDST_EXEC:
                // when we get to the execution phase,
                //  direct execution commands fill in resposnse payload at uuioxbuff
                //   and appropriately set data length at utpptr->hdr[2..3]
                //    this is the standard output format, that allows state branching
                //     directly to CMDST_REPLY
                //      -- all the local memory base read/write commands are in this category
                //  deferred execution commands -- all logical device commands are in this category
                //   they generally issue a message to some other thread
                //    and proceed to CMDST_EXEDONE to wait on the reply
                //     the other thread that accepts the command can fill in uuioxbuff and
                //      utpptr->hdr[2..3] or message this thread how to do it properly...
                //       but for this state machine, the general approach is to set utpptr->hdr[2..3]
                //        data size to (-1), before enetering CMDST_EXEDONE,
                //         and just wait on it to become non-zero to ptoceed to CMDST_REPLY
                //  function key commands can be either direct execution or deferred execution
                //
                // VERBOSITY debug
                //utp_dbgoutput("...arrived CMDST_EXEC\n");
                if (uuiox_cmdexec()) {
                    // fatal command handling error
                    //  resync the whole link
                    sprintf(utpdbgln,"uuiox_cmdexec() failure for cmd 0x%02X\n",
                            (utpptr->uhdr[0]));
                    utp_dbgoutput(utpdbgln);
                    error=1;
                }
                // else {
                // // uaiox_cmdexec() should have appropriatecly changed states
                // //  and is now ready to wait on deferred result or direct output
                // //   so there is implicit state trnasition here, and we're not done
                // }
                break;

            case CMDST_EXEDONE:
                // entry to this state MUST have set utpptr->uhdr[2..3] to (-1)
                //  wait on this to get filled in with the number of return
                //   payload bytes at uuioxbuff
                if (!((*((u16*)(&(utpptr->uhdr[2]))))==0xFFFF)) {
                    // output data has ben placed - ready to proceed, but not done
                    utpptr->cm_state=CMDST_REPLY;
                }
                else {
                    // try again next pass
                    done=1;
                }
                break;

            case CMDST_REPLY:
                // use val to calculate total valid output
                val=(u32)(*((u16*)(&(utpptr->uhdr[2]))));
                // we'll output header + uuioxbuff
                val+=4;
                // is there space for it
                if (((u32)(utpptr->pspace))>=val) {
                    // completely ready to deploy response
                    // send the header
                    error=utp_pfifo_wr((char*)(&(utpptr->uhdr[0])),4);
                    if (!error) {
                        utpptr->pspace=(utpptr->pspace)-4;
                        val-=4;
                        // then send the data payload
                        if (val) {
                            // only send payload if not empty
                            error=utp_pfifo_wr((char*)uuioxbuff,val);
                            if (!error) {
                                utpptr->pspace=(utpptr->pspace)-((u16)val);
                            }
                        }
                    }
                    if (!error) {
                        // once output placed, we're ready to process next command
                        //  so we're not done
                        // VERBOSITY debug
                        // utp_dbgoutput("post CMDST_REPLY\n");
                        // utp_dmp_cfstat();
                        utpptr->cm_state=CMDST_HEADER;
                    }
                }
                else {
                    // forced to wait on output space
                    // try again next pass
                    done=1;
                }
                break;

            default:
                // unknown state - something went really wrong
                //  resync the whole link
                sprintf(utpdbgln,"utp_cmd_uuiox() in unknown state 0x%02X\n",cmst);
                utp_dbgoutput(utpdbgln);
                error=1;
                break;
        }
    }
    if (error) {
        utp_dbgoutput("utp_cmd_uuiox() protocol error\n");
    }
    return(error);
}

//------------------------------------------------------------------------------

void utp_softreset(void) {
    // when either side commands a reset on the fly
    //  it drops all incomplete transactions
    //   but doesn't have to go through the whole process 4 ping heartbeat detection
    // this is how to kill any transactions that are no longer getting serviced
    u32 error;
    error=0;
    // ground state on command processing
    utpptr->cm_state=CMDST_HEADER;
    // clear out any undeliverable console messages
    utpcnsl_flush();
    // clear out any undeliverable output files
    utpofile_flush();
    // clear out any incomplete input files
    utpifile_flush();
    // error or not, this must return void
    // return(error);
}

void utp_restart(void) {
    // close the device if it's open
    //close_utp();
    // set reference time
    //  after a delay we'll try to start it up again
    utpptr->timeref=ACptr->tmr_cnt;
    // that's to avaoid a fast cycling loop of repeating failure
    utpptr->utpstate=UTPST_RETRY;
    // reset any partial transactions that may have been initiated
    //  flush any traffic buffers
    utp_softreset();
}

void utp_isig_srv(u16 msk) {
    // this is edge triggered response (either direction)
    //  msk input is the set of .isih USR signals that changed state

    // handle the .isig signal polarity for each set msk bit
    // for now - does nothing

}

u32 utpxchgpass(void) {
    // return error true on a major crash or protocol violation
    // device status has been collected, and ping handshake has indicated
    //  the other side is connected on the utp link
    //   so now we process traffic
    // in state UTPST XCHG, this routine is responsible for setting up
    //  .sig_a2pc through .pc2abpll which get passed out to the driver
    //   on exit
    // it should not modify the SIGRES or SIG_HB bits of .osig
    //  those are taken care of externally, if a return error is signalled
    //   to take down the utp link
    //
    u32 error;
    int tval;
    u16 obits;
    u16 ebits;
    error=0;
    // we still echo and monitor heartbeat toggles, even during reset
    obits=(utpptr->osig)&(PFL_SIG_HB);
    ebits=(utpptr->eosig)&(PFL_SIG_HB);
    if (obits==ebits) {
        // another heartbeat detected
        // toggle heartbeat signal and record time of event
        utpptr->osig=(utpptr->osig)^PFL_SIG_HB;
        utpptr->timeref=ACptr->tmr_cnt;
    }
    else {
        // no heartbeat
        // get elapsed time
        tval=(ACptr->tmr_cnt)-(utpptr->timeref);
        if (tval>=UTP_LOCK_ZTO) {
            // other side seems to have stopped echo, it went off line
            // so crash the utp link
            utp_dbgoutput("utpxchgpass() heartbeat timeout\n");
            error=1;
        }
        // else do nothing
    }
    // either side can signal a reset during lock
    //  if either side does signal a reset, we tear down any intermediate processing
    //   but it's not cause to drop the link
    if (!error) {
        ebits=((utpptr->osig)|(utpptr->isig))&PFL_SIGRES;
        if (ebits) {
            // one of the sides is signalling reset,
            //  so tear down any incomplete transactions
            utp_softreset();
            // and prepare the reset status
            utp_reset_ostatus();
            // for history tracking, record if remote was commanding reset
            if ((utpptr->isig)&PFL_SIGRES) {
                utpptr->rmtres=1;
            }
            else {
                utpptr->rmtres=0;
            }
            // exit with error=0
        }
        else {
            // non reset behaviour
            // maybe servicing the input signal edges or a protocol violation
            //  at command response will cause a reset and override this
            //   then such an action should enfore necessary rewrite
            //    of .osig and .sig_a2pc
            //     for now we baseline on the assumption they are not changing
            utpptr->utphdr.sig_a2pc=(((int)(utpptr->isig))<<16)|((int)(utpptr->osig));
            // handle input isgnals
            if (utpptr->rmtres) {
                // last pass, remote was commanding reset
                //  then this is the first valid set of .isig
                utp_isig_srv(PFL_SIG_USR);
            }
            else {
                // remote was not resetting last pass
                //  then we only have to service .isig bits that changed
                obits=((utpptr->isig)^(utpptr->zisig))&PFL_SIG_USR;
                if (obits) {
                    utp_isig_srv(obits);
                }
            }
            // for history tracking, remote was not commanding reset this pass
            utpptr->rmtres=0;
            //
            // main stages of traffic data handling
            //  1) command/response handling through the control pipe
            //  2) bulk collection
            //  3) bulk deployment
            //
            // each should supply their own crash message if they return error
            //
            // general command/response handling
            error=utp_cmd_uuiox();
            if (!error) {
                // bulk deployment
                error=utp_blko_FSM();
                if (!error) {
                    // bulk collection
                    error=utp_blki_FSM();
                    if (!error) {
                        // handle utp ofile signalling
                        error=utp_ofile_FSM();
                        if (!error) {
                            // handle utp console signalling
                            error=utp_cnsl_FSM();
                        }
                    }
                    // VERBOSITY debug
                    // else {
                    // utp_dbgoutput("utp_blki_FSM() returned error\n");
                    // }
                }
                // VERBOSITY debug
                // else {
                // utp_dbgoutput("utp_blko_FSM() returned error\n");
                // }
            }
        }
    }
    return(error);
}

//------------------------------------------------------------------------------

void utp_FSM(void) {
    // all errors handled internally
    u8 state;
    u32 done;
    u32 error;
    int tval;
    u16 obits;
    u16 ebits;
    done=0;
    while (!done) {
        //LOGI("UTP FSM pinging....\n");
        state=utpptr->utpstate;
        switch (state) {

            case UTPST_DOWN:
                // the device driver hasn't even been opened
                // reset heartbeat ping detection
                utpptr->pings=0;
                // attempt device open
                error=open_utp();
                if (error) {
                    // no device, can't proceed to next state
                    // record the time
                    utpptr->timeref=ACptr->tmr_cnt;
                    // change state to retry
                    utpptr->utpstate=UTPST_RETRY;
                    // VERBOSITY - keep
                    // sprintf(utpdbgln,"UTPST_DOWN -> UTPST_RETRY\n");
                    // utp_dbgoutput(utpdbgln);
                    // done for this pass
                    done=1;
                }
                else {
                    // device opened and we have fd
                    //  at this point .osig commands SIGRES=1, SIG_HB=0
                    //   but none of that has been communicated out to the device file
                    // at this point, we want to collect first status from the device
                    //  to establish input value, and all our output values are
                    //   implicit
                    error=utp_first_status();
                    if (error) {
                        // reset to UTPST_RETRY
                        utp_restart();
                        done=1;
                    }
                    else {
                        // now we expect to see our own reset output echo'd back to us
                        //  there's a time limit on that
                        utpptr->timeref=ACptr->tmr_cnt;
                        // advance the state
                        utpptr->utpstate=UTPST_RESET;
                        // but we're done this pass, and we'll come back in that state
                        // VERBOSITY - keep
                        // sprintf(utpdbgln,"UTPST_DOWN -> UTPST_RESET\n");
                        // utp_dbgoutput(utpdbgln);
                        done=1;
                    }
                }
                break;

            case UTPST_RETRY:
                // get elapsed time
                tval=(ACptr->tmr_cnt)-(utpptr->timeref);
                if (tval>=UTP_RETRY_Z) {
                    // time to retry
                    utpptr->utpstate=UTPST_DOWN;
                    // try new state immediately (not done)
                    // VERBOSITY - keep
                    // sprintf(utpdbgln,"UTPST_RETRY -> UTPST_DOWN\n");
                    // utp_dbgoutput(utpdbgln);
                }
                else {
                    // done for now, try again next pass
                    done=1;
                }
                break;

            case UTPST_RESET:
                // we're waiting for first echo of reset and !heartbeat ping
                error=utp_getstatus();
                if (!error) {
                    // status is valid
                    obits=(utpptr->osig)&(PFL_SIGRES|PFL_SIG_HB);
                    ebits=(utpptr->eosig)&(PFL_SIGRES|PFL_SIG_HB);
                    if (obits==ebits) {
                        // ready for next state
                        // here is the first toggle of ping
                        //  mark the time of every ping update
                        utpptr->osig=(utpptr->osig)^PFL_SIG_HB;
                        utpptr->timeref=ACptr->tmr_cnt;
                        // advance state
                        utpptr->utpstate=UTPST_PINGS;
                        //  our commanded status is set at .osig
                        //   so just loop back .isig, and set push/pointers to 0
                        utp_reset_ostatus();
                        error=utp_putstatus();
                        // VERBOSITY - keep
                        // sprintf(utpdbgln,"UTPST_RESET -> UTPST_PINGS\n");
                        // utp_dbgoutput(utpdbgln);
                        if (error) {
                            // reset to UTPST_RETRY
                            utp_restart();
                            done=1;
                        }
                        else {
                            done=1;
                        }
                    }
                    else {
                        // no echo yet
                        // get elapsed time
                        tval=(ACptr->tmr_cnt)-(utpptr->timeref);
                        if (tval>=UTP_PING_ZTO) {
                            // this is too long, the other side is not echoing
                            //  host is not maintining this connection...
                            // reset to UTPST_RETRY
                            // VERBOSITY - keep
                            // sprintf(utpdbgln,"UTPST_RESET -> error TIMEOUT\n");
                            // utp_dbgoutput(utpdbgln);
                            utp_restart();
                            done=1;
                        }
                        else {
                            // no echo yet, but still within the window
                            //  our commanded status hasn't changed at .osig
                            //   so just loop back .isig, and set push/pointers to 0
                            utp_reset_ostatus();
                            error=utp_putstatus();
                            if (error) {
                                // reset to UTPST_RETRY
                                utp_restart();
                                done=1;
                            }
                            else {
                                done=1;
                            }
                        }
                    }
                }
                else {
                    // error collecting status
                    // reset to UTPST_RETRY
                    utp_restart();
                    done=1;
                }
                break;

            case UTPST_PINGS:
                // we're waiting for echo of reset and heartbeat
                error=utp_getstatus();
                if (!error) {
                    // status is valid
                    obits=(utpptr->osig)&(PFL_SIGRES|PFL_SIG_HB);
                    ebits=(utpptr->eosig)&(PFL_SIGRES|PFL_SIG_HB);
                    if (obits==ebits) {
                        // another startup heartbeat ping completed
                        tval=((int)(utpptr->pings))+1;
                        utpptr->pings=(u8)tval;
                        if (tval>=UTP_PINGLOCK) {
                            // handshake complete
                            // prepare next toggle
                            //  mark the time of every ping update
                            // but now tear down the reset and advance the state
                            utpptr->osig=((utpptr->osig)^PFL_SIG_HB)&(~PFL_SIGRES);
                            utpptr->timeref=ACptr->tmr_cnt;
                            utpptr->utpstate=UTPST_XCHG;
                            // this is the first time this side is out of reset
                            //  (though other side may still be commanding reset)
                            if ((utpptr->isig)&PFL_SIGRES) {
                                // the other side is still commanding reset
                                utpptr->rmtres=1;
                            }
                            else {
                                // the other side is not reset
                                //  then this is the first instance of valid .isig
                                //   check what all user bits are signalling
                                utp_isig_srv(PFL_SIG_USR);
                                utpptr->rmtres=0;
                            }
                            // although the reset osig is removed now,
                            //  thiis is the last time we use this output combination
                            utp_reset_ostatus();
                            error=utp_putstatus();
                            // VERBOSITY - keep
                            // sprintf(utpdbgln,"UTPST_PINGS -> UTPST_XCHG\n");
                            // utp_dbgoutput(utpdbgln);
                            done=1;
                        }
                        else {
                            // VERBOSITY - keep
                            // sprintf(utpdbgln,"UTPST_PINGS detected ping %d\n",tval);
                            // utp_dbgoutput(utpdbgln);
                            // ping increment recorded, prepare next toggle
                            //  mark the time of every ping update
                            utpptr->osig=(utpptr->osig)^PFL_SIG_HB;
                            utpptr->timeref=ACptr->tmr_cnt;
                            // but stay in this state
                            utp_reset_ostatus();
                            error=utp_putstatus();
                            if (error) {
                                // reset to UTPST_RETRY
                                utp_restart();
                                done=1;
                            }
                            else {
                                done=1;
                            }
                        }
                    }
                    else {
                        // no echo yet
                        // get elapsed time
                        tval=(ACptr->tmr_cnt)-(utpptr->timeref);
                        if (tval>=UTP_PING_ZTO) {
                            // this is too long, the other side is not echoing
                            //  host is not maintining this connection...
                            // reset to UTPST_RETRY
                            // VERBOSITY - keep
                            // sprintf(utpdbgln,"UTPST_PINGS -> error TIMEOUT\n");
                            // utp_dbgoutput(utpdbgln);
                            utp_restart();
                            done=1;
                        }
                        else {
                            // no echo yet, but still within the window
                            //  our commanded status hasn't changed at .osig
                            //   so just loop back .isig, and set push/pointers to 0
                            utp_reset_ostatus();
                            error=utp_putstatus();
                            if (error) {
                                // reset to UTPST_RETRY
                                utp_restart();
                                done=1;
                            }
                            else {
                                done=1;
                            }
                        }
                    }
                }
                else {
                    // error collecting status
                    // reset to UTPST_RETRY
                    utp_restart();
                    done=1;
                }
                break;

            case UTPST_XCHG:
                // in this state, we're usually conductiong traffic
                //  but we do have to soft reset if it commanded by the other side
                error=utp_getstatus();
                if (!error) {
                    // status is valid
                    error=utpxchgpass();
                    if (!error) {
                        // on exit, we always provide a status update
                        //  from .sig_a2pv through .pc2abpll
                        // VERBOSITY deep debug
                        // sprintf(utpdbgln,"at utp_putstatus()  psh %08X pll %08X\n",
                        //                  (utpptr->utphdr.a2pcbpsh),(utpptr->utphdr.a2pcbpll));
                        // utp_dbgoutput(utpdbgln);
                        error=utp_putstatus();
                        if (error) {
                            // reset to UTPST_RETRY
                            utp_restart();
                            done=1;
                        }
                        else {
                            done=1;
                        }
                    }
                    else {
                        // a major protocol violation can crash the link
                        // reset to UTPST_RETRY
                        utp_restart();
                        done=1;
                    }
                }
                else {
                    // error collecting status
                    // reset to UTPST_RETRY
                    utp_restart();
                    done=1;
                }
                break;

            default:
                // unknown state recovery
                // reset to UTPST_RETRY
                utp_restart();
                done=1;
                break;
        }
    }
}

//------------------------------------------------------------------------------

void olxchgring_init(void) {
    // initialize the control uaio_xchg set of open loop carriers
    void* head;
    char* nxt;
    int i;
    head=(void*)(&(olxchg_set[0]));
    lnklst_init((lnklst*)head);
    nxt=(char*)head;
    ((uaio_xchg*)(((char*)head)+0x10))->flg=FLG_uaiox_done;
    for (i=0;i<(OLXCHG_NUM-1);i++) {
        nxt=nxt+sizeof(olxchg_t);
        ((uaio_xchg*)(nxt+0x10))->flg=FLG_uaiox_done;
        lnklst_fins((lnklst*)head,(lnklst*)nxt);
    }
    // the ring is established,
    //  there is nothing unique about the head element, just use it first
    ACptr->olxchg=(lnklst*)head;
}

uaio_xchg* try_olx_carrier(void) {
    // returns null pointer on failure
    int* hwsema;
    uaio_xchg* retval;
    lnklst* olhead;
    int tst;
    // assume failure
    hwsema=(int*)(&(ACptr->hwsema_olx));
    // spin on the semaphore
    tst=1;
    while (tst) {
        tst=__sync_fetch_and_or(hwsema,1);
    }
    // there's no doubt which open loop carrier will get granted
    olhead=ACptr->olxchg;
    retval=(uaio_xchg*)(((char*)olhead)+0x10);
    if ((retval->flg)==FLG_uaiox_done) {
        // the xchg is ready to use (not busy)
        //  so we'll auto-advance next usable carrier
        ACptr->olxchg=(lnklst*)(olhead->rvrs);
    }
    else {
        // fail this attempt and return invalid ptr
        retval=(uaio_xchg*)0;
    }
    // any way out releases semaphore
    tst=__sync_fetch_and_and(hwsema,0);
    // exit
    return(retval);
}

uaio_xchg* get_ol_carrier(void) {
    // spin through sched_yield() until uaio_xchg open loop carrier is granted
    uaio_xchg* retval;
    retval=(uaio_xchg*)0;
    while (!(retval)) {
        retval=try_olx_carrier();
        if (!(retval)) {
            //  try again next pass
            sched_yield();
        }
    }
    return(retval);
}

//------------------------------------------------------------------------------

void init_comiox_ctrl(void) {
    // init once at startup
    lnklst_init(&(ftuptr->cmtxchn));
    lnklst_init(&(ftuptr->cmtachn));
    ftuptr->cmtxque=&comtx0que;
    ftuptr->cmrxque=&comrx0que;
}

void uaiox_SYSresponse(uaio_xchg* xchg) {
    // terminate xchg with *retptr if called for
    u32 retval;
    int* dst;
    dst=(xchg->retptr);
    if (dst) {
        // parent/customer wants response retval
        // retval=error --> 0 when everythinbg OK
        //                  non-zero reports error detection
        retval=(xchg->err);
        // set return error code
        *(dst+1)=retval;
        // set done
        *dst=1;
    }
    // else void termination - no response to issuer
}

void uaiox_quedrop(lnklst* head,u8 errval) {
    uaio_xchg* xchg;
    int done=0;
    while (!done) {
        xchg=(uaio_xchg*)(head->rvrs);
        if (((lnklst*)xchg)==head) {
            // end of list
            done=1;
        }
        else {
            // drop element from chain
            lnklst_drop((lnklst*)xchg);
            // set termination error
            xchg->err=errval;
            // response length = 0
            xchg->len=0;
            // but otherwise mark xchg completed
            xchg->flg=FLG_uaiox_done;
            // optional parent signalling
            uaiox_SYSresponse(xchg);
        }
    }
}

void comiox_reset(void) {
    // reset sequence counters
    ftuptr->cmtseq=0;
    ftuptr->cmtack=0;
    ftuptr->cmrstate=CMDST_HEADER;
    // clobber all txchn exchanges (unsent)
    uaiox_quedrop(&(ftuptr->cmtxchn),ERR_uaiox_lnkdn);
    // clobber all tachn exchanges (awaiting response)
    uaiox_quedrop(&(ftuptr->cmtachn),ERR_uaiox_lnkdn);
}

u32 comiox_lenchk(u32 cmd,u32 len) {
    // returns error true (unreported) when header .len check fails
    //  simultaneously checks .cmd is valid, and that expected .len is valid
    //   for given .cmd
    //  assume fail
    u32 msk;
    u32 cbase;
    u32 retval=1;
    // detect responses
    msk=cmd&0x00000080;
    // and command base
    cbase=cmd&0x0000007F;
    switch (cbase) {
        case UAIOXCMD_WRBYTE:
            // 0x00
            if (msk) {
                // response should have 0 payload
                if (!len) {
                    retval=0;
                }
            }
            else {
                // incoming command -  minimum 4 addr + 1 byte
                if (len>=5) {
                    // maximum 4 addr + UAIOXCMD_MAXDATA bytes
                    if (len<=(UAIOXCMD_MAXDATA+4)) {
                        // no alignment requirements
                        retval=0;
                    }
                }
            }
            break;
        case UAIOXCMD_WRWORD:
            // 0x02
            if (msk) {
                // response should have 0 payload
                if (!len) {
                    retval=0;
                }
            }
            else {
                // incoming command - minimum 4 addr + 1 word
                if (len>=6) {
                    // maximum 4 addr + UAIOXCMD_MAXDATA bytes
                    if (len<=(UAIOXCMD_MAXDATA+4)) {
                        // alignment requirements
                        if (!(len&0x01)) {
                            // size aligned
                            retval=0;
                        }
                    }
                }
            }
            break;
        case UAIOXCMD_WRQUAD:
            // 0x04
            if (msk) {
                // response should have 0 payload
                if (!len) {
                    retval=0;
                }
            }
            else {
                // incoming command - minimum 4 addr + 1 quad
                if (len>=8) {
                    // maximum 4 addr + UAIOXCMD_MAXDATA bytes
                    if (len<=(UAIOXCMD_MAXDATA+4)) {
                        // alignment requirements
                        if (!(len&0x03)) {
                            // size aligned
                            retval=0;
                        }
                    }
                }
            }
            break;
            // for this group, we could check that the response N-bytes
            //  matches what the command asked for... but we don't
            //   it just assumes any non-zero N is good unless it's too big
        case UAIOXCMD_RDBYTE:
            // 0x01
        case UAIOXCMD_RDWORD:
            // 0x03
        case UAIOXCMD_RDQUAD:
            // 0x05
            if (msk) {
                // response somewhat arbitrary N
                if ((len) && (len<=UAIOXCMD_MAXDATA)) {
                    retval=0;
                }
            }
            else {
                // command fixed -- needs 4 addr + u16 numbytes
                if (len==6) {
                    retval=0;
                }
            }
            break;
        case UAIOXCMD_WRLDVC:
            // 0x06
            if (msk) {
                // response is len=0 OK, or len=4 byte with ERRNO
                if ((len==0) || (len==4)) {
                    retval=0;
                }
            }
            else {
                // command - minimum 4 addr + 1 byte + 1 LDVC
                if (len>=6) {
                    // maximum 4 addr + UAIOXCMD_MAXDATA bytes + 1 LDVC
                    if (len<=(UAIOXCMD_MAXDATA+5)) {
                        // no alignment requirements
                        retval=0;
                    }
                }
            }
            break;
            // for this group, we could check that the response N-bytes
            //  matches what the command asked for... but we don't
            //   it just assumes any non-zero N is good unless it's too big
        case UAIOXCMD_RDLDVC:
            // 0x07
            if (msk) {
                // response somewhat arbitrary N
                if ((len) && (len<=UAIOXCMD_MAXDATA)) {
                    retval=0;
                }
            }
            else {
                // command fixed -- needs 4 addr + u16 numbytes + 1 LDVC
                if (len==7) {
                    retval=0;
                }
            }
            break;
        case UAIOXCMD_FNCKEY:
            // 0x09
            if (msk) {
                // response len 4 means ERRNO=error
                //          len 12 means ERRNO=0 with 64 bit retval
                if ((len==4) || (len==12)) {
                    retval=0;
                }
            }
            else {
                // fixed -- 10
                if (len==10) {
                    retval=0;
                }
            }
            break;
        default:
            // unknown - not implemented defaults error
            // case UAIOXCMD_HRTBT: - obsolete
            // 0x08
            // case UAIOXCMD_CONSOLE: - not supported over comiox
            // 0x0A
            // case UAIOXCMD_OFILE: - not supported over comiox
            // 0x0B
            retval=1;
            break;
    }
    return(retval);
}

u32 comiox_rsphdrchk(void) {
    // returns error true
    //  check that arriving response header corellates with expected
    lnklst* head;
    uaio_xchg* xchg;
    u32 retval=0;
    head=&(ftuptr->cmtachn);
    xchg=(uaio_xchg*)(head->rvrs);
    if (((lnklst*)xchg)==head) {
        // when list empty, we've got a problem
        //  the arriving response doesn't correclate to any command
        //   signal and error and exit
        retval=1;
    }
    else {
        if (((ftuptr->cmuhdr[0])&0x7F)==(xchg->cmd)) {
            // response type is correct for .cmd
            if ((ftuptr->cmuhdr[1])==(xchg->seq)) {
                // .seq match command
                //  we haven't confrimed that response seq == .cmtack
                //   but that shouldn't be necessary
                //    just bump .cmtack
                ftuptr->cmtack=(ftuptr->cmtack)+1;
                // we can advance .flg state - the reply header is received
                xchg->flg=FLG_uaiox_rhdr;
                // for commands the request reply payload (example read bytes)
                //  we could verify that response N bytes == request N bytes
                //   but if we passed this much of the protocol
                //    I think it's safe to assume they won't be different
                // so exit success...
            }
            else {
                // sequence number mismatch
                retval=1;
            }
        }
        else {
            // response doesn't match .cmd
            retval=1;
        }
    }
    return(retval);
}

void comiox_rspnullpay(void) {
    // termination of a response without payload
    lnklst* head;
    uaio_xchg* xchg;
    // we know the chain is not empty
    head=&(ftuptr->cmtachn);
    xchg=(uaio_xchg*)(head->rvrs);
    // drop element from cmtachn
    lnklst_drop((lnklst*)xchg);
    // mark xchg completed
    xchg->flg=FLG_uaiox_done;
    // optional parent signalling
    uaiox_SYSresponse(xchg);
}

u32 comiox_rspdata(void) {
    // dispatch response payload
    // returns 0 - error xchg->err set accordingly
    //              else output size, total bytes
    //           - the method varies depending on command
    u32 size;
    u32 scratch;
    u32 retval;
    lnklst* head;
    bptrqueue* que;
    uaio_xchg* xchg;
    // we know the chain is not empty
    head=&(ftuptr->cmtachn);
    xchg=(uaio_xchg*)(head->rvrs);
    // we know size is non-zero
    size=ftuptr->cmrscan;
    que=ftuptr->cmrxque;
    // use xchg->cmd to be 0 referenced (rather than response .cmd with 0x80 offset)
    switch (xchg->cmd) {
        case 1:
            // read byte(s)
        case 3:
            // read word(s)
        case 5:
            // read quad(s)
            // destination address from host
            bptrqueue_rd(que,(char*)(xchg->ahst),size);
            retval=size;
            break;
        case 6:
            // write logical device
            //  this only returns payload when reporting ERRNO (4 bytes)
            //   to discard, dump the bytes to xchg.spr32
            //    so owner can optionally observe it
            bptrqueue_rd(que,(char*)(&(xchg->spr32)),size);
            // we don't really care what the ERRNO is here, but we have to mark
            //  an error return
            xchg->err=ERR_uaiox_reterr;
            // this is done at higher layer
            //  xchg->flg=FLG_uaiox_done;
            // it's still proper signalling of complete transaction
            //  so return size because protocol is still intact
            retval=size;
            break;
        case 7:
            // read logical device
        case 9:
            // fk()
            //   the first quad is 0 (OK) or ERRNO
            if (size>=4) {
                scratch=4;
            }
            else {
                scratch=size;
            }
            // retval is always full size;
            retval=size;
            size-=scratch;
            //   to discard, dump the bytes to xchg.spr32
            //    so owner can optionally observe it
            xchg->spr32=0;
            bptrqueue_rd(que,(char*)(&(xchg->spr32)),scratch);
            // if there is anything left to unload, it's the OK payload
            // destination address from host (0 size is OK)
            bptrqueue_rd(que,(char*)(xchg->ahst),size);
            // if signalling error indicator shows ERRNO
            //  then mark error termination
            if (xchg->spr32) {
                xchg->err=ERR_uaiox_reterr;
            }
            break;
        default:
            // return error - unknown command
            //
            // these should never have resp payload
            //  and shouldn't get here by definition
            // case 0:          // write byte(s)
            // case 2:          // write word(s)
            // case 4:          // write quad(s)
            // case 8:          // heartbeat xchg
            //
            // these are not supported under comiox
            // case 10:         // poll console
            // case 11:         // poll ofile
            //
            retval=0;  // return size unused on link crash
            break;
    }
    return(retval);
}

u32 comiox_sanechk(void) {
    // returns error true (unreported) when command sanity check fails
    //  assume fail
    u32 cmd;
    u32 aoff;
    char* dptr;
    u32 retval=1;
    cmd=(u32)(ftuptr->cmuhdr[0]);
    dptr=(char*)comioxbuff;
    switch (cmd) {
        case UAIOXCMD_WRBYTE:
            // 0x00
            // the address does not need to be aligned
            aoff=*((u32*)dptr);
            // logical memory page is high byte of address
            ftuptr->cmlmempg=aoff>>24;
            aoff=aoff&0x00FFFFFF;
            // actual data length
            ftuptr->cmdlen=(ftuptr->cmrscan)-4;
            // get actual data pointer
            ftuptr->cmdatptr=uaiox_lmembase(ftuptr->cmlmempg);
            if (ftuptr->cmdatptr) {
                // data pointer is valid, adjust for offset
                ftuptr->cmdatptr=(ftuptr->cmdatptr)+aoff;
                // all OK
                retval=0;
            }
            // else return default error
            break;
        case UAIOXCMD_WRWORD:
            // 0x02
            // the address needs to be word aligned
            aoff=*((u32*)dptr);
            if (!(aoff&0x01)) {
                // address is aligned
                // logical memory page is high byte of address
                ftuptr->cmlmempg=aoff>>24;
                aoff=aoff&0x00FFFFFF;
                // actual data length
                ftuptr->cmdlen=(ftuptr->cmrscan)-4;
                // get actual data pointer
                ftuptr->cmdatptr=uaiox_lmembase(ftuptr->cmlmempg);
                if (ftuptr->cmdatptr) {
                    // data pointer is valid, adjust for offset
                    ftuptr->cmdatptr=(ftuptr->cmdatptr)+aoff;
                    // all OK
                    retval=0;
                }
                // else return default error
            }
            // else default fail on non-aligned
            break;
        case UAIOXCMD_WRQUAD:
            // 0x04
            // the address needs to be quad aligned
            aoff=*((u32*)dptr);
            if (!(aoff&0x03)) {
                // address is aligned
                // logical memory page is high byte of address
                ftuptr->cmlmempg=aoff>>24;
                aoff=aoff&0x00FFFFFF;
                // actual data length
                ftuptr->cmdlen=(ftuptr->cmrscan)-4;
                // get actual data pointer
                ftuptr->cmdatptr=uaiox_lmembase(ftuptr->cmlmempg);
                if (ftuptr->cmdatptr) {
                    // data pointer is valid, adjust for offset
                    ftuptr->cmdatptr=(ftuptr->cmdatptr)+aoff;
                    // all OK
                    retval=0;
                }
                // else return default error
            }
            // else default fail on non-aligned
            break;
        case UAIOXCMD_RDBYTE:
            // 0x01
            // the address does not need to be aligned
            aoff=*((u32*)dptr);
            // actual data length
            dptr+=4;
            ftuptr->cmdlen=*((u16*)dptr);
            // logical memory page is high byte of address
            ftuptr->cmlmempg=aoff>>24;
            aoff=aoff&0x00FFFFFF;
            // get actual data pointer
            ftuptr->cmdatptr=uaiox_lmembase(ftuptr->cmlmempg);
            if (ftuptr->cmdatptr) {
                // data pointer is valid, adjust for offset
                ftuptr->cmdatptr=(ftuptr->cmdatptr)+aoff;
                // all OK
                retval=0;
            }
            // else return default error
            break;
        case UAIOXCMD_RDWORD:
            // 0x03
            // the address needs to be word aligned
            aoff=*((u32*)dptr);
            if (!(aoff&0x01)) {
                // address is aligned
                // actual data length
                dptr+=4;
                ftuptr->cmdlen=*((u16*)dptr);
                // logical memory page is high byte of address
                ftuptr->cmlmempg=aoff>>24;
                aoff=aoff&0x00FFFFFF;
                // get actual data pointer
                ftuptr->cmdatptr=uaiox_lmembase(ftuptr->cmlmempg);
                if (ftuptr->cmdatptr) {
                    // data pointer is valid, adjust for offset
                    ftuptr->cmdatptr=(ftuptr->cmdatptr)+aoff;
                    // all OK
                    retval=0;
                }
                // else return default error
            }
            // else default fail on non-aligned
            break;
        case UAIOXCMD_RDQUAD:
            // 0x05
            // the address needs to be quad aligned
            aoff=*((u32*)dptr);
            if (!(aoff&0x03)) {
                // address is aligned
                // actual data length
                dptr+=4;
                ftuptr->cmdlen=*((u16*)dptr);
                // logical memory page is high byte of address
                ftuptr->cmlmempg=aoff>>24;
                aoff=aoff&0x00FFFFFF;
                // get actual data pointer
                ftuptr->cmdatptr=uaiox_lmembase(ftuptr->cmlmempg);
                if (ftuptr->cmdatptr) {
                    // data pointer is valid, adjust for offset
                    ftuptr->cmdatptr=(ftuptr->cmdatptr)+aoff;
                    // all OK
                    retval=0;
                }
                // else return default error
            }
            // else default fail on non-aligned
            break;
        case UAIOXCMD_WRLDVC:
            // 0x06 - implicit byte action
            // logical device is last data byte
            ftuptr->cmldvc=*((u8*)(dptr+(((u32)(ftuptr->cmrscan))-1)));
            if ((ftuptr->cmldvc)<UAIOX_LDVC_BND) {
                // logical device is legal
                // subsequent logical device handling may make use of
                //  ftuptr->cmlmempg and ftuptr->cmdatptr
                // the address has no alignment checks
                ftuptr->cmldva=*((u32*)dptr);
                // actual data length
                ftuptr->cmdlen=(ftuptr->cmrscan)-5;
                // all OK
                retval=0;
            }
            // else return default error
            break;
        case UAIOXCMD_RDLDVC:
            // 0x07 - implicit byte action
            // logical device is last data byte
            ftuptr->cmldvc=*((u8*)(dptr+(((u32)(ftuptr->cmrscan))-1)));
            if ((ftuptr->cmldvc)<UAIOX_LDVC_BND) {
                // logical device is legal
                // subsequent logical device handling may make use of
                //  ftuptr->cmlmempg and ftuptr->cmdatptr
                // the address has no alignment checks
                ftuptr->cmldva=*((u32*)dptr);
                // actual data length
                dptr+=4;
                ftuptr->cmdlen=*((u16*)dptr);
                // all OK
                retval=0;
            }
            // else return default error
            break;
        case UAIOXCMD_FNCKEY:
            // 0x09
            // we know the command is the right length with 0x0A bytes
            // just get the data off the I/O buffer
            //  and transfer to
            //    char*   datptr; - 8 byte / s64 args
            //    u16     dlen;
            memcpy((void*)(&(ftuptr->cmdatptr)),(void*)dptr,0x08);
            dptr+=8;
            ftuptr->cmdlen=*((u16*)dptr);
            // that leaves the 8 bytes datptr up to 64 bit s64 FNC# argument(s)
            //  and dlen=#FNC
            if ((ftuptr->cmdlen)<UAIOX_FK_BND) {
                // generically, we don't have overall rules to check argument values
                //  against FNC# operations
                //   execution of the FNC# has to make that determination
                // all OK
                retval=0;
            }
            // else return default error
            break;
            //
            // this is header only command that should never get here
            //  because it bypasses CMDST_WTPAYLD and CMDST_SANITY
            // case UAIOXCMD_HRTBT:
            //    // 0x08
        default:
            // unknown - not implemented defaults error
            retval=1;
            break;
    }
    return(retval);
}

//------------------------------------------------------------------------------

void comiox_wrbyte(void) {
    char* src;
    char* dst;
    u32 n;
    src=((char*)comioxbuff)+4;
    dst=(char*)(ftuptr->cmdatptr);
    n=(u32)(ftuptr->cmdlen);
    while (n) {
        *dst=*src;
        dst++;
        src++;
        n--;
    }
    // set response payload length
    *((u16*)(&(ftuptr->cmuhdr[2])))=0;
    // ready to deploy
    ftuptr->cmrstate=CMDST_REPLY;
}

void comiox_rdbyte(void) {
    char* src;
    char* dst;
    u32 n;
    u32 s;
    dst=(char*)comioxbuff;
    src=(char*)(ftuptr->cmdatptr);
    n=(u32)(ftuptr->cmdlen);
    s=n;
    while (s) {
        *dst=*src;
        dst++;
        src++;
        s--;
    }
    // set response payload length
    *((u16*)(&(ftuptr->cmuhdr[2])))=(u16)n;
    // ready to deploy
    ftuptr->cmrstate=CMDST_REPLY;
}

void comiox_wrword(void) {
    u16* src;
    u16* dst;
    u32 n;
    src=(u16*)(((char*)comioxbuff)+4);
    dst=(u16*)(ftuptr->cmdatptr);
    n=((u32)(ftuptr->cmdlen))>>1;
    while (n) {
        *dst=*src;
        dst++;
        src++;
        n--;
    }
    // set response payload length
    *((u16*)(&(ftuptr->cmuhdr[2])))=0;
    // ready to deploy
    ftuptr->cmrstate=CMDST_REPLY;
}

void comiox_rdword(void) {
    u16* src;
    u16* dst;
    u32 n;
    u32 s;
    dst=(u16*)comioxbuff;
    src=(u16*)(ftuptr->cmdatptr);
    n=(u32)(ftuptr->cmdlen);
    s=n>>1;
    while (s) {
        *dst=*src;
        dst++;
        src++;
        s--;
    }
    // set response payload length
    *((u16*)(&(ftuptr->cmuhdr[2])))=(u16)n;
    // ready to deploy
    ftuptr->cmrstate=CMDST_REPLY;
}

void comiox_wrquad(void) {
    u32* src;
    u32* dst;
    u32 n;
    src=(u32*)(((char*)comioxbuff)+4);
    dst=(u32*)(ftuptr->cmdatptr);
    n=((u32)(ftuptr->cmdlen))>>2;
    while (n) {
        *dst=*src;
        dst++;
        src++;
        n--;
    }
    // set response payload length
    *((u16*)(&(ftuptr->cmuhdr[2])))=0;
    // ready to deploy
    ftuptr->cmrstate=CMDST_REPLY;
}

void comiox_rdquad(void) {
    u32* src;
    u32* dst;
    u32 n;
    u32 s;
    dst=(u32*)comioxbuff;
    src=(u32*)(ftuptr->cmdatptr);
    n=(u32)(ftuptr->cmdlen);
    s=n>>2;
    while (s) {
        *dst=*src;
        dst++;
        src++;
        s--;
    }
    // set response payload length
    *((u16*)(&(ftuptr->cmuhdr[2])))=(u16)n;
    // ready to deploy
    ftuptr->cmrstate=CMDST_REPLY;
}

void comiox_wrldvc(void) {
    char* src;
    u32 n;
    u32 dvc;
    u32 dva;
    src=((char*)comioxbuff)+4;
    n=(u32)(ftuptr->cmdlen);
    dvc=(u32)(ftuptr->cmldvc);
    dva=ftuptr->cmldva;
    //
    // set response payload length - default (-1), waiting to get filled in
    *((u16*)(&(ftuptr->cmuhdr[2])))=0xFFFF;
    // await response
    ftuptr->cmrstate=CMDST_EXEDONE;
    //
    // given this information, the general approach is to deplay a message
    //  to the appropriate device driver to execute the write
    //
    // the device driver should do one of the follwing, either directly
    //  or by messaging back to the sender thread
    //
    // return OK
    //    -- set u16 at cmioxptr->cmuhdr[2] = 0x0000
    //
    // return ERRNO
    //    -- set quad at comioxbuff = ERRNO
    //    -- set u16 at cmioxptr->cmuhdr[2] = 0x0004
    //
    // so here deploy what we know to logical device handler
    switch (dvc) {

        // QQQQQ
        // known logical device dispatches go here ...

        default:
            // VERBOSE - issue console message (optional)
            // ucnsl_ldvc_fl_acc(
            //    (u32)(utpptr->uhdr[0]),    // cmdnum
            //    (u32)(utpptr->ldvc),       // dvc
            //    (u32)(utpptr->ldva));      // addr
            // unknown device must fail - send ERRNO
            *((u32*)comioxbuff)=UAERR_LDVC_UNKN;
            *((u16*)(&(ftuptr->cmuhdr[2])))=4;
            // so from the CMDST_EXEDONE service -- this is already done
            break;
    }
}

void comiox_rdldvc(void) {
    char* dst;
    u32 n;
    u32 dvc;
    u32 dva;
    dst=(char*)comioxbuff;
    n=(u32)(ftuptr->cmdlen);
    dvc=(u32)(ftuptr->cmldvc);
    dva=ftuptr->cmldva;
    //
    // set response payload length - default (-1), waiting to get filled in
    *((u16*)(&(ftuptr->cmuhdr[2])))=0xFFFF;
    // await response
    ftuptr->cmrstate=CMDST_EXEDONE;
    //
    // given this information, the general approach is to deplay a message
    //  to the appropriate device driver to execute the write
    //
    // the device driver should be one of the follwing, either directly
    //  or by messaging back to the sender thread
    //
    // return ERRNO
    //    -- set quad at dst==comioxbuff = ERRNO
    //    -- set u16 at cmioxptr->cmuhdr[2] = 0x0004
    //
    // return all read data
    //    -- set quad at dst==comioxbuff = 0x00000000 (OK) followed by n bytes
    //    -- set u16 at cmioxptr->cmuhdr[2] = (4+n)
    //
    // so here deploy what we know to logical device handler
    switch (dvc) {

        // QQQQQ
        // known logical device dispatches go here ...

        default:
            // VERBOSE - issue console message (optional) - obsolete implementation
            // ucnsl_ldvc_fl_acc(
            //    (u32)(utpptr->uhdr[0]),    // cmdnum
            //    (u32)(utpptr->ldvc),       // dvc
            //    (u32)(utpptr->ldva));      // addr
            // unknown device must fail - send ERRNO
            *((u32*)comioxbuff)=UAERR_LDVC_UNKN;
            *((u16*)(&(ftuptr->cmuhdr[2])))=4;
            // so from the CMDST_EXEDONE service -- this is already done
            break;
    }
}

void comiox_fnckey(void) {
    u32 fnc;
    char* arg;
    u32 err;
    u32 trm;
    s64 retval;
    char* dst;
    fnc=ftuptr->cmdlen;
    // conceptually function key takes s64 arg and returns s64
    // s64 FNC#(s64* arg)
    //  though arg may also be interpretted as (u8*)arg as pointer to 8 bytes
    arg=(char*)(&(ftuptr->cmdatptr));
    dst=(char*)comioxbuff;
    //
    // set response payload length - default (-1), waiting to get filled in
    *((u16*)(&(ftuptr->cmuhdr[2])))=0xFFFF;
    // await response
    ftuptr->cmrstate=CMDST_EXEDONE;
    //
    // given this information, the general approach is to deplay a message
    //  to the appropriate driver to execute the function, or do it directly here
    //
    // the handler should one of the follwing, either directly
    //  or by messaging back to the sender thread
    //
    // return ERRNO
    //    -- set quad at dst==comioxbuff = ERRNO
    //    -- set u16 at cmioxptr->cmuhdr[2] = 0x0004
    //
    // return s64 return arg
    //    -- set quad at dst==comioxbuff = 0x00000000 (OK) followed by 8 bytes
    //    -- set u16 at cmioxptr->cmuhdr[2] = 0x000C
    //
    // so here deploy what we know to messaged thread handler, or complete locally
    err=0;
    trm=0;
    retval=0;
    // retval.lo=0;
    // retval.hi=0;
    switch (fnc) {
        // any FNC# may hand off to another thread, and leave err/trm=0
        //  or may set err (no data return)
        //  or may set trm and supply retval locally

        // known function key dispatches go here ...

        case 0x0001:
            *((u16*)(&(ftuptr->cmuhdr[2])))=tst_f0001ky(arg,dst);
            break;

        case 0x0002:
            *((u16*)(&(ftuptr->cmuhdr[2])))=tst_f0002ky(arg,dst);
            break;

        case 0x0010:
            // image mode at HDMI
            *((u16*)(&(ftuptr->cmuhdr[2])))=fnc_0010_ky(arg,dst);
            break;

        case 0x0011:
            // Goes to the Commander
            *((u16*)(&(ftuptr->cmuhdr[2])))=fnc_0011_ky(arg, dst);
            break;

        default:
            // VERBOSE - issue console message (optional)
            // ucnsl_fl_fnckey(
            //    fnc,                      // fnum
            //    *((u32*)arg),             // arglo
            //    *((u32*)(arg+4)));        // arghi
            // unknown function key must fail - send ERRNO
            err=UAERR_FNC_UNKN;
            break;

    }
    if (err) {
        *((u32*)dst)=err;
        *((u16*)(&(ftuptr->cmuhdr[2])))=4;
        // so from the CMDST_EXEDONE service -- this is already done
    }
    if (trm) {
        *((u32*)dst)=0;
        dst+=4;
        *((s64*)dst)=retval;
        // *((u32*)dst)=retval.lo;
        // *dst+=4;
        // *((s32*)dst)=retval.hi;
        *((u16*)(&(ftuptr->cmuhdr[2])))=12;
        // so from the CMDST_EXEDONE service -- this is already done
    }
}

//------------------------------------------------------------------------------

u32 comiox_cmdexec(void) {
    // returns error true (unreported) if insurmountable error detected
    //  but execution can "pass" and report error across the link in some cases
    u32 cmd;
    u32 retval=1;
    cmd=(u32)(ftuptr->cmuhdr[0]);
    switch (cmd) {
        case (UAIOXCMD_WRBYTE):
            // 0x00
            comiox_wrbyte();
            retval=0;
            break;
        case (UAIOXCMD_RDBYTE):
            // 0x01
            comiox_rdbyte();
            retval=0;
            break;
        case (UAIOXCMD_WRWORD):
            // 0x02
            comiox_wrword();
            retval=0;
            break;
        case (UAIOXCMD_RDWORD):
            // 0x03
            comiox_rdword();
            retval=0;
            break;
        case (UAIOXCMD_WRQUAD):
            // 0x04
            comiox_wrquad();
            retval=0;
            break;
        case (UAIOXCMD_RDQUAD):
            // 0x05
            comiox_rdquad();
            retval=0;
            break;
        case (UAIOXCMD_WRLDVC):
            // 0x06
            comiox_wrldvc();
            retval=0;
            break;
        case (UAIOXCMD_RDLDVC):
            // 0x07
            comiox_rdldvc();
            retval=0;
            break;
        case (UAIOXCMD_FNCKEY):
            // 0x09
            comiox_fnckey();
            retval=0;
            break;
        default:
            // unknown command should never get here
            // default exit error true
            // case (UAIOXCMD_HRTBT):
            // 0x08
            // case (UAIOXCMD_CONSOLE):
            // 0x0A
            // case (UAIOXCMD_OFILE):
            // 0x0B
            // this is header only command whose data length is already 0
            //  not supported in this version
            break;

    }
    return(retval);
}

u32 comiox_rpath(u32* rhdwy,u32* thdwy) {
    // returns error TRUE - protocol gets externally reset
    u32 done;
    u8 cmst;
    u32 error;
    u32 size;
    error=0;
    done=0;
    while ((!done) && (!error)) {
        cmst=ftuptr->cmrstate;
        switch (cmst) {
            case CMDST_HEADER:
                // implicitly cmrscan=0, waiting for header
                if ((*rhdwy)>=4) {
                    // we can pull a command header, and reduce avail accordingly
                    bptrqueue_rd(ftuptr->cmrxque,(char*)(&(ftuptr->cmuhdr[0])),4);
                    *rhdwy=(*rhdwy)-4;
                    // refresh
                    ftuptr->rfrshq0=(ftuptr->rfrshq0)+4;
                    // VERBOSITY - debug
                    // sprintf(utppb_ln,"comiox_rpath() sees header %08X\n",
                    //                  *((u32*)(&(ftuptr->cmuhdr[0]))));
                    // utp_dbgoutput(utppb_ln);
                    // is known command?
                    if (((ftuptr->cmuhdr[0])&0x7F)<COMIOXCMD_BOUND) {
                        // it looks legal
                        // assume the .len field is valid, no matter what it is
                        //  we may use .uhdr for response generation, so pull out
                        //   the length
                        ftuptr->cmrscan=*((u16*)(&(ftuptr->cmuhdr[2])));
                        error=comiox_lenchk((u32)(ftuptr->cmuhdr[0]),(u32)(ftuptr->cmrscan));
                        if (!error) {
                            // we're OK with the length as reported
                            if ((ftuptr->cmuhdr[0])&0x80) {
                                // processing response
                                error=comiox_rsphdrchk();
                                // VERBOSITY - debug
                                // sprintf(utppb_ln,"comiox_rpath() - comiox_rsphdrchk() returns %02X\n",
                                //                  error);
                                // utp_dbgoutput(utppb_ln);
                                if (!error) {
                                    // this response header matches the xchg at head of ack chain
                                    if (ftuptr->cmrscan) {
                                        // we have to wait on non-zero length payload to proceed
                                        //  (not done yet)
                                        ftuptr->cmrstate=CMDST_WTRPAY;
                                    }
                                    else {
                                        // zero length means this response needs header only
                                        //  and that's generally assumed sane
                                        ftuptr->cmrstate=CMDST_RZERO;
                                    }
                                }
                                // else exit with protocol error marked
                            }
                            else {
                                // processing command
                                if (ftuptr->cmrscan) {
                                    // we have to wait on non-zero length payload to fully assess
                                    //  (not done yet)
                                    ftuptr->cmrstate=CMDST_WTPAYLD;
                                }
                                else {
                                    // zero length means this command needs header only
                                    //  advance to processing state, but not done
                                    //   (a header only command is generally assumed sane
                                    //     and we can simultaneously bypass CMDST_SANITY)
                                    ftuptr->cmrstate=CMDST_EXEC;
                                }
                            }
                        }
                    }
                    else {
                        // unknown command -- echo error?
                        error=1;
                    }
                }
                else {
                    // insufficient data to pull a header
                    // let it slide to next pass
                    done=1;
                }
                break;
            case CMDST_WTPAYLD:
                // only commands enter this state
                // implicitly cmrscan is the size of payload we're waiting on
                if ((*rhdwy)>=(ftuptr->cmrscan)) {
                    // pull the indicated payload
                    bptrqueue_rd(ftuptr->cmrxque,(char*)comioxbuff,(ftuptr->cmrscan));
                    *rhdwy=(*rhdwy)-(ftuptr->cmrscan);
                    // refresh
                    ftuptr->rfrshq0=(ftuptr->rfrshq0)+(ftuptr->cmrscan);
                    ftuptr->cmrstate=CMDST_SANITY;
                }
                else {
                    // we'll have to try again next pass
                    done=1;
                }
                break;
            case CMDST_SANITY:
                // only commands enter this state after their (non-0)
                //  payload has been read into comioxrbuff[]
                error=comiox_sanechk();
                if (!error) {
                    // everything about the payload appears properly formatted
                    //  proceed to the execution phase, though we're not done yet
                    ftuptr->cmrstate=CMDST_EXEC;
                }
                // else exit now on indicated error - debug msg?
                break;
            case CMDST_EXEC:
                // only commands enter this state
                // when we get to the execution phase,
                //  direct execution commands fill in resposnse payload at comioxbuff
                //   and appropriately set data length at cmioxptr->cmuhdr[2..3]
                //    this is the standard output format, that allows state branching
                //     directly to CMDST_REPLY
                //      -- all the local memory base read/write commands are in this category
                //  deferred execution commands -- all logical device commands are in this category
                //   they generally issue a message to some other thread
                //    and proceed to CMDST_EXEDONE to wait on the reply
                //     the other thread that accepts the command can fill in comioxbuff and
                //      cmioxptr->cmuhdr[2..3] or message this thread how to do it properly...
                //       but for this state machine, the general approach is to set cmioxptr->cmuhdr[2..3]
                //        data size to (-1), before enetering CMDST_EXEDONE,
                //         and just wait on it to become non-zero to ptoceed to CMDST_REPLY
                //  function key commands can be either direct execution or deferred execution
                //
                error=comiox_cmdexec();
                // on error, we'd get an error exit - debug msg?
                //  but otherwise, comiox_cmdexec() should have appropriately changed states
                //   and is now ready to wait on deferred result or direct output
                //    so there is implicit state transition here, and we're not done
                break;
            case CMDST_EXEDONE:
                // entry to this state MUST have set cmioxptr->cmuhdr[2..3] to (-1)
                //  wait on this to get filled in with the number of return
                //   payload bytes at comioxbuff
                if (!((*((u16*)(&(ftuptr->cmuhdr[2]))))==0xFFFF)) {
                    // output data has been placed - ready to proceed, but not done
                    ftuptr->cmrstate=CMDST_REPLY;
                }
                else {
                    // try again next pass
                    done=1;
                }
                break;
            case CMDST_REPLY:
                // use val to calculate total valid output
                size=(u32)(*((u16*)(&(ftuptr->cmuhdr[2]))));
                // we'll output header + comioxbuff
                size+=4;
                // is there space for it
                if ((*thdwy)>=size) {
                    // completely ready to deploy response
                    //  send the header - we'll be finishing up this command now
                    //   in the response header, .cmd is modified to indicate response
                    ftuptr->cmuhdr[0]=(ftuptr->cmuhdr[0])|0x80;
                    bptrqueue_wr((bptrqueue*)(ftuptr->cmtxque),(char*)(&(ftuptr->cmuhdr[0])),4);
                    size-=4;
                    *thdwy=(*thdwy)-4;
                    // send the data
                    if (size) {
                        bptrqueue_wr((bptrqueue*)(ftuptr->cmtxque),(char*)comioxbuff,size);
                        *thdwy=(*thdwy)-size;
                    }
                    // once output placed, we're ready to process next command
                    //  so we're not done
                    ftuptr->cmrstate=CMDST_HEADER;
                }
                else {
                    // forced to wait on output space
                    // try again next pass
                    done=1;
                }
                break;
            case CMDST_WTRPAY:
                // only responses enter this state
                // implicitly cmrscan is the size of payload we're waiting on
                if ((*rhdwy)>=(ftuptr->cmrscan)) {
                    // pull the indicated payload
                    size=comiox_rspdata();
                    // then size tokens MUST be refreshed
                    //  but if size is now 0, that signals a protocol violation
                    //   that warrants resynchronizing the link
                    if (!size) {
                        // protocol violation
                        error=1;
                    }
                    else {
                        // indicate what was consumed
                        *rhdwy=(*rhdwy)-size;
                        // refresh
                        ftuptr->rfrshq0=(ftuptr->rfrshq0)+size;
                        // the payload has been handled, so now we can terminate the xchg
                        //  as if it had a null payload
                        comiox_rspnullpay();
                        // loop to start
                        ftuptr->cmrstate=CMDST_HEADER;
                    }
                }
                else {
                    // we'll have to try again next pass
                    done=1;
                }
                break;
            case CMDST_RZERO:
                // only responses without payload enter this state
                comiox_rspnullpay();
                // loop to start
                ftuptr->cmrstate=CMDST_HEADER;
                break;
            default:
                // unknown state - something went really wrong
                //  resync the whole link
                // VERBOSITY - debug
                sprintf(utpdbgln,"Tp08 comiox_rpath() in unknown state 0x%02X\n",cmst);
                utp_dbgoutput(utpdbgln);
                error=1;
                break;
        }
    }
    // VERBOSITY - debug
    if (error) {
        utp_dbgoutput("Tp08 comiox_rpath() protocol error\n");
    }
    return(error);
}

u32 comiox_send(uaio_xchg* xchg,bptrquetx* que) {
    // returns 0 - error, unknown command
    //         else output size, total bytes
    //          - the method varies depending on command
    u32 dlen;
    u32 retval;
    switch (xchg->cmd) {
        case 0:
            // write byte(s)
        case 2:
            // write word(s)
        case 4:
            // write quad(s)
            // send the header
            bptrqueue_wr((bptrqueue*)que,(char*)(&(xchg->cmd)),4);
            dlen=xchg->len;
            retval=dlen+4;
            // 0x00 target address (write to)
            bptrqueue_wr((bptrqueue*)que,(char*)(&(xchg->atrg)),4);
            // that reduces dlen by 4...
            dlen-=4;
            // as the actual data length at host address .ahst
            bptrqueue_wr((bptrqueue*)que,(char*)(xchg->ahst),dlen);
            break;
        case 6:
            // write logical device
            // send the header
            bptrqueue_wr((bptrqueue*)que,(char*)(&(xchg->cmd)),4);
            dlen=xchg->len;
            retval=dlen+4;
            // 0x00 target address (write to)
            bptrqueue_wr((bptrqueue*)que,(char*)(&(xchg->atrg)),4);
            // that reduces dlen by 4...
            //  but reduce it by 5
            dlen-=5;
            // as the actual data length at host address .ahst
            bptrqueue_wr((bptrqueue*)que,(char*)(xchg->ahst),dlen);
            // the last byte appended is logical device from .spr16
            bptrqueue_wr((bptrqueue*)que,(char*)(&(xchg->spr16)),1);
            break;
        case 1:
            // read byte(s)
        case 3:
            // read word(s)
        case 5:
            // read quad(s)
            // send the header
            bptrqueue_wr((bptrqueue*)que,(char*)(&(xchg->cmd)),4);
            // dlen should be 0x06
            dlen=xchg->len;
            retval=dlen+4;
            // 0x00 target address (read from)
            bptrqueue_wr((bptrqueue*)que,(char*)(&(xchg->atrg)),4);
            // 0x04 read transfer size comes from .spr16
            bptrqueue_wr((bptrqueue*)que,(char*)(&(xchg->spr16)),2);
            break;
        case 7:
            // read logical device
            // send the header
            bptrqueue_wr((bptrqueue*)que,(char*)(&(xchg->cmd)),4);
            // dlen should be 0x07
            dlen=xchg->len;
            retval=dlen+4;
            // 0x00 target address (read from)
            bptrqueue_wr((bptrqueue*)que,(char*)(&(xchg->atrg)),4);
            // 0x04 read transfer size comes from .spr16
            bptrqueue_wr((bptrqueue*)que,(char*)(&(xchg->spr16)),2);
            // the last byte appended is logical device from .spr32
            bptrqueue_wr((bptrqueue*)que,(char*)(&(xchg->spr32)),1);
            break;
        case 9:
            // fk()
            // send the header
            bptrqueue_wr((bptrqueue*)que,(char*)(&(xchg->cmd)),4);
            // dlen should be 0x0A
            dlen=xchg->len;
            retval=dlen+4;
            // 0x00 s64 lo input arg
            bptrqueue_wr((bptrqueue*)que,(char*)(&(xchg->atrg)),4);
            // 0x04 s64 hi input arg
            bptrqueue_wr((bptrqueue*)que,(char*)(&(xchg->spr32)),4);
            // 0x08 FNC#
            bptrqueue_wr((bptrqueue*)que,(char*)(&(xchg->spr16)),2);
            break;
        default:
            // return error - unknown command
            //case 8:
            // heartbeat - obsolete
            // case 10:
            // poll console - not supported comiox
            // case 11:
            // poll ofile - not supported comiox
            retval=0;
            break;
    }
    return(retval);
}

u32 comiox_tpath(u32* thdwy) {
    // return error true on protocol failure
    lnklst* head;
    uaio_xchg* xchg;
    u32 size;
    u32 tval;
    u32 error;
    u32 done;
    error=0;
    // transmit what you can from txchn
    if ((*thdwy)>=4) {
        // need at least enough room for header only command
        head=&(ftuptr->cmtxchn);
        done=0;
        while (!done) {
            xchg=(uaio_xchg*)(head->rvrs);
            if (((lnklst*)xchg)==head) {
                // when list empty, we're done
                done=1;
            }
            else {
                // there is an xchg waiting to be sent
                // the size to send is payload size + header(4)
                size=(xchg->len)+4;
                if (size<=(*thdwy)) {
                    // there's room to send the command side of this xchg this pass
                    // we don't want to wrap sequence number
                    //  we limit the sequence number lead
                    tval=((ftuptr->cmtseq)-(ftuptr->cmtack))&0xFF;
                    if (tval<0xE0) {
                        // we know we can transmit this xchg, though the method varies
                        // assign a sequence number
                        xchg->seq=(ftuptr->cmtseq);
                        ftuptr->cmtseq=(ftuptr->cmtseq)+1;
                        // call output handler - get actual size sent
                        size=comiox_send(xchg,ftuptr->cmtxque);
                        if (size) {
                            // output went well, as planned
                            // drop element from txchn
                            lnklst_drop((lnklst*)xchg);
                            // but reconnect it at tachn
                            lnklst_fins(&(ftuptr->cmtachn),(lnklst*)xchg);
                            // mark sent flag
                            xchg->flg=FLG_uaiox_sent;
                            *thdwy=(*thdwy)-size;
                            if ((*thdwy)<4) {
                                // abort loop if not enough for header only command
                                done=1;
                            }
                        }
                        else {
                            // command actually got dropped -- unknown command
                            //  then *thdwy stays where it is
                            //   we didn't use this sequence number after all
                            ftuptr->cmtseq=(ftuptr->cmtseq)-1;
                            // drop element from txchn
                            lnklst_drop((lnklst*)xchg);
                            // set error
                            xchg->err=ERR_uaiox_unknwn;
                            // response length = 0
                            xchg->len=0;
                            // but otherwise mark xchg completed
                            xchg->flg=FLG_uaiox_done;
                            // optional parent signalling
                            uaiox_SYSresponse(xchg);
                        }
                    }
                    else {
                        // sequence leead too big - block sending more
                        done=1;
                    }
                }
                else {
                    // not enough headway,
                    //  can't send this xchg or any following it
                    done=1;
                }
            }
        }
    }
    // in this current form, nothing actually sets error
    return(error);
}

void comiox_prc(void) {
    u32 thdwy;
    u32 rhdwy;
    u32 error;
    // handle once per sweep
    if (ftuptr->comstate==COMXCHG) {
        // what to do when link up
        // check for tx room (usually true)
        thdwy=(u32)bptrqueue_space((bptrqueue*)(ftuptr->cmtxque));
        // ACK what you can from tachn and/or service incoming commands
        //  this way, responses get priority use of tx buffer before we
        //   issue our output commands (which could block responses)
        rhdwy=(u32)bptrqueue_avail(ftuptr->cmrxque);
        error=comiox_rpath(&rhdwy,&thdwy);
        if (!(error)) {
            error=comiox_tpath(&thdwy);
            if (error) {
                // protocol crash at comiox_tpath() - debug msg?
// QQQQQ
                utp_dbgoutput("Tp comiox_prc _tpath error\n");
                comstate_COMSYNC0();
                comiox_reset();
            }
        }
        else {
            // protocol crash at comiox_rpath() - debug msg?
// QQQQQ
            utp_dbgoutput("Tp comiox_prc _rpath error\n");
            comstate_COMSYNC0();
            comiox_reset();
        }
    }
    else {
        // what to do when link intermediate or down
        comiox_reset();
    }
}

//------------------------------------------------------------------------------

void comiox_msg_void(uaio_xchg* xchg) {
    // client calls this to submit void transaction to comiox
    //  what makes it void, is that it doesn't expect
    //   event signalling on completion
    xchg->retptr=(int*)0;
    issue_com_xchg(xchg);
}

u32 comiox_msg_xchg(uaio_xchg* xchg) {
    // returns error code
    // client calls this to submit transaction to comiox
    //   it suspends/waits on tranaction completion signalling through the event
    volatile int ret[2];
    // init "done"=0
    ret[0]=0;
    ret[1]=0;
    xchg->retptr=(int*)(&(ret[0]));
    issue_com_xchg(xchg);
    // now spin on "done"
    while (!(ret[0])) {
        sched_yield();
    }
    // and return the error code
    return((u32)ret[1]);
}

//------------------------------------------------------------------------------

u32 caio_rbbuf(u64 haddr,u32 taddr,u32 numbytes) {
    //             dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    // this form implicitly closed loop
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    uaio_xchg* xchg;
    u32 crashret;
    // need xchg carrier
    xchg=(uaio_xchg*)gblkalloc();
    if (xchg) {
        // have xchg carrier
        // .cmd for read
        xchg->cmd=UAIOXCMD_RDBYTE;
        // .seq gets filled in at xchg processing
        // .len represents address+u16 sizeof read
        //  but .spr16 is actual number of bytes to read
        xchg->len=6;
        xchg->spr16=numbytes;
        // .flg and .err init 0
        *((u16*)(&(xchg->flg)))=0;
        // .atrg set target address
        xchg->atrg=taddr;
        // .ahst is host data ptr (wipe high bytes)
        xchg->ahst=haddr;
        // issue it...
        crashret=comiox_msg_xchg(xchg);
        // any way out has to release the gblk
        gblkfree((void*)xchg);
    }
    else {
        // crash code has been set for app on NULL ptr
        crashret=ERR_uaiox_setup;
    }
    return(crashret);
}

void caio_rbbuf_ol(u64 haddr,u32 taddr,u32 numbytes) {
    //                 dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    // this form implicitly open loop
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    uaio_xchg* xchg;
    // wait for xchg carrier
    xchg=get_ol_carrier();
    // have xchg carrier
    // .cmd for read
    xchg->cmd=UAIOXCMD_RDBYTE;
    // .seq gets filled in at xchg processing
    // .len represents address+u16 sizeof read
    //  but .spr16 is actual number of bytes to read
    xchg->len=6;
    xchg->spr16=numbytes;
    // .flg and .err init 0
    *((u16*)(&(xchg->flg)))=0;
    // .atrg set target address
    xchg->atrg=taddr;
    // .ahst is host data ptr (wipe high bytes)
    xchg->ahst=haddr;
    // issue it...
    comiox_msg_void(xchg);
}

u32 caio_rbbuf_lrg(u64 haddr,u32 taddr,u32 numbytes) {
    //                  dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    //
    // arbitrary (or large) buffer version
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    u32 crashret=0;
    // send all sub-buffers open loop
    //  only the final chunk needs to be closed loop
    if (numbytes) {
        while (numbytes>UAIOXCMD_MAXDATA) {
            caio_rbbuf_ol(haddr,taddr,UAIOXCMD_MAXDATA);
            haddr+=(u64)UAIOXCMD_MAXDATA;
            taddr+=UAIOXCMD_MAXDATA;
            numbytes-=UAIOXCMD_MAXDATA;
        }
        // last chunk
        crashret=caio_rbbuf(haddr,taddr,numbytes);
    }
    return(crashret);
}

u32 caio_rwbuf(u64 haddr,u32 taddr,u32 numbytes) {
    //               dst       src
    // assumes (target) taddr - aligned
    // assumes (host) haddr - no alignment
    // this form implicitly closed loop
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    uaio_xchg* xchg;
    u32 crashret;
    // need xchg carrier
    xchg=(uaio_xchg*)gblkalloc();
    if (xchg) {
        // have xchg carrier
        // .cmd for read
        xchg->cmd=UAIOXCMD_RDWORD;
        // .seq gets filled in at xchg processing
        // .len represents address+u16 sizeof read
        //  but .spr16 is actual number of bytes to read
        xchg->len=6;
        xchg->spr16=numbytes;
        // .flg and .err init 0
        *((u16*)(&(xchg->flg)))=0;
        // .atrg set target address
        xchg->atrg=taddr;
        // .ahst is host data ptr (wipe high bytes)
        xchg->ahst=haddr;
        // issue it...
        crashret=comiox_msg_xchg(xchg);
        // any way out has to release the gblk
        gblkfree((void*)xchg);
    }
    else {
        // crash code has been set for app on NULL ptr
        crashret=ERR_uaiox_setup;
    }
    return(crashret);
}

void caio_rwbuf_ol(u64 haddr,u32 taddr,u32 numbytes) {
    //                  dst       src
    // assumes (target) taddr - aligned
    // assumes (host) haddr - no alignment
    // this form implicitly open loop
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    uaio_xchg* xchg;
    // wait for xchg carrier
    xchg=get_ol_carrier();
    // have xchg carrier
    // .cmd for read
    xchg->cmd=UAIOXCMD_RDWORD;
    // .seq gets filled in at xchg processing
    // .len represents address+u16 sizeof read
    //  but .spr16 is actual number of bytes to read
    xchg->len=6;
    xchg->spr16=numbytes;
    // .flg and .err init 0
    *((u16*)(&(xchg->flg)))=0;
    // .atrg set target address
    xchg->atrg=taddr;
    // .ahst is host data ptr (wipe high bytes)
    xchg->ahst=haddr;
    // issue it...
    comiox_msg_void(xchg);
}

u32 caio_rwbuf_lrg(u64 haddr,u32 taddr,u32 numbytes) {
    //                  dst       src
    // assumes (target) taddr - aligned
    // assumes (host) haddr - no alignment
    //
    // arbitrary (or large) buffer version
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    u32 crashret=0;
    // send all sub-buffers open loop
    //  only the final chunk needs to be closed loop
    if (numbytes) {
        while (numbytes>UAIOXCMD_MAXDATA) {
            caio_rwbuf_ol(haddr,taddr,UAIOXCMD_MAXDATA);
            haddr+=(u64)UAIOXCMD_MAXDATA;
            taddr+=UAIOXCMD_MAXDATA;
            numbytes-=UAIOXCMD_MAXDATA;
        }
        // last chunk
        crashret=caio_rwbuf(haddr,taddr,numbytes);
    }
    return(crashret);
}

u32 caio_rqbuf(u64 haddr,u32 taddr,u32 numbytes) {
    //             dst       src
    // assumes (target) taddr - aligned
    // assumes (host) haddr - no alignment
    // this form implicitly closed loop
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    uaio_xchg* xchg;
    u32 crashret;
    xchg=(uaio_xchg*)gblkalloc();
    if (xchg) {
        // have xchg carrier
        // .cmd for read
        xchg->cmd=UAIOXCMD_RDQUAD;
        // .seq gets filled in at xchg processing
        // .len represents address+u16 sizeof read
        //  but .spr16 is actual number of bytes to read
        xchg->len=6;
        xchg->spr16=numbytes;
        // .flg and .err init 0
        *((u16*)(&(xchg->flg)))=0;
        // .atrg set target address
        xchg->atrg=taddr;
        // .ahst is host data ptr (wipe high bytes)
        xchg->ahst=haddr;
        // issue it...
        crashret=comiox_msg_xchg(xchg);
        // any way out has to release the gblk
        gblkfree((void*)xchg);
    }
    else {
        // crash code has been set for app on NULL ptr
        crashret=ERR_uaiox_setup;
    }
    return(crashret);
}

void caio_rqbuf_ol(u64 haddr,u32 taddr,u32 numbytes) {
    //                 dst       src
    // assumes (target) taddr - aligned
    // assumes (host) haddr - no alignment
    // this form implicitly open loop
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    uaio_xchg* xchg;
    // wait for xchg carrier
    xchg=get_ol_carrier();
    // have xchg carrier
    // .cmd for read
    xchg->cmd=UAIOXCMD_RDQUAD;
    // .seq gets filled in at xchg processing
    // .len represents address+u16 sizeof read
    //  but .spr16 is actual number of bytes to read
    xchg->len=6;
    xchg->spr16=numbytes;
    // .flg and .err init 0
    *((u16*)(&(xchg->flg)))=0;
    // .atrg set target address
    xchg->atrg=taddr;
    // .ahst is host data ptr (wipe high bytes)
    xchg->ahst=haddr;
    // issue it...
    comiox_msg_void(xchg);
}

u32 caio_rqbuf_lrg(u64 haddr,u32 taddr,u32 numbytes) {
    //                 dst       src
    // assumes (target) taddr - aligned
    // assumes (host) haddr - no alignment
    //
    // arbitrary (or large) buffer version
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    u32 crashret=0;
    // send all sub-buffers open loop
    //  only the final chunk needs to be closed loop
    if (numbytes) {
        while (numbytes>UAIOXCMD_MAXDATA) {
            caio_rqbuf_ol(haddr,taddr,UAIOXCMD_MAXDATA);
            haddr+=(u64)UAIOXCMD_MAXDATA;
            taddr+=UAIOXCMD_MAXDATA;
            numbytes-=UAIOXCMD_MAXDATA;
        }
        // last chunk
        crashret=caio_rqbuf(haddr,taddr,numbytes);
    }
    return(crashret);
}

//------------------------------------------------------------------------------

u32 caio_wbbuf(u32 taddr,u64 haddr,u32 numbytes) {
    //             dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    // this form implicitly closed loop
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    uaio_xchg* xchg;
    u32 crashret;
    // need xchg carrier
    xchg=(uaio_xchg*)gblkalloc();
    if (xchg) {
        // have xchg carrier
        // .cmd for read
        xchg->cmd=UAIOXCMD_WRBYTE;
        // .seq gets filled in at xchg processing
        // .len represents address+val payload
        xchg->len=(numbytes+4);
        // .flg and .err init 0
        *((u16*)(&(xchg->flg)))=0;
        // .atrg set target address
        xchg->atrg=taddr;
        // .ahst is host data ptr (wipe high bytes)
        xchg->ahst=haddr;
        // issue it...
        crashret=comiox_msg_xchg(xchg);
        // any way out has to release the gblk
        gblkfree((void*)xchg);
    }
    else {
        // crash code has been set for app on NULL ptr
        crashret=ERR_uaiox_setup;
    }
    return(crashret);
}

void caio_wbbuf_ol(u32 taddr,u64 haddr,u32 numbytes) {
    //                 dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    // this form implicitly open loop
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    uaio_xchg* xchg;
    // wait for xchg carrier
    xchg=get_ol_carrier();
    // have xchg carrier
    // .cmd for read
    xchg->cmd=UAIOXCMD_WRBYTE;
    // .seq gets filled in at xchg processing
    // .len represents address+val payload
    xchg->len=(numbytes+4);
    // .flg and .err init 0
    *((u16*)(&(xchg->flg)))=0;
    // .atrg set target address
    xchg->atrg=taddr;
    // .ahst is host data ptr
    xchg->ahst=haddr;
    // issue it...
    comiox_msg_void(xchg);
}

u32 caio_wbbuf_lrg(u32 taddr,u64 haddr,u32 numbytes) {
    //                 dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    //
    // arbitrary (or large) buffer version
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    u32 crashret=0;
    // send all sub-buffers open loop
    //  only the final chunk needs to be closed loop
    if (numbytes) {
        while (numbytes>UAIOXCMD_MAXDATA) {
            caio_wbbuf_ol(taddr,haddr,UAIOXCMD_MAXDATA);
            haddr+=(u64)UAIOXCMD_MAXDATA;
            taddr+=UAIOXCMD_MAXDATA;
            numbytes-=UAIOXCMD_MAXDATA;
        }
        // last chunk
        crashret=caio_wbbuf(taddr,haddr,numbytes);
    }
    return(crashret);
}

u32 caio_wwbuf(u32 taddr,u64 haddr,u32 numbytes) {
    //             dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    // this form implicitly closed loop
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    uaio_xchg* xchg;
    u32 crashret;
    // need xchg carrier
    xchg=(uaio_xchg*)gblkalloc();
    if (xchg) {
        // have xchg carrier
        // .cmd for read
        xchg->cmd=UAIOXCMD_WRWORD;
        // .seq gets filled in at xchg processing
        // .len represents address+val payload
        xchg->len=(numbytes+4);
        // .flg and .err init 0
        *((u16*)(&(xchg->flg)))=0;
        // .atrg set target address
        xchg->atrg=taddr;
        // .ahst is host data ptr (wipe high bytes)
        xchg->ahst=haddr;
        // issue it...
        crashret=comiox_msg_xchg(xchg);
        // any way out has to release the gblk
        gblkfree((void*)xchg);
    }
    else {
        // crash code has been set for app on NULL ptr
        crashret=ERR_uaiox_setup;
    }
    return(crashret);
}

void caio_wwbuf_ol(u32 taddr,u64 haddr,u32 numbytes) {
    //                 dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    // this form implicitly open loop
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    uaio_xchg* xchg;
    // wait for xchg carrier
    xchg=get_ol_carrier();
    // have xchg carrier
    // .cmd for read
    xchg->cmd=UAIOXCMD_WRWORD;
    // .seq gets filled in at xchg processing
    // .len represents address+val payload
    xchg->len=(numbytes+4);
    // .flg and .err init 0
    *((u16*)(&(xchg->flg)))=0;
    // .atrg set target address
    xchg->atrg=taddr;
    // .ahst is host data ptr
    xchg->ahst=haddr;
    // issue it...
    comiox_msg_void(xchg);
}

u32 caio_wwbuf_lrg(u32 taddr,u64 haddr,u32 numbytes) {
    //                 dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    //
    // arbitrary (or large) buffer version
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    u32 crashret=0;
    // send all sub-buffers open loop
    //  only the final chunk needs to be closed loop
    if (numbytes) {
        while (numbytes>UAIOXCMD_MAXDATA) {
            caio_wwbuf_ol(taddr,haddr,UAIOXCMD_MAXDATA);
            haddr+=(u64)UAIOXCMD_MAXDATA;
            taddr+=UAIOXCMD_MAXDATA;
            numbytes-=UAIOXCMD_MAXDATA;
        }
        // last chunk
        crashret=caio_wwbuf(taddr,haddr,numbytes);
    }
    return(crashret);
}

u32 caio_wqbuf(u32 taddr,u64 haddr,u32 numbytes) {
    //             dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    // this form implicitly closed loop
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    uaio_xchg* xchg;
    u32 crashret;
    // need xchg carrier
    xchg=(uaio_xchg*)gblkalloc();
    if (xchg) {
        // have xchg carrier
        // .cmd for read
        xchg->cmd=UAIOXCMD_WRQUAD;
        // .seq gets filled in at xchg processing
        // .len represents address+val payload
        xchg->len=(numbytes+4);
        // .flg and .err init 0
        *((u16*)(&(xchg->flg)))=0;
        // .atrg set target address
        xchg->atrg=taddr;
        // .ahst is host data ptr (wipe high bytes)
        xchg->ahst=haddr;
        // issue it...
        crashret=comiox_msg_xchg(xchg);
        // any way out has to release the gblk
        gblkfree((void*)xchg);
    }
    else {
        // crash code has been set for app on NULL ptr
        crashret=ERR_uaiox_setup;
    }
    return(crashret);
}

void caio_wqbuf_ol(u32 taddr,u64 haddr,u32 numbytes) {
    //                 dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    // this form implicitly open loop
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    uaio_xchg* xchg;
    // wait for xchg carrier
    xchg=get_ol_carrier();
    // have xchg carrier
    // .cmd for read
    xchg->cmd=UAIOXCMD_WRQUAD;
    // .seq gets filled in at xchg processing
    // .len represents address+val payload
    xchg->len=(numbytes+4);
    // .flg and .err init 0
    *((u16*)(&(xchg->flg)))=0;
    // .atrg set target address
    xchg->atrg=taddr;
    // .ahst is host data ptr
    xchg->ahst=haddr;
    // issue it...
    comiox_msg_void(xchg);
}

u32 caio_wqbuf_lrg(u32 taddr,u64 haddr,u32 numbytes) {
    //                 dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    //
    // arbitrary (or large) buffer version
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    u32 crashret=0;
    // send all sub-buffers open loop
    //  only the final chunk needs to be closed loop
    if (numbytes) {
        while (numbytes>UAIOXCMD_MAXDATA) {
            caio_wqbuf_ol(taddr,haddr,UAIOXCMD_MAXDATA);
            haddr+=(u64)UAIOXCMD_MAXDATA;
            taddr+=UAIOXCMD_MAXDATA;
            numbytes-=UAIOXCMD_MAXDATA;
        }
        // last chunk
        crashret=caio_wqbuf(taddr,haddr,numbytes);
    }
    return(crashret);
}

//------------------------------------------------------------------------------

u32 caio_rbyte(u32 addr,u32* val) {
    // assumes (target) addr - no alignment
    // returns error code
    //         otherwise result at *val
    uaio_xchg* xchg;
    u32 crashret;
    // need xchg carrier
    xchg=(uaio_xchg*)gblkalloc();
    if (xchg) {
        // have xchg carrier
        // .cmd for read byte(s)
        xchg->cmd=UAIOXCMD_RDBYTE;
        // .seq gets filled in at xchg processing
        // .len represents address+u16 sizeof read
        //  but .spr16 is actual number of bytes to read
        xchg->len=6;
        xchg->spr16=1;
        // .flg and .err init 0
        *((u16*)(&(xchg->flg)))=0;
        // .atrg set target address
        xchg->atrg=addr;
        // .ahst is host data ptr (wipe high bytes)
        *val=0;
        xchg->ahst=(u64)val;
        // issue it...
        crashret=comiox_msg_xchg(xchg);
        // any way out has to release the gblk
        gblkfree((void*)xchg);
    }
    else {
        // crash code has been set for app on NULL ptr
        crashret=ERR_uaiox_setup;
    }
    return(crashret);
}

u32 caio_rword(u32 addr,u32* val) {
    // assumes (target) addr - no alignment
    // returns error code
    //         otherwise result at *val
    uaio_xchg* xchg;
    u32 crashret;
    // need xchg carrier
    xchg=(uaio_xchg*)gblkalloc();
    if (xchg) {
        // have xchg carrier
        // .cmd for read byte(s)
        xchg->cmd=UAIOXCMD_RDWORD;
        // .seq gets filled in at xchg processing
        // .len represents address+u16 sizeof read
        //  but .spr16 is actual number of bytes to read
        xchg->len=6;
        xchg->spr16=2;
        // .flg and .err init 0
        *((u16*)(&(xchg->flg)))=0;
        // .atrg set target address
        xchg->atrg=addr;
        // .ahst is host data ptr (wipe high bytes)
        *val=0;
        xchg->ahst=(u64)val;
        // issue it...
        crashret=comiox_msg_xchg(xchg);
        // any way out has to release the gblk
        gblkfree((void*)xchg);
    }
    else {
        // crash code has been set for app on NULL ptr
        crashret=ERR_uaiox_setup;
    }
    return(crashret);
}

u32 caio_rquad(u32 addr,u32* val) {
    // assumes (target) addr - no alignment
    // returns error code
    //         otherwise result at *val
    uaio_xchg* xchg;
    u32 crashret;
    // need xchg carrier
    xchg=(uaio_xchg*)gblkalloc();
    if (xchg) {
        // have xchg carrier
        // .cmd for read byte(s)
        xchg->cmd=UAIOXCMD_RDQUAD;
        // .seq gets filled in at xchg processing
        // .len represents address+u16 sizeof read
        //  but .spr16 is actual number of bytes to read
        xchg->len=6;
        xchg->spr16=4;
        // .flg and .err init 0
        *((u16*)(&(xchg->flg)))=0;
        // .atrg set target address
        xchg->atrg=addr;
        // .ahst is host data ptr (wipe high bytes)
        *val=0;
        xchg->ahst=(u64)val;
        // issue it...
        crashret=comiox_msg_xchg(xchg);
        // any way out has to release the gblk
        gblkfree((void*)xchg);
    }
    else {
        // crash code has been set for app on NULL ptr
        crashret=ERR_uaiox_setup;
    }
    return(crashret);
}

void caio_wbyte(u32 addr,u32 val) {
    // assumes (target) addr - no alignment
    //
    // this is the open loop void form invoked by rb(addr,data)
    // if you want the closed loop form, use wb(addr,data,...)
    //
    uaio_xchg* xchg;
    // wait for xchg carrier
    xchg=get_ol_carrier();
    // have xchg carrier
    // set .cmd for write byte(s)
    xchg->cmd=UAIOXCMD_WRBYTE;
    // .seq gets filled in at xchg processing
    // .len represents address+val payload
    xchg->len=5;
    // .flg and .err init 0
    *((u16*)(&(xchg->flg)))=0;
    // .atrg set target address
    xchg->atrg=addr;
    // .ahst is host data ptr (imported to structure)
    xchg->spr32=val;
    xchg->ahst=(u64)(&(xchg->spr32));
    // issue it...
    comiox_msg_void(xchg);
}

void caio_wword(u32 addr,u32 val) {
    // assumes (target) addr is word aligned
    //
    // this is the open loop void form invoked by rw(addr,data)
    // if you want the closed loop form, use ww(addr,data,...)
    //
    uaio_xchg* xchg;
    // wait for xchg carrier
    xchg=get_ol_carrier();
    // have xchg carrier
    // set .cmd for write word(s)
    xchg->cmd=UAIOXCMD_WRWORD;
    // .seq gets filled in at xchg processing
    // .len represents address+val payload
    xchg->len=6;
    // .flg and .err init 0
    *((u16*)(&(xchg->flg)))=0;
    // .atrg set target address
    xchg->atrg=addr;
    // .ahst is host data ptr (imported to structure)
    xchg->spr32=val;
    xchg->ahst=(u64)(&(xchg->spr32));
    // issue it...
    comiox_msg_void(xchg);
}

void caio_wquad(u32 addr,u32 val) {
    // assumes (target) addr is word aligned
    //
    // this is the open loop void form invoked by rw(addr,data)
    // if you want the closed loop form, use ww(addr,data,...)
    //
    uaio_xchg* xchg;
    // wait for xchg carrier
    xchg=get_ol_carrier();
    // have xchg carrier
    // set .cmd for write word(s)
    xchg->cmd=UAIOXCMD_WRQUAD;
    // .seq gets filled in at xchg processing
    // .len represents address+val payload
    xchg->len=8;
    // .flg and .err init 0
    *((u16*)(&(xchg->flg)))=0;
    // .atrg set target address
    xchg->atrg=addr;
    // .ahst is host data ptr (imported to structure)
    xchg->spr32=val;
    xchg->ahst=(u64)(&(xchg->spr32));
    // issue it...
    comiox_msg_void(xchg);
}

//------------------------------------------------------------------------------

u32 caio_rldvcbuf(u32 ldvc,u64 haddr,u32 taddr,u32 numbytes) {
    //                         dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    // this form implicitly closed loop
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    uaio_xchg* xchg;
    u32 crashret;
    // need xchg carrier
    xchg=(uaio_xchg*)gblkalloc();
    if (xchg) {
        // have xchg carrier
        // .cmd for read
        xchg->cmd=UAIOXCMD_RDLDVC;
        // .seq gets filled in at xchg processing
        // .len represents address+u16(sizeof read)+1(ldvc)
        //  but .spr16 is actual number of bytes to read
        xchg->len=7;
        xchg->spr16=numbytes;
        // .flg and .err init 0
        *((u16*)(&(xchg->flg)))=0;
        // .atrg set target address
        xchg->atrg=taddr;
        // .ahst is host data ptr
        xchg->ahst=haddr;
        // logical device gets transferred in low byte of .spr32
        xchg->spr32=ldvc;
        // issue it...
        crashret=comiox_msg_xchg(xchg);
        // any way out has to release the gblk
        gblkfree((void*)xchg);
    }
    else {
        // crash code has been set for app on NULL ptr
        crashret=ERR_uaiox_setup;
    }
    return(crashret);
}

void caio_rldvcbol(u32 ldvc,u64 haddr,u32 taddr,u32 numbytes) {
    //                          dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    // this form implicitly open loop
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    uaio_xchg* xchg;
    // wait for xchg carrier
    xchg=get_ol_carrier();
    // have xchg carrier
    // .cmd for read
    xchg->cmd=UAIOXCMD_RDLDVC;
    // .seq gets filled in at xchg processing
    // .len represents address+u16(sizeof read)+1(ldvc)
    //  but .spr16 is actual number of bytes to read
    xchg->len=7;
    xchg->spr16=numbytes;
    // .flg and .err init 0
    *((u16*)(&(xchg->flg)))=0;
    // .atrg set target address
    xchg->atrg=taddr;
    // .ahst is host data ptr
    xchg->ahst=haddr;
    // logical device gets transferred in low byte of .spr32
    xchg->spr32=ldvc;
    // issue it...
    comiox_msg_void(xchg);
}

u32 caio_rldvc_lrg(u32 ldvc,u64 haddr,u32 taddr,u32 numbytes) {
    //                          dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    //
    // arbitrary (or large) buffer version
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    u32 crashret=0;
    // send all sub-buffers open loop
    //  only the final chunk needs to be closed loop
    if (numbytes) {
        while (numbytes>UAIOXCMD_MAXDATA) {
            caio_rldvcbol(ldvc,haddr,taddr,UAIOXCMD_MAXDATA);
            haddr+=(u64)UAIOXCMD_MAXDATA;
            taddr+=UAIOXCMD_MAXDATA;
            numbytes-=UAIOXCMD_MAXDATA;
        }
        // last chunk
        crashret=caio_rldvcbuf(ldvc,haddr,taddr,numbytes);
    }
    return(crashret);
}

u32 caio_wldvcbuf(u32 ldvc,u32 taddr,u64 haddr,u32 numbytes) {
    //                         dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    // this form implicitly closed loop
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    uaio_xchg* xchg;
    u32 crashret;
    // need xchg carrier
    xchg=(uaio_xchg*)gblkalloc();
    if (xchg) {
        // have xchg carrier
        // .cmd for read
        xchg->cmd=UAIOXCMD_WRLDVC;
        // .seq gets filled in at xchg processing
        // .len represents address+numbytes+ldvc
        xchg->len=(numbytes+5);
        // .flg and .err init 0
        *((u16*)(&(xchg->flg)))=0;
        // .atrg set target address
        xchg->atrg=taddr;
        // .ahst is host data ptr (wipe high bytes)
        xchg->ahst=haddr;
        // logical device travels in low half of .spr16
        xchg->spr16=(u16)ldvc;
        // issue it...
        crashret=comiox_msg_xchg(xchg);
        // any way out has to release the gblk
        gblkfree((void*)xchg);
    }
    else {
        // crash code has been set for app on NULL ptr
        crashret=ERR_uaiox_setup;
    }
    return(crashret);
}

void caio_wldvcbol(u32 ldvc,u32 taddr,u64 haddr,u32 numbytes) {
    //                          dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    // this form implicitly open loop
    //
    uaio_xchg* xchg;
    // wait for xchg carrier
    xchg=get_ol_carrier();
    // have xchg carrier
    // .cmd for read
    xchg->cmd=UAIOXCMD_WRBYTE;
    // .seq gets filled in at xchg processing
    // .len represents address+numbytes+ldvc
    xchg->len=(numbytes+5);
    // .flg and .err init 0
    *((u16*)(&(xchg->flg)))=0;
    // .atrg set target address
    xchg->atrg=taddr;
    // .ahst is host data ptr
    xchg->ahst=haddr;
    // logical device travels in low half of .spr16
    xchg->spr16=(u16)ldvc;
    // issue it...
    comiox_msg_void(xchg);
}

u32 caio_wldvc_lrg(u32 ldvc,u32 taddr,u64 haddr,u32 numbytes) {
    //                          dst       src
    // assumes (target) taddr - no alignment
    // assumes (host) haddr - no alignment
    //
    // arbitrary (or large) buffer version
    //
    // returns error code
    //         otherwise result is implicitly numbytes transferred
    u32 crashret=0;
    // send all sub-buffers open loop
    //  only the final chunk needs to be closed loop
    if (numbytes) {
        while (numbytes>UAIOXCMD_MAXDATA) {
            caio_wldvcbol(ldvc,taddr,haddr,UAIOXCMD_MAXDATA);
            haddr+=(u64)UAIOXCMD_MAXDATA;
            taddr+=UAIOXCMD_MAXDATA;
            numbytes-=UAIOXCMD_MAXDATA;
        }
        // last chunk
        crashret=caio_wldvcbuf(ldvc,taddr,haddr,numbytes);
    }
    return(crashret);
}

//------------------------------------------------------------------------------

u32 caio_rldvc_reg(u32 ldvc,u32 addr,u32* val,u32 vsz) {
    // vsz !=0 vsz=(1,2,3,4)
    // assumes (target) addr - no alignment
    // returns error code
    //         otherwise result at *val
    //
    uaio_xchg* xchg;
    u32 crashret;
    // need xchg carrier
    xchg=(uaio_xchg*)gblkalloc();
    if (xchg) {
        // have xchg carrier
        // .cmd for read quad(s)
        xchg->cmd=UAIOXCMD_RDLDVC;
        // .seq gets filled in at xchg processing
        // .len represents address+u16(sizeof read)+1(ldvc)
        //  but .spr16 is actual number of bytes to read
        xchg->len=7;
        xchg->spr16=vsz;
        // .flg and .err init 0
        *((u16*)(&(xchg->flg)))=0;
        // .atrg set target address
        xchg->atrg=addr;
        // .ahst is host data ptr (wipe high bytes)
        //   if we pick up byte or word, high stuff is all 0's
        *val=0;
        xchg->ahst=(u64)val;
        // logical device gets transferred in low byte of .spr32
        xchg->spr32=ldvc;
        // issue it...
        crashret=comiox_msg_xchg(xchg);
        // any way out has to release the gblk
        gblkfree((void*)xchg);
    }
    else {
        // crash code has been set for app on NULL ptr
        crashret=ERR_uaiox_setup;
        *val=0;
    }
    return(crashret);
}

void caio_wldvc_reg(u32 ldvc,u32 addr,u32 val,u32 vsz) {
    // vsz !=0, vsz=(1,2,3,4) -- as numbytes
    //  write is open loop
    //
    uaio_xchg* xchg;
    // wait for xchg carrier
    xchg=get_ol_carrier();
    // have xchg carrier
    // set .cmd for write logical device (always bytes)
    xchg->cmd=UAIOXCMD_WRLDVC;
    // .seq gets filled in at xchg processing
    // .len represents address+val(vsz)+1(ldvc)
    xchg->len=(5+vsz);
    // .flg and .err init 0
    *((u16*)(&(xchg->flg)))=0;
    // .atrg set target address
    xchg->atrg=addr;
    // .ahst is host data ptr (imported to structure)
    xchg->spr32=val;
    xchg->ahst=(u64)(&(xchg->spr32));
    // logical device travels in low half of .spr16
    xchg->spr16=(u16)ldvc;
    // issue it...
    comiox_msg_void(xchg);
}

//------------------------------------------------------------------------------

u32 caio_fnckey(u32 findx,u32 arglo,u32 arghi,s64* val) {
    // returns error code
    //         otherwise result at *val
    uaio_xchg* xchg;
    u32 crashret;
    // need xchg carrier
    xchg=(uaio_xchg*)gblkalloc();
    if (xchg) {
        // have xchg carrier
        // .cmd for fk()
        xchg->cmd=UAIOXCMD_FNCKEY;
        // .seq gets filled in at xchg processing
        // .len represents arglo,arghi,(u16)findx in that order
        xchg->len=10;
        // .flg and .err init 0
        *((u16*)(&(xchg->flg)))=0;
        // set 64 bits of arg (s64 or (2 x u32))
        xchg->atrg=arglo;
        xchg->spr32=arghi;
        // spr16 passes FNC#
        xchg->spr16=(u16)findx;
        // .ahst is host data ptr for 64 bit return
        xchg->ahst=(u64)val;
        // issue it...
        crashret=comiox_msg_xchg(xchg);
        // any way out has to release the gblk
        gblkfree((void*)xchg);
    }
    else {
        // crash code has been set for app on NULL ptr
        crashret=ERR_uaiox_setup;
        // default val return
        *val=0;
    }
    return(crashret);
}

//------------------------------------------------------------------------------



//------------------------------------------------------------------------------

int msg_ftu_show_D2xx(void* ioptr) {
    u32 phi;
    u32 plo;
    phi=(u32)((((s64)(ftuptr->D2xx_ptr))>>32)&0xFFFFFFFF);
    plo=(u32)(((s64)(ftuptr->D2xx_ptr))&0xFFFFFFFF);
    sprintf(utppb_ln,"Tp08 thread4 has D2xx_ptr %08X_%08X\n",phi,plo);
    utp_dbgoutput(utppb_ln);
    // return int 0 - auto destroy message
    return(0);
}

void ftu_show_D2xx(void) {
    // status dump (associated with an F# pushbutton)
    msg_queue* queue;
    u32 error;
    // the message gets passed to MU_thread4
    queue=&((thr_ctrl+4)->mqueue);
    error=msgexec_gblkvoid_a2(queue,msg_ftu_show_D2xx,0,0);
}

int msg_set_D2xx(void* ioptr) {
    // this is a message handler to deal with java side signalling D2xx setup
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    //  dstptr - unused
    //  arg0 - D2xx_ptr
    //  arg1 - unused
    //  arg2 - unused
    //  doneptr - unused
    void* ptr;
    ptr=(void*)(*(((s64*)ioptr)+1));
    ftuptr->D2xx_ptr=ptr;
    // VERBOSITY
    // optional echo - returns int and calling arg unused
    // msg_ftu_show_D2xx(ptr);
    // return int 0 - auto destroy message
    return(0);
}

//------------------------------------------------------------------------------

int fttx_avail(void) {
    // returns tx byte count committed for FTDI transmit
    int retval;
    int dcnt;
    bptrquetx* bq;
    bq=ftuptr->txqueue;
    // can only send with tokens permit
    retval=bq->tok;
    if (retval) {
        // tokens grant permission of some paylad
        //  bound the transfer to max size, but we know it's still non-zero
        if (retval>FTDI_XFER_MAX) {
            retval=FTDI_XFER_MAX;
        }
        // dcnt is amount of tx data on deck
        dcnt=bptrqueue_avail((bptrqueue*) bq);
        // bound the transfer to data available
        if (retval>dcnt) {
            retval=dcnt;
        }
    }
    // else blocked from sending without tokens
    return(retval);
}

int fttx_commit(void) {
    // returns next tx byte for FTDI transmit
    int retval;
    bptrquetx* bq;
    bq=ftuptr->txqueue;
    retval=0;
    bptrqueue_rd((bptrqueue*)bq,(char*)(&retval),1);
    // reduce tokens
    bq->tok=(bq->tok)-1;
    return(retval);
}

void ftrx_commit(int val) {
    // commits num byte from src to receive bptrqueue
    //  assumes data will fit without overrun
    bptrqueue* bq;
    bq=ftuptr->rxqueue;
    bptrqueue_wr(bq,(char*)(&val),1);
}

//------------------------------------------------------------------------------

void ft_fe_loopback(void) {
    // test handler that just performs loopback on FTDI uart front end
    bptrqueue* bqr;
    bptrquetx* bqt;
    int num;
    int spc;
    // check for rx data
    bqr=ftuptr->rxqueue;
    num=bptrqueue_avail(bqr);
    if (num) {
        // rx data detected
        // check for loopback room
        bqt=ftuptr->txqueue;
        spc=bptrqueue_space((bptrqueue*)bqt);
        if (spc) {
            // some tx data room
            // use lower bound
            if (num>spc) {
                num=spc;
            }
            // known num is non-zero
            bptr_que_to_que((bptrqueue*)bqt,bqr,num);
            // auto-supply tx tokens for this test
            bqt->tok=(bqt->tok)+num;
        }
    }
}

//------------------------------------------------------------------------------
// general utilities

void com_offline(void) {
    // message any clients that need to know of link failure
    //  link dropout corrective actions
    //
    // do_nothing -- TBD placeholder
}

void com_online(void) {
    // message any clients that need to know of link startup
    //  startup actions
    //
    // do_nothing -- TBD placeholder
}

//------------------------------------------------------------------------------
// state transitions

void comstate_COMSYNC0(void) {
    u8 paststate;
    // called on entering this state
    paststate=ftuptr->comstate;
    // if drop-out from XCHG, alert clients
    if (paststate==COMXCHG) {
        com_offline();
    }
    ftuptr->comstate=COMSYNC0;
    fttxqueue.tok=0;
    // wipe all refresh counts
    ftuptr->rfrshfe=0;
    ftuptr->rfrshq0=0;
    // reset .comtcnt .comrcnt
    ftuptr->comtcnt=0;
    ftuptr->comrcnt=0;
    // .rxcollect resets to 0
    ftuptr->rxcollect=0;
    // all sequence numbers reset to 0
    *((int*)(&(ftuptr->txseq)))=0;
    // fake like we just recevied some input (for TIMEOUT)
    ftuptr->rx_in_time=ACptr->tmr_cnt;
    // fake like we just sent some output (for generation interval)
    ftuptr->tx_op_time=ACptr->tmr_cnt;
    // verbosity
    if (paststate==COMXCHG) {
        utp_dbgoutput("Tp COM: re-synchronizing\n");
    }
    // VERBOSITY - debug
    // utp_dbgoutput("Tp comstate: COMSYNC0\n");
}

void comstate_COMSYNC1(void) {
    // called on entering this state
    ftuptr->comstate=COMSYNC1;
    // reset .comtcnt .comrcnt
    ftuptr->comtcnt=0;
    ftuptr->comrcnt=0;
    // verbosity
    // utp_dbgoutput("Tp comstate: COMSYNC1\n");
}

void comstate_COMSYNC2(void) {
    // called on entering this state
    ftuptr->comstate=COMSYNC2;
    // reset .comtcnt .comrcnt
    ftuptr->comtcnt=0;
    ftuptr->comrcnt=0;
    // verbosity
    // utp_dbgoutput("Tp comstate: COMSYNC2\n");
}

void comstate_COMLOCK(void) {
    // called on entering this state
    ftuptr->comstate=COMLOCK;
    // reset .comtcnt .comrcnt
    ftuptr->comtcnt=0;
    ftuptr->comrcnt=0;
    // verbosity
    // utp_dbgoutput("Tp comstate: COMLOCK\n");
    //
    //  as we go to COMLOCK state - reset the client queues
    //   then RX will recieved TX(0) and set all initial tokens
    //    and as we go to COMXCHG state -
    //     we can signal the clients that the queue's are on line
    //
    // sub-queue's get reset
    bptrquetx_init(&comtx0que,comtx0body,COM_SQBUFF_SIZE);
    bptrqueue_init(&comrx0que,comrx0body,COM_SQBUFF_SIZE);
}

void comstate_COMXCHG(void) {
    // called on entering this state
    // reset .comtcnt .comrcnt
    // COM_ctrl.comtcnt=0;
    // COM_ctrl.comrcnt=0;
    ftuptr->comstate=COMXCHG;
    //
    // PRIORITY - set comstate first as we have done here
    //  if we need to signal clients that link is up by messaging, do it now
    //
    // alert clients link is up
    com_online();
    //
    // verbosity
    utp_dbgoutput("Tp comstate: COMXCHG\n");
    // setting state on exit of routine is most conservative
    //  so that clients won't try to use the interface until it is up
}

void com_send_SYNCx(char* src) {
    // general routine to submit any SYNC packet
    //  src points at 4 byte sync pattern
    bptrqueue_wr(((bptrqueue*)(&fttxqueue)),src,4);
    // SYNC packets supply their own tokens
    fttxqueue.tok=(fttxqueue.tok)+4;
    // update the time we sent something
    ftuptr->tx_op_time=ACptr->tmr_cnt;
}

void com_send_TX_0(void) {
    // build the TX(0) packet at pkt_com_TXhdr[]
    //  and send it
    char* dst;
    dst=(char*)pkt_com_TXhdr;
    // identifier TXPKT
    *dst=UART_TXPKT;
    dst++;
    // sequence 0
    *dst=0;
    dst++;
    // the next TXPKT we would send, would have to be index 1
    ftuptr->txseq=1;
    // payload count=0
    *((u16*)(dst))=0;
    dst+=2;
    // front end tokens
    *((u16*)(dst))=FTDI_BPTR_DEPTH-COM_BUFF_SAFEPAD;
    dst+=2;
    // subqueue tokens
    *((u16*)(dst))=COM_SQBUFF_SIZE-COM_BUFF_SAFEPAD;
    dst+=2;
    dst=(char*)pkt_com_TXhdr;
    // submit to front end Tx queue
    bptrqueue_wr(((bptrqueue*)(&fttxqueue)),dst,8);
    // SYNC packets supply their own tokens
    fttxqueue.tok=(fttxqueue.tok)+8;
    // update the time we sent something
    ftuptr->tx_op_time=ACptr->tmr_cnt;
}

void com_send_ACK_0(void) {
    char* dst;
    dst=(char*)pkt_com_TXhdr;
    // identifier ACKPKT
    *dst=UART_ACKPKT;
    // sequence 0
    *(dst+1)=0;
    //  submit to front end Tx queue
    bptrqueue_wr(((bptrqueue*)(&fttxqueue)),dst,2);
    // SYNC packets supply their own tokens
    fttxqueue.tok=(fttxqueue.tok)+2;
    // .rxackseq is 0 and we just sent it
    // the next ACK response should be 1
    ftuptr->rxackseq=1;
    // update the time we sent something
    ftuptr->tx_op_time=ACptr->tmr_cnt;
}

void comrx_txpkt_onlyfet(void) {
    // the received packet is TXPKT containing only tokens
    //  deploy all the information in the packet
    u8* src;
    u16 fet;
    src=(u8*)pkt_com_RXhdr;
    // get front end tokens
    fet=*((u16*)(src+4));
    // these get applied as refresh to fttxqueue
    fttxqueue.tok=(fttxqueue.tok)+fet;
    // get subque tokens
    fet=*((u16*)(src+6));
    // these get applied as refresh to comtx0que
    comtx0que.tok=(comtx0que.tok)+fet;
    // sequence number has passed sanity checks
    //  so just bump the sequence number (folded counter)
    if  ((ftuptr->rxseq)==(0xFF)) {
        ftuptr->rxseq=1;
    }
    else {
        ftuptr->rxseq=(ftuptr->rxseq)+1;
    }
    // some extra consideration for TX(0) reception
    if ((ftuptr->comstate)==COMLOCK) {
        // record this event happened
        ftuptr->comrcnt=(ftuptr->comrcnt)|0x01;
        // during sync, the Rx process auto-ack's TX(0)
        //  with ACK(0)
        com_send_ACK_0();
    }
}

u32 com_rxhdr_deploy(u8 hdrtype) {
    // if fully deployed, needs to reset COM_ctrl.rxcollect to 0
    //   (internally make any necessary adjustments to sequence numbers)
    // if not fully deployed, need to update .rxcollect accordingly
    //   (sequence numbers are unchanged)
    // returns upper layer "done" that decides if com_fe_extract while loop
    //  exits (done=1) or loops (done=0)
    //
    // assume nothing
    u32 moredata;
    u32 retval;
    switch (hdrtype) {
        case 0:
            // SYNC0
            switch (ftuptr->comstate) {
                case COMSYNC0:
                    // wait on double detection
                    //  raise comrcnt to maximum 2
                    if ((ftuptr->comrcnt)<UART_SYNC0_RTRG) {
                        ftuptr->comrcnt=(ftuptr->comrcnt)+1;
                    }
                    // simple discard resets header collection
                    ftuptr->rxcollect=0;
                    // retval as upper layer "done"=0
                    retval=0;
                    break;
                case COMSYNC1:
                    // we advanced state before our peer
                    //  the peer is still in COMSYNC0 state, sending these
                    //   .. so throw them away, we left a busrt of SYNC0
                    //    and the other side should be joining us in COMSYNC1 soon
                    // other states should not see these by uart_rxhdr_mask[]
                    // case COMSYNC2:
                    // case COMLOCK:
                    // case COMXCHG:
                default:
                    // simple discard resets header collection
                    ftuptr->rxcollect=0;
                    // retval as upper layer "done"=0
                    retval=0;
                    break;
            }
            break;
        case 1:
            // SYNC1
            switch (ftuptr->comstate) {
                case COMSYNC1:
                    // we're in COMSYNC1, and the other side has joined us
                    //  just mark the detection
                    ftuptr->comrcnt=1;
                    // simple discard resets header collection
                    ftuptr->rxcollect=0;
                    // retval as upper layer "done"=0
                    retval=0;
                    break;
                case COMSYNC2:
                    // we advanced state and expect the same of the peer
                    //  COMSYNC2 migration is SYNC1->SYNC2 transition on the line
                    //   .. so throw them away, waiting for the SYNC2
                    // other states should not see these by uart_rxhdr_mask[]
                    // case COMSYNC0:
                    // case COMLOCK:
                    // case COMXCHG:
                default:
                    // simple discard resets header collection
                    ftuptr->rxcollect=0;
                    // retval as upper layer "done"=0
                    retval=0;
                    break;
            }
            break;
        case 2:
            // SYNC2
            switch (ftuptr->comstate) {
                case COMSYNC2:
                    // this is the only state that can accept this packet
                    //  it's the detection of the sync one-shot event
                    // direct output TX(0) from here
                    com_send_TX_0();
                    // transition state
                    comstate_COMLOCK();
                    // simple discard resets header collection
                    ftuptr->rxcollect=0;
                    // retval as upper layer "done"=0
                    retval=0;
                    break;
                    // other states should not see these by uart_rxhdr_mask[]
                    // case COMSYNC0:
                    // case COMSYNC1:
                    // case COMLOCK:
                    // case COMXCHG:
                default:
                    // simple discard resets header collection
                    ftuptr->rxcollect=0;
                    // retval as upper layer "done"=0
                    retval=0;
                    break;
            }
            break;
        case 3:
            // TXPKT
            switch (ftuptr->comstate) {
                case COMLOCK:
                case COMXCHG:
                    // other states should not see these by uart_rxhdr_mask[]
                    //  but there is no other handling for TXPKT in any other state
                    // case COMSYNC0:
                    // case COMSYNC1:
                    // case COMSYNC2:
                default:
                    // the packet has passed header checks
                    //  but must be elongated for payload
                    moredata=(u32)(*((u16*)(((char*)pkt_com_RXhdr)+2)));
                    // it's unlikely, but the result could be 0,
                    //  in which case, this packet is complete as is
                    if (moredata) {
                        // VERBOSITY - debug
                        // sprintf(utppb_ln,"com_rxhdr_deploy() has moredata %04X\n",moredata);
                        // utp_dbgoutput(utppb_ln);
                        // take action to collect the rest of the packet
                        // it will continue at index 8
                        ftuptr->rxcollect=8;
                        // set .rxvlen as more_data to collect
                        ftuptr->rxvlen=moredata;
                        // retval as upper layer "done"=0
                        retval=0;
                    }
                    else {
                        // service this packet now
                        //  the only thing it can be carrying is front end tokens
                        //   and we consume the sequence number
                        comrx_txpkt_onlyfet();
                        // then we're done with this packet
                        // simple discard resets header collection
                        ftuptr->rxcollect=0;
                        // retval as upper layer "done"=0
                        retval=0;
                    }
            }
            break;
        case 4:
            // ACKPKT
            switch (ftuptr->comstate) {
                case COMLOCK:
                    // if we passed all the checks to here, we satisfied
                    //  the sync startup protocol
                    comstate_COMXCHG();
                    // flows into case COMXCHG
                case COMXCHG:
                    // we know sanity confirmed it is .txackseq match
                    //  so just bump the sequence number (folded counter)
                    if  ((ftuptr->txackseq)==(0xFF)) {
                        ftuptr->txackseq=1;
                    }
                    else {
                        ftuptr->txackseq=(ftuptr->txackseq)+1;
                    }
                    ftuptr->acktime=ACptr->tmr_cnt;
                    // simple discard resets header collection
                    ftuptr->rxcollect=0;
                    // retval as upper layer "done"=0
                    retval=0;
                    break;
                    // other states should not see these by uart_rxhdr_mask[]
                    // case COMSYNC0:
                    // case COMSYNC1:
                    // case COMSYNC2:
                default:
                    // simple discard resets header collection
                    ftuptr->rxcollect=0;
                    // retval as upper layer "done"=0
                    retval=0;
                    break;
            }
            break;
    }
    return(retval);
}

void com_send_ACK(void) {
    char* dst;
    dst=(char*)pkt_com_TXhdr;
    // identifier ACKPKT
    *dst=UART_ACKPKT;
    // sequence
    *(dst+1)=ftuptr->rxackseq;
    //  submit to front end Tx queue
    bptrqueue_wr(((bptrqueue*)(&fttxqueue)),dst,2);
    // raise .rxackseq
    // bump the sequence number (folded counter)
    if  ((ftuptr->rxackseq)==(0xFF)) {
        ftuptr->rxackseq=1;
    }
    else {
        ftuptr->rxackseq=(ftuptr->rxackseq)+1;
    }
    // update the time we sent something
    ftuptr->tx_op_time=ACptr->tmr_cnt;
}

//------------------------------------------------------------------------------

u32 com_TX0hdr_sane(u8* src) {
    // previously BOOL -- use 0/1 FALSE/TRUE
    // returns TRUE for accept
    //         FALSE fails header
    // assume fail
    u32 retval=0;
    // COMLOCK tests
    if ((*(src+1))==0) {
        // sequence # is 0
        if ((*((u16*)(src+2)))==0) {
            // there is no payload
            // front end tokens should be non-zero
            //  but if they didn't issue any tokens, we can't send them anything
            //   so strictly it's not an error and doesm't get checked
            if (!(ftuptr->comrcnt)) {
                // this is the first and only TX(0) arrival during COMLOCK
                retval=1;
            }
        }
    }
    return(retval);
}

u32 com_TXhdr_sane(u8* src) {
    // previously BOOL -- use 0/1 FALSE/TRUE
    // returns TRUE for accept
    //         FALSE fails header
    // assume fail
    u16 payld;
    u32 retval=0;
    // COMXCHG tests
    // the sequence number must match expected .rxseq
    if ((*(src+1))==(ftuptr->rxseq)) {
        // sequence number is expected value
        payld =*((u16*)(src+2));
        if (payld<=COM_TXPKT_MAXPL) {
            // payload is a legal size
            retval=1;
        }
    }
    return(retval);
}

u32 com_rxhdr_sane(char* src) {
    // previously BOOL -- use 0/1 FALSE/TRUE
    // returns TRUE for accept
    //         FALSE fails header
    u32 hdrtype;
    // assume pass
    u32 retval=1;
    hdrtype=(u32)(*((u8*)(src)));
    switch (hdrtype) {
        // we know it is valid hdr type
        case 0:
            // SYNC0
            if (!((*((int*)(src)))==kpkt_com_SYNC0)) {
                // failed 4 byte pattern match
                retval=0;
            }
            break;
        case 1:
            // SYNC1
            if (!((*((int*)(src)))==kpkt_com_SYNC1)) {
                // failed 4 byte pattern match
                retval=0;
            }
            break;
        case 2:
            // SYNC2
            if (!((*((int*)(src)))==kpkt_com_SYNC2)) {
                // failed 4 byte pattern match
                retval=0;
            }
            break;
        case 3:
            // TX
            if ((ftuptr->comstate)==COMLOCK) {
                // COMLOCK must see specifically TX(0)
                retval=com_TX0hdr_sane((u8*)src);
            }
            else {
                // implicit COMXCHG rules
                retval=com_TXhdr_sane((u8*)src);
            }
            break;
        case 4:
            // ACK
            // sequence must match expected .txackseq
            if ((*((u8*)(src+1)))==ftuptr->txackseq) {
                // expected sequence match
                if ((ftuptr->comstate)==COMLOCK) {
                    // additional checks for COMLOCK
                    // this can only be accepted in order -
                    //  non-redundant, and after reception of TX(0) from other side
                    if (!((ftuptr->comrcnt)==1)) {
                        // failed sequential check
                        retval=0;
                    }
                }
                // else - under COMXCHG, all checks passed
            }
            else {
                retval=0;
            }
            break;
    }
    return(retval);
}

void comrx_txpkt_all(void) {
    // the received packet is TXPKT containing variable length content
    //  deploy all the information in the packet
    u8* src;
    u16 fet;
    src=(u8*)pkt_com_RXhdr;
    // get front end tokens
    fet=*((u16*)(src+4));
    // these get applied as refresh to comtfeque
    fttxqueue.tok=(fttxqueue.tok)+fet;
    // get subque tokens
    fet=*((u16*)(src+6));
    // these get applied as refresh to comtx0que
    comtx0que.tok=(comtx0que.tok)+fet;
    // get transfer count
    fet=*((u16*)(src+2));
    // and those bytes get copied into the subque
    src+=8;
    // VERBOSITY debug
    // sprintf(utppb_ln,"comrx_txpkt_all() data xfer %04X - %08X\n",fet,*((u32*)src));
    // utp_dbgoutput(utppb_ln);
    bptrqueue_wr(&comrx0que,(char*)src,fet);
    // sequence number has passed sanity checks
    //  so just bump the sequence number (folded counter)
    if  ((ftuptr->rxseq)==(0xFF)) {
        ftuptr->rxseq=1;
    }
    else {
        ftuptr->rxseq=(ftuptr->rxseq)+1;
    }
    // some extra consideration for TX(0) reception
    if ((ftuptr->comstate)==COMLOCK) {
        // record this event happened
        ftuptr->comrcnt=(ftuptr->comrcnt)|0x01;
        // during sync, the Rx process auto-ack's TX(0)
        //  with ACK(0)
        com_send_ACK_0();
    }
}

void com_fesend_XCHG(void) {
    //
    // the last thing the other side sent,
    //  if it wants to be quiet,
    //   is the ACK to the last thing we sent
    //    so when we get that, it free's 2 refresh tokens
    //     -- so to avoid an endless loop of every token return, there is
    //         some lower threshold for "that can wait"
    // as a policy, we should return every sub-queue token
    //  but if both sides are only negotiating front end tokens, they can shut up
    //   to talk at minimum packet rate of ~1/s to keep the heartbeat
    //    #define UART_PKT_TICKS   (64)
    // if we send a TXPKT, we must get an ACK for it, though we don't strictly police it
    //  we can't send another TXPKT if the ACK backlog is too big
    //   if it is too big and never changes, we send nothing...
    //    so that violates the heartbeat rule
    //     the other side will timeout on idle line
    //      and then they should send us SYNC0 that will crash the protocol on this side
    // if we are expecting any ACK's... we know the time of the last ACK arrival
    //  then failure of the next ACK to arrive within ~2 sec drops the link
    //   if we're not expecting any ACK's we don't monitor this
    //    #define UART_ACK_TICKS   (130)
    // if we send or get a pure heartbeat,
    //  it's a TXPKT with no payload and no sub-queue tokens
    //   it can only contain front end tokens and has a minimum size 8
    //    so when we fully decode one of those arrivals, it free's up 8 tokens
    //     and when we receive an ACK for a heartbeat we sent, it free's up 2 token
    //      so if we only have like 10 refresh tokens on the front end, it doesn't warrant
    //       the issue of another TXPKT, or we'll be in an endless loop enchanging
    //        front end tokens only about the tokens we eat to supply that status
    // so we'll threshold that front end token returns >16 warrant a packet generation
    //  otherwise we can hold them until the next 1 second boundary
    //   #define UART_TX_RFRSHFE   0x10
    //
    //---------------------------------------------------------------------------
    u32 acklead;
    u32 space;
    u32 txgen;
    u32 bqdata;
    u32 pktsz;
    u32 rmsz;
    u32 now;
    char* pptr;
    u32 done=0;
    now=(u32)(ACptr->tmr_cnt);
    //
    // get the ACK backlog
    //  although it's the difference of folded counters, it's good enough
    //   to discriminate a "big" difference
    //
    acklead=((ftuptr->txseq)-(ftuptr->txackseq))&0xFF;
    if (acklead) {
        // for non-0 acklead, we're waiting on an ACK
        // .acktime is the time of the last received ACK, or sent TXPKT
        if ((now-(ftuptr->acktime))>UART_ACK_TICKS) {
            // time out on ACK delay
            // resets the protocol, not the link
// QQQQQ
            utp_dbgoutput("Tp com_fesend_XCHG fail UART_ACK_TICKS\n");
            comstate_COMSYNC0();
            utp_dbgoutput(com_ack_fault);
            done=1;
        }
    }
    if (!done) {
        // space is output budget at the front end
        space=(u32)(bptrqueue_space((bptrqueue*)(&fttxqueue)));
        // our own (rx) ACK backlog should get cleared first
        while (!done) {
            if (space>2) {
                // we have room for an ACK
                if (!((ftuptr->rxseq)==(ftuptr->rxackseq))) {
                    // we have an ACK to fill
                    com_send_ACK();
                    space-=2;
                }
                else {
                    done=1;
                }
            }
            else {
                done=1;
            }
        }
        // then we can generate TXPKT's
        done=0;
        while (!done) {
            // no generation if there is a backlog of ACK's
            if (acklead<UART_ACK_LIMIT) {
                if (space>=UART_TX_PKT_THR) {
                    // a packet could go out -- there is room for it
                    //  now check if it's warranted
                    txgen=0;
                    // scan for payload supply
                    bqdata=(u32)(bptrquetx_avail(&comtx0que));
                    if (bqdata) {
                        // data supply means a TXPKT will go out
                        //  with bqdata available from bq
                        txgen=1;
                        // we also have to bound this
                        //  it is already bounded by the bptrquetx tokens
                        //   issued from the other side, and is safe to send
                        //    in that respect
                        // but our own internal staging buffer for output
                        //  (pkt_com_TXhdr) has a built in size assumption
                        //    that it doesn't exceed COM_TXPKT_MAXPL
                        //
                        // HEY! - be aware that the tokens issued on a queue from
                        //  the other side may actually be larger than the circular
                        //   buffer we implemented on this side
                        // AND - our subque can actually be larger than
                        //   the front end queue
                        //
                        if (bqdata>COM_TXPKT_MAXPL) {
                            bqdata=COM_TXPKT_MAXPL;
                        }
                    }
                    // check for subque refresh
                    if (ftuptr->rfrshq0) {
                        txgen=1;
                    }
                    // check if front end refresh demands packet
                    if ((ftuptr->rfrshfe)>UART_TX_RFRSHFE) {
                        txgen=1;
                    }
                    // if there really is no data to go,
                    //  we still have to send a packet to sustain link heartbeat
                    if ((now-(ftuptr->tx_op_time))>=UART_PKT_TICKS) {
                        txgen=1;
                    }
                    //-----------
                    // so all decision points are assessed - do we send?
                    if (txgen) {
                        // send a packet and loop
                        // start with minimum pktsz;
                        pktsz=8;
                        pptr=(char*)pkt_com_TXhdr;
                        // packet type/ID
                        *pptr=UART_TXPKT;
                        pptr++;
                        // sequence
                        *pptr=ftuptr->txseq;
                        pptr++;
                        if (bqdata) {
                            // there will be payload appended
                            // now use rmsz as the packet budget
                            rmsz=space-pktsz;
                            // the actual payload size is the lesser of <bqdata,rmsz>
                            // leave it at bqdata
                            if (rmsz<bqdata) {
                                bqdata=rmsz;
                            }
                        }
                        // zero or otherwise, set packet payload size from bqdata
                        *((u16*)pptr)=bqdata;
                        pptr+=2;
                        // front end refresh tokens
                        *((u16*)pptr)=ftuptr->rfrshfe;
                        pptr+=2;
                        ftuptr->rfrshfe=0;
                        // append sub-que refresh tokens
                        *((u16*)pptr)=ftuptr->rfrshq0;
                        pptr+=2;
                        ftuptr->rfrshq0=0;
                        // append the payload
                        if (bqdata) {
                            bptrqueue_rd((bptrqueue*)(&comtx0que),pptr,bqdata);
                            // the extraction consumes the tokens
                            comtx0que.tok=(comtx0que.tok)-bqdata;
                            // the total packet size
                            pktsz+=bqdata;
                        }
                        // submit to front end Tx queue
                        // VERBOSITY debug - once packets with payload start - might want to trace here
                        // if (bqdata) {
                        //    sprintf(comdbgbuffer,"pkthdr %04X %04X",
                        //            (pkt_com_TXhdr[0]),
                        //            (pkt_com_TXhdr[1]));
                        //    log1linestatus(comdbgbuffer);
                        //    }
                        bptrqueue_wr(((bptrqueue*)(&fttxqueue)),(char*)pkt_com_TXhdr,pktsz);
                        //  bump the sequence number (folded counter)
                        if ((ftuptr->txseq)==(0xFF)) {
                            ftuptr->txseq=1;
                        }
                        else {
                            ftuptr->txseq=(ftuptr->txseq)+1;
                        }
                        //-----------
                        // some of the clean up is redundant now, with only one subque
                        // now the output budget is reduced when we loop
                        space-=pktsz;
                        // update the last time we sent something
                        ftuptr->acktime=now;
                        // since we just sent, there is increment in backlog of ACK's
                        acklead++;
                        // update the time we sent something
                        ftuptr->tx_op_time=now;
                        // with only one subque now handled, there is no need to loop
                        done=1;
                    }
                    else {
                        done=1;
                    }
                }
                else {
                    done=1;
                }
            }
            else {
                done=1;
            }
        }
    }
}

//------------------------------------------------------------------------------

void com_fe_extract(void) {
    //
    // this is the general wrapper looking for input packets
    //
    u32 avail;
    u32 num;
    u8 tchar;
    char* dst;
    u32 done;
    done=0;
    // no input, nothing can really change

// QQQQQ
    // if (((ACptr->tmr_cnt)-(ftuptr->rx_in_time))>uarttimeout) {
    num=(ACptr->tmr_cnt)-(ftuptr->rx_in_time);
    if (num>uarttimeout) {

        /// // this means the line is dead - target not connected, or not talking (reset)
        /// //  skip it if we're already at the lowest state
        /// if (!((ftuptr->comstate)==COMSYNC0)) {
        ///    // VERBOSITY - debug
        ///  // QQQQQ
        ///    // utp_dbgoutput("Tp COM: timeout idle Rx\n");
        ///    sprintf(utpdbgln,"Tp COM: timeout idle Rx %d\n",num);
        ///    utp_dbgoutput(utpdbgln);
        ///    // resets the protocol, not the link
        ///    comstate_COMSYNC0();
        ///    // NO - whack the UART and restart completely
        ///    ft_uart_restart();
        ///    }

// QQQQQ
        // utp_dbgoutput("Tp COM: timeout idle Rx\n");
        sprintf(utpdbgln,"Tp COM: timeout idle Rx %d\n",num);
        utp_dbgoutput(utpdbgln);
        // whack the UART and restart completely
        ft_uart_restart();

        done=1;
    }
    avail=bptrqueue_avail(&ftrxqueue);
    if (avail) {
        // now try to consume what we have
        while ((avail) && (!(done))) {
            dst=(char*)pkt_com_RXhdr;
            dst+=(ftuptr->rxcollect);
            switch (ftuptr->rxcollect) {
                // .rxcollect is basically an indication of
                //  how many bytes are already accumulated at
                //   pkt_com_RXhdr
                // we can't do anything until we've collected a valid packet header
                case 0:
                    // VERBOSITY - debug
                    // sprintf(utppb_ln,"com_fe_extract() at .rxcollect=0 avail=%d\n",avail);
                    // utp_dbgoutput(utppb_ln);
                    // inspect potential first byte
                    tchar=(u8)bptrqueue_prvw(&ftrxqueue,0);
                    if (tchar<=UART_ACKPKT) {
                        // potentially valid header identifier
                        if ((1<<tchar) & (uart_rxhdr_mask[ftuptr->comstate])) {
                            // this type of packet is accepted in this state
                            // consume the character
                            if ((ftuptr->comstate)==COMXCHG) {
                                ftuptr->rfrshfe=(ftuptr->rfrshfe)+1;
                            }
                            avail--;
                            bptrqueue_drop(&ftrxqueue,1);
                            // commit it to collection buffer
                            ftuptr->rxcollect=1;
                            *dst=tchar;
                            // loop exit
                        }
                        else {
                            // otherwise re-SYNC
// QQQQQ
                            // utp_dbgoutput("Tp com_fe_extract case 0 sync\n");
                            sprintf(utpdbgln,"Tp com_fe_extract case 0 comstate=%02X keybyte=%02X\n",
                                    (ftuptr->comstate),tchar);
                            utp_dbgoutput(utpdbgln);


                            comstate_COMSYNC0();
                            // and drop the character
                            // discard it
                            avail--;
                            bptrqueue_drop(&ftrxqueue,1);
                        }
                    }
                    else {
                        // not a header identifier
                        // discard it
                        avail--;
                        bptrqueue_drop(&ftrxqueue,1);
                        // any state higher than SYNC0 has to decay at this point
                        if ((ftuptr->comstate)>COMSYNC0) {
// QQQQQ
                            utp_dbgoutput("Tp com_fe_extract case 0 elevated\n");
                            comstate_COMSYNC0();
                        }
                        else {
                            if (((ftuptr->comstate)==COMSYNC0) && (ftuptr->comrcnt)) {
                                // even COMSYNC0 resets if it has already seen a SYNC0
                                //  whatever this is... it shouldn't be here
// QQQQQ
                                utp_dbgoutput("Tp com_fe_extract case 0 COMSYNC0\n");
                                comstate_COMSYNC0();
                            }
                            // else - we're in COMSYNC0 hunting for the start of
                            //  SYNC sequence -- so drop it and keep hunting
                            //   we know .rxcollect is 0, so pkt_com_RXhdr is still reset
                        }
                    }
                    break;
                case 1:
                    // VERBOSITY - debug
                    // sprintf(utppb_ln,"com_fe_extract() at .rxcollect=1 avail=%d\n",avail);
                    // utp_dbgoutput(utppb_ln);
                    // so far, we've collected the first byte of a header
                    //  this is a re-entrant state machine, so the packet type we were
                    //   collecting may no longer be valid if the .comstate changed
                    tchar=*((u8*)pkt_com_RXhdr);
                    if ((1<<tchar) & (uart_rxhdr_mask[ftuptr->comstate])) {
                        // this type of packet is (still) accepted in this state
                        // do the bytes available fulfill minimum header
                        if ((ftuptr->comstate)==COMSYNC0) {
                            // lowest sync state verifies SYNC0 at each input byte
                            tchar=(u8)bptrqueue_prvw(&ftrxqueue,0);
                            if (tchar==(*(((u8*)(&kpkt_com_SYNC0))+1))) {
                                // next expected byte arrived
                                avail--;
                                bptrqueue_drop(&ftrxqueue,1);
                                // commit it to collection buffer
                                ftuptr->rxcollect=2;
                                *dst=tchar;
                            }
                            else {
// QQQQQ
                                sprintf(utpdbgln,"Tp SYNC0 byte[1]= %02X\n",tchar);
                                utp_dbgoutput(utpdbgln);
                                comstate_COMSYNC0();
                            }
                        }
                        else {
                            // past state COMSYNC0, packets should be aligned

                            num=uart_rxhdr_minacc[tchar];
                            if (avail>=num) {
                                // the full header seed is available
                                // pull it from que to packet accumulator
                                bptrqueue_rd(&ftrxqueue,dst,num);
                                if ((ftuptr->comstate)==COMXCHG) {
                                    ftuptr->rfrshfe=(ftuptr->rfrshfe)+num;
                                }
                                avail-=num;
                                if (com_rxhdr_sane((char*)pkt_com_RXhdr)) {
                                    done=com_rxhdr_deploy(tchar);
                                }
                                else {
                                    // packet contents fail sanity screen
                                    // re-SYNC and loop to top
// QQQQQ
                                    utp_dbgoutput("Tp com_fe_extract fail com_rxhdr_sane\n");
                                    comstate_COMSYNC0();
                                }
                            }
                            else {
                                // no point to evaluate it any further yet
                                // so exit doing nothing for now... wait for next pass
                                done=1;
                            }

                        }


                    }
                    else {
                        // otherwise re-SYNC without consuming anything else
// QQQQQ
                        utp_dbgoutput("Tp case 1\n");
                        comstate_COMSYNC0();
                        // and loop to top
                    }
                    break;

                case 2:
                    // only for COMSYNC0
                    tchar=*((u8*)pkt_com_RXhdr);
                    if ((1<<tchar) & (uart_rxhdr_mask[ftuptr->comstate])) {
                        // this type of packet is (still) accepted in this state
                        tchar=(u8)bptrqueue_prvw(&ftrxqueue,0);
                        if (tchar==(*(((u8*)(&kpkt_com_SYNC0))+2))) {
                            // next expected byte arrived
                            avail--;
                            bptrqueue_drop(&ftrxqueue,1);
                            // commit it to collection buffer
                            ftuptr->rxcollect=3;
                            *dst=tchar;
                        }
                        else {
// QQQQQ
                            sprintf(utpdbgln,"Tp SYNC0 byte[2]= %02X\n",tchar);
                            utp_dbgoutput(utpdbgln);
                            comstate_COMSYNC0();
                        }
                    }
                    else {
                        // otherwise re-SYNC without consuming anything else
// QQQQQ
                        utp_dbgoutput("Tp case 2\n");
                        comstate_COMSYNC0();
                        // and loop to top
                    }
                    break;

                case 3:
                    // only for COMSYNC0
                    tchar=*((u8*)pkt_com_RXhdr);
                    if ((1<<tchar) & (uart_rxhdr_mask[ftuptr->comstate])) {
                        // this type of packet is (still) accepted in this state
                        tchar=(u8)bptrqueue_prvw(&ftrxqueue,0);
                        if (tchar==(*(((u8*)(&kpkt_com_SYNC0))+3))) {
                            // next expected byte arrived
                            avail--;
                            bptrqueue_drop(&ftrxqueue,1);
                            // commit it to collection buffer
                            // ftuptr->rxcollect=4;
                            *dst=tchar;
                            //don't need to check sanity, already did that for each byte
                            // secifically deploy type 0
                            done=com_rxhdr_deploy(0);
                        }
                        else {
// QQQQQ
                            sprintf(utpdbgln,"Tp SYNC0 byte[3]= %02X\n",tchar);
                            utp_dbgoutput(utpdbgln);
                            comstate_COMSYNC0();
                        }
                    }
                    else {
                        // otherwise re-SYNC without consuming anything else
// QQQQQ
                        utp_dbgoutput("Tp case 3\n");
                        comstate_COMSYNC0();
                        // and loop to top
                    }
                    break;

                case 8:
                    // VERBOSITY - debug
                    // sprintf(utppb_ln,"com_fe_extract() at .rxcollect=8 avail=%d\n",avail);
                    // utp_dbgoutput(utppb_ln);
                    // so far, we've collected 8 bytes of data and need more
                    //  this can only happen for TXPKT
                    //   this is a re-entrant state machine, so the packet type we were
                    //    collecting may no longer be valid if the .comstate changed
                    tchar=*((u8*)pkt_com_RXhdr);
                    if ((1<<tchar) & (uart_rxhdr_mask[ftuptr->comstate])) {
                        // this type of packet is (still) accepted in this state
                        //  do the bytes available fulfill what's needed to complete
                        //   packet
                        num=ftuptr->rxvlen;
                        if (avail>=num) {
                            // the rest of the packet is available
                            // pull it from que to packet accumulator
                            // VERBOSITY - debug
                            // sprintf(utppb_ln,"com_fe_extract() collecting %04X payload bytes\n",num);
                            // utp_dbgoutput(utppb_ln);
                            bptrqueue_rd(&ftrxqueue,dst,num);
                            if ((ftuptr->comstate)==COMXCHG) {
                                ftuptr->rfrshfe=(ftuptr->rfrshfe)+num;
                            }
                            avail-=num;
                            // deploy the (long) TXPKT
                            comrx_txpkt_all();
                            // we've consumed entire packet, start a new one
                            ftuptr->rxcollect=0;
                            // and loop to top
                        }
                        else {
                            // no point to evaluate it any further yet
                            // so exit doing nothing for now... wait for next pass
                            done=1;
                        }
                    }
                    else {
                        // otherwise re-SYNC without consuming anything else
// QQQQQ
                        utp_dbgoutput("Tp com_fe_extract case 8\n");
                        comstate_COMSYNC0();
                        // and loop to top
                    }
                    break;
                default:
                    // VERBOSITY - debug
// QQQQQ
                    utp_dbgoutput("Tp ERROR com_fe_extract() at .rxcollect=?");
                    // anything else, is illegal...
                    //  the protocol is broken -- reset/crash the protocol
                    comstate_COMSYNC0();
                    break;
            }
        }
    }
}

void com_fe_send(void) {
    //
    // this is the general wrapper to stuff output at the front end
    //
    int i;

    switch (ftuptr->comstate) {

        case COMSYNC0:
            if ((ftuptr->comrcnt)<UART_SYNC0_RTRG) {
                // below detection threshold, trickle out SYNC0 packets
                if   ((ftuptr->comtcnt)<=UART_SYNC0_DROP) {
                    // keep trying, but regulate rate based on time
                    if (((ACptr->tmr_cnt)-(ftuptr->tx_op_time))>=UART_SYNC_SEP) {
                        com_send_SYNCx((char*)(&kpkt_com_SYNC0));
                        ftuptr->comtcnt=ftuptr->comtcnt+1;
                    }
                }
                else {
                    // but within reason, if other side won't send us any SYNC0 packets
                    //  we could eventually quit sending them also ...
                    //   but the target doesn't do that, and tries to establish SYNC link always
                    //    we're already in COMSYNC0 state, but should reset the counters
                    //     to effectively start over in this state
                    // VERBOSITY
                    // utp_dbgoutput("Tp COM: timeout SYNC0 detection\n");
                    // resets the protocol, not the link
// QQQQQ
                    utp_dbgoutput("Tp com_fe_send UART_SYNC0_DROP\n");
                    comstate_COMSYNC0();
                }
            }
            else {
                // if we heard the other side, blast out enough that
                //  other side should detect
                //   they may have already detected our partial trickle output
                for (i=0;i<(UART_SYNC0_RTRG+1);i++) {
                    com_send_SYNCx((char*)(&kpkt_com_SYNC0));
                }
                // and then advance state
                comstate_COMSYNC1();
            }
            break;

        case COMSYNC1:
            //  1 timer tick is ~ 10..15 msec...
            if (ftuptr->comrcnt) {
                // other side has joined us at COMSYNC1 state
                // output 2 SYNC1 packets and 1 SYNC2 packet
                com_send_SYNCx((char*)(&kpkt_com_SYNC1));
                com_send_SYNCx((char*)(&kpkt_com_SYNC1));
                com_send_SYNCx((char*)(&kpkt_com_SYNC2));
                comstate_COMSYNC2();
            }
            else {
                if ((ftuptr->comtcnt)<UART_SYNC1_DROP) {
                    // we're still in the grace period for the other side to join
                    //  us at COMSYNC1 state -- then trickle SYNC1 packets
                    if (((ACptr->tmr_cnt)-(ftuptr->tx_op_time))>=UART_SYNC_SEP) {
                        com_send_SYNCx((char*)(&kpkt_com_SYNC1));
                        ftuptr->comtcnt=ftuptr->comtcnt+1;
                    }
                }
                else {
                    // the other side hasn't joined, and never will if we keep sending COMSYNC1
                    //  so return to COMSYNC0 state to try to bring us up together again
// QQQQQ
                    utp_dbgoutput("Tp com_fe_send UART_SYNC1_DROP\n");
                    comstate_COMSYNC0();
                    // and in this state, we trickle SYNC0 packets
                    com_send_SYNCx((char*)(&kpkt_com_SYNC0));
                }
            }
            break;

        case COMSYNC2:
            // this state doesn't output anything from here
            //  it completely reacts to the sync one-shot at com_rxhdr_deploy()
            break;

        case COMLOCK:
            // this state doesn't output anything from here
            //  it completely reacts to Rx events at com_rxhdr_deploy()
            break;

        case COMXCHG:
            com_fesend_XCHG();
            break;

        case COMDOWN:
        default:
            // do nothing
            break;
    }
}

//------------------------------------------------------------------------------

void ft_bpque_init(void) {
    bptrqueue_init(&ftrxqueue,ftrxbody,FTDI_BPTR_DEPTH);
    bptrquetx_init(&fttxqueue,fttxbody,FTDI_BPTR_DEPTH);
    ftuptr->rxqueue=&ftrxqueue;
    ftuptr->txqueue=&fttxqueue;
}

void ft_uart_restart(void) {
    // totally reset the FTDI UART and restart
    ftuptr->ft_state=FTUART_BOOT;
    ftuptr->comstate=COMDOWN;
    // there's nothing we can do to fix D2xx_ptr if it's broken
    // reset the circular queue's
    ft_bpque_init();
}

void ft_uart_FSM(void) {
    // all errors handled internally
    u8 state;
    int tval;
    int error;
    u32 done;
    done=0;
    while (!done) {
        state=ftuptr->ft_state;
        switch (state) {

            case FTUART_BOOT:
                // program startup only happens once
                //  but sometimes on crash or user intervention
                //   in any case, boot attempts don't have to be fast
                //    enforce a delay between successive attempts so if there is
                //     no UART attached, we only do a plug-and-play check like
                //      once per second....
                // record start time and advance state
                ftuptr->reftime=(ACptr->tmr_cnt);
                ftuptr->ft_state=FTUART_DSCVR;
                // immediately proceed next state (not done)
                break;

            case FTUART_DSCVR:
                // time to check for plug-and-play UART?
                tval=(ACptr->tmr_cnt)-(ftuptr->reftime);
                if (tval>FTDI_RETRY_Z) {
                    // do a discovery poll - has D2xx registered?
                    if (ftuptr->D2xx_ptr) {
                        // Java side indicates D2xx manager present
                        //  throw it to FTDI Java library to try to set up connection
                        tval=j2j_get_FTDI_uart();
                        // this returns the port number assigned, (-1) on failure
                        if (tval<0) {
                            // D2xx is up, but apparently no UART's on USB2
                            error=1;
                        }
                        else {
                            // OK we have a port index at tval (and could note it we wanted to,
                            //  but we don't really care which uart...
                            error=0;
                        }
                    }
                    else {
                        // this is unlikely - we generally expect the D2xx Manager present
                        //  even if there is no FTDI UART plugged in
                        error=1;
                    }
                    if (error) {
                        // start over from the beginning
                        ftuptr->ft_state=FTUART_BOOT;
                        // exit to new state immediately (not done)
                        // VERBOSITY - show advancing status
                        // sprintf(utppb_ln,"ft_uart_FSM() looping from FTUART_DSCVR to FTUART_BOOT\n");
                        // utp_dbgoutput(utppb_ln);
                    }
                    else {
                        // we have a UART connection and it's been set up
                        //  proceed to next state
                        ftuptr->ft_state=FTUART_PHYON;
                        // VERBOSITY - show advancing status
                        // sprintf(utppb_ln,"ft_uart_FSM() advance from FTUART_DSCVR to FTUART_XCHG tval=%d\n",tval);
                        // utp_dbgoutput(utppb_ln);
                        // exit to new state immediately (not done)
// QQQQQ
                        utp_dbgoutput("Tp ft_uart_FSM first SYNC0\n");
                        comstate_COMSYNC0();
                    }
                }
                else {
                    // not time yet for another discovery attempt
                    //  bail to save CPU cycles
                    done=1;
                }
                break;

            case FTUART_PHYON:

                // -- simple loopback ----------------
                //     // temporary test - just loopback front end Tx gets Rx
                //     ft_fe_loopback();

                // -- protocol driven ----------------
                if (!(ftuptr->comstate==COMDOWN)) {
                    // process input
                    com_fe_extract();
                    // generate output
                    com_fe_send();
                }
                // support comiox protocol on subque[0]
                comiox_prc();

                // -- common exit --------------------
                done=1;
                break;

            default:
                // unknown state recovery
                // reset to FTUART_BOOT
                ft_uart_restart();
                done=1;
                break;
        }
    }
}
//------------------------------------------------------------------------------

u32 fnc_0010_ky(char* arg,char* dst) {
    //
    // assume this is called from thread4 - USB/uaiox handler thread
    //  invoking a function key from
    //   void uuiox_fnckey(void)
    //
    // fk(0x10,mode)  - Image mode at HDMI
    //                0x00 - live texture YUV
    //                0x01 - feed bitmap RAW from thread2
    //                0x02 - feed bitmap RAW (tst_inframe)
    //                0x03 - feed bitmap RAW (tst_inframe) -> thread3 HASS/GPU
    //                0x04 - feed bitmap RAW -> thread3 HASS/GPU
    //                ...     test pattern? color test?
    //
    // return ERRNO
    //    -- set quad at dst = ERRNO
    //    -- set retval = 0x0004
    //
    // return s64 return arg
    //    -- set quad at dst = (OK) followed by 8 bytes
    //    -- set retval = 0x000C
    //
    u32 retval=12;  // always successful
    u32 modeval;
    modeval=*((u32*)arg);
    // VERBOSITY - debug
    // LOGI("fnc_0010_ky(%08X)\n",modeval);
    switch (modeval) {
        case 0x00:
            // live texture YUV
            // image bitmap renderer dispose all
            set_ibm_msk(IBM_MSK_NONE);
            // thread1 defaults
            thr1flgs_setclr(0,(T1FLG_TSTIMG|T1FLG_HASSEN));
            // set TextureView visible
            j2j_set_imgvw(0);
            break;
        case 0x02:
            // feed bitmap RAW (tst_inframe)
            // image bitmap renderer only pass RAW
            set_ibm_msk(IBM_MSK_RAW);
            // enable tst_inframe[] substitution
            thr1flgs_setclr(T1FLG_TSTIMG,T1FLG_HASSEN);
            // set ImageView visible
            j2j_set_imgvw(1);
            break;
        case 0x03:
            // feed bitmap RAW (tst_inframe) -> thread3 HASS/GPU
            // image bitmap renderer only pass HASS
            set_ibm_msk(IBM_MSK_HASS);
            // enable tst_inframe[] substitution
            thr1flgs_setclr((T1FLG_TSTIMG|T1FLG_HASSEN),0);
            // set ImageView visible
            j2j_set_imgvw(1);
            break;
        case 0x04:
            // feed bitmap RAW -> thread3 HASS/GPU (normal HASS mode)
            // image bitmap renderer only pass HASS
            set_ibm_msk(IBM_MSK_HASS);
            // disable tst_inframe[] substitution
            thr1flgs_setclr(T1FLG_HASSEN,T1FLG_TSTIMG);
            // set ImageView visible
            j2j_set_imgvw(1);
            break;
        case 0x01:
            // feed bitmap RAW from thread 2
        default:
            // image bitmap renderer only pass RAW
            set_ibm_msk(IBM_MSK_RAW);
            // thread1 defaults
            thr1flgs_setclr(0,(T1FLG_TSTIMG|T1FLG_HASSEN));
            // set ImageView visible
            j2j_set_imgvw(1);
            //
            // whatever else is needed to make RAW image flow...
            //   that means configuring output steering of thread 1 -> 2
            //
            break;
    }
    // success on this fk() have to set ERRNO=0, and s64 return value
    *((u32*)dst)=0;
    *((u32*)(dst+4))=0;
    *((u32*)(dst+8))=0;
    return(retval);
}

u32 fnc_0011_ky(char* arg,char* dst) {
    //
    // assume this is called from thread4 - USB/uaiox handler thread
    //  invoking a function key from
    //   void uuiox_fnckey(void)
    //
    // return ERRNO
    //    -- set quad at dst = ERRNO
    //    -- set retval = 0x0004
    //
    // return s64 return arg
    //    -- set quad at dst = (OK) followed by 8 bytes
    //    -- set retval = 0x000C
    //
    u32 retval=12;  // always successful
    u32 command = *((u32*)arg);
    u32 argument = *(((u32*)arg)+1);
    int arg1 = argument & 0xFFFF;
    int arg2 = (argument >> 16) & 0xFFFF;

    int ret = commanderExecute(command, arg1, arg2);

    sprintf(utpdbgln, "Command %d, arg1 %d, arg2 %d\n",
            command, arg1, arg2);
    utp_dbgoutput(utpdbgln);

    // success on this fk() have to set ERRNO=0, and s64 return value
    *((u32*)dst)=0;
    *((u32*)(dst+4))=ret;
    *((u32*)(dst+8))=0;
    return(retval);
}

void* MU_thread0(void* ptr) {
    thread_ctrl* thr_ptr;
    void* retval;
    int now_ms;
    msg_gblkfnc* msg;
    //---------------------------------
    // the input ptr points to this thread's thread_ctrl structure
    thr_ptr=(thread_ctrl*)ptr;
    // then implicitly we know our thread index
    //  and the home of the shared data
    // seed the message queue empty
    lnklst_init((lnklst*)(&(thr_ptr->mqueue)));
    // init command path for jni2java commands
    init_j2j_ctrl(&(ACptr->j2jctrl));
    // it gets wiped to 0 at startup init of ACptr
    //  but really, the shutdown timer hasn't started yet
    ACptr->shtdn_timer=(SHTDN_MAXTM+1);
    // immediately mark this thread started
    thr_ptr->strt_flg=1;

    //---------------------------------
    // what the thread does while alive --
    //  once per pass, check for shutdown
    while (!(thr_ptr->stop_flg)) {
        // maintain the clock, even during shutdown
        now_ms=(int)now_ms_i64();
        ACptr->tmr_cnt=now_ms-(ACptr->tmr_zoff);
        (ACptr->thr0cnt)++;
        if (!(ACptr->hwsema_crsh)) {
            // drag along the shutdown timestamp
            ACptr->shtdn_tstmp=now_ms;
            // once per loop, service msg queue
            msgqueue_service((msg_queue*)(&(thr_ptr->mqueue)));
            // thread specific

            // run the J2J state machine
            j2j_FSM(&(ACptr->j2jctrl));

        }
        else {
            // during shutdown, also maintain the shutdown timer
            //  that starts at #define  SHTDWN_MAXTM -- threads.h
            //   and counts down to 0
            now_ms=SHTDN_MAXTM-(now_ms-(ACptr->shtdn_tstmp));
            if (now_ms>=0) {
                ACptr->shtdn_timer=now_ms;
            }
            else {
                ACptr->shtdn_timer=0;
            }
        }
        wait_thread();
    }

    //---------------------------------
    // on exit, we have to set done flag
    //  and our own return value is pointer to done_flg
    retval=(void*)(&(thr_ptr->done_flg));
    // set the done flag
    *((int*)retval)=1;
    // complete the termination
    pthread_exit(retval);
    //---------------------------------
}

void* MU_thread1(void* ptr) {
    thread1_ctrl* thr_ptr;
    void* vptr;
    void* retval;
    //---------------------------------
    // the input ptr points to this thread's thread_ctrl structure
    thr_ptr=(thread1_ctrl*)ptr;
    // then implicitly we know our thread index
    //  and the home of the shared data
    // seed the message queue empty
    lnklst_init((lnklst*)(&(thr_ptr->mqueue)));

    // set raw image dimension that we accept
    //  cfedim = cfeydim||cfexdim
    //    X = 1920 = 0x780
    //    Y = 1080 = 0x438
    // thr_ptr->cfedim=0x04380780;
    // now based on hassproc.h
    //  #define H_def_WIDTH       1920
    //  #define H_def_HEIGHT      1080
    thr_ptr->cfedim=(H_def_HEIGHT<<16)|H_def_WIDTH;
    // init the cfei and cfeg buffer pool empty
    lnklst_init((lnklst*)(&(thr_ptr->cfepool)));
    lnklst_init((lnklst*)(&(thr_ptr->cfgpool)));
    // the number of snapshot requests is empty
    // thr_ptr->snapreq=0;    // by default
    // seed at least 2 carrier buffers in the pool
    //  (if this fails, we don't crash yet... but we likely will
    //    when data starts streaming and we have nothing in the pool
    //     generally, we should have enough resources so that we can
    //      assume the system will start up normally)
    init_cfepool(thr_ptr);

    // immediately mark this thread started
    thr_ptr->strt_flg=1;
    //---------------------------------
    // what the thread does while alive --
    //  once per pass, check for shutdown
    while (!(thr_ptr->stop_flg)) {
        // if pending shutdown detected, just wait for it
        if (!(ACptr->hwsema_crsh)) {
            // once per loop, service msg queue
            msgqueue_service((msg_queue*)(&(thr_ptr->mqueue)));
            // thread specific

            // ZZZZZ

        }
        wait_thread();
    }
    //---------------------------------
    // clean up
    //---------------------------------
    // on exit, we have to set done flag
    //  and our own return value is pointer to done_flg
    retval=(void*)(&(thr_ptr->done_flg));
    // set the done flag
    *((int*)retval)=1;
    // complete the termination
    pthread_exit(retval);
    //---------------------------------
}

void* MU_thread2(void* ptr) {
    thread2_ctrl* thr_ptr;
    u32 x;
    u32 y;
    void* vptr;
    void* retval;
    //---------------------------------
    // the input ptr points to this thread's thread_ctrl structure
    thr_ptr=(thread2_ctrl*)ptr;
    // then implicitly we know our thread index
    //  and the home of the shared data
    // seed the message queue empty
    lnklst_init((lnklst*)(&(thr_ptr->mqueue)));

    // set raw image dimension that we accept
    //  cfedim = cfeydim||cfexdim
    //    X = 1920 = 0x780
    //    Y = 1080 = 0x438
    x=0x780;
    y=0x438;
    thr_ptr->imodim=(u32)((y<<16)|x);
    thr_ptr->imobytes=(x*y)<<2;
    // semaphore defaults 0 anyhow
    // thr_ptr->hwsema_rdy=0;
    // the output buffer pool and ready chains are empty
    lnklst_init((lnklst*)(&(thr_ptr->imopool)));
    lnklst_init((lnklst*)(&(thr_ptr->imo_rdy)));
    // the color output file carrier pool is empty
    lnklst_init((lnklst*)(&(ACptr->coflpool)));
    // seed at least 2 carrier buffers in the pool
    //  (if this fails, we don't crash yet... but we likely will
    //    when data starts streaming and we have nothing in the pool
    //     generally, we should have enough resources so that we can
    //      assume the system will start up normally)
    vptr=imo_getbuffer(thr_ptr);
    if (vptr) {
        // new resource goes to pool
        lnklst_fins((&(thr_ptr->imopool)),((lnklst*)vptr));
        vptr=imo_getbuffer(thr_ptr);
        if (vptr) {
            // new resource goes to pool
            lnklst_fins((&(thr_ptr->imopool)),((lnklst*)vptr));
        }
    }
    // likewise for the coflpool
    vptr=imo_getbuffer(thr_ptr);
    if (vptr) {
        // new resource goes to pool
        lnklst_fins((&(ACptr->coflpool)),((lnklst*)vptr));
        vptr=imo_getbuffer(thr_ptr);
        if (vptr) {
            // new resource goes to pool
            lnklst_fins((&(ACptr->coflpool)),((lnklst*)vptr));
        }
    }

    // immediately mark this thread started
    thr_ptr->strt_flg=1;
    //---------------------------------
    // what the thread does while alive --
    //  once per pass, check for shutdown
    while (!(thr_ptr->stop_flg)) {
        // if pending shutdown detected, just wait for it
        if (!(ACptr->hwsema_crsh)) {
            // once per loop, service msg queue
            msgqueue_service((msg_queue*)(&(thr_ptr->mqueue)));
            // thread specific

            // test pattern insertion
            tp_monitor();

        }
        wait_thread();
    }
    //---------------------------------
    // clean up
    //---------------------------------
    // on exit, we have to set done flag
    //  and our own return value is pointer to done_flg
    retval=(void*)(&(thr_ptr->done_flg));
    // set the done flag
    *((int*)retval)=1;
    // complete the termination
    pthread_exit(retval);
    //---------------------------------
}

void* MU_thread3(void* ptr) {
    thread3_ctrl* thr_ptr;
    void* retval;
    short* plnout;
    int svm_open=0;
    //---------------------------------
    // the input ptr points to this thread's thread_ctrl structure
    thr_ptr=(thread3_ctrl*)ptr;
    // then implicitly we know our thread index
    //  and the home of the shared data
    // seed the message queue empty
    lnklst_init((lnklst*)(&(thr_ptr->mqueue)));

    // VERBOSITY - need some startup information?
    //  drop in a print status line here
    // LOGI("sizeof(Hass_ctrl)=0x%04X\n",sizeof(Hass_ctrl));
    // LOGI("thread 3 sizeof(cl_int) = %d\n",sizeof(cl_int));

    // GO ON, BEFORE ALL KERNELS COMPILED?...
    //  let the other threads boot
    thr_ptr->strt_flg=1;

    // nothing in CL_startup() cares about the size of the image
    //  but we want CL up, in case buffer allocations of Haas_init
    //   want to appeal to the CL_context to get buffers with CL properties
//    CL_startup();

    // allocate the huge encompassing SVM buffer
//    Hassptr->svmbase=alloc_SVM(OCL_SVM_SIZE);

    // this does an initial allocation of most image pipeline buffers
//    Hass_init();

    // transfer default filter to SVM
//    hass_fConfig_SVM(&filterConfig);
    // hass_coef -> SVM
    //  note this is dependent on fConfig
    //   a change in fConfig ripples to a re-calculation of hass_coef[]
//    hass_coef_SVM(hass_coef);
    // shrink coef vector (used in 2D)
//    shrnk_coef_SVM((float*)(&shrink_coef[0]));
    // set reconCoeffs (in SVM memory)
//    hass_reconcoe_SVM();
    // set general kernel parameters in SVM
//    hass_parms_SVM();
    // combine coef vector (used in 2D)
//    comb_coef_SVM((float*)(&combine_coef[0]));
    // combine zoom vector (used in 2D)
//    zoom_coef_SVM((float*)(&zoom_coef[0]));
    // hass_post plain gains to SVM
//    hass_gains_SVM();
    // starting gamma table assumptions
//    hass_gamma_SVM((char*)(&(gammaLut[0])),(gammaTblLen<<1));
    // populate thread1 queue with available input GPU frames
//    init_cfegpool();
    // the default setup for HASS planesOutput is open for read only
//    plnout=(short*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNOUT);
//    clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_READ,
//                    (void*)plnout,SVMMAX_PLNOUT,0,NULL,NULL);
    // mark remote svm buffers open
//    svm_open=1;
    // default kernel sets output planes at 10 bpp
    //  (always >= 8)
//    Hassptr->okbpp=10;

    // a simple test pattern for testing
//    gen_tst_inframe();

    // ORIGINALLY, we held off signalling this thread is up,
    //  until all CL kernels were collected and compiled...
    //   moved this signalling earlier in the thread startup, above.
    // thr_ptr->strt_flg=1;

    //---------------------------------
    // what the thread does while alive --
    //  once per pass, check for shutdown
//    while (!(thr_ptr->stop_flg)) {
    // if pending shutdown detected, just wait for it
//        if (!(ACptr->hwsema_crsh)) {
    // once per loop, service msg queue
//            msgqueue_service((msg_queue*)(&(thr_ptr->mqueue)));
    // thread specific

    // ZZZZZ

//        }
//        else {
    // cfegptr buffers are always outstanding...
    //  it may not matter, but try to close them
    //   another thread mat be writing to one at shutdown, and is
    //    just slower than this thread to dtect the shutdown
    //     so after a reasonable delay, all threads know not to
    //       touch SVM buffers any more ... THEN close them
    // likewise, close the planesOutput
//            if ((svm_open) && (CL_queue) && ((ACptr->shtdn_timer)<SHTDN_SVM)) {
//                drop_cfegpool();
//                clEnqueueSVMUnmap(CL_queue,(void*)plnout,0,NULL,NULL);
//                svm_open=0;
//            }
//        }
//        sched_yield();
//    }
    //---------------------------------
    // clean up
    // free buffers first, so they can return to CL_context if necessary
//    reset_hass_imgdim();
    // release the SVM buffer
//    free_SVM(Hassptr->svmbase);

    // then tear down the CL connection/interface
//    CL_shutdown();
    //---------------------------------
    // on exit, we have to set done flag
    //  and our own return value is pointer to done_flg
//    retval=(void*)(&(thr_ptr->done_flg));
    // set the done flag
//    *((int*)retval)=1;
    // complete the termination
//    pthread_exit(retval);
    pthread_exit(1);
    //---------------------------------
}

void* MU_thread4(void* ptr) {
    thread4_ctrl* thr_ptr;
    void* retval;
    ofil_carrier* ofil;
    //---------------------------------
    // the input ptr points to this thread's thread_ctrl structure
    thr_ptr=(thread4_ctrl*)ptr;
    // then implicitly we know our thread index
    //  and the home of the shared data
    // seed the message queue empty
    lnklst_init((lnklst*)(&(thr_ptr->mqueue)));
    // bring UTP on line
    utp_startup();
    lnklst_init((lnklst*)(&(utpptr->ocnslchn)));
    lnklst_init((lnklst*)(&(utpptr->oflchn)));
    lnklst_init((lnklst*)(&(utpptr->iflchn)));
    lnklst_init((lnklst*)(&(utpptr->bochn)));

    // set up comiox_ctrl
    init_comiox_ctrl();
    // reset FT uart handler
    //ft_uart_restart();
    // immediately mark this thread started
    thr_ptr->strt_flg=1;
    //---------------------------------
    // what the thread does while alive --
    //  once per pass, check for shutdown
    while (!(thr_ptr->stop_flg)) {
        // if pending shutdown detected, just wait for it
        if (!(ACptr->hwsema_crsh)) {
            // once per loop, service msg queue
            msgqueue_service((msg_queue*)(&(thr_ptr->mqueue)));
            // thread specific - service UTP
            utp_FSM();
            // thread specific - handle FTDI uart comms for dual camera co-ordination
            //ft_uart_FSM();
        }
        wait_thread();
    }

    //---------------------------------
    // clean up UTP
    utp_shutdown();
    //---------------------------------
    // on exit, we have to set done flag
    //  and our own return value is pointer to done_flg
    retval=(void*)(&(thr_ptr->done_flg));
    // set the done flag
    *((int*)retval)=1;
    // complete the termination
    pthread_exit(retval);
    //---------------------------------
}

void* MU_thread5(void* ptr) {
    thread5_ctrl* thr_ptr;
    void* retval;
    //---------------------------------
    // the input ptr points to this thread's thread_ctrl structure
    thr_ptr=(thread5_ctrl*)ptr;
    // then implicitly we know our thread index
    //  and the home of the shared data
    // seed the message queue empty
    lnklst_init((lnklst*)(&(thr_ptr->mqueue)));
    // local inits
    // immediately mark this thread started
    thr_ptr->strt_flg=1;
    //---------------------------------
    // what the thread does while alive --
    //  once per pass, check for shutdown
    while (!(thr_ptr->stop_flg)) {
        // if pending shutdown detected, just wait for it
        if (!(ACptr->hwsema_crsh)) {
            // once per loop, service msg queue
            msgqueue_service((msg_queue*)(&(thr_ptr->mqueue)));
            // thread specific

        }
        wait_thread();
    }
    //---------------------------------
    // clean up
    //---------------------------------
    // on exit, we have to set done flag
    //  and our own return value is pointer to done_flg
    retval=(void*)(&(thr_ptr->done_flg));
    // set the done flag
    *((int*)retval)=1;
    // complete the termination
    pthread_exit(retval);
    //---------------------------------
}

pthread_attr_t thr_attr;

void threads_startup(void) {
    int err;
    int i;
    thread_ctrl* thr_ptr;

    // thread[0]
    i=0;
    thr_ptr=thr_ctrl;
    pthread_attr_init(&thr_attr);
    // customize attributes if you must...

    // granted assumption - there will always be at least 1 working thread
    //  ie minimum MU_THREADWRK is 1
    // thread [0..(MU_THREADWRK-1)]
    for (i=0;i<MU_THREADWRK;i++) {
        if (i == 3) {
            thr_ptr++;
            continue;
        }
        thr_ptr->this_thr=0;
        thr_ptr->shrptr=shrMEM_ptr;
        thr_ptr->thr_indx=i;
        // thread must set the start and done flags,
        //  we'll issue the stop flag at some point in the future
        thr_ptr->strt_flg=0;
        thr_ptr->stop_flg=0;
        thr_ptr->done_flg=0;
        // no special attributes
        err=pthread_create((&(thr_ptr->this_thr)),&thr_attr,MU_threads[i],(void*)(thr_ptr));
        LOGI("THREAD %d create returned %08X\n",i,err);
        thr_ptr++;
    }

    // more threads may be allocated, but in fact never get launched,
    //  and really don't exist
    if (MU_THREADWRK<MU_THREADNUM) {
        // thread [MU_THREADWRK..(MU_THREADNUM-1)]
        for (i=MU_THREADWRK;i<MU_THREADNUM;i++) {
            thr_ptr->this_thr=0;
            thr_ptr->shrptr=shrMEM_ptr;
            thr_ptr->thr_indx=i;
            // still unused ideces - no thread create
            //  simulate thread was started, stopped, and issued done
            thr_ptr->strt_flg=1;
            thr_ptr->stop_flg=1;
            thr_ptr->done_flg=1;
            thr_ptr++;
        }
    }

    // destroy is a misnomer, as it doesn't go anywhere
    //  but to reuse it with different values, you have to "destroy" it
    //   to re- "init" it differently
    //pthread_attr_destroy(&thr_attr);

    // wait for threads to check in at start-up
    thr_ptr=thr_ctrl;
    for (i=0;i<MU_THREADNUM;i++) {
        if (i == 3) {
            thr_ptr++;
            continue;
        }
        while (!(thr_ptr->strt_flg)) {
            sched_yield();
        }
        thr_ptr++;
    }
}

void threads_shutdown(void) {
    int i;
    thread_ctrl* thr_ptr;
    // first command each thread to gracefully halt ...
    thr_ptr=thr_ctrl;
    for (i=0;i<MU_THREADNUM;i++) {
        if (i == 3) {
            thr_ptr++;
            continue;
        }
        thr_ptr->stop_flg=1;
        thr_ptr++;
    }
    // ... then wait for each thread to check out
    thr_ptr=thr_ctrl;
    for (i=0;i<MU_THREADNUM;i++) {
        if (i == 3) {
            thr_ptr++;
            continue;
        }
        if (thr_ptr->strt_flg == 1) {
            while (!(thr_ptr->done_flg)) {
                sched_yield();
            }
        }
        void* status;
        pthread_join(&(thr_ptr->this_thr), &status);
        thr_ptr++;
    }

    pthread_attr_destroy(&thr_attr);
}

void thread_set_crash(u32 crashcode) {
//
// any child thread can shut down the program on detecting a fatal crash condition
//  but only the first failure is captured
//   whoever captures ACptr->hwsema_crsh leaves it set indefinitely
//    so that all others can detect that they don't need to overwrite the first
//     detected crash, which is the only one that initiates the shutdown action
//
    int* hwsema;
    int tst;
    // even if we're not the first, report the crash
    LOGI("CRASH code 0x%08X reported\n",crashcode);
    hwsema=(int*)(&(ACptr->hwsema_crsh));
    tst=__sync_fetch_and_or(hwsema,1);
    // if it's already set, do nothing, as some other thread has already started
    //  to shut down the application - then we'vre read and written a 1
    //   so no corrective action is required
    if (!tst) {
        // this attempt is the first successful attempt to report crash condition
        //  which means we set the crash code
        ACptr->crsh_code=crashcode;
        // then also sync the shutdown timer
        ACptr->shtdn_tstmp=(int)now_ms_i64();
        ACptr->shtdn_timer=SHTDN_MAXTM;
    }
}

// MU_thread1 is the Camera Front End (cfe) that
//  manages the arrival of (should be) periodic images
//   it will try to pass RAW image samples downstream at the same rate
//    but is self regulating if it needs to resoprt to dropping frames
//

void wait_on_texture(void) {
    thread1_ctrl* thr_ptr;
    u32 done;
    thr_ptr=(thread1_ctrl*)(thr_ctrl+1);
    // implicitly, on entry, we own the camCapt semaphore
    // now release the semaphore, we'll spin waiting for it
    camRlse();
    done=0;
    while (!done) {
        camCapt();
        done=(thr_ptr->wt_texture);
        if (!done) {
            camRlse();
            sched_yield();
        }
    }
    // when we exit, we're the owner of the semaphore
    //  as we were when we entered
}

void set_wt_texture(void) {
    thread1_ctrl* thr_ptr;
    thr_ptr=(thread1_ctrl*)(thr_ctrl+1);
    // can only change it under semaphore protection
    camCapt();
    thr_ptr->wt_texture=1;
    camRlse();
}

void clr_wt_texture(void) {
    thread1_ctrl* thr_ptr;
    thr_ptr=(thread1_ctrl*)(thr_ctrl+1);
    // can only change it under semaphore protection
    camCapt();
    thr_ptr->wt_texture=0;
    camRlse();
}

void* cfe_getbuffer(thread1_ctrl* thr_ptr) {
    void* retval;
    // retval==0 means a crash was recorded for out of resources
    u32 size;
    u32 x;
    u32 y;
    // by whatever means, allocate a buffer big enough to accept a raw image
    //  we only pass on images of known size
    //   camera image may be bigger, but we crop
    // the carrier holds a gblk_64, followed by the image data
    x=(u32)(*((u16*)(&(thr_ptr->cfedim))));
    y=(u32)(*(((u16*)(&(thr_ptr->cfedim)))+1));
    size=((x*y)<<1)+(u32)sizeof(gblk_64);
    // but also add 1K for USB transport end padding
    // size+=0x400;
    //
    // for now allocate from shared memory, but eventually this will
    //  likely change to SVM compatible OpenCL buffer

    // however we do this, we don't want to appeal to thread messaging to do it
    //  alloc and free on these resources should be executable from within this thread
    retval=appmalloc(size);

    // retval==0, implies crash was recorded
    if (retval) {
        // preload the size of the image expected to carry
        ((cfei_image*)retval)->xdim=(u16)x;
        ((cfei_image*)retval)->ydim=(u16)y;
    }
    return(retval);
}

int init_cfepool(thread1_ctrl* thr_ptr) {
    // returns error/crash true
    //  otherwise initialize the cfeiptr pool with 4 buffers on deck
    void* vptr;
    int n;
    int error=0;
    n=4;
    while ((n) && (!error)) {
        vptr=cfe_getbuffer(thr_ptr);
        if (vptr) {
            // new resource goes to pool
            lnklst_fins((&(thr_ptr->cfepool)),((lnklst*)vptr));
            n--;
        }
        else {
            error=1;
        }
    }
    return(error);
}

int msg_snapshot(void* ioptr) {
    // this is a message handler to take a snapshot RAW file at the next opportunity
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    //               - no args
    thread1_ctrl* thr_ptr;
    int n;
    n=(int)(*(((s64*)ioptr)+1));
    thr_ptr=(thread1_ctrl*)(thr_ctrl+1);
    thr_ptr->snapreq=(thr_ptr->snapreq)+n;
    // return int 0 - auto destroy message
    return(0);
}

int req_snapshot(int n) {
    // returns 1 on crash abort, else 0
    // request n (hopefully consecutive) snapshots from another thread
    msg_queue* queue;
    int retval;
    // the message gets passed to MU_thread1
    queue=&((thr_ctrl+1)->mqueue);
    retval=msgexec_gblkvoid(queue,msg_snapshot,(s64)n,0,0);
    return(retval);
}

int msg_cfe_refill(void* ioptr) {
    // eventually, the downstream traffic process should return the cfeiptr
    //  so we can replenish the cfepool
    //
    int error;
    thread1_ctrl* thr_ptr;
    cfei_image* cfeiptr;
    thr_ptr=(thread1_ctrl*)(thr_ctrl+1);
    cfeiptr=(cfei_image*)(*(((s64*)ioptr)+1));
    // so just recycle this image buffer
    lnklst_fins((&(thr_ptr->cfepool)),((lnklst*)cfeiptr));
    // return int 0 - auto destroy message
    return(0);
}

int cfe_refill(cfei_image* cfeiptr) {
    // returns 1 on crash abort, else 0
    // normal return of cfei_image (from any thread)
    msg_queue* queue;
    int retval;
    // the message gets passed to MU_thread1
    queue=&((thr_ctrl+1)->mqueue);
    retval=msgexec_gblkvoid(queue,msg_cfe_refill,((s64)cfeiptr),0,0);
    return(retval);
}

int msg_cfeg_refill(void* ioptr) {
    // eventually, the downstream traffic process should return the cfegptr
    //  so we can replenish the cfepool
    //
    int error;
    thread1_ctrl* thr_ptr;
    cfeg_image* cfegptr;
    thr_ptr=(thread1_ctrl*)(thr_ctrl+1);
    cfegptr=(cfeg_image*)(*(((s64*)ioptr)+1));
    // so just recycle this image buffer
    lnklst_fins((&(thr_ptr->cfgpool)),((lnklst*)cfegptr));
    // return int 0 - auto destroy message
    return(0);
}

int cfeg_refill(cfeg_image* cfegptr) {
    // returns 1 on crash abort, else 0
    //  normal return of cfeg_image (from any thread)
    //   actually, this only gets called from thread3 after processing
    //    the image at the GPU
    // so the combined idea is that the SVM buffer has to be "open"
    //  for write - WRITE ONLY - to accept a new input data block
    msg_queue* queue;
    int retval;
    // open the SVM buffer for WRITE ONLY
    retval=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_WRITE,
                           (cfegptr->svmptr),SVMMAX_IFRM,0,NULL,NULL);
    if (!retval) {
        // the message gets passed to MU_thread1
        queue=&((thr_ctrl+1)->mqueue);
        retval=msgexec_gblkvoid(queue,msg_cfeg_refill,((s64)cfegptr),0,0);
    }
    else {
        // error already detected and marked in retval
        //  issue a crash for it
        thread_set_crash(CRASH_OPENCL);
    }
    return(retval);
}

int cfe_file_image(cfei_image* cfeiptr) {
    // return value 0 indicates success
    //              1 indicates some crash recorded - error true
    //
    // this hands off the image to pass out a RAW capture file
    //  either here, or in subsequent called routine,
    //   the responsibility to evetually release the cfei image buffer
    //    travels with it
    //
    int error;
    ofil_carrier* ofil;
    u8 ichar;
    //raw_filhdr* hdr;
    raw_sizehdr* hdr;
    // if traffic is blocking, we're wasting our time
    error=(int)utp_trafblk();
    if (!error) {
        // ofile system is accepting traffic
        ofil=(ofil_carrier*)gblkalloc();
        if (ofil) {
            // we have a carrier for it
            ofil->ofltype=OFLTYP_CFE;
            // need logical transfer index (auto increment)
            ichar=ACptr->snpraw;
            ACptr->snpraw=(ichar+1)&0x0F;
            // text index
            ichar=hexASCII[ichar];
            snprobin_fname[3]=ichar;
            // insert name
            strcpy((char*)(&(ofil->filname)),snprobin_fname);
            // point header
            //hdr=(raw_filhdr*)(((char*)cfeiptr)+0x30);
            hdr=(raw_sizehdr*)(((char*)cfeiptr)+0x30);
            ofil->refptr=(char*)cfeiptr;
            // insert size header + pixels
            //ofil->filsize=sizeof(raw_filhdr)+((((u32)(hdr->xdim))*((u32)(hdr->ydim)))<<1);
            ofil->filsize=sizeof(raw_sizehdr)+((((u32)(hdr->xdim))*((u32)(hdr->ydim)))<<1) + META_DATA_SIZE;
            // ready to submit
            send_utpofile(ofil);
        }
        else {
            // no carrier, we have to drop it
            error=1;
        }
    }
    if (error) {
        // file is currently undeliverable, so discard it
        //  treat as void, return the error we already have
        // error=cfe_refill(cfeiptr);
        cfe_refill(cfeiptr);
    }
    return(error);
}

void cfeiptr_rawhdr(cfei_image* cfeiptr) {
    // configure the header of a raw file carrier
    //  can be used for cfegptr as well with casting
    // insert arrival time stamp in units of msec
    cfeiptr->tstmp=ACptr->tmr_cnt;
    //
    // really we should complete the entire raw_filhdr here
    //  we're not currently supporting date in timestamps, but could in the future
    //   so .tdate is just invalid
    //  xdim ydim were fixed at the allocation of the buffer
    //  pbpp is a property of the sensor - assumed system constant for now
    //   same for medge, pmod, flgs
    //
    cfeiptr->tdate=0;
    cfeiptr->pbpp=RSENSOR_BPP;
    cfeiptr->medge=RSENSOR_MEDGE;
    cfeiptr->pmod=RSENSOR_PMOD;
    cfeiptr->flgs=RSENSOR_FLGS;
}

int get_raw_cfei(u32 ix,u32 iy,s64* pix,s64* dst) {
    int error=0;
    u32 ox;
    u32 oy;
    u32 xc;
    u32 yc;
    u32 tmp;
    u32 strto;
    u32 ilpitch;
    u32 olpitch;

    u8* src;
    u8* srcln;
    u16* udst;
    u16 tpix;
    u32 p5i;
    u32 p5m;

    udst=(u16*)dst;

    thread1_ctrl* thr_ptr;
    //
    // monitoring option
    // u16 mpix;
    //
    thr_ptr=(thread1_ctrl*)(thr_ctrl+1);
    ox=(u32)(*((u16*)(&(thr_ptr->cfedim))));
    oy=(u32)(*(((u16*)(&(thr_ptr->cfedim)))+1));
    // the input frame must be larger, or equal to the pickup carrier
    //  we can crop, but we havn't done the code that fills yet
    //   (and probably won't go there)
    // it's assumed that the output width is divisible by 4
    //  so we can do 64 bit transfers
    //   and that the number of lines is divisible by 2
    // we also assume alignment is preserved, so ix has to be divisible by 4
    if (((ix>=ox) && (iy>=oy)) && (!(ix&0x03)) && (!(ox&0x03))) {
        // VERBOSITY
        // LOGI("get_raw_cfei ix=%d iy=%d ox=%d oy=%d",ix,iy,ox,oy);
        // all assumptions good for the transfer algorithm to work
        // cropped destination is always contiguous
        // dst is calling arg
        if (!((ACptr->thr1flgs)&T1FLG_TSTIMG)) {
            //
            // monitoring option
            //       mpix=0;
            //
            // normally crop the data from sensor raw input frame
            // input line pitch in units of s64's
            // RAW10
            ilpitch=(ix>>2)*5;
            // u16
            olpitch=ox;

            // input starting x offset is half the total padding, rounded down
            // strto is pixel based units, not compensated for pitch/RAW10
            //  it include mosaic aligning offset as well
            // src always stays line aligned
            strto=((ix-ox)/2)-3;
            src=(u8*)pix;

            // input starting y offset is half the total padding, rounded down
            //  to even (mosaic remains unchanged)
            tmp=((iy-oy)/2)&(~0x01);
            // include tuning term for mosaic alignment
            src+=((tmp-1)*ilpitch);

            // transfer the cropped area
            for (yc=0;yc<oy;yc++) {
                srcln=src;
                for (xc=0;xc<olpitch;xc++) {

                    // RAW10 format
                    //          bit 7     bit 6     bit 5     bit 4     bit 3     bit 2     bit 1     bit 0
                    // Byte 0:  P0[9]     P0[8]     P0[7]     P0[6]     P0[5]     P0[4]     P0[3]     P0[2]
                    // Byte 1:  P1[9]     P1[8]     P1[7]     P1[6]     P1[5]     P1[4]     P1[3]     P1[2]
                    // Byte 2:  P2[9]     P2[8]     P2[7]     P2[6]     P2[5]     P2[4]     P2[3]     P2[2]
                    // Byte 3:  P3[9]     P3[8]     P3[7]     P3[6]     P3[5]     P3[4]     P3[3]     P3[2]
                    // Byte 4:  P3[1]     P3[0]     P2[1]     P2[0]     P1[1]     P1[0]     P0[1]     P0[0]

                    // pixel offset in the line is (strto+xc);
                    p5i=strto+xc;
                    // index within the 5 group is p5m
                    p5m=p5i&0x03;
                    // so every 4 pixels is 5 byte step
                    p5i=((p5i&(0xFFFFFFFC))>>2)*5;

                    switch (p5m) {
                        case 0:
                            tpix=((*(srcln+(p5i+0)))<<2) | ((*(srcln+(p5i+4)))&0x03);
                            break;
                        case 1:
                            tpix=((*(srcln+(p5i+1)))<<2) | (((*(srcln+(p5i+4)))>>2)&0x03);
                            break;
                        case 2:
                            tpix=((*(srcln+(p5i+2)))<<2) | (((*(srcln+(p5i+4)))>>4)&0x03);
                            break;
                        case 3:
                            tpix=((*(srcln+(p5i+3)))<<2) | (((*(srcln+(p5i+4)))>>6)&0x03);
                            break;
                    }
                    //
                    // monitoring option
                    //             if (tpix>mpix) {
                    //                mpix=tpix;
                    //                }
                    //
                    *udst=tpix;
                    udst++;
                }
                src+=ilpitch;
            }
            //
            // monitoring option
            //       LOGI("get_raw_cfei() -> max pixel in frame is 0x%04X\n",mpix);
            //
        }
        else {
            // test override substitutes data from tst_inframe[]
            //  which is already 1920x1080

// switch to u16 for adjustment
//       src=(s64*)(&(tst_inframe));
//       src=(u16*)(&(tst_inframe));
//            src=(u8*)(&(tst_inframe));
            src=(u8*)pix;
            //
            // it should be this simple but it isn't....
            //
            // // xc calculates byte transfer size
            // xc=(ox*oy)<<1;
            // memcpy((void*)dst,(void*)src,xc);
            //
            // our golden images are 8 bit pixels and the IMX214 is 10 bit
            //  so to compensate for now, we have to shift each pixel 2 bits
            //   and we know they are 8 bit... don't need to check field overrun
            // however, to keep the gain consistent with PC model for HASS tests
            //  we want to keep them as 8 bit pixels if they are going to the GPU
            //
            // xc calculates s64 transfer size
            // xc=(ox*oy)>>2;
            xc=(ox*oy);

            while (xc) {
                if ((ACptr->thr1flgs)&T1FLG_HASSEN) {
                    // going to GPU stays 8 bit pixels
                    //  for C-model working match
                    // *dst=(*src);
                    *udst=(*((u16*)src));
                }
                else {
                    // to display RAW, we're set for 10 bit pixels
                    // *dst=(*src)<<2;
                    *udst=(*((u16*)src))<<2;
//                    *udst=(*((u16*)src));
                }
                // src++;
                src+=2;;
                // dst++;
                udst++;
                xc--;
            }
        }
    }
    else {
        // can't process this input .. set a crash code and go on
        thread_set_crash(CRASH_CFEIFMT);
        error=1;
    }
    return(error);
}

int get_raw2x_cfei(u32 ix,u32 iy,s64* pix,s64* dst0,s64* dst1) {
    int error=0;
    u32 ox;
    u32 oy;
    u32 xc;
    u32 yc;
    u32 tmp;
    u32 strto;
    u32 ilpitch;
    u32 olpitch;

// switched to u16 for adjustment
    // s64* src;
    // s64* srcln;
    u8* src;
    u8* srcln;
    u16* udst0;
    u16* udst1;
    u16 tpix;
    u32 p5i;
    u32 p5m;

    udst0=(u16*)dst0;
    udst1=(u16*)dst1;

    thread1_ctrl* thr_ptr;
    thr_ptr=(thread1_ctrl*)(thr_ctrl+1);
    ox=(u32)(*((u16*)(&(thr_ptr->cfedim))));
    oy=(u32)(*(((u16*)(&(thr_ptr->cfedim)))+1));
    // the input frame must be larger, or equal to the pickup carrier
    //  we can crop, but we havn't done the code that fills yet
    //   (and probably won't go there)
    // it's assumed that the output width is divisible by 4
    //  so we can do 64 bit transfers
    //   and that the number of lines is divisible by 2
    // we also assume alignment is preserved, so ix has to be divisible by 4
    if (((ix>=ox) && (iy>=oy)) && (!(ix&0x03)) && (!(ox&0x03))) {
        // VERBOSITY
        // LOGI("get_raw_cfei ox=%d oy=%d",ox,oy);
        // all assumptions good for the transfer algorithm to work
        // cropped destination is always contiguous
        // dst0,dst1 is calling arg
        if (!((ACptr->thr1flgs)&T1FLG_TSTIMG)) {
            // normally crop the data from sensor raw input frame
            // input line pitch in units of s64's
            // RAW10
            ilpitch=ix+(ix>>2);
            // u16
            olpitch=ox;

            // input starting x offset is half the total padding, rounded down
            // strto is pixel based units, not compensated for pitch/RAW10
            //  it include mosaic aligning offset as well
            // src always stays line aligned
            strto=((ix-ox)/2)-3;
            src=(u8*)pix;

            // input starting y offset is half the total padding, rounded down
            //  to even (mosaic remains unchanged)
            tmp=((iy-oy)/2)&(~0x01);
            // include tuning term for mosaic alignment
            src+=((tmp-1)*ilpitch);

            // transfer the cropped area
            for (yc=0;yc<oy;yc++) {
                srcln=src;
                for (xc=0;xc<olpitch;xc++) {

                    // RAW10 format
                    //          bit 7     bit 6     bit 5     bit 4     bit 3     bit 2     bit 1     bit 0
                    // Byte 0:  P0[9]     P0[8]     P0[7]     P0[6]     P0[5]     P0[4]     P0[3]     P0[2]
                    // Byte 1:  P1[9]     P1[8]     P1[7]     P1[6]     P1[5]     P1[4]     P1[3]     P1[2]
                    // Byte 2:  P2[9]     P2[8]     P2[7]     P2[6]     P2[5]     P2[4]     P2[3]     P2[2]
                    // Byte 3:  P3[9]     P3[8]     P3[7]     P3[6]     P3[5]     P3[4]     P3[3]     P3[2]
                    // Byte 4:  P3[1]     P3[0]     P2[1]     P2[0]     P1[1]     P1[0]     P0[1]     P0[0]

                    // pixel offset in the line is (strto+xc);
                    p5i=strto+xc;
                    // index within the 5 group is p5m
                    p5m=p5i&0x03;
                    // so every 4 pixels is 5 byte step
                    p5i=(p5i&(0xFFFFFFFC))+(p5i>>2);

                    switch (p5m) {
                        case 0:
                            tpix=((*(srcln+(p5i+0)))<<2) | ((*(srcln+(p5i+4)))&0x03);
                            break;
                        case 1:
                            tpix=((*(srcln+(p5i+1)))<<2) | (((*(srcln+(p5i+4)))>>2)&0x03);
                            break;
                        case 2:
                            tpix=((*(srcln+(p5i+2)))<<2) | (((*(srcln+(p5i+4)))>>4)&0x03);
                            break;
                        case 3:
                            tpix=((*(srcln+(p5i+3)))<<2) | (((*(srcln+(p5i+4)))>>6)&0x03);
                            break;
                    }

                    *udst0=tpix;
                    *udst1=tpix;
                    udst0++;
                    udst1++;
                }
                src+=ilpitch;
            }
        }
        else {
            // test override substitutes data from tst_inframe[]
            //  which is already 1920x1080

// switch to u16 for adjustment
//       src=(s64*)(&(tst_inframe));
//       src=(u16*)(&(tst_inframe));
            src=(u8*)(&(tst_inframe));

            //
            // it should be this simple but it isn't....
            //
            // // xc calculates byte transfer size
            // xc=(ox*oy)<<1;
            // memcpy((void*)dst,(void*)src,xc);
            //
            // our golden images are 8 bit pixels and the IMX214 is 10 bit
            //  so to compensate for now, we have to shift each pixel 2 bits
            //   and we know they are 8 bit... don't need to check field overrun
            // however, to keep the gain consistent with PC model for HASS tests
            //  we want to keep them as 8 bit pixels if they are going to the GPU
            //
            // xc calculates s64 transfer size

            // xc=(ox*oy)>>2;
            xc=(ox*oy);

            while (xc) {
                if ((ACptr->thr1flgs)&T1FLG_HASSEN) {
                    // going to GPU stays 8 bit pixels
                    //  for C-model working match
                    // *dst0=(*src);
                    // *dst1=(*src);
                    *udst0=(*((u16*)src));
                    *udst1=(*((u16*)src));
                }
                else {
                    // to display RAW, we're set for 10 bit pixels
                    // *dst0=(*src)<<2;
                    // *dst1=(*src)<<2;
                    *udst0=(*((u16*)src))<<2;
                    *udst1=(*((u16*)src))<<2;
                }
//          src++;
                src+=2;
                // dst0++;
                // dst1++;
                udst0++;
                udst1++;
                xc--;
            }
        }
    }
    else {
        // can't process this input .. set a crash code and go on
        thread_set_crash(CRASH_CFEIFMT);
        error=1;
    }
    return(error);
}

int msg_fe_frame(void* ioptr) {
    // this is a message handler to deal with arrival of input frame
    // this is what we do on the arrival of a new image available from the camera
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    //  dstptr - unused
    //  arg0 - u16 width
    //  arg1 - u16 height
    //  arg2 - *pixels
    //  doneptr - unused
    thread1_ctrl* thr_ptr;
    volatile int* doneptr;
    cfei_image* cfeiptr0;
    s64* dst0;
    cfei_image* cfeiptr1;
    s64* dst1;
    s64* pix;
    u32 ix;
    u32 iy;
    int error;
    thr_ptr=(thread1_ctrl*)(thr_ctrl+1);
    // whether we really accept it or not, the number of input frames increments
    (thr_ptr->cfeicnt)++;
    doneptr=(volatile int*)(&(thr_ptr->cfedone));
    if  ((ACptr->thr1flgs)&T1FLG_HASSEN) {
        // when set, the image gets HASS processing at thread3
        //  to import the raw image, we need a carrier from the pool
        //   it's cast as a cfeiptr, but it's really a cfegptr
        cfeiptr0=(cfei_image*)lnklst_fpop((&(thr_ptr->cfgpool)));
        if (cfeiptr0) {
            // primary use carrier available
            // then the first destination crop pointer
            dst0=(s64*)(((cfeg_image*)cfeiptr0)->svmptr);
            // have image carrier to accept input
            // collect args
            ix=(u32)(*(((s64*)ioptr)+1));
            iy=(u32)(*(((s64*)ioptr)+2));
            pix=(s64*)(*(((s64*)ioptr)+3));
            // configure header on primary
            cfeiptr_rawhdr(cfeiptr0);
            // check for file capture request
            if (thr_ptr->snapreq) {
                // a snapshot has been requested - always standard cfeiptr
                // to import the raw image, we need a carrier from the pool
                cfeiptr1=(cfei_image*)lnklst_fpop((&(thr_ptr->cfepool)));
                if (cfeiptr1) {
                    // secondary use carrier available
                    // then the second destination crop pointer
                    dst1=(s64*)(((s64)cfeiptr1)+64);
                    // configure header on secondary
                    cfeiptr_rawhdr(cfeiptr1);
                    // primary and secondary
                    error=get_raw2x_cfei(ix,iy,pix,dst0,dst1);
                    error=raw2hass_fwrd((cfeg_image*)cfeiptr0);
                    // secondary cancels snapreq
                    thr_ptr->snapreq=(thr_ptr->snapreq)-1;
                    error=cfe_file_image(cfeiptr1);
                    *doneptr=1;
                }
                else {
                    //only primary
                    error=get_raw_cfei(ix,iy,pix,dst0);
                    error=raw2hass_fwrd((cfeg_image*)cfeiptr0);
                    *doneptr=1;
                }
            }
            else {
                // only primary
                error=get_raw_cfei(ix,iy,pix,dst0);
                error=raw2hass_fwrd((cfeg_image*)cfeiptr0);
                *doneptr=1;
            }
        }
        else {
            // no primary image carrier available
            //  so as far as the camera is concerned, we've absorbed the image
            //   but it is a dropped frame
            (thr_ptr->cfeidrop)++;
            *doneptr=1;
        }
    }
    else {
        // when clr (default), RAW images get passed directly to HDMI (thread2)
        //  to display RAW
        // to import the raw image, we need a carrier from the pool
        cfeiptr0=(cfei_image*)lnklst_fpop((&(thr_ptr->cfepool)));
        if (cfeiptr0) {
            // primary use carrier available
            // then the first destination crop pointer
            dst0=(s64*)(((char*)cfeiptr0)+64);
            // have image carrier to accept input
            // collect args
            ix=(u32)(*(((s64*)ioptr)+1));
            iy=(u32)(*(((s64*)ioptr)+2));
            pix=(s64*)(*(((s64*)ioptr)+3));
            // configure header on primary
            cfeiptr_rawhdr(cfeiptr0);
            // check for file capture request
            if (thr_ptr->snapreq) {
                // a snapshot has been requested - always standard cfeiptr
                // to import the raw image, we need a carrier from the pool
                cfeiptr1=(cfei_image*)lnklst_fpop((&(thr_ptr->cfepool)));
                if (cfeiptr1) {
                    // secondary use carrier available
                    // then the second destination crop pointer
                    dst1=(s64*)(((s64)cfeiptr1)+64);
                    // configure header on secondary
                    cfeiptr_rawhdr(cfeiptr1);
                    // primary and secondary
                    error=get_raw2x_cfei(ix,iy,pix,dst0,dst1);
                    error=raw2pixo_fwrd(cfeiptr0);
                    // secondary cancels snapreq
                    thr_ptr->snapreq=(thr_ptr->snapreq)-1;
                    error=cfe_file_image(cfeiptr1);
                    *doneptr=1;
                }
                else {
                    //only primary
                    error=get_raw_cfei(ix,iy,pix,dst0);
                    error=raw2pixo_fwrd(cfeiptr0);
                    *doneptr=1;
                }
            }
            else {
                // only primary
                error=get_raw_cfei(ix,iy,pix,dst0);
                error=cfe_file_image(cfeiptr0);
                //error=raw2pixo_fwrd(cfeiptr0);
                *doneptr=1;
            }
        }
        else {
            // no primary image carrier available
            //  so as far as the camera is concerned, we've absorbed the image
            //   but it is a dropped frame
            (thr_ptr->cfeidrop)++;
            *doneptr=1;
        }
    }
    // return int 0 - auto destroy message
    return(0);
}

//------------------------------------------------------------------------------

int msg_thr1flgs(void* ioptr) {
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    // arg0 is mask of SET flags
    // arg1 is set of CLR flags
    // update ACptr->thr1flgs
    //  (applied in that order means CLR dominates simultaneous SET)
    //
    u32 sflgs;
    u32 cflgs;
    sflgs=(u32)(*(((s64*)ioptr)+1));
    cflgs=(u32)(*(((s64*)ioptr)+2));
    ACptr->thr1flgs=((ACptr->thr1flgs)|sflgs)&(~cflgs);
    // return int 0 - auto destroy message
    return(0);
}

int thr1flgs_setclr(u32 setmsk,u32 clrmsk) {
    // returns 1 on crash abort, else 0
    //  any thread can call this
    msg_queue* queue;
    int retval;
    // the message gets passed to MU_thread1
    queue=&((thr_ctrl+1)->mqueue);
    retval=msgexec_gblkvoid(queue,msg_thr1flgs,(s64)setmsk,(s64)clrmsk,0);
    return(retval);
}

//------------------------------------------------------------------------------




// MU_thread2 manages generating output pixel images for display
//  it may spawn off others for JPEG recording or USB
//   that's TBD...

void* imo_getbuffer(thread2_ctrl* thr_ptr) {
    void* retval;
    // retval==0 means a crash was recorded for out of resources
    u32 size;
    u32 x;
    u32 y;
    // by whatever means, allocate a buffer big enough to accept
    //  output pixel image ARGB_8888 format
    //  we only pass on images of known size

    //   camera image may be bigger, but we crop
    // the carrier holds a gblk_64, followed by the image data
    x=(u32)(*((u16*)(&(thr_ptr->imodim))));
    y=(u32)(*(((u16*)(&(thr_ptr->imodim)))+1));
    size=(thr_ptr->imobytes)+(u32)sizeof(gblk_64);
    // but also add 1K for USB transport end padding
    // size+=0x400;
    //
    // for now allocate from shared memory, but eventually this will
    //  likely change to SVM compatible OpenCL buffer

    // however we do this, we don't want to appeal to thread messaging to do it
    //  alloc and free on these resources should be executable from within this thread
    retval=appmalloc(size);

    // retval==0, implies crash was recorded
    if (retval) {
        // preload the size of the image expected to carry
        ((pixo_image*)retval)->xdim=(u16)x;
        ((pixo_image*)retval)->ydim=(u16)y;
    }
    return(retval);
}

void imo_freebuffer(void* buff) {
    // this is the free method that stays compatible with the allocation
    //  method of imo_getbuffer()

    // however we do this, we don't want to appeal to thread messaging to do it
    //  alloc and free on these resources should be executable from within this thread
    appfree(buff);

}

void imo_cleanup(thread2_ctrl* thr_ptr) {
//
// on a crash, we should free any imo buffers to avoid memory leaks
//  we should free system resources
//   at this point it is a best effort mechanism, messaging may be down,
//    appmalloc and appfree my be broken
// however we do this, we don't want to appeal to thread messaging to do it
//  alloc and free on these resources should be executable from within this thread
//   there's no need to signal up any new errors
//
    int done=0;
    void *vptr;
    // behavior may be different on a crash exit...
    // if (!(ACptr->hwsema_crsh)) {

    // free unused pixo buffers in pool
    while (!done) {
        vptr=(void*)lnklst_fpop(&(thr_ptr->imopool));
        if (vptr) {
            imo_freebuffer(vptr);
        }
        else {
            done=1;
        }
    }
}

//------------------------------------------------------------------------------

int msg_imo_refill(void* ioptr) {
    // eventually, the display update process should return the pixo_image
    //  so we can replenish the imopool
    //   if it takes too long, the pool won't get refilled
    //    which eventually leads to incoming frames getting dropped
    // this naturally regulates the throughput frame rate whether or not we can
    //  keep up with the delivery speed from the sensor
    // this is a message handler to accept return of pixo_image
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    //               - arg0 pixo_image being returned from display update
    //
    int error;
    thread2_ctrl* thr_ptr;
    pixo_image* pixoptr;
    thr_ptr=(thread2_ctrl*)(thr_ctrl+2);
    pixoptr=(pixo_image*)(*(((s64*)ioptr)+1));
    lnklst_fins((&(thr_ptr->imopool)),((lnklst*)pixoptr));
    // return int 0 - auto destroy message
    return(0);
}

int imo_refill(pixo_image* pixoptr) {
    // returns 1 on crash abort, else 0
    // normal return of pixo_image (from any thread)
    msg_queue* queue;
    int retval;
    // the message gets passed to MU_thread2
    queue=&((thr_ctrl+2)->mqueue);
    retval=msgexec_gblkvoid(queue,msg_imo_refill,((s64)pixoptr),0,0);
    return(retval);
}

//------------------------------------------------------------------------------

int msg_cofl_refill(void* ioptr) {
    // eventually, the downstream traffic process should return the coflptr
    //  so we can replenish the cfepool
    //
    int error;
    cofl_image* coflptr;
    coflptr=(cofl_image*)(*(((s64*)ioptr)+1));
    // so just recycle this image buffer
    lnklst_fins(&(ACptr->coflpool),(lnklst*)coflptr);
    // return int 0 - auto destroy message
    return(0);
}

int cofl_refill(cofl_image* coflptr) {
    // returns 1 on crash abort, else 0
    // normal return of cofl_image (from any thread)
    msg_queue* queue;
    int retval;
    // the message gets passed to MU_thread2
    queue=&((thr_ctrl+2)->mqueue);
    retval=msgexec_gblkvoid(queue,msg_cofl_refill,((s64)coflptr),0,0);
    return(retval);
}

//------------------------------------------------------------------------------

int msg_ibm_msk(void* ioptr) {
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    u8 mskval;
    thread2_ctrl* thr_ptr;
    mskval=(u8)(*(((s64*)ioptr)+1));
    ACptr->ibm_msk=mskval;
    // may have to add calling arg qualifiers...
    if (mskval==IBM_MSK_NONE) {
        //  ...but generally, this only happens going from ImgVw to texture view state
        //   so then we want clobber any stale ImgVw image
        //    force test pattern insertion
        thr_ptr=(thread2_ctrl*)(thr_ctrl+2);
        // pixel value
        thr_ptr->tpxptr=(void*)(0xFF000000);
        // mode - blnkscreen
        thr_ptr->tpmode=0;
        // enable test pattern generation
        thr_ptr->tpflg=1;
    }
    // return int 0 - auto destroy message
    return(0);
}

int set_ibm_msk(u32 mskval) {
    // returns 1 on crash abort, else 0
    //  signals thread2 to permit only output frames of msk
    msg_queue* queue;
    int retval;
    // the message gets passed to MU_thread2
    queue=&((thr_ctrl+2)->mqueue);
    retval=msgexec_gblkvoid_a2(queue,msg_ibm_msk,(s64)mskval,0);
    return(retval);
}

//------------------------------------------------------------------------------

int msg_osnapshot(void* ioptr) {
    // this is a message handler to take a snapshot output file at the next opportunity
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    //               - no args
    thread2_ctrl* thr_ptr;
    int n;
    n=(int)(*(((s64*)ioptr)+1));
    thr_ptr=(thread2_ctrl*)(thr_ctrl+2);
    thr_ptr->osnapreq=(thr_ptr->osnapreq)+n;
    // return int 0 - auto destroy message
    return(0);
}

int req_osnapshot(int n) {
    // returns 1 on crash abort, else 0
    // request n (hopefully consecutive) snapshots from another thread
    msg_queue* queue;
    int retval;
    // the message gets passed to MU_thread2 for output files
    queue=&((thr_ctrl+2)->mqueue);
    retval=msgexec_gblkvoid(queue,msg_osnapshot,(s64)n,0,0);
    return(retval);
}

//------------------------------------------------------------------------------

void tp_monitor(void) {
    // monitors test pattern generation to ImgVw
    thread2_ctrl* thr_ptr;
    pixo_image* pixoptr;
    u32* dst;
    u32 oy;
    u32 ox;
    u32 pix;
    thr_ptr=(thread2_ctrl*)(thr_ctrl+2);
    if ((ACptr->ibm_msk)==IBM_MSK_NONE) {
        // this is only active for IBM_MSK_NONE
        if (thr_ptr->tpflg) {
            // test pattern generation request on deck
            if (ACptr->vwdim) {
                // can only do this if the ImgVw size has been established
                pixoptr=(pixo_image*)lnklst_fpop(&(thr_ptr->imopool));
                if (pixoptr) {
                    // have an image carrier available
                    // cancel the request - we'll fill it now
                    thr_ptr->tpflg=0;
                    switch (thr_ptr->tpmode) {
                        // case 1:
                        //    break;
                        default:
                            // includes case 0 - blankscreen
                            //  more generally, fill screen from pixel cast at .tpptr
                            dst=(u32*)(((char*)pixoptr)+64);
                            // right sized output dimensions
                            oy=ACptr->vwdim;
                            ox=oy&0xFFFF;
                            oy=oy>>16;
                            // total pixels
                            oy=ox*oy;
                            // everywhere pixel is taken from tpptr
                            pix=(u32)(thr_ptr->tpxptr);
                            while (oy) {
                                *dst++=pix;
                                oy--;
                            }
                            break;
                    }
                    // deploy the test pattern pixo_image
                    hsp_lnklst_fins((int*)(&(thr_ptr->hwsema_rdy)),(&(thr_ptr->imo_rdy)),((lnklst*)pixoptr));
                    // VERBOSITY - debug
                    // LOGI("tp_monitor() issuing test pattern\n");
                }
                // else - can't service this request yet
                //  wait for carrier
            }
            // else - ImgVw size still unknown
            //  you can't service this request yet...
            //   so leave it on deck and exit
        }
        // else - no test pattern request active
    }
    else {
        // any other IBM_MSK_<mode> cancels any test pattern requests
        thr_ptr->tpflg=0;
    }
}

//------------------------------------------------------------------------------

int msg_raw2pixo_frame(void* ioptr) {
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    //               - arg0 cfei_image
    //
    thread2_ctrl* thr_ptr;
    cfei_image* cfeiptr;
    pixo_image* pixoptr;
    u16* src;
    u16* srcln;
    u32* dst;
    u32 oy;
    u32 ox;
    u32 iy;
    u32 ix;
    u32 ilpitch;
    u32 ishft;
    u32 ipix;
    u32 opix;
    int error;
    //
    // // monitoring option
    // u32 pixmsk;
    // u32 pixmax;
    // u32 pixtrnc;
    //
    thr_ptr=(thread2_ctrl*)(thr_ctrl+2);
    // bump number of arriving frames
    (thr_ptr->pixocnt)++;
    // cast the cfei
    cfeiptr=(cfei_image*)(*(((s64*)ioptr)+1));
    if ((ACptr->ibm_msk)==IBM_MSK_RAW) {
        // we're passing raw frames only
        pixoptr=(pixo_image*)lnklst_fpop(&(thr_ptr->imopool));
        if (pixoptr) {
            // have output carrier to complete transaction
            // for building pix map to image update area
            //  we don't need to carry any part of the header
            //   if we want to issue an output file...
            //    that's a separate consideration, and probably should carry
            //     a copy of the raw_filhdr on the fork/clone deployment
            src=(u16*)(((char*)cfeiptr)+64);
            dst=(u32*)(((char*)pixoptr)+64);
            // right sized output dimensions
            oy=ACptr->vwdim;
            ox=oy&0xFFFF;
            oy=oy>>16;
            // input dimensions
            iy=cfeiptr->ydim;
            ix=cfeiptr->xdim;
            ilpitch=ix;
            // iy calculates cropping home y shift
            //   keep mosaic alignment ?
            //    would need to calculate that based on cfeiptr.medge
            iy=(iy-oy)>>1;
            src+=(iy*ilpitch);
            // ix calculates cropping home x shift
            //   keep mosaic alignment ?
            //    would need to calculate that based on cfeiptr.medge
            ix=(ix-ox)>>1;
            src+=ix;
            // 8 bit justify
            ishft=(cfeiptr->pbpp)-8;
            // transfer the cropped area
            //
            // // monitoring option
            // pixmax=0;
            // pixmsk=(1<<(cfeiptr->pbpp))-1;
            // pixtrnc=0;
            //
            for (iy=0;iy<oy;iy++) {
                srcln=src;
                for (ix=0;ix<ox;ix++) {
                    // BPP sample -> 8 bit  -- no clamp detection needed on IMX214
                    ipix=(u32)(*srcln);
                    //
                    // // monitoring option
                    // if (ipix>pixmax) {
                    //    pixmax=ipix;
                    //    }
                    // if (ipix>pixmsk) {
                    //    ipix=pixmsk;
                    //    pixtrnc++;
                    //    }
                    //
                    ipix=ipix>>ishft;
                    // grayscale
                    opix=ipix|(ipix<<8)|(ipix<<16)|0xFF000000;
                    *dst=opix;
                    // update pointers
                    srcln++;
                    dst++;
                }
                src+=ilpitch;
            }
            //
            // // monitoring option
            // LOGI("msg_raw2pixo_frame() pixmsk=%04X pixmax=%04X pixtrnc=%08X\n",
            //       pixmsk,pixmax,pixtrnc);
            //
            hsp_lnklst_fins((int*)(&(thr_ptr->hwsema_rdy)),(&(thr_ptr->imo_rdy)),((lnklst*)pixoptr));
            // VERBOSITY - debug
            // LOGI("msg_raw2pixo_frame(jni.2) raw to pixo frame on deck\n");
        }
        else {
            // no output buffer available - have to drop the frame
            // bump number of dropped frames
            (thr_ptr->pixodrop)++;
            // VERBOSITY
            // LOGI("raw to pixo frame drop\n");
        }
    }
    else {
        // since we're not passing raw frames, it gets drpped
        // bump number of dropped frames
        (thr_ptr->pixodrop)++;
        // VERBOSITY
        // LOGI("raw to pixo frame masked\n");
    }
    // any way out returns the RAW image (buffer)
    //  we can detect a crash abort, but can't really do anything about it here
    error=cfe_refill(cfeiptr);
    // return int 0 - auto destroy message
    return(0);
}

int raw2pixo_fwrd(cfei_image* cfeiptr) {
    // returns 1 on crash abort, else 0
    //  direct RAW -> display shortcut
    msg_queue* queue;
    int retval;
    // the message gets passed to MU_thread2
    queue=&((thr_ctrl+2)->mqueue);
    // use custom message pass to avoid touching the raw_filhdr
    retval=msgexec_gblkvoid_a2(queue,msg_raw2pixo_frame,((s64)cfeiptr),0);
    return(retval);
}

//------------------------------------------------------------------------------

int cofl_file_image(cofl_image* coflptr) {
    // return value 0 indicates success
    //              1 indicates some crash recorded - error true
    //
    // this hands off the image to pass out Colour(RGB) capture file
    //  either here, or in subsequent called routine,
    //   the responsibility to evetually release the cofl image buffer
    //    travels with it
    //
    int error;
    ofil_carrier* ofil;
    rgb_filhdr* hdr;
    u8 ichar;
    // if traffic is blocking, we're wasting our time
    error=(int)utp_trafblk();
    if (!error) {
        // ofile system is accepting traffic
        ofil=(ofil_carrier*)gblkalloc();
        //ofil=(ofil_carrier*)lnklst_fpop(&(ACptr->ofilpool));
        if (ofil) {
            // we have a carrier for it
            ofil->ofltype=OFLTYP_COFL;
            // need logical transfer index (auto increment)
            ichar=ACptr->snprgb;
            ACptr->snprgb=(ichar+1)&0x0F;
            // text index
            ichar=hexASCII[ichar];
            snprgbot_fname[3]=ichar;
            // insert name
            strcpy((char*)(&(ofil->filname)),snprgbot_fname);
            // point header
            hdr=(rgb_filhdr*)(((char*)coflptr)+0x30);
            ofil->refptr=(char*)coflptr;
            // insert size header + pixels
            ofil->filsize=sizeof(rgb_filhdr)+((((u32)(hdr->xdim))*((u32)(hdr->ydim)))<<2);
            // ready to submit
            send_utpofile(ofil);
        }
        else {
            // no carrier, we have to drop it
            error=1;
            utp_dbgoutput("no ofil available\n");
        }
    }
    if (error) {
        // file is currently undeliverable, so discard it
        //  treat as void, return the error we already have
        // error=cofl_refill(coflptr);
        cofl_refill(coflptr);
    }
    return(error);
}

//------------------------------------------------------------------------------

int msg_hass2pixo_frame(void* ioptr) {
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    thread2_ctrl* thr_ptr;
    u32 hassspec;
    pixo_image* pixoptr;
    u16* srcr;
    u16* srcg;
    u16* srcb;
    u16* srcrl;
    u16* srcgl;
    u16* srcbl;
    u32 plnsep;
    u32* dst;
    u32 oy;
    u32 ox;
    u32 iy;
    u32 ix;
    u32 ilpitch;
    u32 r;
    u32 g;
    u32 b;
    u32 pix;
    int pshft;
    //-------------------
    s64 rfh0;
    s64 rfh1;
    // the raw_filhdr from the starting RAW frame is available here
    rfh0=(*(((s64*)ioptr)+1));
    rfh1=(*(((s64*)ioptr)+2));
    //  so if you need timestamp information to fork off file capture
    //   it's available here
    //-------------------
    // at start read spec from thread 3
    hassspec=ACptr->hassoblk;
    // it MUST be non-0
    if (hassspec) {
        // the spec may modify what we do to render HASS output image
        thr_ptr=(thread2_ctrl*)(thr_ctrl+2);
        // bump number of arriving frames
        (thr_ptr->pixocnt)++;
        if ((ACptr->ibm_msk)==IBM_MSK_HASS) {
            // we're passing hass frames only
            // need an output carrier available
            pixoptr=(pixo_image*)lnklst_fpop(&(thr_ptr->imopool));
            if (pixoptr) {
                // have output carrier to complete transaction
                // for building pix map to image update area
                //  we don't need to carry any sort of header information
                //   if we want to issue an output file...
                //    that's a separate consideration, and probably should carry
                //     some reference information in the fork/clone deployment
                //
                // for now, only 3 plane -> RGB
                //
                srcb=(u16*)(Hassptr->planesOutput);
                dst=(u32*)(((char*)pixoptr)+64);
                // right sized output dimensions
                oy=ACptr->vwdim;
                ox=oy&0xFFFF;
                oy=oy>>16;
                // input dimensions
                iy=Hassptr->height;
                ix=Hassptr->width;
                ilpitch=ix;
                //
                // the 3 planes are assumed 8 bit pixels
                //  the gain stage of Hass process should be tuned insure this
                //   and provide the necessary clamping upstream
                //    -- if not, we can modify what we do here, or make it configurable
                //
                // iy calculates cropping home y shift
                //   keep mosaic alignment ?
                //    would need to calculate that based on mosaic edge size
                iy=(iy-oy)>>1;
                srcb+=(iy*ilpitch);
                // ix calculates cropping home x shift
                //   keep mosaic alignment ?
                //    would need to calculate that based on mosaic edge size
                ix=(ix-ox)>>1;
                srcb+=ix;
                // other planes get same starting crop offsets
                plnsep=(Hassptr->height)*(Hassptr->width);
                srcg=srcb+plnsep;
                srcr=srcg+plnsep;
                // adjustment for planes bpp
                pshft=(Hassptr->okbpp)-8;
                for (iy=0;iy<oy;iy++) {
                    srcrl=srcr;
                    srcgl=srcg;
                    srcbl=srcb;
                    for (ix=0;ix<ox;ix++) {
                        // collect input
                        r=((u32)(*srcrl))>>pshft;
                        g=((u32)(*srcgl))>>pshft;
                        b=((u32)(*srcbl))>>pshft;
                        // set output
                        // I've actually ordered them bgr at planesOutput (incrementally)
                        pix=(r<<16)|(g<<8)|b|0xFF000000;
                        *dst=pix;
                        // update pointers
                        srcrl++;
                        srcgl++;
                        srcbl++;
                        dst++;
                    }
                    srcr+=ilpitch;
                    srcg+=ilpitch;
                    srcb+=ilpitch;
                }
                hsp_lnklst_fins((int*)(&(thr_ptr->hwsema_rdy)),(&(thr_ptr->imo_rdy)),((lnklst*)pixoptr));
                // VERBOSITY - debug
                // LOGI("msg_hass2pixo_frame(jni.2) hass to pixo frame on deck\n");
                //
                //--- outputting color file is purely optional
                //     and may be different size than ImgVw bitmap
                //
                if (thr_ptr->osnapreq) {
                    // there's a request for output file snapshot
                    pixoptr=(pixo_image*)lnklst_fpop(&(ACptr->coflpool));
                    if (pixoptr) {
                        // we have image carrier (which is really cofl_image*)
                        srcb=(u16*)(Hassptr->planesOutput);
                        srcg=srcb+plnsep;
                        srcr=srcg+plnsep;
                        // construct the header
                        //  as default, get the header from the original RAW cfe
                        *((s64*)(((char*)pixoptr)+0x30))=rfh0;
                        *((s64*)(((char*)pixoptr)+0x38))=rfh1;
                        //
                        // nested rgb_filhdr (oconsole.h)
                        // u16          xdim;        // 0x30
                        // u16          ydim;        // 0x32
                        // u32          tstmp;       // 0x34
                        // u32          tdate;       // 0x38
                        // u8           pbpp;        // 0x3C
                        // u8           spr;         // 0x3D <tbd>
                        // u8           pmod;        // 0x3E pmod - could encode mosaic type
                        // u8           flgs;        // 0x3F flgs could be context dependent
                        //
                        // dst points rgb_filhdr
                        dst=(u32*)(((char*)pixoptr)+0x30);
                        // copy xdim ydim
                        ix=(u32)(*((u32*)(&(thr_ptr->imodim))));
                        *dst=ix;
                        // time and date stamp stay as they are
                        // overwrite pbpp=8 (don't support any other modes), u8spr=0
                        *((u16*)(dst+3))=0x0008;
                        // keep pmod and flgs inherited from RAW cfe
                        // total pixels
                        oy=(thr_ptr->imobytes)>>2;
                        // output pixel area
                        dst=(u32*)(((char*)pixoptr)+0x40);
                        for (iy=0;iy<oy;iy++) {
                            // collect input
                            r=((u32)(*srcr))>>pshft;
                            g=((u32)(*srcg))>>pshft;
                            b=((u32)(*srcb))>>pshft;
                            // set output (unused bits are all 0)
                            //  note this is backward from Android pixel map
                            pix=(b<<16)|(g<<8)|r;
                            *dst=pix;
                            // update pointers
                            srcr++;
                            srcg++;
                            srcb++;
                            dst++;
                        }
                        // you can message/send it to another thread to process the file
                        //  but we do it all locally
                        // error=cofl_file_image((cofl_image*)pixoptr);
                        //  call as void
                        cofl_file_image((cofl_image*)pixoptr);
                        // the handoff is now responsible to return the carrier to the pool
                        // decrement outstanding snapshot requests
                        thr_ptr->osnapreq=(thr_ptr->osnapreq)-1;
                    }
                }
            }
            else {
                // no output buffer available - have to drop the frame
                // bump number of dropped frames
                (thr_ptr->pixodrop)++;
                // VERBOSITY
                // LOGI("hass to pixo frame drop\n");
            }
        }
        else {
            // no longer passing hass frames - late frame gets dropped
            // bump number of dropped frames
            (thr_ptr->pixodrop)++;
            // VERBOSITY
            // LOGI("hass to pixo frame masked\n");
        }
        // any way out has to clear the thread2 handshake blocker
        ACptr->hassoblk=0;
    }
    // else hassspec is NOT set... don't know what to do...
    // we're already signalling done...
    // return int 0 - auto destroy message
    return(0);
}

int hass2pixo_fwrd(u32 hassspec,s64 hdr0,s64 hdr1) {
    // returns 1 on crash abort, else 0
    //  thread2 signals thread3 that hass results are ready for display
    msg_queue* queue;
    int retval;
    // the message gets passed to MU_thread2
    queue=&((thr_ctrl+2)->mqueue);
    // use custom message pass to avoid touching any filhdr
    //  there really are no calling arguments
    //   pass spec which sets blocker, marking this path busy
    ACptr->hassoblk=hassspec;
    retval=msgexec_gblkvoid_a2(queue,msg_hass2pixo_frame,hdr0,hdr1);
    return(retval);
}

//------------------------------------------------------------------------------


int set_hass_gains(int N,float* tbl) {
    // returns error true
    //   -- N==0, N>maxHassComp
    //
    int error=0;
    float* dst;
    if (!((N>0) && (N<=maxHassComp))) {
        error=1;
    }
    if (error) {
        LOGE("Error - set_hass_gains() illegal args\n");
    }
    else {
        // commit table to Hassctrl
        dst=(float*)(&(Hassptr->postGainPln[0]));
        while (N) {
            *dst++=*tbl++;
            N--;
        }
    }
    return(error);
}

void set_hass_mode(int Npln,int gammaEnab,int nrTaps) {
    // set the operation mode
    Hassptr->numPlane=Npln;
    Hassptr->gamma_en=gammaEnab;
    if (nrTaps<0) {
        Hassptr->nrTapSize=0;
    }
    else {
        if (nrTaps>9) {
            Hassptr->nrTapSize=9;
        }
        else {
            Hassptr->nrTapSize=nrTaps;
        }
    }
}

int set_hass_lambda(int N,int* tbl) {
    // returns error true
    //   -- N==0, N>maxHassComp
    //   -- any of the lambda are not in range of
    //        LAMBDA_350,            // 0x00 ...
    //        LAMBDA_1000            // 0x41
    //
    int error=0;
    int i;
    int tstlmbda;
    int* dst;
    if ((N>0) && (N<=maxHassComp)) {
        for (i=0;i<N;i++) {
            tstlmbda=*(tbl+i);
            if (!((tstlmbda>=LAMBDA_350) && (tstlmbda<=LAMBDA_1000))) {
                error=error|0x01;
            }
        }
    }
    else {
        error=1;
    }
    if (error) {
        LOGE("Error - set_hass_lambda() illegal args\n");
    }
    else {
        // commit table to Hassctrl
        dst=(int*)(&(Hassptr->waveLngthTbl[0]));
        while (N) {
            *dst++=*tbl++;
            N--;
        }
    }
    return(error);
}

//------------------------------------------------------------------------------

void* alloc_SVM(int bytesize) {
    void* retval;
    // must succeed and return valid pointer
    //  or set crash code and return NULL pointer
    //   should execute autonomously - no appeals to messaging
    retval=clSVMAlloc(CL_context,CL_MEM_READ_WRITE,bytesize,0);
    // VERBOSITY
    // LOGI("alloc_SVM(%08X) returned %016X\n",bytesize,(u64)(retval));
    if (!retval) {
        thread_set_crash(CRASH_OPENCL);
    }
    return(retval);
}

void free_SVM(void* ptr) {
    if (ptr) {
        clSVMFree(CL_context,ptr);
    }
}

//------------------------------------------------------------------------------

int hass_fConfig_SVM(hassMosaic* fConfig) {
    // returns error true
    //  generally gets called once at startup,
    //   (seldom on filter update)
    int error;
    int* dst;
    dst=(int*)(((char*)(Hassptr->svmbase))+SVMOFF_FCONFIG);
    // to open up the SVM
    error=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_WRITE,
                          (void*)dst,SVMMAX_FCONFIG,0,NULL,NULL);
    if (!error) {
        // opened 256 int window for growth, but only sending 64 int's
        memcpy((void*)dst,(void*)(&(fConfig[0][0])),256);
        // close the SVM window
        error=clEnqueueSVMUnmap(CL_queue,(void*)dst,0,NULL,NULL);
    }
    if (error) {
        LOGE("Error - hass_fConfig_SVM()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

// note this is dependent on fConfig
//  a change in fConfig ripples to a re-calculation of hass_coef[]
int hass_coef_SVM(float* coef) {
    // returns error true
    //  generally gets called once at startup,
    //   (seldom on filter update)
    int error;
    float* dst;
    dst=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_HCOEF);
    // to open up the SVM
    error=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_WRITE,
                          (void*)dst,SVMMAX_HCOEF,0,NULL,NULL);
    if (!error) {
        // opened 16KB float window for growth, but only sending 4KB
        memcpy((void*)dst,(void*)coef,4096);
        // close the SVM window
        error=clEnqueueSVMUnmap(CL_queue,(void*)dst,0,NULL,NULL);
    }
    if (error) {
        LOGE("Error - hass_coef_SVM()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

int shrnk_coef_SVM(float* coef) {
    // returns error true
    //  generally gets called once at startup,
    //   (seldom on filter update)
    int error;
    float* dst;
    dst=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_SHRCOE);
    // to open up the SVM
    error=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_WRITE,
                          (void*)dst,SVMMAX_SHRCOE,0,NULL,NULL);
    if (!error) {
        // opened 1KB float window for growth, but only sending 8 samples
        memcpy((void*)dst,(void*)coef,8*sizeof(float));
        // close the SVM window
        error=clEnqueueSVMUnmap(CL_queue,(void*)dst,0,NULL,NULL);
    }
    if (error) {
        LOGE("Error - shrnk_coef_SVM()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

int hass_parms_SVM(void) {
    // returns error true
    //
    //  see _frameA or _frameM for hass_set_kparms()
    int error;
    char* dst;
    dst=(char*)(Hassptr->svmbase);
    // to open up the SVM
    error=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_WRITE,
                          (void*)dst,SVMMAX_PRMS,0,NULL,NULL);
    if (!error) {
        // insert the frame data
        hass_set_kparms();
        // close the SVM window
        error=clEnqueueSVMUnmap(CL_queue,(void*)dst,0,NULL,NULL);
    }
    if (error) {
        LOGE("Error - hass_parms_SVM()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

int hass_reconcoe_SVM(void) {
    // returns error true
    int error;
    float* rcnCoe;
    rcnCoe=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_RECON);
    // to open up the SVM
    error=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_WRITE,
                          (void*)rcnCoe,SVMMAX_RECON,0,NULL,NULL);
    if (!error) {
        // insert the frame data
        hass_rcnCoe_setup((reconCoeTbl*)rcnCoe);
        // close the SVM window
        error=clEnqueueSVMUnmap(CL_queue,(void*)rcnCoe,0,NULL,NULL);
    }
    if (error) {
        LOGE("Error - hass_reconcoe_SVM()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

int comb_coef_SVM(float* coef) {
    // returns error true
    //  generally gets called once at startup,
    //   (seldom on filter update)
    int error;
    float* dst;
    dst=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_CMBCOE);
    // to open up the SVM
    error=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_WRITE,
                          (void*)dst,SVMMAX_CMBCOE,0,NULL,NULL);
    if (!error) {
        // opened 1KB float window for growth, but only sending 6 samples
        memcpy((void*)dst,(void*)coef,6*sizeof(float));
        // close the SVM window
        error=clEnqueueSVMUnmap(CL_queue,(void*)dst,0,NULL,NULL);
    }
    if (error) {
        LOGE("Error - comb_coef_SVM()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

int zoom_coef_SVM(float* coef) {
    // returns error true
    //  generally gets called once at startup,
    //   (seldom on filter update)
    int error;
    float* dst;
    dst=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_ZMCOE);
    // to open up the SVM
    error=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_WRITE,
                          (void*)dst,SVMMAX_ZMCOE,0,NULL,NULL);
    if (!error) {
        // opened 1KB float window for growth, but only sending 6 samples
        memcpy((void*)dst,(void*)coef,8*sizeof(float));
        // close the SVM window
        error=clEnqueueSVMUnmap(CL_queue,(void*)dst,0,NULL,NULL);
    }
    if (error) {
        LOGE("Error - zoom_coef_SVM()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

int hass_gains_SVM(void) {
    // returns error true
    int error;
    float* dst;
    float* src;
    dst=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLGAIN);
    // to open up the SVM
    error=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_WRITE,
                          (void*)dst,SVMMAX_PLGAIN,0,NULL,NULL);
    if (!error) {
        // insert the gain data
        src=(float*)(&(Hassptr->postGainPln[0]));
        // whether or not Npln is set for max (16),
        //  always transfer all 16 floats -> 64 bytes
        memcpy((void*)dst,(void*)src,16*sizeof(float));
        // close the SVM window
        error=clEnqueueSVMUnmap(CL_queue,(void*)dst,0,NULL,NULL);
    }
    if (error) {
        LOGE("Error - hass_gains_SVM()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

int hass_gamma_SVM(char* src,int bytesize) {
    // returns error true
    //  the source buffer is char*, in dimension bytes
    //   though it is likely an array of short
    int error;
    float* dst;
    dst=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_GAMMA);
    // to open up the SVM
    error=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_WRITE,
                          (void*)dst,SVMMAX_GAMMA,0,NULL,NULL);
    if (!error) {
        // insert the data
        memcpy((void*)dst,(void*)src,bytesize);
        // close the SVM window
        error=clEnqueueSVMUnmap(CL_queue,(void*)dst,0,NULL,NULL);
    }
    if (error) {
        LOGE("Error - hass_gamma_SVM()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

//------------------------------------------------------------------------------

int set_hass_imgdim(int x,int y) {
    // set has image plane dimensions
    // returns error true, 0 OK, no faults
    int error=0;
    // allocate output plane(s)
    Hassptr->width=x;
    Hassptr->height=y;
    Hassptr->planesOutput=(short*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNOUT);
    Hassptr->planeGuide=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNGD);

    // ZZZZZ
    // all intermediate buffers have to be right-sized
    LOGI("set_hass_imgdim() allocated all buffers\n");

    if (error) {
        LOGI("set_hass_imgdim error code 0x%04X\n",error);
    }
    return(error);
}

void reset_hass_imgdim(void) {
    // if you change image dimensions,
    //   before you resize and set new set_hass_imgdim(x,y) that sets up size
    //    dependent buffers, you should free the old buffers based on old sizes
    Hassptr->width=0;
    Hassptr->height=0;
    if (Hassptr->planesOutput) {
        Hassptr->planesOutput=(short*)0;
    }
    if (Hassptr->planeGuide) {
        Hassptr->planeGuide=(float*)0;
    }
    if (Hassptr->planeShrink) {
        Hassptr->planeShrink=(float*)0;
    }
    if (Hassptr->planeNR) {
        Hassptr->planeNR=(float*)0;
    }
    if (Hassptr->planeRecon) {
        Hassptr->planeRecon=(float*)0;
    }
    if (Hassptr->planeComb) {
        Hassptr->planeComb=(float*)0;
    }
    if (Hassptr->planeProc) {
        Hassptr->planeProc=(float*)0;
    }
}

int resize_hass_imgdim(int x,int y) {
    // returns error true, 0 OK, no faults
    int retval=0;
    reset_hass_imgdim();
    retval=set_hass_imgdim(x,y);
    return(retval);
}

//-----------------------------------------------------------------------------

void gen_hass_coef(void) {
    // this is really static from startup, and only needs to be calculated once
    //  (hass_coef[] could be calculated offline at compile time)
    int col;
    int yy;
    int xx;
    int y;
    int x;
    int cx;
    int cy;
    int pcx;
    int pcy;
    int tapnum;
    int coe_indx;
    tapnum=repUnitSize+1;

    //calculate number of same color
    for(col=0;col<numOfHassCh;col++) {
        hass_power[col] = (float)0.0;
    }
    for (yy=0;yy<repUnitSize;yy++) {
        for(xx=0;xx<repUnitSize;xx++) {
            hass_power[filterConfig[yy][xx]]+= (float)1.0;
        }
    }

    //calculate number of same color in taps
    for (y=0;y<(repUnitSize*repUnitSize*numOfHassCh);y++) {
        hass_coef[y] = (float)0.0;
    }
    for (y=0;y<repUnitSize;y++) {
        for (x=0;x<repUnitSize;x++) {
            for(col=0;col<numOfHassCh;col++) {
                coe_indx=(((y*repUnitSize)+x)*numOfHassCh)+col;
                for(yy=0;yy<tapnum;yy++) {
                    for(xx=0;xx<tapnum;xx++) {
                        cx = (x + xx - tapnum/2);
                        cy = (y + yy - tapnum/2);
                        pcx = (cx + repUnitSize)%repUnitSize;
                        pcy = (cy + repUnitSize)%repUnitSize;
                        if (col == filterConfig[pcy][pcx]){
                            hass_coef[coe_indx] += (float)1.0;
                        }
                    }
                }
            }
            //combine power and coef
            for(col=0;col<numOfHassCh;col++) {
                coe_indx=(((y*repUnitSize)+x)*numOfHassCh)+col;

// QQQQQ  (test effect on image)
                hass_coef[coe_indx] = hass_power[col] / hass_coef[coe_indx];
                // hass_coef[coe_indx] =  (float)1.0;

                // VERBOSITY
                // printf("y=%d x=%d col=%d indx=%d value=%10.3e\n",y,x,col,coe_indx,hass_coef[coe_indx]);
            }
        }
    }
}

void gen_shrink_coef(void) {
    int i;
    float ds;
    for (i=0;i<repUnitSize;i++) {
        ds = (float)i/(float)repUnitSize;
        // bilinear
        shrink_coef[i]=((float)1.0)-ds;
        // bicubic
        //shrink_coef[i] = ((float)2.0)*ds*ds*ds - ((float)3.0*ds*ds + ((float)1.0);
    }
}

void gen_zoom_coef(void) {
    int i;
    float ds;
    for (i=0;i<repUnitSize;i++) {
        ds = (float)i/(float)repUnitSize;
        // bilinear
        zoom_coef[i]=((float)1.0)-ds;
        // bicubic
        //zoom_coef[i] = ((float)2.0)*ds*ds*ds - ((float)3.0*ds*ds + ((float)1.0);
    }
}

void gen_combine_coef(void) {
    int i;
    int combine_tap_center;
    int combine_tap_side;
    int combine_tap;
    float ds;

    combine_tap_center = repUnitSize/2;
    combine_tap_side   = repUnitSize/4;
    combine_tap = combine_tap_center + combine_tap_side;
    // this is always visible without constructor/destructor
    // float combine_coeff[6];
    for(i = 0; i < combine_tap_center; i++) {
        combine_coef[i] = ((float)1.0);
    }
    for (i = combine_tap_center ; i < combine_tap; i++) {
        ds = (float)(i-combine_tap_center) / (float)combine_tap_center;
        combine_coef[i] = ((float)1.0) - ds;
    }
}

// you have to call Hass_init somewhere at startup
void Hass_init(void) {
    int error;
    // assumed fixed constants
    Hassptr->mosaicptr=&filterConfig;
    Hassptr->waveMix=&lutWaveLength;
    Hassptr->gammaptr=gammaLut;
    // this is done per frame, through hiframe_accept<_g>
    //  by way of  int hass_kfrm_admit(void)
    //   which calls   void hass_rcnCoe_setup(reconCoeTbl* rcncoeptr)
    //    Hassptr->reconCoeptr= &reconCoeffs or SVM address;
    //
    // need some default values
    //
    // INFO
    // some usefull suggestions
    //
    //  // RGB
    //  waveLngthTbl[0]=LAMBDA_600;
    //  waveLngthTbl[1]=LAMBDA_530;
    //  waveLngthTbl[2]=LAMBDA_460;
    //  // R + IR
    //  waveLngthTbl[0]=LAMBDA_650;
    //  waveLngthTbl[1]=LAMBDA_750;
    //
    // we dfefault RGB

    // Npln gammaEnab nrTaps
    // set_hass_mode(   3,     1,      5);
    // set_hass_mode(   3,     1,      3);
    set_hass_mode(   3,     1,      1);

    // not concerned about error here
    //  setting startup config should know what they are doing...
    set_hass_lambda(hass_dflt_plns,hass_dflt_lmbda);
    set_hass_gains(hass_dflt_plns,hass_dflt_gains);
    // only do this once
    gen_hass_coef();
    gen_shrink_coef();
    gen_zoom_coef();
    gen_combine_coef();
    // dimensions
    error=set_hass_imgdim(H_def_WIDTH,H_def_HEIGHT);
}

//-----------------------------------------------------------------------------

int gen_tst_inframe(void) {
    int x;
    int y;
    short* dst;
    short pix;
    int dim;
    int retval=0;
    if ((f_rawin=fopen(rawin_fname,"rb")) == NULL) {
        LOGE("ERROR: could not open %s\n",rawin_fname);
        retval=HERROR_FILE_OP;
    }
    else {
        fread((void*)(&dim),1,4,f_rawin);
        if (dim==0x0455079C) {
            //right sized for expectations
            // was drop 7 lines
            //    fseek(f_rawin,0x6a88,SEEK_CUR);
            // only drop 2 lines
            fseek(f_rawin,0x1E70,SEEK_CUR);
            dst=tst_inframe;
            // NOTE: invert the frame
            //    dst+=((H_def_WIDTH*H_def_HEIGHT)-1);
            // non-inverted frame - do nothing
            for (y=0;y<H_def_HEIGHT;y++) {
                for (x=0;x<H_def_WIDTH;x++) {
                    fread((void*)(&pix),1,2,f_rawin);
                    *dst=pix;
                    // NOTE: invert the frame - decremental
                    //          dst--;
                    // NOTE: invert the frame - incremental
                    dst++;
                }
                // between lines 28 pixels is 56 bytes
                fseek(f_rawin,0x38,SEEK_CUR);
            }
        }
        else {
            LOGE("ERROR: %s indicates size 0x%08X\n",rawin_fname,dim);
            retval=1;
        }
        fclose(f_rawin);
    }
    return(retval);
}

int dump_hass_coef(void) {
    int len;
    float* src;
    int retval=0;
    hasscoef_fname[25]=testrec_key;
    if ((f_hasscoef=fopen(hasscoef_fname,"wb")) == NULL) {
        LOGE("ERROR: could not open %s\n",hasscoef_fname);
        retval=HERROR_FILE_OP;
    }
    else {
        src=hass_coef;
        len=repUnitSize*repUnitSize*numOfHassCh*sizeof(float);
        fwrite((void*)src,1,len,f_hasscoef);
        fclose(f_hasscoef);
    }
    return(retval);
}

int dump_reconCoeffs(float* src) {
    int len;
    int retval=0;
    reconcoe_fname[25]=testrec_key;
    if ((f_reconcoe=fopen(reconcoe_fname,"wb")) == NULL) {
        LOGE("ERROR: could not open %s\n",reconcoe_fname);
        retval=HERROR_FILE_OP;
    }
    else {
        // now calling argument
        // src=(float*)(&(reconCoeffs[0][0]));
        len=(Hassptr->numPlane)*numOfHassCh*sizeof(float);
        fwrite((void*)src,1,len,f_reconcoe);
        fclose(f_reconcoe);
    }
    return(retval);
}

int dmp_SVM_reconCoeffs(void) {
    // returns error true
    int error;
    float* src;
    src=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_RECON);
    // to open up the SVM
    error=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_READ,
                          (void*)src,SVMMAX_RECON,0,NULL,NULL);
    if (!error) {
        error=dump_reconCoeffs(src);
        // close the SVM window (discard error)
        clEnqueueSVMUnmap(CL_queue,(void*)src,0,NULL,NULL);
    }
    if (error) {
        LOGE("Error - dmp_SVM_reconCoeffs()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

int dump_planeGuide(float* src) {
    int len;
    int retval=0;
    plnguide_fname[25]=testrec_key;
    if ((f_plnguide=fopen(plnguide_fname,"wb")) == NULL) {
        LOGE("ERROR: could not open %s\n",plnguide_fname);
        retval=HERROR_FILE_OP;
    }
    else {
        // now calling argument
        // src=Hassptr->planeGuide;
        len=(Hassptr->height)*(Hassptr->width)*sizeof(float);
        fwrite((void*)src,1,len,f_plnguide);
        fclose(f_plnguide);
    }
    return(retval);
}

int dmp_SVM_planeGuide(void) {
    // returns error true
    int error;
    float* src;
    src=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNGD);
    // to open up the SVM
    error=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_READ,
                          (void*)src,SVMMAX_PLNGD,0,NULL,NULL);
    if (!error) {
        error=dump_planeGuide(src);
        // close the SVM window (discard error)
        clEnqueueSVMUnmap(CL_queue,(void*)src,0,NULL,NULL);
    }
    if (error) {
        LOGE("Error - dmp_SVM_planeGuide()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

int dump_planeShrink(float* src) {
    int len;
    int retval=0;
    plnshrink_fname[25]=testrec_key;
    if ((f_plnshrink=fopen(plnshrink_fname,"wb")) == NULL) {
        LOGE("ERROR: could not open %s\n",plnshrink_fname);
        retval=HERROR_FILE_OP;
    }
    else {
        // now calling arg
        // src=Hassptr->planeShrink;
        len=numOfHassCh*(((Hassptr->height)/8)+1)*(((Hassptr->width)/8)+1)*sizeof(float);
        fwrite((void*)src,1,len,f_plnshrink);
        fclose(f_plnshrink);
    }
    return(retval);
}

int dmp_SVM_planeShrink(void) {
    // returns error true
    int error;
    float* src;
    src=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNSHR);
    // to open up the SVM
    error=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_READ,
                          (void*)src,SVMMAX_PLNSHR,0,NULL,NULL);
    if (!error) {
        error=dump_planeShrink(src);
        // close the SVM window (discard error)
        clEnqueueSVMUnmap(CL_queue,(void*)src,0,NULL,NULL);
    }
    if (error) {
        LOGE("Error - dmp_SVM_planeShrink()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

int dump_planeNRfnc(float* src) {
    int len;
    int retval=0;
    plnnrfnc_fname[25]=testrec_key;
    if ((f_plnnrfnc=fopen(plnnrfnc_fname,"wb")) == NULL) {
        LOGE("ERROR: could not open %s\n",plnnrfnc_fname);
        retval=HERROR_FILE_OP;
    }
    else {
        // now calling arg
        // src=Hassptr->planeNR;
        len=numOfHassCh*(((Hassptr->height)/8)+1)*(((Hassptr->width)/8)+1)*sizeof(float);
        fwrite((void*)src,1,len,f_plnnrfnc);
        fclose(f_plnnrfnc);
    }
    return(retval);
}

int dmp_SVM_planeNRfnc(void) {
    // returns error true
    int error;
    float* src;
    if ((Hassptr->nrTapSize)>1) {
        src=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNNR);
    }
    else {
        src=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNSHR);
    }
    // to open up the SVM
    error=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_READ,
                          (void*)src,SVMMAX_PLNNR,0,NULL,NULL);
    if (!error) {
        error=dump_planeNRfnc(src);
        // close the SVM window (discard error)
        clEnqueueSVMUnmap(CL_queue,(void*)src,0,NULL,NULL);
    }
    if (error) {
        LOGE("Error - dmp_SVM_planeNRfnc()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

int dump_planeRecon(float* src) {
    int len;
    int retval=0;
    plnrecon_fname[25]=testrec_key;
    if ((f_plnrecon=fopen(plnrecon_fname,"wb")) == NULL) {
        LOGE("ERROR: could not open %s\n",plnrecon_fname);
        retval=HERROR_FILE_OP;
    }
    else {
        // now calling arg
        // src=Hassptr->planeRecon;
        len=(Hassptr->numPlane)*(((Hassptr->height)/8)+1)*(((Hassptr->width)/8)+1)*sizeof(float);
        fwrite((void*)src,1,len,f_plnrecon);
        fclose(f_plnrecon);
    }
    return(retval);
}

int dmp_SVM_planeRecon(void) {
    // returns error true
    int error;
    float* src;
    src=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNRCN);
    // to open up the SVM
    error=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_READ,
                          (void*)src,SVMMAX_PLNRCN,0,NULL,NULL);
    if (!error) {
        error=dump_planeRecon(src);
        // close the SVM window (discard error)
        clEnqueueSVMUnmap(CL_queue,(void*)src,0,NULL,NULL);
    }
    if (error) {
        LOGE("Error - dmp_SVM_planeRecon()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

int dump_planeComb(float* src) {
    int len;

// DEBUG - proof that planeComb basically equals 1.0 everywhere
//   int x;
//   int y;

    int retval=0;
    plncomb_fname[25]=testrec_key;
    if ((f_plncomb=fopen(plncomb_fname,"wb")) == NULL) {
        LOGE("ERROR: could not open %s\n",plncomb_fname);
        retval=HERROR_FILE_OP;
    }
    else {
        // now calling arg
        // src=Hassptr->planeComb;
        len=(Hassptr->height)*(Hassptr->width)*sizeof(float);
        fwrite((void*)src,1,len,f_plncomb);
        fclose(f_plncomb);

// DEBUG - proof that planeComb basically equals 1.0 everywhere

//      for (y=0;y<(Hassptr->height);y++) {
//         for (x=0;x<(Hassptr->width);x++) {
//            printf("y %04X x %04X planeComb %10.3f\n",y,x,*src);
//            src++;
//            }
//         }


    }
    return(retval);
}

int dmp_SVM_planeComb(void) {
    // returns error true
    int error;
    float* src;
    src=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNCMB);
    // to open up the SVM
    error=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_READ,
                          (void*)src,SVMMAX_PLNCMB,0,NULL,NULL);
    if (!error) {
        error=dump_planeComb(src);
        // close the SVM window (discard error)
        clEnqueueSVMUnmap(CL_queue,(void*)src,0,NULL,NULL);
    }
    if (error) {
        LOGE("Error - dmp_SVM_planeComb()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

int dump_planeProc(float* src) {
    int len;
    int retval=0;
    plnproc_fname[25]=testrec_key;
    if ((f_plnproc=fopen(plnproc_fname,"wb")) == NULL) {
        LOGE("ERROR: could not open %s\n",plnproc_fname);
        retval=HERROR_FILE_OP;
    }
    else {
        // now calling arg
        // src=Hassptr->planeProc;
        len=(Hassptr->numPlane)*(Hassptr->height)*(Hassptr->width)*sizeof(float);
        fwrite((void*)src,1,len,f_plnproc);
        fclose(f_plnproc);
    }
    return(retval);
}

int dmp_SVM_planeProc(void) {
    // returns error true
    int error;
    float* src;
    src=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNPRC);
    // to open up the SVM
    error=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_READ,
                          (void*)src,SVMMAX_PLNPRC,0,NULL,NULL);
    if (!error) {
        error=dump_planeProc(src);
        // close the SVM window (discard error)
        clEnqueueSVMUnmap(CL_queue,(void*)src,0,NULL,NULL);
    }
    if (error) {
        LOGE("Error - dmp_SVM_planeProc()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

int dump_planesOutput(short* src) {
    int len;
    int retval=0;
    plnoutput_fname[25]=testrec_key;
    if ((f_plnoutput=fopen(plnoutput_fname,"wb")) == NULL) {
        LOGE("ERROR: could not open %s\n",plnoutput_fname);
        retval=HERROR_FILE_OP;
    }
    else {
        // now calling arg
        // src=Hassptr->planesOutput;
        len=(Hassptr->numPlane)*(Hassptr->height)*(Hassptr->width)*sizeof(short);
        fwrite((void*)src,1,len,f_plnoutput);
        fclose(f_plnoutput);
    }
    return(retval);
}

int dmp_SVM_planesOutput(void) {
    // returns error true
    int error=0;
    short* src;
    src=(short*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNOUT);
    error=dump_planesOutput(src);
    if (error) {
        LOGE("Error - dmp_SVM_planesOutput()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

void range_planesOutput(short* src) {
    int y;
    int x;
    int c;
    short val;
    short min;
    short max;
    // now calling arg
    // src=Hassptr->planesOutput;
    for (c=0;c<(Hassptr->numPlane);c++) {
        min=*src;
        max=*src;
        for (y=0;y<(Hassptr->height);y++) {
            for (x=0;x<(Hassptr->width);x++) {
                val=*src;
                src++;
                if (val<min) {
                    min=val;
                }
                else {
                    if (val>max) {
                        max=val;
                    }
                    // else do nothing
                }
            }
        }
        LOGI("plansOutput[%d] range min=0x%04X max=0x%04X\n",c,min,max);
    }
}

int rng_SVM_planesOutput(void) {
    // returns error true
    int error=0;
    short* src;
    src=(short*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNOUT);
    range_planesOutput(src);
    if (error) {
        LOGE("Error - rng_SVM_planesOutput()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

int dump_RGBfile(short* src) {
    short* srcr;
    short* srcg;
    short* srcb;
    int plnsep;
    int x;
    int y;
    int truncr;
    int truncg;
    int truncb;
    int r;
    int g;
    int b;
    int pix;
    int retval=0;
    rgboutput_fname[25]=testrec_key;
    if ((f_rgboutput=fopen(rgboutput_fname,"wb")) == NULL) {
        LOGE("ERROR: could not open %s\n",rgboutput_fname);
        retval=HERROR_FILE_OP;
    }
    else {
        // use calling arg
        // srcb=Hassptr->planesOutput;
        srcb=src;
        plnsep=(Hassptr->height)*(Hassptr->width);
        srcg=srcb+plnsep;
        srcr=srcg+plnsep;
        truncr=0;
        truncg=0;
        truncb=0;
        for (y=0;y<(Hassptr->height);y++) {
            for (x=0;x<(Hassptr->width);x++) {
                r=(int)(*srcr);
                srcr++;
                if (r&0xFFFFFF00) {
                    truncr++;
                }
                r=r&0xFF;
                g=(int)(*srcg);
                srcg++;
                if (g&0xFFFFFF00) {
                    truncg++;
                }
                g=g&0xFF;
                b=(int)(*srcb);
                srcb++;
                if (b&0xFFFFFF00) {
                    truncb++;
                }
                b=b&0xFF;
                pix=(b<<16)|(g<<8)|r;
                fwrite((void*)(&pix),1,4,f_rgboutput);
            }
        }
        LOGI("RGB trunc stats r=%d g=%d b=%d\n",truncr,truncg,truncb);
        fclose(f_rgboutput);
    }
    return(retval);
}

int dmp_SVM_RGBfile(void) {
    // returns error true
    int error=0;
    short* src;
    src=(short*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNOUT);
    error=dump_RGBfile(src);
    if (error) {
        LOGE("Error - dmp_SVM_RGBfile()\n");
        thread_set_crash(CRASH_OPENCL);
    }
    return(error);
}

void hass_tmr_report(hass_tmracc* tmracc) {
    double psum;
    double dvsr;
    double tval;
    // each pass sum starts at 0
    psum=0.0;
    // averaging divisor is number of frames
    dvsr=1.0/((double)(tmracc->numfr));
    LOGI("Kernel timing (msec)\n");
    // coefficients
    tval=((double)(tmracc->tcoeptr))*dvsr;
    psum+=tval;
    LOGI("Coeff setup           %10.3f\n",tval);
    tval=((double)(tmracc->tguide))*dvsr;
    psum+=tval;
    LOGI("planeGuide            %10.3f\n",tval);
    tval=((double)(tmracc->tshrink))*dvsr;
    psum+=tval;
    LOGI("planeShrink           %10.3f\n",tval);
    tval=((double)(tmracc->tNR))*dvsr;
    psum+=tval;
    LOGI("planeNR               %10.3f\n",tval);
    tval=((double)(tmracc->trecon))*dvsr;
    psum+=tval;
    LOGI("planeRecon            %10.3f\n",tval);
    tval=((double)(tmracc->tapp))*dvsr;
    psum+=tval;
    LOGI("<planeApp>            %10.3f\n",tval);
    tval=((double)(tmracc->tcomb))*dvsr;
    psum+=tval;
    LOGI("planeComb             %10.3f\n",tval);
    tval=((double)(tmracc->tproc))*dvsr;
    psum+=tval;
    LOGI("planeProc <zoom>      %10.3f\n",tval);
    tval=((double)(tmracc->tout))*dvsr;
    psum+=tval;
    LOGI("planesOutput          %10.3f\n",tval);
    // TOTAL
    LOGI("TOTAL kernel          %10.3f\n",psum);
    tval=((double)(tmracc->tstrt))*dvsr;
    psum+=tval;
    LOGI("TOTAL w overhead      %10.3f\n",tval);
}



//------------------------------------------------------------------------------
// frame running options...

int msg_hass_captr(void* ioptr) {
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    //  only calling arg is mask of files to be captured from
    //   the next (availble) frame processing
    //    -- gets pre-empted by hass timing operations
    //
    u16 msk;
    msk=(u16)(*(((s64*)ioptr)+1));
    ACptr->hasscaptr=msk;
    // default return - auto-destroy the carrier
    return(0);
}

int set_hass_capture(u32 msk) {
    //
    // returns error true, 0 OK
    //
    // start a measurement cycle over N frames
    //  thread0 will isse a report of some sort when done
    //
    msg_queue* queue;
    int retval;
    // the message gets passed to MU_thread3
    queue=&((thr_ctrl+3)->mqueue);
    retval=msgexec_gblkvoid(queue,msg_hass_captr,((s64)msk),0,0);
    return(retval);
}

int msg_hass_tmren(void* ioptr) {
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    //  only calling arg is N of frames to be captured from
    //   the next (availble) frame processing
    //
    u16 N;
    N=(u16)(*(((s64*)ioptr)+1));
    ACptr->hasstmren=N;
    // VERBOSITY - debug (but timing measurement should use minimal overhead)
    // LOGI("msg_hass_tmren() signalled %d frames\n",N);
    // default return - auto-destroy the carrier
    return(0);
}

int set_hass_tmren(u32 N) {
    //
    // returns error true, 0 OK
    //
    // start a measurement cycle over N frames
    //
    msg_queue* queue;
    int retval;
    // the message gets passed to MU_thread3
    queue=&((thr_ctrl+3)->mqueue);
    retval=msgexec_gblkvoid(queue,msg_hass_tmren,((s64)N),0,0);
    return(retval);
}

//------------------------------------------------------------------------------

void hass_set_kparms(void) {
//
// [0] width
// [1] height
// [2] width-1
// [3] height-1
// [4] width/8
// [5] height/8
// [6] (width/8)-1
// [7] (height/8)-1
// [8] numOfPlanes N
// [9] nrTapSize/2
// [10] gamma_mode   1 enabled / 0 disabled
// [11] gammalen-1
// [12] width/4
// [13] height/4
// [14] (width/4)-1
// [15] (height/4)-1
//
    int* dst;
    int x;
    int y;
    dst=(int*)(Hassptr->svmbase);
    //----------------
    // pixel dimensions
    x=(Hassptr->width);
    y=(Hassptr->height);
    // PRM[0] = width
    *dst=x;
    dst++;
    // PRM[1] = height
    *dst=y;
    dst++;
    // PRM[2] = (width-1)
    *dst=x-1;
    dst++;
    // PRM[3] = (height-1)
    *dst=y-1;
    dst++;
    //----------------
    // mosaic dimension
    x=x>>3;
    y=y>>3;
    // PRM[4] = width/8
    *dst=x;
    dst++;
    // PRM[5] = height/8
    *dst=y;
    dst++;
    // PRM[6] = ((width/8)-1)
    *dst=x-1;
    dst++;
    // PRM[7] = ((height/8)-1)
    *dst=y-1;
    dst++;
    //----------------
    // PRM[8] = numPlane
    *dst=(Hassptr->numPlane);
    dst++;
    // PRM[9] = nrTapSize/2
    *dst=(Hassptr->nrTapSize)>>1;
    dst++;
    //----------------
    // PRM[10] = gamma_mode
    *dst=(Hassptr->gamma_en);
    dst++;
    // PRM[11] = gammalen-1
    *dst=gammaTblLen-1;
    dst++;
    //----------------
    // planeGuide support
    x=x<<1;
    y=y<<1;
    // PRM[12] = width/4
    *dst=x;
    dst++;
    // PRM[13] = height/4
    *dst=y;
    dst++;
    // PRM[14] = ((width/4)-1)
    *dst=x-1;
    dst++;
    // PRM[15] = ((height/4)-1)
    *dst=y-1;
    dst++;
    //----------------
    // further enhancements as needed for general case
}

void hass_rcnCoe_setup(reconCoeTbl* rcncoeptr) {
    char* dst;
    int i;
    int indx;
    int Npln;
    int coesz;
    //
    dst=(char*)rcncoeptr;
    // sync the Hassptr to this indirection
    Hassptr->reconCoeptr=rcncoeptr;
    // the list for this pass is known
    Npln=(Hassptr->numPlane);
    coesz=sizeof(float)*numOfHassCh;
    for (i=0;i<Npln;i++) {
        indx=Hassptr->waveLngthTbl[i];
        memcpy((void*)dst,(void*)(&(lutWaveLength[indx][0])),coesz);
        dst+=coesz;
    }
}

int hiframe_accept(void) {
    // return error code true, or 0 for no_error, all OK
    int error;
    // all sanity checks have been removed, now just do it...
    // close the SVM window
    error=clEnqueueSVMUnmap(CL_queue,(void*)(Hassptr->planeHassRaw),0,NULL,NULL);
    if (error) {
        error=HERROR_ACCEPT;
        LOGE("hiframe_accept error code 0x%04X\n",error);
    }
    return(error);
}

//------------------------------------------------------------------------------
// Android run a HASS frame - just do it fast as possible, no options

int hass_input_frame(cfeg_image* cfegptr) {
    // accept/process input frame
    //  eventually leads to output frame generation
    // returns error true, 0 OK, no faults
    short* iframe;
    int error=0;
    // point at the pixels
    iframe=(short*)(cfegptr->svmptr);
    Hassptr->planeHassRaw=iframe;
    // input frame is valid pointer
    if (Hassptr->planesOutput) {
        // output area has been set up
        error=hiframe_accept();
        if (!error) {
            kgen_planeGuide();
            khass_shrink();

            // HERE WE'RE DONE WITH THE RAW INPUT
            // THIS MEANS - RETURN RAW BUFFER TO THREAD 1
            // call as void - we want the output error code to reflect
            //  only HASS GPU execution issues - assume this succeeds here
            cfeg_refill(cfegptr);

            // identity NR calculation is bypassed
            if ((Hassptr->nrTapSize)>1) {
                khass_NRfnc();
            }

            kgen_planeRecon();
            kgen_planeComb();
            khass_zoom();

            // Hassptr->planesOutput is the common buffer interface with thread2
            //  for HDMI handoff, and maybe another thread for file capture
            //   we can't overwrite this area with the next step results
            //    until we know the other processes aren't still trying to
            //     read the buffers
            // so spin here until the downstream processes are "done"
            //  we could hang here if the other threads never reset their blockers
            //   but honestly, we really expect to be the slowest thread as far
            //    as processing a full frame goes, and will likely never stall here
            while (ACptr->hassoblk) {
                sched_yield();
            }
            khass_post();
            // VERBOSITY
            // LOGI("hass_input_frame() output ready for handoff\n");
        }
        // else {
        // error reported
        // ACK/return the input frame done
    }
    else {
        // no output buffer set up, but has to ACK
        // ACK/return the input frame
        // THIS MEANS - RETURN RAW BUFFER TO THREAD 1
        // call as void - we already have an error
        //  error=cfe_refill(cfeiptr);
        cfeg_refill(cfegptr);
        error=HERROR_HOFRAME;
        LOGE("hass_input_frame() error code 0x%04X\n",error);
    }
    return(error);
}

//------------------------------------------------------------------------------
// Android run a HASS frame - and capture intermediate result files
//  this is to support comparison of results with msc13\hass
//   this also runs range_planesOutput();
//    that you can use to check if gain tuning is set up well

int hass_input_frame_cap(cfeg_image* cfegptr,u32 hassopts) {
    // accept/process input frame
    //  eventually leads to output frame generation
    // returns error true, 0 OK, no faults
    short* iframe;
    int error=0;
    // point at the pixels
    iframe=(short*)(cfegptr->svmptr);
    Hassptr->planeHassRaw=iframe;
    // input frame is valid pointer
    if (Hassptr->planesOutput) {
        // output area has been set up
        error=hiframe_accept();
        if (!error) {
            if (hassopts & HSOPT_CAP_COE) {
                error=dmp_SVM_reconCoeffs();
            }
            kgen_planeGuide();
            if (hassopts & HSOPT_CAP_GDE) {
                error=dmp_SVM_planeGuide();
            }
            khass_shrink();
            if (hassopts & HSOPT_CAP_SHR) {
                error=dmp_SVM_planeShrink();
            }

            // HERE WE'RE DONE WITH THE RAW INPUT
            // THIS MEANS - RETURN RAW BUFFER TO THREAD 1
            // call as void - we want the output error code to reflect
            //  only HASS GPU execution issues - assume this succeeds here
            cfeg_refill(cfegptr);

            // identity NR calculation is bypassed
            if ((Hassptr->nrTapSize)>1) {
                khass_NRfnc();
            }

            if (hassopts & HSOPT_CAP_NR) {
                error=dmp_SVM_planeNRfnc();
            }
            kgen_planeRecon();
            if (hassopts & HSOPT_CAP_RCN) {
                error=dmp_SVM_planeRecon();
            }

            // there is no App
            // #define  HSOPT_CAP_APP    (0x0020)

            kgen_planeComb();
            if (hassopts & HSOPT_CAP_CMB) {
                error=dmp_SVM_planeComb();
            }
            khass_zoom();
            if (hassopts & HSOPT_CAP_PRC) {
                error=dmp_SVM_planeProc();
            }

            // Hassptr->planesOutput is the common buffer interface with thread2
            //  for HDMI handoff, and maybe another thread for file capture
            //   we can't overwrite this area with the next step results
            //    until we know the other processes aren't still trying to
            //     read the buffers
            // so spin here until the downstream processes are "done"
            //  we could hang here if the other threads never reset their blockers
            //   but honestly, we really expect to be the slowest thread as far
            //    as processing a full frame goes, and will likely never stall here
            while (ACptr->hassoblk) {
                sched_yield();
            }

            khass_post();
            if (hassopts & HSOPT_CAP_OUT) {
                error=dmp_SVM_planesOutput();
                error=rng_SVM_planesOutput();
            }
            if (hassopts & HSOPT_CAP_RGB) {
                error=dmp_SVM_RGBfile();
            }

            // at the end of capture, cancel the capture request(s)
            ACptr->hasscaptr=0;
            // LOGI("hass_input_frame_cap() output ready for handoff\n");
        }
        // else {
        // error reported
        // ACK/return the input frame - done
    }
    else {
        // no output buffer set up, but has to ACK
        // ACK/return the input frame
        // THIS MEANS - RETURN RAW BUFFER TO THREAD 1
        // call as void - we already have an error
        //  error=cfe_refill(cfeiptr);
        cfeg_refill(cfegptr);
        error=HERROR_HOFRAME;
        LOGE("hass_input_frame_cap() error code 0x%04X\n",error);
    }
    return(error);
}

//------------------------------------------------------------------------------
// Android run a HASS frame - and collect timing/delay information

int hass_input_frame_tmr(cfeg_image* cfegptr,hass_timer* tmrptr) {
    // accept/process input frame
    //  performs timer measurements - this module decides whether to
    //   forward the timer results and issue timer cancel
    //    (error aborted run doesn't do either)
    //  eventually leads to output frame generation
    // returns error true, 0 OK, no faults
    short* iframe;
    msg_queue* queue;
    int error=0;
    // timestamp
    tmrptr->tstrt=(u32)(ACptr->tmr_cnt);
    // point at the pixels
    iframe=(short*)(cfegptr->svmptr);
    Hassptr->planeHassRaw=iframe;
    // input frame is valid pointer
    if (Hassptr->planesOutput) {
        // output area has been set up
        error=hiframe_accept();
        // timestamp
        tmrptr->tcoeptr=(u32)(ACptr->tmr_cnt);
        if (!error) {
            kgen_planeGuide();
            tmrptr->tguide=(u32)(ACptr->tmr_cnt);
            khass_shrink();
            tmrptr->tshrink=(u32)(ACptr->tmr_cnt);

            // HERE WE'RE DONE WITH THE RAW INPUT
            // THIS MEANS - RETURN RAW BUFFER TO THREAD 1
            // call as void - we want the output error code to reflect
            //  only HASS GPU execution issues - assume this succeeds here
            cfeg_refill(cfegptr);

            // identity NR calculation is bypassed
            if ((Hassptr->nrTapSize)>1) {
                khass_NRfnc();
            }

            tmrptr->tNR=(u32)(ACptr->tmr_cnt);
            kgen_planeRecon();
            tmrptr->trecon=(u32)(ACptr->tmr_cnt);

            // there is no Recon -> App stage
            //  for timing they are the same for now
            tmrptr->tapp=(tmrptr->trecon);

            kgen_planeComb();
            tmrptr->tcomb=(u32)(ACptr->tmr_cnt);
            khass_zoom();
            tmrptr->tproc=(u32)(ACptr->tmr_cnt);

            // Hassptr->planesOutput is the common buffer interface with thread2
            //  for HDMI handoff, and maybe another thread for file capture
            //   we can't overwrite this area with the next step results
            //    until we know the other processes aren't still trying to
            //     read the buffers
            // so spin here until the downstream processes are "done"
            //  we could hang here if the other threads never reset their blockers
            //   but honestly, we really expect to be the slowest thread as far
            //    as processing a full frame goes, and will likely never stall here
            while (ACptr->hassoblk) {
                sched_yield();
            }
            khass_post();
            tmrptr->tout=(u32)(ACptr->tmr_cnt);
            // VERBOSITY
            // LOGI("hass_input_frame_t() output ready for handoff\n");
            //
            // if we get here... timer management executes
            //
            // cancel the timer measurement request - one down
            ACptr->hasstmren=(ACptr->hasstmren)-1;
            // auto advance index to use alternate timer carrier next pass
            ACptr->hasstmridx=((ACptr->hasstmridx)+1)&0x01;
            // send this timer report to thread0
            queue=&((thr_ctrl)->mqueue);
            msgexec_gblkv_carr(queue,msg_hass_tmeasu,(msg_gblkfnc*)tmrptr);
            // VERBOSITY - debug (but timing measurement should use minimal overhead)
            // LOGI("hass_input_frame_tmr() now at frame %d\n",(ACptr->hasstmren));
        }
        // else {
        // error reported
        // ACK/return the input frame - done
        // no timer info sent or cancelled
    }
    else {
        // no output buffer set up, but has to ACK
        // ACK/return the input frame
        // THIS MEANS - RETURN RAW BUFFER TO THREAD 1
        // call as void - we already have an error
        //  error=cfe_refill(cfeiptr);
        cfeg_refill(cfegptr);
        error=HERROR_HOFRAME;
        LOGE("hass_input_frame_t() error code 0x%04X\n",error);
        // no timer info sent or cancelled
    }
    return(error);
}

//------------------------------------------------------------------------------


//------------------------------------------------------------------------------

// OpenCl.c

void vfvdummy(void) {
    // do nothing dummy function
    //  return Void Function that takes Void argument -- vfv
}

void CLCreateContext(cl_device_type type,cl_context* context, cl_device_id* device) {
    cl_int err = CL_SUCCESS;
    cl_platform_id platformId;
    char buffer[256];
    cl_context_properties cps[3] = {
            CL_CONTEXT_PLATFORM,
            (cl_context_properties)0,
            0
    };
    err=clGetPlatformIDs(1,&platformId,NULL);
    if (!err) {
        err=clGetPlatformInfo(platformId,CL_PLATFORM_NAME,sizeof(buffer),buffer,NULL);
        // we'll assume this succeeds if first call passed without error
        LOGI("OpenCL Platform Name: %s\n", buffer);
        cps[1]=(cl_context_properties)platformId;
        *context = clCreateContextFromType(cps, type, NULL, NULL, &err);
        err=clGetDeviceIDs(platformId, type, 1, device, NULL);
    }
    else {
        LOGE("No OpenCL support!\n");
        *context=(cl_context)0;
        *device=(cl_device_id)0;
        thread_set_crash(CRASH_OPENCL);
    }
}

char* CLReadFileAsArray(char* filename,u32* size) {
    FILE* ifile;
    char* retval=NULL;
    *size=0;
    ifile = fopen(filename, "rb");
    if (ifile) {
        fseek(ifile, 0L, SEEK_END);
        // we're OK with file size <32 bits in this app
        *size =(u32)ftell(ifile);
        rewind(ifile);
        retval=(char*)appmalloc((*size)+1);
        if (retval) {
            // memory allocated
            fread((void*)retval,1,(*size),ifile);
            *(retval+(*size))=0;
        }
        // else error return retval NULL (crash reported)
        fclose(ifile);
    }
    else {
        // can't find necessary file
        LOGE("CLReadFileAsArray failed on file %s\n",filename);
        thread_set_crash(CRASH_OPENCL);
    }
    return(retval);
}

void CLDisplayBuildErrorsAndWarnings(cl_program program, cl_device_id device) {
    cl_int err;
    size_t logSize;
    FILE* flog;
    LOGE("enter CLDisplayBuildErrorsAndWarnings\n");
    err = clGetProgramBuildInfo(program,device,CL_PROGRAM_BUILD_LOG,0,NULL,&logSize);
    if (!err) {
        if (logSize > 2) {
            char* log = (char*)malloc(logSize);
            memset(log, 0, logSize);
            err = clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG,logSize,log,NULL);
            // it succeeded first time
            if ((flog=fopen(kfile_bld_info,"wb")) != NULL) {
                fwrite((void*)log,1,logSize,flog);
                fclose(flog);
            }
            free(log);
        }
    }
    else {
        LOGE("CLDisplayBuildErrorsAndWarnings failed clGetProgramBuildInfo()\n");
    }
}

int CLBuildProgram_dbg(cl_program* program,cl_device_id device,cl_context context,
                       char* src,u32 srclen,char* compilerOptions) {
    // returns error true (crash reported) else OK => 0
    int retval;
    cl_int err;
    size_t lsize=(size_t)srclen;
    *program=clCreateProgramWithSource(context,1,(const char **)(&src),&lsize,&err);
    if (!err) {
        err =clBuildProgram(*program,1,&device,compilerOptions,NULL,NULL);
    }
    if (err) {
        LOGE("CLBuildProgram_dbg() failure\n");
        // try to dump the log "kbld_log.txt"
        CLDisplayBuildErrorsAndWarnings(*program,device);
        thread_set_crash(CRASH_OPENCL);
        retval=1;
    }
    else {
        LOGE("CLBuildProgram_dbg() success\n");
        retval=0;
    }
    return(retval);
}

int CLBuildProgram(cl_program* program,cl_device_id device,cl_context context,
                   char* src,u32 srclen,char* compilerOptions) {
    // returns error true (crash reported) else OK => 0
    int retval;
    cl_int err;
    size_t lsize=(size_t)srclen;
    *program=clCreateProgramWithSource(context,1,(const char **)(&src),&lsize,&err);
    if (!err) {
        err =clBuildProgram(*program,1,&device,compilerOptions,NULL,NULL);
    }
    if (err) {
        LOGE("CLBuildProgram failure\n");
        thread_set_crash(CRASH_OPENCL);
        retval=1;
    }
    else {
        retval=0;
    }
    return(retval);
}

//------------------------------------------------------------------------------

void CL_kernel_build_dbg(CL_proc_ctrl* kctrl) {
    int error;
    cl_int clerr;
    error=CLBuildProgram_dbg((&(kctrl->program)),CL_device,CL_context,
                             (kctrl->ksrc),(kctrl->klen),CL_COMPILER_OPTIONS);
    if (!error) {
        kctrl->kernel=clCreateKernel((kctrl->program),(kctrl->kname),&clerr);
        if (clerr) {
            kctrl->kernel=(cl_kernel)0;
            LOGE("CL_kernel_build_dbg() failed create kernel for %s\n",(kctrl->kname));
            thread_set_crash(CRASH_OPENCL);
        }
        else {
            LOGE("CL_kernel_build_dbg() successful for %s\n",(kctrl->kname));
        }
    }
}

void CL_kernel_build(CL_proc_ctrl* kctrl) {
    int error;
    cl_int clerr;
    error=CLBuildProgram((&(kctrl->program)),CL_device,CL_context,
                         (kctrl->ksrc),(kctrl->klen),CL_COMPILER_OPTIONS);
    if (!error) {
        kctrl->kernel=clCreateKernel((kctrl->program),(kctrl->kname),&clerr);
        if (clerr) {
            kctrl->kernel=(cl_kernel)0;
            LOGE("CL_kernel_build failed create kernel for %s\n",(kctrl->kname));
            thread_set_crash(CRASH_OPENCL);
        }
    }
}

void CL_kernel_setup_dbg(int kindx) {
    CL_proc_ctrl* kctrl;
    char* ktext;
    // abort on earlier crash
    if (!(ACptr->hwsema_crsh)) {

        // ZZZZZ
        LOGI("CL_kernel_setup_dbg() for kindx=%d\n",kindx);

        kctrl=(CL_proc_ctrl*)(&(kern_ctrl[kindx]));
        if (kctrl->ksrc) {
            // source text built into application
            CL_kernel_build(kctrl);
        }
        else {
            // working from file - easier to edit in development
            // but if there's no filename either, there's a problem
            if (kctrl->kfilename) {
                // have filename to load with auto malloc()
                ktext=CLReadFileAsArray(kctrl->kfilename,(&(kctrl->klen)));
                if (ktext) {

                    // ZZZZZ
                    LOGI("CL_kernel_setup_dbg loaded file %s size %d 0x%08X\n",
                         kctrl->kfilename,kctrl->klen,kctrl->klen);

                    // now we have resident source code
                    kctrl->ksrc=ktext;
                    CL_kernel_build_dbg(kctrl);
                    // whether it compiles or not, release the malloc
                    appfree((void*)(kctrl->ksrc));
                }
                // else failed to load source - crash reported
            }
            else {
                // no source of any kind - why did we call ths routine
                LOGE("CL_kernel_setup has no source for kindx=%d\n",kindx);
                thread_set_crash(CRASH_OPENCL);
            }
        }
    }
}

void CL_kernel_setup(int kindx) {
    CL_proc_ctrl* kctrl;
    char* ktext;
    // abort on earlier crash
    if (!(ACptr->hwsema_crsh)) {
        kctrl=(CL_proc_ctrl*)(&(kern_ctrl[kindx]));
        if (kctrl->ksrc) {
            // source text built into application
            CL_kernel_build(kctrl);
        }
        else {
            // working from file - easier to edit in development
            // but if there's no filename either, there's a problem
            if (kctrl->kfilename) {
                // have filename to load with auto malloc()
                ktext=CLReadFileAsArray(kctrl->kfilename,(&(kctrl->klen)));
                if (ktext) {
                    // now we have resident source code
                    kctrl->ksrc=ktext;
                    CL_kernel_build(kctrl);
                    // whether it compiles or not, release the malloc
                    appfree((void*)(kctrl->ksrc));
                }
                // else failed to load source - crash reported
            }
            else {
                // no source of any kind - why did we call ths routine
                LOGE("CL_kernel_setup has no source for kindx=%d\n",kindx);
                thread_set_crash(CRASH_OPENCL);
            }
        }
    }
}

void CL_kernel_build_suite(void) {
    // builds all the kernels's we'll use for this app, or crashes trying
    //  some kernels my be run in native arm during development
    //   bring additional kernels on line as available
    CL_kernel_setup(HASS_KINDX_PLANEGUID0);
    CL_kernel_setup(HASS_KINDX_PLANEGUID1);
    CL_kernel_setup(HASS_KINDX_SHR0);
    CL_kernel_setup(HASS_KINDX_SHR1);
    CL_kernel_setup(HASS_KINDX_SHR2);
    CL_kernel_setup(HASS_KINDX_SHR3);
    CL_kernel_setup(HASS_KINDX_SHR4);
    CL_kernel_setup(HASS_KINDX_SHR5);
    CL_kernel_setup(HASS_KINDX_NRF0);
    CL_kernel_setup(HASS_KINDX_NRF1);
    CL_kernel_setup(HASS_KINDX_PLANERECON);
    CL_kernel_setup(HASS_KINDX_PLANECMB0);
    CL_kernel_setup(HASS_KINDX_PLANECMB1);
    CL_kernel_setup(HASS_KINDX_ZOOM);
    CL_kernel_setup(HASS_KINDX_POST);
}

void CL_startup(void) {
    cl_int err;
    // initially get device and context
    CLCreateContext(CL_type,&CL_context,&CL_device);
    if (CL_context) {
        CL_queue=clCreateCommandQueueWithProperties(CL_context,CL_device,NULL,&err);
        if (!err) {
            // then we have interface queue...
            //  pre-compile all the kernels we'll use - will report any crash
            CL_kernel_build_suite();
        }
        else {
            LOGE("CL_startup can't establish CL_queue\n");
            thread_set_crash(CRASH_OPENCL);
        }
    }
    // else, without a CL_context, crash has been recorded
}

//------------------------------------------------------------------------------

// void CL_kernel_run(int kindx) {
// //
// // executing a kernel is a matter of running its fnc*
// //  no checks... assumes the kernel is compiled and ready to go
// //   with all necessary input data on deck
// //
// // CL_kernel_run(HASS_KINDX_PLANEGUIDE);
// // CL_kernel_run(HASS_KINDX_SHRINK);
// // CL_kernel_run(HASS_KINDX_NRFNC);
// // CL_kernel_run(HASS_KINDX_PLANERECON);
// // CL_kernel_run(HASS_KINDX_PLANECOMB);
// // CL_kernel_run(HASS_KINDX_ZOOM);
// // CL_kernel_run(HASS_KINDX_POST);
// //
//    CL_proc_ctrl* kctrl;
//    kctrl=(CL_proc_ctrl*)(&(kern_ctrl[kindx]));
//    (*(kctrl->fnc))();
//    }

//------------------------------------------------------------------------------

void CL_kernel_release(int kindx) {
    cl_int err;
    CL_proc_ctrl* kctrl;
    // we're shutting down
    //  if any part of this fails, it doesn't really matter in the sense
    //   that there's nothing else we can do about it...
    //
    kctrl=(CL_proc_ctrl*)(&(kern_ctrl[kindx]));
    if (kctrl->kernel) {
        err = clReleaseKernel((kctrl->kernel));
        kctrl->kernel=(cl_kernel)0;
    }
    if (kctrl->program) {
        err = clReleaseProgram((kctrl->program));
        kctrl->program=(cl_program)0;
    }
}

void CL_shutdown(void) {
    // best effort to completely shutdown OpenCL
    //  even in light of other possible crash inhibitors that may cause
    //   additional errors here
    cl_int err;
    CL_kernel_release(HASS_KINDX_PLANEGUID0);
    CL_kernel_release(HASS_KINDX_PLANEGUID1);
    CL_kernel_release(HASS_KINDX_SHR0);
    CL_kernel_release(HASS_KINDX_SHR1);
    CL_kernel_release(HASS_KINDX_SHR2);
    CL_kernel_release(HASS_KINDX_SHR3);
    CL_kernel_release(HASS_KINDX_SHR4);
    CL_kernel_release(HASS_KINDX_SHR5);
    CL_kernel_release(HASS_KINDX_NRF0);
    CL_kernel_release(HASS_KINDX_NRF1);
    CL_kernel_release(HASS_KINDX_PLANERECON);
    CL_kernel_release(HASS_KINDX_PLANECMB0);
    CL_kernel_release(HASS_KINDX_PLANECMB1);
    CL_kernel_release(HASS_KINDX_ZOOM);
    CL_kernel_release(HASS_KINDX_POST);
    if (CL_context) {
        err=clReleaseContext(CL_context);
        CL_context=(cl_context)0;
    }
}

//------------------------------------------------------------------------------

void kgen_planeGuide(void) {

    CL_proc_ctrl* kctrl;
    cl_int err;
    size_t globalWorkItems[2];
    size_t localWorkItems[2];
    float* plngd;

    plngd=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNGD);
    err=0;

    if (!(ACptr->hwsema_crsh)) {

        // CREATE KERNEL - precompiled
        //  __kernel void PLANEGUID0(
        //     __constant int* PRM,
        //     __global ushort4* PHRI,
        //     __global float4* PGO
        //     )
        kctrl=(CL_proc_ctrl*)(&(kern_ctrl[HASS_KINDX_PLANEGUID0]));

        // SET KERNEL ARGS
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),0,(Hassptr->svmbase));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),1,(void*)(Hassptr->planeHassRaw));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),2,(void*)plngd);
        }

        // DIMENSIONING and KERNEL LAUNCH 0
        if (!err) {
            // assign global work-items and group work-items
            globalWorkItems[0]=(Hassptr->height)/8;            // 135
            globalWorkItems[1]=8;                              // 8   y indexes 0..1079
            localWorkItems[0]=1;
            localWorkItems[1]=8;
            // launch CL kernel
            err=clEnqueueNDRangeKernel(CL_queue,(kctrl->kernel),2,NULL,
                                       globalWorkItems,localWorkItems,0,NULL,NULL);
        }

        // CREATE KERNEL - precompiled
        //  __kernel void PLANEGUID1(
        //     __constant int* PRM,
        //     __global float* PGO
        //     )
        kctrl=(CL_proc_ctrl*)(&(kern_ctrl[HASS_KINDX_PLANEGUID1]));

        // SET KERNEL ARGS
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),0,(Hassptr->svmbase));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),1,(void*)plngd);
        }

        // DIMENSIONING and KERNEL LAUNCH 1
        if (!err) {
            // assign global work-items and group work-items
            globalWorkItems[0]=(Hassptr->width)/8;             // 240
            globalWorkItems[1]=8;                              // 8   x indexes 0..1919
            localWorkItems[0]=16;
            localWorkItems[1]=8;
            // launch CL kernel
            err=clEnqueueNDRangeKernel(CL_queue,(kctrl->kernel),2,NULL,
                                       globalWorkItems,localWorkItems,0,NULL,NULL);
        }

        // SUMMARY
        if (err) {
            LOGE("kgen_planeGuide() failure\n");
            thread_set_crash(CRASH_OPENCL);
        }
    }
}

//------------------------------------------------------------------------------

void khass_shrink(void) {

    CL_proc_ctrl* kctrl;
    cl_int err;
    size_t globalWorkItems[2];
    size_t localWorkItems[2];

    float* plnshr;
    float* plnnr;
    float* plnapp;

    plnshr=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNSHR);
    plnnr=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNNR);
    plnapp=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNAPP);
    err=0;

    if (!(ACptr->hwsema_crsh)) {

        // CREATE KERNEL - precompiled
        // SET KERNEL ARGS
        //  __kernel void HASSSHR0(
        //     __constant int* PRM,
        //     __global float16* PNR,
        //     __global float16* APP
        //     )
        kctrl=(CL_proc_ctrl*)(&(kern_ctrl[HASS_KINDX_SHR0]));

        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),0,(Hassptr->svmbase));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),1,(void*)plnnr);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),2,(void*)plnapp);
        }

        // DIMENSIONING and KERNEL LAUNCH
        if (!err) {
            // assign global work-items and group work-items
            globalWorkItems[0]=((Hassptr->width)/8)+1;
            globalWorkItems[1]=((Hassptr->height)/8)+1;
            localWorkItems[0]=1;
            localWorkItems[1]=8;
            // launch CL kernel
            err=clEnqueueNDRangeKernel(CL_queue,(kctrl->kernel),2,NULL,
                                       globalWorkItems,localWorkItems,0,NULL,NULL);
        }

        //------------------------------------------------------------------------

        // CREATE KERNEL - precompiled
        kctrl=(CL_proc_ctrl*)(&(kern_ctrl[HASS_KINDX_SHR1]));

        // SET KERNEL ARGS
        //  __kernel void HASSSHR1(
        //     __constant int* PRM,
        //     __global ushort8* PHRI,
        //     __global float16* PNR,
        //     __global float16* APP
        //     )

        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),0,(Hassptr->svmbase));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),1,(void*)(Hassptr->planeHassRaw));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),2,(void*)plnnr);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),3,(void*)plnapp);
        }

        // DIMENSIONING and KERNEL LAUNCH
        if (!err) {
            // assign global work-items and group work-items
            globalWorkItems[0]=((Hassptr->width)/8);
            globalWorkItems[1]=((Hassptr->height)/8);
            localWorkItems[0]=16;
            localWorkItems[1]=1;
            // launch CL kernel
            err=clEnqueueNDRangeKernel(CL_queue,(kctrl->kernel),2,NULL,
                                       globalWorkItems,localWorkItems,0,NULL,NULL);
        }

        //------------------------------------------------------------------------

        // CREATE KERNEL - precompiled
        kctrl=(CL_proc_ctrl*)(&(kern_ctrl[HASS_KINDX_SHR2]));

        // SET KERNEL ARGS
        //  __kernel void HASSSHR2(
        //     __constant int* PRM,
        //     __global ushort8* PHRI,
        //     __global float16* PNR,
        //     __global float16* APP
        //     )

        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),0,(Hassptr->svmbase));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),1,(void*)(Hassptr->planeHassRaw));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),2,(void*)plnnr);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),3,(void*)plnapp);
        }

        // DIMENSIONING and KERNEL LAUNCH
        if (!err) {
            // assign global work-items and group work-items
            globalWorkItems[0]=((Hassptr->width)/8);
            globalWorkItems[1]=((Hassptr->height)/8);
            localWorkItems[0]=16;
            localWorkItems[1]=1;
            // launch CL kernel
            err=clEnqueueNDRangeKernel(CL_queue,(kctrl->kernel),2,NULL,
                                       globalWorkItems,localWorkItems,0,NULL,NULL);
        }

        //------------------------------------------------------------------------

        // CREATE KERNEL - precompiled
        kctrl=(CL_proc_ctrl*)(&(kern_ctrl[HASS_KINDX_SHR3]));

        // SET KERNEL ARGS
        //  __kernel void HASSSHR3(
        //     __constant int* PRM,
        //     __global ushort8* PHRI,
        //     __global float16* PNR,
        //     __global float16* APP
        //     )

        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),0,(Hassptr->svmbase));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),1,(void*)(Hassptr->planeHassRaw));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),2,(void*)plnnr);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),3,(void*)plnapp);
        }

        // DIMENSIONING and KERNEL LAUNCH
        if (!err) {
            // assign global work-items and group work-items
            globalWorkItems[0]=((Hassptr->width)/8);
            globalWorkItems[1]=((Hassptr->height)/8);
            localWorkItems[0]=16;
            localWorkItems[1]=1;
            // launch CL kernel
            err=clEnqueueNDRangeKernel(CL_queue,(kctrl->kernel),2,NULL,
                                       globalWorkItems,localWorkItems,0,NULL,NULL);
        }

        //------------------------------------------------------------------------

        // CREATE KERNEL - precompiled
        kctrl=(CL_proc_ctrl*)(&(kern_ctrl[HASS_KINDX_SHR4]));

        // SET KERNEL ARGS
        //  __kernel void HASSSHR4(
        //     __constant int* PRM,
        //     __global ushort8* PHRI,
        //     __global float16* PNR,
        //     __global float16* APP
        //     )

        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),0,(Hassptr->svmbase));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),1,(void*)(Hassptr->planeHassRaw));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),2,(void*)plnnr);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),3,(void*)plnapp);
        }

        // DIMENSIONING and KERNEL LAUNCH
        if (!err) {
            // assign global work-items and group work-items
            globalWorkItems[0]=((Hassptr->width)/8);
            globalWorkItems[1]=((Hassptr->height)/8);
            localWorkItems[0]=16;
            localWorkItems[1]=1;
            // launch CL kernel
            err=clEnqueueNDRangeKernel(CL_queue,(kctrl->kernel),2,NULL,
                                       globalWorkItems,localWorkItems,0,NULL,NULL);
        }

        //------------------------------------------------------------------------

        // CREATE KERNEL - precompiled
        kctrl=(CL_proc_ctrl*)(&(kern_ctrl[HASS_KINDX_SHR5]));

        // SET KERNEL ARGS
        //  __kernel void HASSSHR5(
        //     __constant int* PRM,
        //     __global float16* PNR,
        //     __global float16* APP,
        //     __global float16* PSO
        //     )

        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),0,(Hassptr->svmbase));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),1,(void*)plnnr);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),2,(void*)plnapp);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),3,(void*)plnshr);
        }

        // DIMENSIONING and KERNEL LAUNCH
        if (!err) {
            // assign global work-items and group work-items
            globalWorkItems[0]=((Hassptr->width)/8)+1;
            globalWorkItems[1]=((Hassptr->height)/8)+1;
            localWorkItems[0]=1;
            localWorkItems[1]=8;
            // launch CL kernel
            err=clEnqueueNDRangeKernel(CL_queue,(kctrl->kernel),2,NULL,
                                       globalWorkItems,localWorkItems,0,NULL,NULL);
        }

        // SUMMARY
        if (err) {
            LOGE("khass_shrink() failure\n");
            thread_set_crash(CRASH_OPENCL);
        }
    }
}

//------------------------------------------------------------------------------

void khass_NRfnc(void) {

    CL_proc_ctrl* kctrl;
    cl_int err;
    size_t globalWorkItems[2];
    size_t localWorkItems[2];

    float* plnshr;
    float* plnnr;

    plnshr=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNSHR);
    plnnr=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNNR);
    err=0;

    if (!(ACptr->hwsema_crsh)) {

        // CREATE KERNEL - precompiled
        //  __kernel void HASSNRF0(
        //     __constant int* PRM,
        //     __global float* PSI,
        //     __global float* PNRO
        //     )
        kctrl=(CL_proc_ctrl*)(&(kern_ctrl[HASS_KINDX_NRF0]));

        // SET KERNEL ARGS
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),0,(Hassptr->svmbase));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),1,(void*)plnshr);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),2,(void*)plnnr);
        }

        // DIMENSIONING and KERNEL LAUNCH
        if (!err) {
            // assign global work-items and group work-items
            globalWorkItems[0]=(((Hassptr->width)/8)+1);
            globalWorkItems[1]=numOfHassCh;
            localWorkItems[0]=1;
            localWorkItems[1]=numOfHassCh;
            // launch CL kernel
            err=clEnqueueNDRangeKernel(CL_queue,(kctrl->kernel),2,NULL,
                                       globalWorkItems,localWorkItems,0,NULL,NULL);
        }

        // CREATE KERNEL - precompiled
        //  __kernel void HASSNRF1(
        //     __constant int* PRM,
        //     __global float* PNRO
        //     )
        kctrl=(CL_proc_ctrl*)(&(kern_ctrl[HASS_KINDX_NRF1]));

        // SET KERNEL ARGS
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),0,(Hassptr->svmbase));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),1,(void*)plnnr);
        }

        // DIMENSIONING and KERNEL LAUNCH
        if (!err) {
            // assign global work-items and group work-items
            globalWorkItems[0]=(((Hassptr->height)/8)+1);
            globalWorkItems[1]=numOfHassCh;
            localWorkItems[0]=8;
            localWorkItems[1]=numOfHassCh;
            // launch CL kernel
            err=clEnqueueNDRangeKernel(CL_queue,(kctrl->kernel),2,NULL,
                                       globalWorkItems,localWorkItems,0,NULL,NULL);
        }

        // SUMMARY
        if (err) {
            LOGE("khass_NRfnc() failure\n");
            thread_set_crash(CRASH_OPENCL);
        }
    }
}

//------------------------------------------------------------------------------

void kgen_planeRecon(void) {
    //
    //  __kernel void PLANERECON(
    //     __constant int* PRM,
    //     __constant float* RCC,
    //     __global float* PNRI,
    //     __global float* PRCO
    //     )

    CL_proc_ctrl* kctrl;
    cl_int err;
    size_t globalWorkItems[2];
    size_t localWorkItems[2];
    float* rcncoe;
    float* plnnr;
    float* plnrcn;

    kctrl=(CL_proc_ctrl*)(&(kern_ctrl[HASS_KINDX_PLANERECON]));
    rcncoe=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_RECON);
    // be aware of NR calculation bypass
    if ((Hassptr->nrTapSize)>1) {
        plnnr=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNNR);
    }
    else {
        plnnr=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNSHR);
    }
    plnrcn=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNRCN);
    err=0;

    if (!(ACptr->hwsema_crsh)) {

        // CREATE KERNEL - precompiled

        // SET KERNEL ARGS
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),0,(Hassptr->svmbase));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),1,(void*)rcncoe);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),2,(void*)plnnr);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),3,(void*)plnrcn);
        }

        // DIMENSIONING and KERNEL LAUNCH
        if (!err) {
            // assign global work-items and group work-items
            globalWorkItems[0]=((Hassptr->width)/8)+1;
            globalWorkItems[1]=((Hassptr->height)/8)+1;
            localWorkItems[0]=1;
            localWorkItems[1]=8;
            // launch CL kernel
            err=clEnqueueNDRangeKernel(CL_queue,(kctrl->kernel),2,NULL,
                                       globalWorkItems,localWorkItems,0,NULL,NULL);
        }

        // SUMMARY
        if (err) {
            LOGE("kgen_planeRecon() failure\n");
            thread_set_crash(CRASH_OPENCL);
        }
    }
}

//------------------------------------------------------------------------------

void kgen_planeComb(void) {

    CL_proc_ctrl* kctrl;
    cl_int err;
    size_t globalWorkItems[3];
    size_t localWorkItems[3];
    float* plngd;
    float* plncmb;

    plngd=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNGD);
    plncmb=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNCMB);
    err=0;

    if (!(ACptr->hwsema_crsh)) {

        // CREATE KERNEL - precompiled
        //  __kernel void PLANECMB0(
        //     __constant int* PRM,
        //     __global float* PGI,
        //     __global float* PCO
        //     )
        kctrl=(CL_proc_ctrl*)(&(kern_ctrl[HASS_KINDX_PLANECMB0]));

        // SET KERNEL ARGS
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),0,(Hassptr->svmbase));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),1,(void*)plngd);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),2,(void*)plncmb);
        }

        // DIMENSIONING and KERNEL LAUNCH 0
        if (!err) {
            // assign global work-items and group work-items
            globalWorkItems[0]=(Hassptr->width)/8;             // 240
            globalWorkItems[1]=8;                              // 8   x indexes 0..1919
            localWorkItems[0]=16;
            localWorkItems[1]=8;
            // launch CL kernel
            err=clEnqueueNDRangeKernel(CL_queue,(kctrl->kernel),2,NULL,
                                       globalWorkItems,localWorkItems,0,NULL,NULL);
        }

        // CREATE KERNEL - precompiled
        //  __kernel void PLANECMB1(
        //     __constant int* PRM,
        //     __global float* PGI,
        //     __global float* PCO
        //     )
        kctrl=(CL_proc_ctrl*)(&(kern_ctrl[HASS_KINDX_PLANECMB1]));

        // SET KERNEL ARGS
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),0,(Hassptr->svmbase));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),1,(void*)plngd);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),2,(void*)plncmb);
        }

        // DIMENSIONING and KERNEL LAUNCH 1
        if (!err) {
            // assign global work-items and group work-items
            globalWorkItems[0]=(Hassptr->height)/8;            // 135
            globalWorkItems[1]=8;                              // 8   y indexes 0..1079
            localWorkItems[0]=1;
            localWorkItems[1]=8;
            // launch CL kernel
            err=clEnqueueNDRangeKernel(CL_queue,(kctrl->kernel),2,NULL,
                                       globalWorkItems,localWorkItems,0,NULL,NULL);
        }

        // SUMMARY
        if (err) {
            LOGE("kgen_planeComb() failure\n");
            thread_set_crash(CRASH_OPENCL);
        }
    }
}

//------------------------------------------------------------------------------

void khass_zoom(void) {
    // __kernel void HASSZOOM(
    //    __constant int* PRM,
    //    __global float* PRCI,
    //    __global float4* PCI,
    //    __global float4* PPRO
    //    )

    CL_proc_ctrl* kctrl;
    cl_int err;
    size_t globalWorkItems[3];
    size_t localWorkItems[3];
    float* plnrcn;
    float* plncmb;
    float* plnprc;

    kctrl=(CL_proc_ctrl*)(&(kern_ctrl[HASS_KINDX_ZOOM]));
    plnrcn=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNRCN);
    plncmb=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNCMB);
    plnprc=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNPRC);
    err=0;

    if (!(ACptr->hwsema_crsh)) {

        // CREATE KERNEL - precompiled

        // SET KERNEL ARGS
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),0,(Hassptr->svmbase));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),1,(void*)plnrcn);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),2,(void*)plncmb);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),3,(void*)plnprc);
        }

        // DIMENSIONING and KERNEL LAUNCH
        if (!err) {
            // assign global work-items and group work-items
            globalWorkItems[0]=(Hassptr->width)/8;
            globalWorkItems[1]=(Hassptr->height)/8;
            globalWorkItems[2]=(Hassptr->numPlane);
            localWorkItems[0]=16;
            localWorkItems[1]=1;
            localWorkItems[2]=1;
            // launch CL kernel
            err=clEnqueueNDRangeKernel(CL_queue,(kctrl->kernel),3,NULL,
                                       globalWorkItems,localWorkItems,0,NULL,NULL);
        }

        // SUMMARY
        if (err) {
            LOGE("khass_zoom() failure\n");
            thread_set_crash(CRASH_OPENCL);
        }
    }
}

//------------------------------------------------------------------------------

void khass_post(void) {
    //  __kernel void HASSPOST(
    //     __constant int* PRM,
    //     __global float* PPI,
    //     __constant float* PGP,
    //     __global short* GMA,
    //     __global short* PO
    //     )

    CL_proc_ctrl* kctrl;
    cl_int err;
    size_t globalWorkItems[3];
    size_t localWorkItems[3];
    float* plnprc;
    float* plgn;
    short* gmma;
    short* plnout;

    kctrl=(CL_proc_ctrl*)(&(kern_ctrl[HASS_KINDX_POST]));
    plnprc=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNPRC);
    plgn=(float*)(((char*)(Hassptr->svmbase))+SVMOFF_PLGAIN);
    gmma=(short*)(((char*)(Hassptr->svmbase))+SVMOFF_GAMMA);
    plnout=(short*)(((char*)(Hassptr->svmbase))+SVMOFF_PLNOUT);
    err=0;

    if (!(ACptr->hwsema_crsh)) {

        // close for GPU manipulation
        if (!err) {
            err=clEnqueueSVMUnmap(CL_queue,(void*)plnout,0,NULL,NULL);
        }

        // CREATE KERNEL - precompiled

        // SET KERNEL ARGS
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),0,(Hassptr->svmbase));
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),1,(void*)plnprc);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),2,(void*)plgn);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),3,(void*)gmma);
        }
        if (!err) {
            err=clSetKernelArgSVMPointer((kctrl->kernel),4,(void*)plnout);
        }

        // DIMENSIONING and KERNEL LAUNCH
        if (!err) {
            // assign global work-items and group work-items
            globalWorkItems[0]=(Hassptr->width)/8;
            globalWorkItems[1]=(Hassptr->height)/8;
            globalWorkItems[2]=(Hassptr->numPlane)*8;
            localWorkItems[0]=16;
            localWorkItems[1]=1;
            localWorkItems[2]=8;
            // launch CL kernel
            err=clEnqueueNDRangeKernel(CL_queue,(kctrl->kernel),3,NULL,
                                       globalWorkItems,localWorkItems,0,NULL,NULL);
        }

        // re-open planesOutput as read only
        if (!err) {
            err=clEnqueueSVMMap(CL_queue,CL_TRUE,CL_MAP_READ,
                                (void*)plnout,SVMMAX_PLNOUT,0,NULL,NULL);
        }

        // SUMMARY
        if (err) {
            LOGE("khass_post() failure\n");
            thread_set_crash(CRASH_OPENCL);
        }
    }
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

int init_cfegptr(cfeg_image* cfegptr,void* svmhome) {
    // returns error/crash true
    //  otherwise sends thread1 a seed cfegptr
    u32 x;
    u32 y;
    int error=0;
    cfegptr->svmptr=svmhome;
    x=Hassptr->width;
    y=Hassptr->height;
    // preload the size of the image expected to carry
    cfegptr->xdim=(u16)x;
    cfegptr->ydim=(u16)y;
    // send it to thread 1
    error=cfeg_refill(cfegptr);
    return(error);
}

int create_cfegptr(void* svmhome) {
    // returns error/crash true
    //  otherwise sends thread1 a seed cfegptr
    void* vptr;
    int error=0;
    vptr=appmalloc(sizeof(cfeg_image));
    if (vptr) {
        error=init_cfegptr((cfeg_image*)vptr,svmhome);
    }
    else {
        // error/crash reported
        error=1;
    }
    return(error);
}

void init_cfegpool(void) {
    // the buffers allocated are implicit in hass_SVM.h
    // #define  SVMOFF_IFRM      0x00100000
    // #define  SVMOFF_IFRM2     0x01F00000
    //
    // errors not expected at init
    //
    void* svmhome;
    svmhome=(void*)(((char*)(Hassptr->svmbase))+SVMOFF_IFRM);
    create_cfegptr(svmhome);
    svmhome=(void*)(((char*)(Hassptr->svmbase))+SVMOFF_IFRM2);
    create_cfegptr(svmhome);
}

//
// clean up requirements
//  in theory SVM write only cfeg buffers are always outstanding
//   meaning there is never any signalling to return them to thread3
//    under GPU processing, the buffer gets closed, and read by the kernels
//     but then the buffer is re-opend as write only, and handed back to thread1
//      to await next camera capture...
// so really, anywhere in transit in a message queue, the buffer is open for write
//  on crash cleanup, we just close them all, the same way we initially
//   just deployed them all
//

void drop_cfegpool(void) {
    // the buffers allocated are implicit in hass_SVM.h
    // #define  SVMOFF_IFRM      0x00100000
    // #define  SVMOFF_IFRM2     0x01F00000
    //
    // errors can be ignored at shutdown
    //
    void* svmhome;
    svmhome=(void*)(((char*)(Hassptr->svmbase))+SVMOFF_IFRM);
    clEnqueueSVMUnmap(CL_queue,svmhome,0,NULL,NULL);
    svmhome=(void*)(((char*)(Hassptr->svmbase))+SVMOFF_IFRM2);
    clEnqueueSVMUnmap(CL_queue,svmhome,0,NULL,NULL);
}

//------------------------------------------------------------------------------

int msg_raw2hass_frame(void* ioptr) {
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    thread3_ctrl* thr_ptr;
    cfeg_image* cfegptr;
    u32 hassspec;
    u32 hassopts;
    hass_timer* tmrptr;
    s64 rfh0;
    s64 rfh1;
    int error=0;
    // this thread control structure
    thr_ptr=(thread3_ctrl*)(thr_ctrl+3);
    cfegptr=(cfeg_image*)(*(((s64*)ioptr)+1));
    // get a copy of the raw_filhdr information from the input raw frame
    rfh0=*((s64*)(((char*)cfegptr)+0x30));
    rfh1=*((s64*)(((char*)cfegptr)+0x38));
    // run the suite of GPU algorithms
    // get the options - hasstmren||hasscaptr
    hassopts=*((u32*)(&(ACptr->hasscaptr)));
    if (hassopts) {
        // doing either a timer run or capture run
        //  timer runs must be contiguous - so they dominate
        //   if capture request is coincident a timing test,
        //    capture gets deferred until timing test completes
        if (hassopts & HSOPT_TMEAS) {
            // timer dominates - use the next timer on deck
            tmrptr=&(hasstmr_pool[ACptr->hasstmridx]);
            //
            // so if the frame errors out,
            //  there's a frame slip in the timer measurement
            //   and the total with overhead would be "off" in that case
            //    but I think we can reasonably assume that if the
            //     frames are flowing... things will work well
            //  like - what if they shut off RAW feed and turn on YUV for
            //   a while during a timer measurement...
            //    have to keep some reasonable assumptions
            //     We're counting on the operator to properly sequence
            //      test requests
            //
            error=hass_input_frame_tmr(cfegptr,tmrptr);
        }
        else {
            // capture request gets serviced this pass
            error=hass_input_frame_cap(cfegptr,hassopts);
        }
    }
    else {
        // default - run as fast as possible
        error=hass_input_frame(cfegptr);
    }
    // if that craps out, there is no forwarding
    if (!error) {
        //
        // there's no definition for information in hassspec yet
        //  as it may be interpretted by thread3
        //   for now, the only requirement is a non-0 value
        hassspec=1;
        //
        error=hass2pixo_fwrd(hassspec,rfh0,rfh1);
        // if that doesn't work, we likely have a crash,
        //  but there's nothing we can do about it here
        //
        // if we want to dispatch a fork of clone of the data for
        //  file capture or download
        //   it's best to implement an alternal deployment routine to another thread
        //    (any file capture would be full image size, where HDMI display may well
        //      be cropped for the small display -- or may be disabled completely)
        //    (any saved files would need header info - timestamp, bpp, etc)
        //
    }
    // return int 0 - auto destroy message
    return(0);
}

int raw2hass_fwrd(cfeg_image* cfegptr) {
    // returns 1 on crash abort, else 0
    //  direct RAW -> thread3 HASS GPU handling
    msg_queue* queue;
    int retval;
    // the message gets passed to MU_thread3
    queue=&((thr_ctrl+3)->mqueue);
    // use custom message pass to avoid touching the raw_filhdr
    retval=msgexec_gblkvoid_a2(queue,msg_raw2hass_frame,((s64)cfegptr),0);
    return(retval);
}

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

int msg_utp_fhdr_state(void* ioptr) {
    sprintf(utppb_ln,"Tp08 sees sig_pc2a %08X sig_a2pc %08X\n",
            (utpptr->utphdr.sig_pc2a),(utpptr->utphdr.sig_a2pc));
    utp_dbgoutput(utppb_ln);
    sprintf(utppb_ln,"pc2a_psh %08X pc2a_pll %08X a2pc_psh %08X a2pc_pll %08X\n",
            (utpptr->utphdr.pc2a_psh),(utpptr->utphdr.pc2a_pll),
            (utpptr->utphdr.a2pc_psh),(utpptr->utphdr.a2pc_pll));
    utp_dbgoutput(utppb_ln);
    sprintf(utppb_ln,"pc2abpsh %08X pc2abpll %08X a2pcbpsh %08X a2pcbpll %08X\n",
            (utpptr->utphdr.pc2abpsh),(utpptr->utphdr.pc2abpll),
            (utpptr->utphdr.a2pcbpsh),(utpptr->utphdr.a2pcbpll));
    utp_dbgoutput(utppb_ln);
    // return int 0 - auto destroy message
    return(0);
}

void utp_fhdr_state(void) {
    // status dump (associated with an F# pushbutton)
    msg_queue* queue;
    u32 error;
    // the message gets passed to MU_thread4
    queue=&((thr_ctrl+4)->mqueue);
    error=msgexec_gblkvoid_a2(queue, msg_utp_fhdr_state,0,0);
}

//------------------------------------------------------------------------------

void utp_frc_lseek(int fd,off_t offset, int whence) {
    u32 done=0;
    while (!done) {
        if (lseek(fd,offset,whence)!=(-1)) {
            done=1;
        }
    }
}

void utp_frc_read(int fd,void *buf,size_t n,off_t offset) {
    size_t dn;
    dn=0;
    while (dn!=n) {
        utp_frc_lseek(fd,offset,SEEK_SET);
        dn=read(fd,buf,n);
    }
}

void utp_frc_write(int fd,const void *buf,size_t n,off_t offset) {
    size_t dn;
    dn=0;
    while (dn!=n) {
        utp_frc_lseek(fd,offset,SEEK_SET);
        dn=write(fd,buf,n);
    }
}

ssize_t utp_blk_read(int fd,void *buf,size_t n,off_t offset) {
    ssize_t retval;
    // we have to expect this could return a different size
    //  as the driver encodes error returns as a read size of UTP_BLK_ERROR
    utp_frc_lseek(fd,offset,SEEK_SET);
    retval=read(fd,buf,n);
    return(retval);
}

ssize_t utp_blk_write(int fd,const void *buf,size_t n,off_t offset) {
    ssize_t retval;
    // we have to expect this could return a different size
    //  as the driver encodes error returns as a read size of UTP_BLK_ERROR
    utp_frc_lseek(fd,offset,SEEK_SET);
    retval=write(fd,buf,n);
    return(retval);
}

//------------------------------------------------------------------------------

int msg_com_xchg(void* ioptr) {
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    lnklst* xchg;
    xchg=(lnklst*)(*(((s64*)ioptr)+1));
    // just attach this xchg carrier into .cmtxchn
    lnklst_fins((&(ftuptr->cmtxchn)),xchg);
    // return int 0 - auto destroy message
    return(0);
}

void issue_com_xchg(uaio_xchg* xchg) {
    // returns 1 on crash abort, else 0
    msg_queue* queue;
    u32 error;
    // the message gets passed to MU_thread4
    queue=&((thr_ctrl+4)->mqueue);
    error=msgexec_gblkvoid_a2(queue, msg_com_xchg,((s64)xchg),0);
}

//------------------------------------------------------------------------------

int msg_comio_tst(void* ioptr) {
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    u32 tstN;
    tstN=(u32)(*(((s64*)ioptr)+1));
    switch (tstN) {
        case 0:
            tst_comio_00();
            break;
        case 1:
            tst_comio_01();
            break;
        case 2:
            tst_comio_02();
            break;
        case 3:
            tst_comio_03();
            break;
        default:
            // do nothing
            break;
    }
    // return int 0 - auto destroy message
    return(0);
}

int comio_tst_N(u32 N) {
    // returns 1 on crash abort, else 0
    msg_queue* queue;
    int retval;
    // the message gets passed to MU_thread5
    queue=&((thr_ctrl+5)->mqueue);
    // use custom message pass to avoid touching the raw_filhdr
    retval=msgexec_gblkvoid_a2(queue,msg_comio_tst,((s64)N),0);
    return(retval);
}

//------------------------------------------------------------------------------

int msg_comv_fk_mode(void* ioptr) {
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    //
    // it really does a void funckey() call over comiox
    //  to remote
    u32 findx;
    u32 arglo;
    u32 arghi;
    s64 val;
    u32 error;
    findx=(u32)(*(((s64*)ioptr)+1));
    arglo=(u32)(*(((s64*)ioptr)+2));
    arghi=(u32)(*(((s64*)ioptr)+3));
    error=caio_fnckey(findx,arglo,arghi,&val);
    // return int 0 - auto destroy message
    return(0);
}

int comv_fk_mode(u32 findx,u32 arglo,u32 arghi) {
    // returns 1 on crash abort, else 0
    msg_queue* queue;
    int retval;
    // the message gets passed to MU_thread5
    queue=&((thr_ctrl+5)->mqueue);
    retval=msgexec_gblkvoid(queue,msg_comv_fk_mode,((s64)findx),((s64)arglo),((s64)arghi));
    return(retval);
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

u32 tst_f0001ky(char* arg,char* dst) {
    //
    // assume this is called from thread4 - USB/uaiox handler thread
    //  the filename and header are hard coded constants
    //   invoking a function key from
    //    void uuiox_fnckey(void)
    //
    // return ERRNO
    //    -- set quad at dst = ERRNO
    //    -- set retval = 0x0004
    //
    // return s64 return arg
    //    -- set quad at dst = (OK) followed by 8 bytes
    //    -- set retval = 0x000C
    //
    u32 retval;
    u32 arglo;
    u32 arghi;
    u32 error=0;
    arglo=(u32)(*((u32*)arg));
    arghi=(u32)(*(((u32*)arg)+1));
    switch (arglo) {
        case 0:
            // capture with msk
            error=(u32)set_hass_capture(arghi);
            break;
        case 1:
            // timer test N frames
            error=(u32)set_hass_tmeasr(arghi);
            break;
        case 2:
            // request raw snapshots
            error=(u32)req_snapshot((int)arghi);
            break;
        case 3:
            // request rgb snapshots
            error=(u32)req_osnapshot((int)arghi);
            break;
        default:
            // it's an unknown test
            error=HERROR_FNCUNKN;
            break;
    }
    // generic fk() exit ------------
    if (error) {
        // ERRNO return
        retval=4;
        *((u32*)dst)=error;
    }
    else {
        // assume dst has been fully set 3 quads 0,(retlo),(rethi)
        //  in this case, do it here
        *((u32*)dst)=0;
        *((u32*)(dst+4))=0;
        *((u32*)(dst+8))=0;
        retval=12;
    }
    return(retval);
}

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

void tst_comio_00(void) {
    char* lclsrc;
    char* lcldst;
    char* ptr;
    u32 i;
    u32 tval;
    u32 error;
    lclsrc=(char*)(((u64)shrMEM_ptr)+0x220);
    lcldst=lclsrc+0x20;
    // generate a pattern
    ptr=lclsrc;
    tval=0x40;
    for (i=0;i<0x20;i+=1) {
        *ptr=(char)tval;
        ptr++;
        tval++;
    }
    // send over comiox
    error=caio_wbbuf_lrg(0x200,(u64)lclsrc,0x20);
    if (!error) {
        // write success - read over comiox
        error=caio_rbbuf_lrg((u64)lcldst,0x200,0x20);
    }
    /// if (error) {
    ///    // debug message?
    ///    }
}

void tst_comio_01(void) {
    char* lclsrc;
    char* lcldst;
    u16* ptr;
    u32 i;
    u32 tval;
    u32 error;
    lclsrc=(char*)(((u64)shrMEM_ptr)+0x220);
    lcldst=lclsrc+0x20;
    // generate a pattern
    ptr=(u16*)lclsrc;
    tval=0x5000;
    for (i=0;i<0x20;i+=2) {
        *ptr=(u16)tval;
        ptr++;
        tval++;
    }
    // send over comiox
    error=caio_wwbuf_lrg(0x200,(u64)lclsrc,0x20);
    if (!error) {
        // write success - read over comiox
        error=caio_rwbuf_lrg((u64)lcldst,0x200,0x20);
    }
    /// if (error) {
    ///    // debug message?
    ///    }
}

void tst_comio_02(void) {
    char* lclsrc;
    char* lcldst;
    u32* ptr;
    u32 i;
    u32 tval;
    u32 error;
    lclsrc=(char*)(((u64)shrMEM_ptr)+0x220);
    lcldst=lclsrc+0x20;
    // generate a pattern
    ptr=(u32*)lclsrc;
    tval=0x10203070;
    for (i=0;i<0x20;i+=4) {
        *ptr=(u32)tval;
        ptr++;
        tval++;
    }
    // send over comiox
    error=caio_wqbuf_lrg(0x200,(u64)lclsrc,0x20);
    if (!error) {
        // write success - read over comiox
        error=caio_rqbuf_lrg((u64)lcldst,0x200,0x20);
    }
    /// if (error) {
    ///    // debug message?
    ///    }
}

void tst_comio_03(void) {
    char* lclsrc;
    char* lcldst;
    char* lclref;
    u32 error;
    lclsrc=(char*)(((u64)shrMEM_ptr)+0x220);
    lcldst=lclsrc+0x20;
    lclref=lclsrc-0x20;
    error=0;
    // a byte swap
    if (!error) {
        // read element there
        error=caio_rbyte(0x200,(u32*)lclref);
        if (!error) {
            // set new value
            caio_wbyte(0x200,0xF1);
        }
    }
    // a word swap
    lclref+=4;
    if (!error) {
        // read element there
        error=caio_rword(0x204,(u32*)lclref);
        if (!error) {
            // set new value
            caio_wword(0x204,0xFFF2);
        }
    }
    // a quad swap
    lclref+=4;
    if (!error) {
        // read element there
        error=caio_rquad(0x208,(u32*)lclref);
        if (!error) {
            // set new value
            caio_wquad(0x208,0x0EEEFFF4);
        }
    }
    // read all over comiox
    if (!error) {
        error=caio_rwbuf_lrg((u64)lcldst,0x200,0x20);
    }
    /// if (error) {
    ///    // debug message?
    ///    }
}

u32 tst_f0002ky(char* arg,char* dst) {
    //
    // assume this is called from thread4 - USB/uaiox handler thread
    //  the filename and header are hard coded constants
    //   invoking a function key from
    //    void uuiox_fnckey(void) or comiox_fnckey(void)
    //
    // return ERRNO
    //    -- set quad at dst = ERRNO
    //    -- set retval = 0x0004
    //
    // return s64 return arg
    //    -- set quad at dst = (OK) followed by 8 bytes
    //    -- set retval = 0x000C
    //
    u32 retval;
    u32 arglo;
    u32 arghi;
    u32 error=0;
    arglo=(u32)(*((u32*)arg));
    arghi=(u32)(*(((u32*)arg)+1));
    error=comio_tst_N(arglo);
    // generic fk() exit ------------
    if (error) {
        // ERRNO return
        retval=4;
        *((u32*)dst)=error;
    }
    else {
        // assume dst has been fully set 3 quads 0,(retlo),(rethi)
        //  in this case, do it here
        *((u32*)dst)=0;
        *((u32*)(dst+4))=0;
        *((u32*)(dst+8))=0;
        retval=12;
    }
    return(retval);
}

int msg_tst_comrx_stat(void* ioptr) {
    u32 comst;
    u32 cmrst;
    u32 rhdwy;
    //
    comst=ftuptr->comstate;
    cmrst=ftuptr->cmrstate;
    rhdwy=(u32)bptrqueue_avail(ftuptr->cmrxque);
    //
    sprintf(utppb_ln,"tst_comrx_stat() comstate %02X cmrstate %02X rhdwy %04X\n",
            comst,cmrst,rhdwy
    );
    utp_dbgoutput(utppb_ln);
    //
    // return int 0 - auto destroy message
    return(0);
}

void tst_comrx_stat(void) {
    // status dump (associated with an F# pushbutton)
    msg_queue* queue;
    u32 error;
    // the message gets passed to MU_thread4
    queue=&((thr_ctrl+4)->mqueue);
    error=msgexec_gblkvoid_a2(queue,msg_tst_comrx_stat,0,0);
}

//------------------------------------------------------------------------------

jint Java_com_sony_isdc_musarc_MainActivity_JNIcreate(JNIEnv* env,jobject thiz) {
    // what gets returned is upper java layer launcherr - init default 1
    int retval=0;
    u32 ptrh;
    u32 ptrl;

    LOGI("JNI create\n");
    // set up shared memory across threads
    shrMEM_ptr=mmap((void*)0,shrMEM_size,
                    (PROT_READ|PROT_WRITE),
                    (MAP_SHARED|MAP_ANONYMOUS),
                    (-1),0);
    ptrh=(u32)((((s64)(shrMEM_ptr))>>32)&0xFFFFFFFF);
    ptrl=(u32)(((s64)(shrMEM_ptr))&0xFFFFFFFF);
    LOGI("shrMEM alloc size 0x%08X returned 0x%08X%08X\n",shrMEM_size,ptrh,ptrl);
    if (shrMEM_ptr==MAP_FAILED) {
        // no point to continue...
        retval=2;      // error value - shrMEM !!
    }
    else {
        // wipe the AppCtrlBlk
        memset(shrMEM_ptr,0,ACB_FIXED_SIZE);

        // ZZZZZ this is unnecessary, you can't use them to autonomously
        //  access the Java side .....
        //// // record android main thread jclass and JNIenv
        //// //  so we can autonomously call android methods from threads
        //// ACptr->topclass=(*env)->GetObjectClass(env,thiz);
        //// ACptr->topenv=env;
        //// ACptr->topthiz=thiz;
        // initial alc_state

        ACptr->alc_state=0;
        // initialize start of time
        ACptr->tmr_zoff=((int)(now_ms_i64()));
        ACptr->tmr_cnt=0;
        ACptr->thr0cnt=0;
        // memctrl setup happens once and there is no teardown
        //  the entire memory pool goes back to the system when the
        //   shared memory is un-mapped, so there is no other cleanup needed
        // allocate the AppCtrlBlk at the start of shared memory
        //  and the rest of shared memeory becomes a memctrl unit attached
        //   at the end
        ACptr->memc=memctrl_init( (((u64)shrMEM_ptr)+((u64)ACB_FIXED_SIZE)) ,
                                  ((u64)(shrMEM_size-ACB_FIXED_SIZE))  );
        // the glbk chain will draw from memctrl as needed to create more gblk's
        //  but the head must be initialized
        init_gblkchain();
        // init open loop xchg carriers
        //  this is really needed for interfaces to thread4 comms,
        //   but should be on line early -- any thread may try to send a message
        //    before thread4 is fully initialized
        olxchgring_init();
        // start threads
        // LOGI("enter threads_startup()\n");
        //threads_startup();
        // LOGI("exit threads_startup()\n");
    }
    // retval should be returned, this function should return int (jint)
    return ((jint)retval);
}
static int jni_state = 0;
void Java_com_sony_isdc_musarc_MainActivity_JNIstart(JNIEnv* env,jobject thiz) {
    
    if(jni_state==1) {
        LOGI("JNI already started\n");
        return;
    }
    jni_state = 1;

    LOGI("JNI start\n");
    // final alc_state
    ACptr->alc_state=1;
    //thr1flgs_setclr(T1FLG_TSTIMG, 0);
    threads_startup();
}

void Java_com_sony_isdc_musarc_MainActivity_JNIresume(JNIEnv* env,jobject thiz) {
    jclass thisClass;
    jmethodID mid;
    LOGI("JNI resume\n");
    // start camera from Java code
    // env=ACptr->topenv;
    // thiz=ACptr->topthiz;
    thisClass = (*env)->GetObjectClass(env,thiz);
    mid = (*env)->GetMethodID(env,thisClass,"cameraStartup","()V");
    (*env)->CallVoidMethod(env,thiz,mid);
    // final alc_state
    ACptr->alc_state=2;
}

void Java_com_sony_isdc_musarc_MainActivity_JNIpause(JNIEnv* env,jobject thiz) {
    jclass thisClass;
    jmethodID mid;
    //
    // evidently, I can get a call to JNIpause() after a JNIdestroy()
    // 01-06 05:34:04.975 28672-28672/com.example.tp I/Tp_NATIVE: JNI stop
    // 01-06 05:34:04.975 28672-28672/com.example.tp I/Tp_NATIVE: JNI destroy
    // 01-06 05:34:05.238 28672-28672/com.example.tp I/Tp_NATIVE: JNI pause
    //
    if (shrMEM_ptr!=MAP_FAILED) {
        LOGI("JNI pause\n");
        // tear down camera from Java code
        // env=ACptr->topenv;
        // thiz=ACptr->topthiz;
        thisClass = (*env)->GetObjectClass(env,thiz);
        mid = (*env)->GetMethodID(env,thisClass,"cameraTeardown","()V");
        (*env)->CallVoidMethod(env,thiz,mid);
        // final alc_state
        ACptr->alc_state=3;
    }
}

void Java_com_sony_isdc_musarc_MainActivity_JNIstop(JNIEnv* env,jobject thiz) {
    //
    // evidently, I can get a call to JNIstop() after a JNIdestroy()
    // 01-06 05:39:51.781 28997-28997/com.example.tp I/Tp_NATIVE: JNI stop
    // 01-06 05:39:51.781 28997-28997/com.example.tp I/Tp_NATIVE: JNI destroy
    // 01-06 05:39:52.204 28997-28997/com.example.tp I/Tp_NATIVE: JNI stop
    //
    if (shrMEM_ptr!=MAP_FAILED) {
        if(jni_state==0) {
            LOGI("JNI hasnt started yet\n");
            return;
        }
        jni_state = 0;
        LOGI("JNI stop\n");
        // final alc_state
        ACptr->alc_state=4;
        threads_shutdown();
    }
}

void Java_com_sony_isdc_musarc_MainActivity_JNIdestroy(JNIEnv* env,jobject thiz) {
    jclass thisClass;
    jmethodID mid;
    int* hwsema;
    int tst;
    jni_state = 0;
    LOGI("JNI destroy\n");
    // tear down and release shared memory
    if (!(shrMEM_ptr==MAP_FAILED)) {
        // block any other thread attempts to start shutdown of program
        //  if somebody set crash code, no problem
        //   if we're the first here, then main android thread is exiting normally
        //    in which case we can leave the default crashcode==0
        hwsema=(int*)(&(ACptr->hwsema_crsh));
        tst=__sync_fetch_and_or(hwsema,1);
        // disconnect FTDI uart if it's online
        thisClass = (*env)->GetObjectClass(env,thiz);
        mid = (*env)->GetMethodID(env,thisClass,"ft_disconnect","()V");
        (*env)->CallVoidMethod(env,thiz,mid);
        // shut down threads
        threads_shutdown();
        // release shared memory
        munmap(shrMEM_ptr,shrMEM_size);
        shrMEM_ptr=MAP_FAILED;
    }
    // final alc_state - of course, you can't do this when the
    //  the shared memory has been unmapped
    // ACptr->alc_state=5;
}

//------------------------------------------------------------------------------

// Java callable variants
jint Java_com_example_tp_TopActivity_getALCstate(JNIEnv* env,jobject thiz) {
    if (shrMEM_ptr!=MAP_FAILED) {
        return ((jint)(ACptr->alc_state));
    }
    else {
        // when shared memory has been shut down, signal like
        //  we're in JNIdestroy state
        return ((jint)5);
    }
}

jint Java_com_example_tp_TopActivity_tstCrash(JNIEnv* env,jobject thiz) {
    if (shrMEM_ptr!=MAP_FAILED) {
        return ((jint)(ACptr->hwsema_crsh));
    }
    else {
        // when shared memory has been shut down, signal crash true
        return ((jint)1);
    }
}

jint Java_com_example_tp_TopActivity_shtdnTimer(JNIEnv* env,jobject thiz) {
    if (shrMEM_ptr!=MAP_FAILED) {
        return ((jint)(ACptr->shtdn_timer));
    }
    else {
        // when shared memory has been shut down, signal timer expired
        return ((jint)0);
    }
}

void Java_com_sony_isdc_musarc_ImageProcessing_camCaptNative(JNIEnv* env,jobject thiz) {
    if (shrMEM_ptr!=MAP_FAILED) {
        camCapt();
    }
}

void Java_com_sony_isdc_musarc_ImageProcessing_camRlseNative(JNIEnv* env,jobject thiz) {
    if (shrMEM_ptr!=MAP_FAILED) {
        camRlse();
    }
}

void Java_com_example_tp_TopActivity_camJcrash(JNIEnv* env,jobject thiz) {
    thread_set_crash(CRASH_JCAMERA);
}

void Java_com_sony_isdc_musarc_ImageProcessing_cfeRawFrame(JNIEnv* env,jobject thiz,
                                                           int x,int y,jobject jBbuff) {
    // the Java camera interface hands off a RAW frame to the cfe
    char* src;
    thread1_ctrl* thr_ptr;
    msg_queue* queue;
    u32 err;
    src =(char*)((*env)->GetDirectBufferAddress(env,jBbuff));
    // the cfe is thread 1
    queue=&((thr_ctrl+1)->mqueue);
    thr_ptr=(thread1_ctrl*)(thr_ctrl+1);
    // wipe cfedone
    thr_ptr->cfedone=0;
    // pass void function and wait for ack of buffer received
    // VERBOSITY
    // LOGI("Calling -> msg_fe_frame X=%d Y=%d ptr=0x%016X\n",x,y,(s64)src);
    err=msgexec_gblkvoid(queue,msg_fe_frame,(s64)x,(s64)y,(s64)src);
}

int Java_com_example_tp_TopActivity_tstCfeDone(JNIEnv* env,jobject* thiz) {
    // polls thread1_ctrl.cfedone returning value
    //  if it's set, it automatically gets cleared
    thread1_ctrl* thr_ptr;
    int* hwsema;
    int retval;
    thr_ptr=(thread1_ctrl*)(thr_ctrl+1);
    hwsema=(int*)(&(thr_ptr->cfedone));
    retval=__sync_fetch_and_and(hwsema,0);
    return(retval);
}

void Java_com_example_tp_TopActivity_clrTexture(JNIEnv* env,jobject thiz) {
    clr_wt_texture();
}

void Java_com_example_tp_TopActivity_avlTexture(JNIEnv* env,jobject thiz,int x,int y) {
    ACptr->vwdim=(u32)((y<<16)|x);
    ACptr->vwpix=(u32)(y*x);
    set_wt_texture();
}

void Java_com_example_tp_TopActivity_waitTexture(JNIEnv* env,jobject thiz) {
    wait_on_texture();
}

jlong Java_com_example_tp_TopActivity_getImgAvail(JNIEnv* env,jobject* thiz) {
    thread2_ctrl* thr_ptr;
    int* hwsema;
    void* retval;
    thr_ptr=(thread2_ctrl*)(thr_ctrl+2);
    hwsema=(int*)(&(thr_ptr->hwsema_rdy));
    // pop available pixo_image
    retval=(void*)hsp_lnklst_fpop(hwsema,(&(thr_ptr->imo_rdy)));
    // return it as jlong
    return((jlong)retval);
}

void Java_com_example_tp_TopActivity_releaseImg(JNIEnv* env,jobject* thiz,jlong hndl) {
    pixo_image* pixoptr;
    int error;
    pixoptr=(pixo_image*)hndl;
    // detect crash, but nothing we can do about it here
    error=imo_refill(pixoptr);
}

jobject Java_com_example_tp_TopActivity_getImBytBuf(JNIEnv* env,jobject* thiz,jlong hndl) {
    // recall jni.h ->   typedef void* jobject;
    jobject retval;
    pixo_image* pixoptr;
    thread2_ctrl* thr_ptr;
    char* addr;
    jlong size;
    pixoptr=(pixo_image*)hndl;
    // addr points at the actual data
    addr=((char*)pixoptr)+64;
    // the size of the pixels is known
    thr_ptr=(thread2_ctrl*)(thr_ctrl+2);
    size=(jlong)(thr_ptr->imobytes);
    // then we can construct a ByteBuffer for return
    retval=(*env)->NewDirectByteBuffer(env,(void*)addr,size);
    return(retval);
}

void Java_com_example_tp_TopActivity_camStartMode(JNIEnv* env,jobject thiz,int viewsel) {
    // look to f0010ky for guide, never gets called for viewsel 0 - YUV is default
    if (viewsel==1) {
        // image bitmap renderer only pass RAW
        // (actually, leave the value of ACptr->ibm_msk as
        //   whatever it was... or minimally use its past value
        //    as part of the recovery decision process to get back to the same
        //     functional mode
        //
        // we no longer set this fixed hard coded default - stick with past value
        // set_ibm_msk(IBM_MSK_RAW);
        //
        // thread1 defaults - stay as they were, whether RAW, test image or HASS
        // thr1flgs_setclr(0,(T1FLG_TSTIMG|T1FLG_HASSEN));
        // set ImageView visible
        j2j_set_imgvw(1);
    }
    // else do nothing
}


void Java_com_sony_isdc_musarc_Serial_setD2xxPtr(JNIEnv* env,jobject thiz,jobject jD2xx) {
    // recall jni.h ->   typedef void* jobject;
    void* ptr;
    msg_queue* queue;
    u32 err;
    ptr=jD2xx;
    // thread 4 manages all comms
    queue=&((thr_ctrl+4)->mqueue);
    err=msgexec_gblkvoid(queue,msg_set_D2xx,(s64)ptr,0,0);
}

int Java_com_sony_isdc_musarc_Serial_ftTxAvail(JNIEnv* env,jobject thiz) {
    int retval;
    retval=fttx_avail();
    return(retval);
}

int Java_com_sony_isdc_musarc_Serial_ftTxCommit(JNIEnv* env,jobject thiz) {
    int retval;
    retval=fttx_commit();
    return(retval);
}

void Java_com_sony_isdc_musarc_Serial_ftRxCommit(JNIEnv* env,jobject thiz,int val) {
    ftrx_commit(val);
}

void Java_com_sony_isdc_musarc_Serial_ftRxTstamp(JNIEnv* env,jobject thiz) {
    ftuptr->rx_in_time=ACptr->tmr_cnt;
}

//------------------------------------------------------------------------------

void Java_com_example_tp_TopActivity_B0native(JNIEnv* env,jobject thiz) {
    // native action in response to screen F0
    // fk(0x10,mode)  - Image mode at HDMI
    //                0x00 - live texture YUV
    // live texture YUV
    // image bitmap renderer dispose all
    set_ibm_msk(IBM_MSK_NONE);
    // thread1 defaults
    thr1flgs_setclr(0,(T1FLG_TSTIMG|T1FLG_HASSEN));
    // set TextureView visible
    j2j_set_imgvw(0);
}

void Java_com_example_tp_TopActivity_B1native(JNIEnv* env,jobject thiz) {
    // native action in response to screen F1
    // fk(0x10,mode)  - Image mode at HDMI
    //                0x01 - feed bitmap RAW from thread2
    // feed bitmap RAW from thread 2
    // image bitmap renderer only pass RAW
    set_ibm_msk(IBM_MSK_RAW);
    // thread1 defaults
    thr1flgs_setclr(0,(T1FLG_TSTIMG|T1FLG_HASSEN));
    // set ImageView visible
    j2j_set_imgvw(1);
}

void Java_com_example_tp_TopActivity_B2native(JNIEnv* env,jobject thiz) {
    // native action in response to screen F2
    // fk(0x10,mode)  - Image mode at HDMI
    //                0x04 - feed bitmap RAW -> thread3 HASS/GPU
    // feed bitmap RAW -> thread3 HASS/GPU (normal HASS mode)
    // image bitmap renderer only pass HASS
    set_ibm_msk(IBM_MSK_HASS);
    // disable tst_inframe[] substitution
    thr1flgs_setclr(T1FLG_HASSEN,T1FLG_TSTIMG);
    // set ImageView visible
    j2j_set_imgvw(1);
}

void Java_com_example_tp_TopActivity_B3native(JNIEnv* env,jobject thiz) {
    // native action in response to screen F3

// QQQQQ
    // show what thread4 assumes is current state of driver
    //  dump to debug console
    utp_fhdr_state();

}

void Java_com_example_tp_TopActivity_B4native(JNIEnv* env,jobject thiz) {
    // native action in response to screen F4

// QQQQQ

}

void Java_com_example_tp_TopActivity_B5native(JNIEnv* env,jobject thiz) {
    // native action in response to screen F5

// QQQQQ
    tst_comrx_stat();

}

void Java_com_example_tp_TopActivity_B6native(JNIEnv* env,jobject thiz) {
    // native action in response to screen F6

// QQQQQ
    ftu_show_D2xx();

}

void Java_com_sony_isdc_musarc_Commander_writeUsb2Data(JNIEnv* env, jobject obj,
                                                       jbyteArray data) {
    jsize size = (*env)->GetArrayLength(env, data);
    (*env)->GetByteArrayRegion(env, data, 0, size, ACptr->usb2);
}

jbyteArray Java_com_sony_isdc_musarc_Commander_readUsb2Data(JNIEnv* env, jobject obj,
                                                            jint size) {
    jbyteArray data = (*env)->NewByteArray(env, size);
    (*env)->SetByteArrayRegion(env, data, 0, size, ACptr->usb2);
    return data;
}

void Java_com_sony_isdc_musarc_USB_rgbFrame(JNIEnv* env, jobject obj,
                                            jobject buffer, jlong timestamp,
                                            jboolean mainAlsEnabled, jobject main,
                                            jboolean slaveAlsEnabled, jobject slave,
                                            jboolean gpsEnabled, jobject location) {
    // the Java camera interface hands off a RGB frame to the cfe
    thread2_ctrl* thr_ptr = (thread2_ctrl*)thr_ctrl+2;
    msg_queue* queue = &(thr_ptr->mqueue);

    if (mainAlsEnabled) {
        jclass alsClass = (*env)->GetObjectClass(env, main);
        jfieldID alsFieldRed = (*env)->GetFieldID(env, alsClass, "red", "I");
        ACptr->usbmainals.red = (*env)->GetIntField(env, main, alsFieldRed);
        jfieldID alsFieldGreen = (*env)->GetFieldID(env, alsClass, "green", "I");
        ACptr->usbmainals.green = (*env)->GetIntField(env, main, alsFieldGreen);
        jfieldID alsFieldBlue = (*env)->GetFieldID(env, alsClass, "blue", "I");
        ACptr->usbmainals.blue = (*env)->GetIntField(env, main, alsFieldBlue);
        jfieldID alsFieldClear = (*env)->GetFieldID(env, alsClass, "clear", "I");
        ACptr->usbmainals.clear = (*env)->GetIntField(env, main, alsFieldClear);
        jfieldID alsFieldInfrared = (*env)->GetFieldID(env, alsClass, "infrared", "I");
        ACptr->usbmainals.infrared = (*env)->GetIntField(env, main, alsFieldInfrared);
    }

    if (slaveAlsEnabled) {
        jclass alsClass = (*env)->GetObjectClass(env, slave);
        jfieldID alsFieldRed = (*env)->GetFieldID(env, alsClass, "red", "I");
        ACptr->usbslaveals.red = (*env)->GetIntField(env, slave, alsFieldRed);
        jfieldID alsFieldGreen = (*env)->GetFieldID(env, alsClass, "green", "I");
        ACptr->usbslaveals.green = (*env)->GetIntField(env, slave, alsFieldGreen);
        jfieldID alsFieldBlue = (*env)->GetFieldID(env, alsClass, "blue", "I");
        ACptr->usbslaveals.blue = (*env)->GetIntField(env, slave, alsFieldBlue);
        jfieldID alsFieldClear = (*env)->GetFieldID(env, alsClass, "clear", "I");
        ACptr->usbslaveals.clear = (*env)->GetIntField(env, slave, alsFieldClear);
        jfieldID alsFieldInfrared = (*env)->GetFieldID(env, alsClass, "infrared", "I");
        ACptr->usbslaveals.infrared = (*env)->GetIntField(env, slave, alsFieldInfrared);
    }

    if (gpsEnabled && (location != NULL)) {
        jclass locationClass = (*env)->GetObjectClass(env, location);
        jmethodID getTime = (*env)->GetMethodID(env, locationClass, "getTime", "()J");
        ACptr->gps.time = (*env)->CallLongMethod(env, location, getTime);
        jmethodID getLatitude = (*env)->GetMethodID(env, locationClass, "getLatitude", "()D");
        ACptr->gps.latitude = (*env)->CallDoubleMethod(env, location, getLatitude);
        jmethodID getLongitude = (*env)->GetMethodID(env, locationClass, "getLongitude", "()D");
        ACptr->gps.longitude = (*env)->CallDoubleMethod(env, location, getLongitude);
        jmethodID getAltitude = (*env)->GetMethodID(env, locationClass, "getAltitude", "()D");
        ACptr->gps.altitude = (*env)->CallDoubleMethod(env, location, getAltitude);
        jmethodID getSpeed = (*env)->GetMethodID(env, locationClass, "getSpeed", "()F");
        ACptr->gps.speed = (*env)->CallFloatMethod(env, location, getSpeed);
        jmethodID getBearing = (*env)->GetMethodID(env, locationClass, "getBearing", "()F");
        ACptr->gps.bearing = (*env)->CallFloatMethod(env, location, getBearing);
    }

    // pass void function and wait for ack of buffer received
    // VERBOSITY
    // LOGI("Calling -> msg_fe_frame X=%d Y=%d ptr=0x%016X\n",x,y,(s64)src);
    int options = 0;
    if (mainAlsEnabled) {
        options |= 0x0001;
    }
    if (slaveAlsEnabled) {
        options |= 0x0010;
    }
    if (gpsEnabled && (location != NULL)) {
        options |= 0x0100;
    }
    void* src =((*env)->GetDirectBufferAddress(env,buffer));
    u32 err=msgexec_gblkvoid(queue,msg_rgb2pixo_frame,(s64)src,(s64)timestamp,(s64)options);
}

void Java_com_sony_isdc_musarc_USB_rawFrame(JNIEnv* env, jobject obj,
                                            jobject buffer, jlong timestamp,
                                            jboolean mainAlsEnabled, jobject main,
                                            jboolean slaveAlsEnabled, jobject slave,
                                            jboolean gpsEnabled, jobject location) {
    // the Java camera interface hands off a RAW frame to the cfe
    thread1_ctrl* thr_ptr = (thread1_ctrl*)(thr_ctrl+1);
    msg_queue* queue = &(thr_ptr->mqueue);

    if (mainAlsEnabled) {
        jclass alsClass = (*env)->GetObjectClass(env, main);
        jfieldID alsFieldRed = (*env)->GetFieldID(env, alsClass, "red", "I");
        ACptr->usbmainals.red = (*env)->GetIntField(env, main, alsFieldRed);
        jfieldID alsFieldGreen = (*env)->GetFieldID(env, alsClass, "green", "I");
        ACptr->usbmainals.green = (*env)->GetIntField(env, main, alsFieldGreen);
        jfieldID alsFieldBlue = (*env)->GetFieldID(env, alsClass, "blue", "I");
        ACptr->usbmainals.blue = (*env)->GetIntField(env, main, alsFieldBlue);
        jfieldID alsFieldClear = (*env)->GetFieldID(env, alsClass, "clear", "I");
        ACptr->usbmainals.clear = (*env)->GetIntField(env, main, alsFieldClear);
        jfieldID alsFieldInfrared = (*env)->GetFieldID(env, alsClass, "infrared", "I");
        ACptr->usbmainals.infrared = (*env)->GetIntField(env, main, alsFieldInfrared);
    }

    if (slaveAlsEnabled) {
        jclass alsClass = (*env)->GetObjectClass(env, slave);
        jfieldID alsFieldRed = (*env)->GetFieldID(env, alsClass, "red", "I");
        ACptr->usbslaveals.red = (*env)->GetIntField(env, slave, alsFieldRed);
        jfieldID alsFieldGreen = (*env)->GetFieldID(env, alsClass, "green", "I");
        ACptr->usbslaveals.green = (*env)->GetIntField(env, slave, alsFieldGreen);
        jfieldID alsFieldBlue = (*env)->GetFieldID(env, alsClass, "blue", "I");
        ACptr->usbslaveals.blue = (*env)->GetIntField(env, slave, alsFieldBlue);
        jfieldID alsFieldClear = (*env)->GetFieldID(env, alsClass, "clear", "I");
        ACptr->usbslaveals.clear = (*env)->GetIntField(env, slave, alsFieldClear);
        jfieldID alsFieldInfrared = (*env)->GetFieldID(env, alsClass, "infrared", "I");
        ACptr->usbslaveals.infrared = (*env)->GetIntField(env, slave, alsFieldInfrared);
    }

    if (gpsEnabled && (location != NULL)) {
        jclass locationClass = (*env)->GetObjectClass(env, location);
        jmethodID getTime = (*env)->GetMethodID(env, locationClass, "getTime", "()J");
        ACptr->gps.time = (*env)->CallLongMethod(env, location, getTime);
        jmethodID getLatitude = (*env)->GetMethodID(env, locationClass, "getLatitude", "()D");
        ACptr->gps.latitude = (*env)->CallDoubleMethod(env, location, getLatitude);
        jmethodID getLongitude = (*env)->GetMethodID(env, locationClass, "getLongitude", "()D");
        ACptr->gps.longitude = (*env)->CallDoubleMethod(env, location, getLongitude);
        jmethodID getAltitude = (*env)->GetMethodID(env, locationClass, "getAltitude", "()D");
        ACptr->gps.altitude = (*env)->CallDoubleMethod(env, location, getAltitude);
        jmethodID getSpeed = (*env)->GetMethodID(env, locationClass, "getSpeed", "()F");
        ACptr->gps.speed = (*env)->CallFloatMethod(env, location, getSpeed);
        jmethodID getBearing = (*env)->GetMethodID(env, locationClass, "getBearing", "()F");
        ACptr->gps.bearing = (*env)->CallFloatMethod(env, location, getBearing);
    }

    // pass void function and wait for ack of buffer received
    // VERBOSITY
    // LOGI("Calling -> msg_fe_frame X=%d Y=%d ptr=0x%016X\n",x,y,(s64)src);
    int options = 0;
    if (mainAlsEnabled) {
        options |= 0x0001;
    }
    if (slaveAlsEnabled) {
        options |= 0x0010;
    }
    if (gpsEnabled && (location != NULL)) {
        options |= 0x0100;
    }
    void* src =((*env)->GetDirectBufferAddress(env,buffer));
    u32 err=msgexec_gblkvoid(queue,msg_raw2cfe_frame,(s64)src,(s64)timestamp,(s64)options);
}

void Java_com_sony_isdc_musarc_NativeMemory_write(JNIEnv* env, jobject obj,
                                                  jint address, jobject data, jint size) {
    void* buffer = (*env)->GetDirectBufferAddress(env, data);
    void* baseAddress = ACptr;
    memcpy(baseAddress + address, buffer, size);
}

void Java_com_sony_isdc_musarc_NativeMemory_read(JNIEnv* env, jobject obj,
                                                 jint address, jobject data, jint size) {
    void* buffer = (*env)->GetDirectBufferAddress(env, data);
    void* baseAddress = ACptr;
    memcpy(buffer, baseAddress + address, size);
}

//------------------------------------------------------------------------------

int msg_rgb2pixo_frame(void* ioptr) {
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    thread2_ctrl* thr_ptr;
    pixo_image* pixoptr;
    u32 oy;
    u32 ox;
    u32 iy;
    u32 ix;
    u32 r;
    u32 g;
    u32 b;
    u32 pix;
    int pshft;
    //-------------------
    s64 timestamp;
    s64 options;
    s32* srcImage;
    int error;

    error=(int)utp_trafblk();
    if (!error) {
        srcImage = (*(((s64 *) ioptr) + 1));
        timestamp = (*(((s64 *) ioptr) + 2));
        options = (*(((s64 *) ioptr) + 3));

        thr_ptr = (thread2_ctrl *) (thr_ctrl + 2);

        // there's a request for output file snapshot
        pixoptr = (pixo_image *) lnklst_fpop(&(ACptr->coflpool));
        if (pixoptr) {
            // construct the header
            // nested rgb_filhdr (oconsole.h)
            // u16          xdim;        // 0x30
            // u16          ydim;        // 0x32
            // u32          tstmp;       // 0x34
            // u32          tdate;       // 0x38
            // u8           pbpp;        // 0x3C
            // u8           spr;         // 0x3D <tbd>
            // u8           pmod;        // 0x3E pmod - could encode mosaic type
            // u8           flgs;        // 0x3F flgs could be context dependent
            // als          main;        // 0x40 main ALS
            // als          slave;       // 0x50 slave ALS

            // dst points rgb_filhdr
            // copy xdim ydim
            u32* dim = (u32 *) (((char *) pixoptr) + 0x30);
            *dim = (u32) (*((u32 *) (&(thr_ptr->imodim))));
            // copy timestamp
            u64* time = (u64*) (((char *) pixoptr) + 0x38);
            *time = timestamp;

            u16* main_als = (u16*)(((char*)pixoptr) + 0x40);
            if (options & 0x0001) {
                *(main_als) = 1;
                *(main_als + 1) = ACptr->usbmainals.red;
                *(main_als + 2) = ACptr->usbmainals.green;
                *(main_als + 3) = ACptr->usbmainals.blue;
                *(main_als + 4) = ACptr->usbmainals.clear;
                *(main_als + 5) = ACptr->usbmainals.infrared;
            } else {
                *(main_als) = 0;
            }

            u16* slave_als = (u16*)(((char*)pixoptr) + 0x50);
            if (options & 0x0010) {
                *(slave_als) = 1;
                *(slave_als + 1) = ACptr->usbslaveals.red;
                *(slave_als + 2) = ACptr->usbslaveals.green;
                *(slave_als + 3) = ACptr->usbslaveals.blue;
                *(slave_als + 4) = ACptr->usbslaveals.clear;
                *(slave_als + 5) = ACptr->usbslaveals.infrared;
            } else {
                *(slave_als) = 0;
            }

            u32* gps = (u32*)(((char*)pixoptr) + 0x60);
            if (options & 0x0100) {
                *(gps) = 1;
                memcpy(gps + 2, &ACptr->gps.time, sizeof(long));
                memcpy(gps + 4, &ACptr->gps.latitude, sizeof(double));
                memcpy(gps + 6, &ACptr->gps.longitude, sizeof(double));
                memcpy(gps + 8, &ACptr->gps.altitude, sizeof(double));
                memcpy(gps + 10, &ACptr->gps.speed, sizeof(float));
                memcpy(gps + 11, &ACptr->gps.bearing, sizeof(float));
            } else {
                *(gps) = 0;
            }

            // keep pmod and flgs inherited from RAW cfe
            // total pixels
            oy = (thr_ptr->imobytes) >> 2;
            // output pixel area
            u32* pixel = (u32 *) (((char *) pixoptr) + 0x80);
            for (iy = 0; iy < oy; iy++) {
                // collect input
                b = ((u32) (*srcImage)) & 0xFF;
                g = (((u32) (*srcImage)) >> 8) & 0xFF;
                r = (((u32) (*srcImage)) >> 16) & 0xFF;
                // set output (unused bits are all 0)
                //  note this is backward from Android pixel map
                u32 pix = (b << 16) | (g << 8) | r;
                *pixel = pix;
                // update pointers
                srcImage++;
                pixel++;
            }
            // you can message/send it to another thread to process the file
            //  but we do it all locally
            // error=cofl_file_image((cofl_image*)pixoptr);
            //  call as void
            cofl_file_image((cofl_image *) pixoptr);
        }
        else {
            utp_dbgoutput("no cofl available\n");
            // no output buffer available - have to drop the frame
            // bump number of dropped frames
            (thr_ptr->pixodrop)++;
            // VERBOSITY
            // LOGI("hass to pixo frame drop\n");
            //ofilRlse();
            usbTransferFail();
        }
    } else {
        utp_dbgoutput("utp link down - rgb\n");
        //ofilRlse();
        usbTransferFail();
    }

    // else hassspec is NOT set... don't know what to do...
    // we're already signalling done...
    // return int 0 - auto destroy message
    return(0);
}

int msg_raw2cfe_frame(void* ioptr) {
    // modelled on
    //  typedef int (*gblkfnc)(void* ioptr);
    // ignore dstptr - this is really void
    thread1_ctrl* thr_ptr;
    cfei_image* cfeiptr;
    u32 oy;
    u32 ox;
    u32 iy;
    u32 ix;
    u32 pix;
    int pshft;
    //-------------------
    s64 timestamp;
    s64 options;
    u16* srcImage;
    int error;

    error=(int)utp_trafblk();
    if (!error) {
        srcImage = (*(((s64 *) ioptr) + 1));
        timestamp = (*(((s64 *) ioptr) + 2));
        options = (*(((s64 *) ioptr) + 3));

        thr_ptr=(thread1_ctrl*)(thr_ctrl+1);

        cfeiptr=(cfei_image*)lnklst_fpop((&(thr_ptr->cfepool)));
        if (cfeiptr) {
            // construct the header
            // nested rgb_filhdr (oconsole.h)
            // u16          xdim;        // 0x30
            // u16          ydim;        // 0x32
            // u32          tstmp;       // 0x34
            // u32          tdate;       // 0x38
            // u8           pbpp;        // 0x3C
            // u8           spr;         // 0x3D <tbd>
            // u8           pmod;        // 0x3E pmod - could encode mosaic type
            // u8           flgs;        // 0x3F flgs could be context dependent
            // als          main;        // 0x40 main ALS
            // als          slave;       // 0x50 slave ALS

            cfeiptr_rawhdr(cfeiptr);
            // copy xdim ydim
            u16* dim = (u16*)(((char*)cfeiptr) + 0x30);
            *(dim) = (*((u16*)(&(thr_ptr->cfedim))));
            *(dim+1) = (*((u16*)(&(thr_ptr->cfedim))+1));
            
            //2019.3.27 remove header data
            #if 0
            // copy timestamp
            u64* time = (u64*)(((char*)cfeiptr) + 0x38);
            *(time) = timestamp;

            u16* main_als = (u16*)(((char*)cfeiptr) + 0x40);
            if (options & 0x0001) {
                *(main_als) = 1;
                *(main_als + 1) = ACptr->usbmainals.red;
                *(main_als + 2) = ACptr->usbmainals.green;
                *(main_als + 3) = ACptr->usbmainals.blue;
                *(main_als + 4) = ACptr->usbmainals.clear;
                *(main_als + 5) = ACptr->usbmainals.infrared;
            } else {
                *(main_als) = 0;
            }

            u16* slave_als = (u16*)(((char*)cfeiptr) + 0x50);
            if (options & 0x0010) {
                *(slave_als) = 1;
                *(slave_als + 1) = ACptr->usbslaveals.red;
                *(slave_als + 2) = ACptr->usbslaveals.green;
                *(slave_als + 3) = ACptr->usbslaveals.blue;
                *(slave_als + 4) = ACptr->usbslaveals.clear;
                *(slave_als + 5) = ACptr->usbslaveals.infrared;
            } else {
                *(slave_als) = 0;
            }

            u32* gps = (u32*)(((char*)cfeiptr) + 0x60);
            if (options & 0x0100) {
                *(gps) = 1;
                memcpy(gps + 2, &ACptr->gps.time, sizeof(long));
                memcpy(gps + 4, &ACptr->gps.latitude, sizeof(double));
                memcpy(gps + 6, &ACptr->gps.longitude, sizeof(double));
                memcpy(gps + 8, &ACptr->gps.altitude, sizeof(double));
                memcpy(gps + 10, &ACptr->gps.speed, sizeof(float));
                memcpy(gps + 11, &ACptr->gps.bearing, sizeof(float));
            } else {
                *(gps) = 0;
            }
            #endif

            // output pixel area
            u16* pixel = (u16*) (((char*) cfeiptr) + 0x34);
            int size = cfeiptr->xdim * cfeiptr->ydim * sizeof(short) + META_DATA_SIZE;
            memcpy(pixel, srcImage, size);

            error=cfe_file_image(cfeiptr);
        }
        else {
            // no output buffer available - have to drop the frame
            // bump number of dropped frames
            (thr_ptr->cfeidrop)++;
            // VERBOSITY
            utp_dbgoutput("no cfe available\n");
            // LOGI("hass to pixo frame drop\n");
            //ofilRlse();
            usbTransferFail();
        }
    } else {
        utp_dbgoutput("utp link down - raw\n");
        //ofilRlse();
        usbTransferFail();
    }

    // else hassspec is NOT set... don't know what to do...
    // we're already signalling done...
    // return int 0 - auto destroy message
    return(0);
}


//------------------------------------------------------------------------------

int msg_ofil_refill(void* ioptr) {
    // eventually, the downstream traffic process should return the coflptr
    //  so we can replenish the cfepool
    //
    int error;
    ofil_carrier* ofilptr;
    ofilptr=(ofil_carrier*)(*(((s64*)ioptr)+1));
    // so just recycle this image buffer
    //lnklst_fins(&(ACptr->ofilpool),(lnklst*)ofilptr);
    // return int 0 - auto destroy message
    return(0);
}

int ofil_refill(ofil_carrier* ofilptr) {
    // returns 1 on crash abort, else 0
    // normal return of cofl_image (from any thread)
    msg_queue* queue;
    int retval;
    // the message gets passed to MU_thread2
    queue=&((thr_ctrl+4)->mqueue);
    retval=msgexec_gblkvoid(queue,msg_ofil_refill,((s64)ofilptr),0,0);
    return(retval);
}

static JavaVM* jvm;
static jobject commanderObj;
static jmethodID commanderExecuteMethod;
static jobject usbObj;
static jmethodID usbTransferCompleteMethod;

jint JNI_OnLoad(JavaVM* vm, void* reserved) {
    jvm = vm;

    JNIEnv* env;
    if ((*jvm)->GetEnv(jvm, &env, JNI_VERSION_1_6) != JNI_OK) {
        return JNI_EVERSION;
    }

    return JNI_VERSION_1_6;
}

jboolean Java_com_sony_isdc_musarc_Commander_reigsterNativeMethods(JNIEnv* env, jobject obj) {
    jclass commander = (*env)->GetObjectClass(env, obj);
    if (commander == NULL) {
        return JNI_FALSE;
    }

    commanderExecuteMethod = (*env)->GetMethodID(env, commander, "nativeExecute", "(III)I");
    if (commanderExecuteMethod == NULL) {
        return JNI_FALSE;
    }

    commanderObj = (*env)->NewGlobalRef(env, obj);
    return JNI_TRUE;
}

int commanderExecute(int command, int arg1, int arg2) {
    jboolean attached = JNI_FALSE;
    JNIEnv* env;
    if ((*jvm)->AttachCurrentThread(jvm, &env, NULL) == JNI_OK) {
        attached = JNI_TRUE;
    }
    if (attached) {
        jint ret = (*env)->CallIntMethod(env, commanderObj, commanderExecuteMethod, command, arg1, arg2);
        (*jvm)->DetachCurrentThread(jvm);
        return ret;
    } else {
        return -1;
    }
}

jboolean Java_com_sony_isdc_musarc_USB_reigsterNativeMethods(JNIEnv* env, jobject obj) {
    jclass usb = (*env)->GetObjectClass(env, obj);
    if (usb == NULL) {
        return JNI_FALSE;
    }

    usbTransferCompleteMethod = (*env)->GetMethodID(env, usb, "transferComplete", "(Z)V");
    if (usbTransferCompleteMethod == NULL) {
        return JNI_FALSE;
    }

    usbObj = (*env)->NewGlobalRef(env, obj);
    return JNI_TRUE;
}

void Java_com_sony_isdc_musarc_USB_setAccessoryMode(JNIEnv* env, jobject obj) {
    setUSBAccssry();
}

void Java_com_sony_isdc_musarc_USB_setAdbMode(JNIEnv* env, jobject obj) {
    setUSBAdb();
}

void setScalingGovernor(int mode) {
    char kstr_interactive[]="interactive";
    char kstr_performance[]="performance";
    char kstr_conservative[]="conservative";
    char kstr_powersave[]="powersave";

    int fd = open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor", O_WRONLY);
    if (fd != -1) {
        switch(mode) {
            case 0: write(fd, (void*)kstr_interactive, 11); break;
            case 1: write(fd, (void*)kstr_performance, 11); break;
            case 2: write(fd, (void*)kstr_conservative, 12); break;
            case 3: write(fd, (void*)kstr_powersave, 9); break;
        }
        close(fd);
    }
}

void Java_com_sony_isdc_musarc_camera_CameraContext_setInteractiveMode(JNIEnv* env, jobject obj) {
    setScalingGovernor(0);
}

void Java_com_sony_isdc_musarc_camera_CameraContext_setPerformanceMode(JNIEnv* env, jobject obj) {
    setCameraOptimizationOff();
    setScalingGovernor(2);
}

void usbTransferSuccess() {
    jboolean attached = JNI_FALSE;
    JNIEnv* env;
    if ((*jvm)->AttachCurrentThread(jvm, &env, NULL) == JNI_OK) {
        attached = JNI_TRUE;
    }
    if (attached) {
        (*env)->CallVoidMethod(env, usbObj, usbTransferCompleteMethod, JNI_TRUE);
        (*jvm)->DetachCurrentThread(jvm);
    }
}

void usbTransferFail() {
    jboolean attached = JNI_FALSE;
    JNIEnv* env;
    if ((*jvm)->AttachCurrentThread(jvm, &env, NULL) == JNI_OK) {
        attached = JNI_TRUE;
    }
    if (attached) {
        (*env)->CallVoidMethod(env, usbObj, usbTransferCompleteMethod, JNI_FALSE);
        (*jvm)->DetachCurrentThread(jvm);
    }
}

void Java_com_sony_isdc_musarc_MainActivity_test(JNIEnv* env, jobject obj) {
    commanderExecute(10, 0, 0);
}
