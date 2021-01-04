/*
Description:
    Configure Lora concentrator and forward packets to a server
    Use hwtimer for packet timestamping.

Maintainer: Forest-Rain
*/

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */
/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
    #define _XOPEN_SOURCE 600
#else
    #define _XOPEN_SOURCE 500
#endif
#include "rtthread.h"        /* C99 types */
#include <stdint.h>         /* C99 types */
#include <stdbool.h>        /* bool type */
#include <stdio.h>          /* printf, fprintf, snprintf, fopen, fputs */

#include <string.h>         /* memset */
#include <signal.h>         /* sigaction */

#include <time.h>           /* time, clock_gettime, strftime, gmtime */
#include <stdlib.h>         /* atoi, exit */
#include <errno.h>          /* error messages */
#include <math.h>           /* modf */
#include <assert.h>

#include <sys/socket.h>     /* socket specific definitions */
#ifdef RT_USING_AT
#include <at_device.h>
#endif

#include <netdb.h>          /* gai_strerror */
#ifdef RT_USING_PTHREADS
#include <pthread.h>
#endif
#include "trace.h"
#include "jitqueue.h"
#include "timersync.h"
#include "parson.h"
#include "base64.h"

#include "clock_time.h" //for  CLOCK_MONOTONIC

#ifdef PKG_USING_LORA_RADIO_DRIVER
#include "lora-radio-timer.h"
#include "lora-radio.h"
#endif

#ifdef RT_USING_ULOG
#define DRV_DEBUG
#define LOG_TAG             "lgwsc.pkt.fwd"
#include <drv_log.h>
#include <ulog.h> 
#else
#define LOG_D rt_kprintf
#define LOG_I rt_kprintf
#endif

#ifdef PKG_USING_EASYFLASH
#include <easyflash.h>
#endif


/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))
#define STRINGIFY(x)    #x
#define STR(x)          STRINGIFY(x)
// 
#define exit(x) 
/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#ifndef VERSION_STRING
  #define VERSION_STRING "undefined"
#endif

#if 1 //def LGW_USING_LORAGW_THINGS_QCLOUD_COM
#define DEFAULT_SERVER      loragw.things.qcloud.com  /* hostname also supported */
#define DEFAULT_PORT_UP     1700
#define DEFAULT_PORT_DW     1700
#else
#define DEFAULT_SERVER      127.0.0.1   /* hostname also supported */
#define DEFAULT_PORT_UP     1780
#define DEFAULT_PORT_DW     1782
#endif
#define DEFAULT_KEEPALIVE   5           /* default time interval for downstream keep-alive packet */
#define DEFAULT_STAT        30          /* default time interval for statistics */
#define PUSH_TIMEOUT_MS     100
#define PULL_TIMEOUT_MS     200
#define GPS_REF_MAX_AGE     30          /* maximum admitted delay in seconds of GPS loss before considering latest GPS sync unusable */
#define FETCH_SLEEP_MS      10          /* nb of ms waited when a fetch return no packets */
#define BEACON_POLL_MS      50          /* time in ms between polling of beacon TX status */

#define PROTOCOL_VERSION    2           /* v1.3 */

#define XERR_INIT_AVG       128         /* nb of measurements the XTAL correction is averaged on as initial value */
#define XERR_FILT_COEF      256         /* coefficient for low-pass XTAL error tracking */

#define PKT_PUSH_DATA   0
#define PKT_PUSH_ACK    1
#define PKT_PULL_DATA   2
#define PKT_PULL_RESP   3
#define PKT_PULL_ACK    4
#define PKT_TX_ACK      5

#define NB_PKT_MAX      1 /* max number of packets per fetch/send cycle */

#define MIN_LORA_PREAMB 6 /* minimum Lora preamble length for this application */
#define STD_LORA_PREAMB 8
#define MIN_FSK_PREAMB  3 /* minimum FSK preamble length for this application */
#define STD_FSK_PREAMB  5

#define STATUS_SIZE     200
#define TX_BUFF_SIZE    ((540 * NB_PKT_MAX) + 30 + STATUS_SIZE)

#define UNIX_GPS_EPOCH_OFFSET 315964800 /* Number of seconds ellapsed between 01.Jan.1970 00:00:00
                                                                          and 06.Jan.1980 00:00:00 */

//  rado event
#define EV_RADIO_INIT            0x0001
#define EV_RADIO_TX_START        0x0002
#define EV_RADIO_TX_DONE         0x0004
#define EV_RADIO_TX_TIMEOUT      0x0008
#define EV_RADIO_RX_DONE         0x0010
#define EV_RADIO_RX_TIMEOUT      0x0020
#define EV_RADIO_RX_ERROR        0x0040
#define EV_RADIO_ALL             0x003F

#define DEBUG_PKT_FWD   1//0
#define DEBUG_LOG       1


//#define RF_FREQUENCY_RX                             470300000 // Hz - CH0
//#define RF_FREQUENCY_TX                             500300000 // Hz - CH0
#define RF_FREQUENCY_RX                             471900000 // Hz - CH8
#define RF_FREQUENCY_TX                             501900000 // Hz - CH8

#define TX_OUTPUT_POWER                             14        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_FIX_LENGTH_PAYLOAD_OFF                 false
#define LORA_IQ_INVERSION_ON                        false
#define LORA_IQ_INVERSION_OFF                       false
/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

/* signal handling variables */
volatile bool exit_sig = false; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
volatile bool quit_sig = false; /* 1 -> application terminates without shutting down the hardware */

/* packets filtering configuration variables */
static bool fwd_valid_pkt = true; /* packets with PAYLOAD CRC OK are forwarded */
static bool fwd_error_pkt = false; /* packets with PAYLOAD CRC ERROR are NOT forwarded */
//static bool fwd_nocrc_pkt = false; /* packets with NO PAYLOAD CRC are NOT forwarded */


/* gateway <-> MAC protocol variables */
static uint32_t net_mac_h; /* Most Significant Nibble, network order */
static uint32_t net_mac_l; /* Least Significant Nibble, network order */

/* network sockets */
static int sock_up; /* socket for upstream traffic */
static int sock_down; /* socket for downstream traffic */

/* network protocol variables */
static struct timeval push_timeout_half = {0, (PUSH_TIMEOUT_MS * 500)}; /* cut in half, critical for throughput */
static struct timeval pull_timeout = {0, (PULL_TIMEOUT_MS * 1000)}; /* non critical for throughput */

/* hardware access control and correction */
pthread_mutex_t mx_concent = PTHREAD_MUTEX_INITIALIZER; /* control access to the concentrator */

/* measurements to establish statistics */
static pthread_mutex_t mx_meas_up = PTHREAD_MUTEX_INITIALIZER; /* control access to the upstream measurements */
static uint32_t meas_nb_rx_rcv = 0; /* count packets received */
static uint32_t meas_nb_rx_ok = 0; /* count packets received with PAYLOAD CRC OK */
static uint32_t meas_nb_rx_bad = 0; /* count packets received with PAYLOAD CRC ERROR */
static uint32_t meas_nb_rx_nocrc = 0; /* count packets received with NO PAYLOAD CRC */
static uint32_t meas_up_pkt_fwd = 0; /* number of radio packet forwarded to the server */
static uint32_t meas_up_network_byte = 0; /* sum of UDP bytes sent for upstream traffic */
static uint32_t meas_up_payload_byte = 0; /* sum of radio payload bytes sent for upstream traffic */
static uint32_t meas_up_dgram_sent = 0; /* number of datagrams sent for upstream traffic */
static uint32_t meas_up_ack_rcv = 0; /* number of datagrams acknowledged for upstream traffic */

//static pthread_mutex_t mx_meas_dw = PTHREAD_MUTEX_INITIALIZER; /* control access to the downstream measurements */
 pthread_mutex_t mx_meas_dw = PTHREAD_MUTEX_INITIALIZER; 
static uint32_t meas_dw_pull_sent = 0; /* number of PULL requests sent for downstream traffic */
static uint32_t meas_dw_ack_rcv = 0; /* number of PULL requests acknowledged for downstream traffic */
static uint32_t meas_dw_dgram_rcv = 0; /* count PULL response packets received for downstream traffic */
static uint32_t meas_dw_network_byte = 0; /* sum of UDP bytes sent for upstream traffic */
static uint32_t meas_dw_payload_byte = 0; /* sum of radio payload bytes sent for upstream traffic */
static uint32_t meas_nb_tx_ok = 0; /* count packets emitted successfully */
static uint32_t meas_nb_tx_fail = 0; /* count packets were TX failed for other reasons */
static uint32_t meas_nb_tx_requested = 0; /* count TX request from server (downlinks) */
static uint32_t meas_nb_tx_rejected_collision_packet = 0; /* count packets were TX request were rejected due to collision with another packet already programmed */
static uint32_t meas_nb_tx_rejected_collision_beacon = 0; /* count packets were TX request were rejected due to collision with a beacon already programmed */
static uint32_t meas_nb_tx_rejected_too_late = 0; /* count packets were TX request were rejected because it is too late to program it */
static uint32_t meas_nb_tx_rejected_too_early = 0; /* count packets were TX request were rejected because timestamp is too much in advance */
static uint32_t meas_nb_beacon_queued = 0; /* count beacon inserted in jit queue */
static uint32_t meas_nb_beacon_sent = 0; /* count beacon actually sent to concentrator */
static uint32_t meas_nb_beacon_rejected = 0; /* count beacon rejected for queuing */

static pthread_mutex_t mx_stat_rep = PTHREAD_MUTEX_INITIALIZER; /* control access to the status report */
static bool report_ready = false; /* true when there is a new report to send to the server */
static char status_report[STATUS_SIZE]; /* status report as a JSON object */

/* auto-quit function */
static uint32_t autoquit_threshold = 0; /* enable auto-quit after a number of non-acknowledged PULL_DATA (0 = disabled)*/

/* Just In Time TX scheduling */
static struct jit_queue_s jit_queue;

/* Gateway specificities */
static int8_t antenna_gain = 0;

/* TX capabilities */
static struct lgw_tx_gain_lut_s txlut =  /* TX gain table */
{
    {0, 2, 4, 6, 8, 10, 12, 14, 15, 16, 17, 18, 20},
    13,
};
/* lowest frequency supported by TX chain */
static uint32_t tx_freq_min[1]=
{
    470000000,
}; 
/* highest frequency supported by TX chain */
static uint32_t tx_freq_max[1]=
{
    525000000,
};
  
struct lgw_pkt_tx_s lgwsc_tx_pkt = { 0 };
struct lgw_pkt_rx_s lgwsc_rx_pkt = { 0 };

struct lgwsc_conf_params_s lgwsc_conf = 
{
    .txc = 
        {
            .freq_hz         = RF_FREQUENCY_TX,
            .rf_power        = TX_OUTPUT_POWER,
            
            // lora
            .modulation      = MOD_LORA,
            .datarate        = DR_LORA_SF12,
            .bandwidth       = BW_125KHZ,
            .coderate        = CR_LORA_4_5,
            .preamble        = LORA_PREAMBLE_LENGTH,
            
            .invert_pol      = true,
        },
        
    .rxc =
        {
            .freq_hz         = RF_FREQUENCY_RX,

            // lora
            .modulation      = MOD_LORA, 
            .datarate        = DR_LORA_SF12,
            .bandwidth       = BW_125KHZ,
            .coderate        = CR_LORA_4_5,
        },
        
    .gweui = { 0x00,0x95,0x69,0xFF,0xFE,0x00,0x00,0x01 },
    
    /* network configuration variables */
    .serv_addr = STR(DEFAULT_SERVER),/* address of the server (host name or IPv4/IPv6) */
    .serv_port_up = STR(DEFAULT_PORT_UP), /* server port for upstream traffic */
    .serv_port_down = STR(DEFAULT_PORT_DW), /* server port for downstream traffic */
    .keepalive_time = DEFAULT_KEEPALIVE,/* send a PULL_DATA request every X seconds, negative = disabled */

    /* statistics collection configuration variables */
    .stat_interval = DEFAULT_STAT,/* time interval (in sec) at which statistics are collected and displayed */
};


const uint32_t lora_radio_dr_table[] = { 5,6,7,8,9,10,11,12 }; 
const uint32_t lora_radio_bw_table[] = { 0,1,2,3};
const char *bw_table_string[3] = {"125","250","500"};
const RadioModems_t lora_radio_mod_table[] = { MODEM_LORA, MODEM_FSK };
uint8_t lora_radio_dr_index = 1;
uint8_t lora_radio_tx_status = 0;

static struct rt_event loragw_radio_event;
/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

/* threads */
static void thread_up(void);
static void thread_down(void);
static void thread_jit(void);

static int lgwsc_get_cfg_len(void);
static int lgwsc_read_cfg_info(void *buff, int len);
static void lgwsc_pkt_fwd_thread(void* parameter);
static void lgwsc_radio_config(void);
static void lgwsc_send_packet(void);
    
/*!
 * \brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
static void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
static void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
static void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
static void OnRxError( void );

static void wait_ms(unsigned long a);

#ifdef LORAGW_PKT_FWD_USING_OLED_DISPLAY
void lgwsc_oled_init(void);
void lgwsc_oled_update(float temperature,uint32_t cp_nb_rx_rcv);
#endif

TimerEvent_t lgwsc_tx_on_timestamp_timer;

void OnTxDone( void )
{
    Radio.Sleep( );
    rt_event_send(&loragw_radio_event, EV_RADIO_TX_DONE);
    lora_radio_tx_status = TX_FREE;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    lgwsc_rx_pkt.size = size;
    rt_memcpy( lgwsc_rx_pkt.payload, payload, lgwsc_rx_pkt.size );
    lgwsc_rx_pkt.rssi = rssi;
    lgwsc_rx_pkt.snr = snr;
    
#ifdef LORA_RADIO_DRIVER_USING_RXDONE_TIMESTAMP 
    lgwsc_rx_pkt.count_us = Radio.GetRxdoneTimestamp(); 
#endif    
    lgwsc_rx_pkt.status = STAT_CRC_OK;
 
    rt_event_send(&loragw_radio_event, EV_RADIO_RX_DONE);
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    rt_event_send(&loragw_radio_event, EV_RADIO_TX_TIMEOUT);
    lora_radio_tx_status = TX_FREE;
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    rt_event_send(&loragw_radio_event, EV_RADIO_RX_TIMEOUT);
}

void OnRxError( void )
{
    Radio.Sleep( );
    rt_event_send(&loragw_radio_event, EV_RADIO_RX_ERROR);
}

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */
static double difftimespec(struct timespec end, struct timespec beginning) {
    double x;

    x = 1E-9 * (double)(end.tv_nsec - beginning.tv_nsec);
    x += (double)(end.tv_sec - beginning.tv_sec);

    return x;
}

static int send_tx_ack(uint8_t token_h, uint8_t token_l, enum jit_error_e error) {
    uint8_t buff_ack[64]; /* buffer to give feedback to server */
    int buff_index;

    /* reset buffer */
    memset(&buff_ack, 0, sizeof buff_ack);

    /* Prepare downlink feedback to be sent to server */
    buff_ack[0] = PROTOCOL_VERSION;
    buff_ack[1] = token_h;
    buff_ack[2] = token_l;
    buff_ack[3] = PKT_TX_ACK;
    *(uint32_t *)(buff_ack + 4) = net_mac_h;
    *(uint32_t *)(buff_ack + 8) = net_mac_l;
    buff_index = 12; /* 12-byte header */

    /* Put no JSON string if there is nothing to report */
    if (error != JIT_ERROR_OK) {
        /* start of JSON structure */
        memcpy((void *)(buff_ack + buff_index), (void *)"{\"txpk_ack\":{", 13);
        buff_index += 13;
        /* set downlink error status in JSON structure */
        memcpy((void *)(buff_ack + buff_index), (void *)"\"error\":", 8);
        buff_index += 8;
        switch (error) {
            case JIT_ERROR_FULL:
            case JIT_ERROR_COLLISION_PACKET:
                memcpy((void *)(buff_ack + buff_index), (void *)"\"COLLISION_PACKET\"", 18);
                buff_index += 18;
                /* update stats */
                pthread_mutex_lock(&mx_meas_dw);
                meas_nb_tx_rejected_collision_packet += 1;
                pthread_mutex_unlock(&mx_meas_dw);
                break;
            case JIT_ERROR_TOO_LATE:
                memcpy((void *)(buff_ack + buff_index), (void *)"\"TOO_LATE\"", 10);
                buff_index += 10;
                /* update stats */
                pthread_mutex_lock(&mx_meas_dw);
                meas_nb_tx_rejected_too_late += 1;
                pthread_mutex_unlock(&mx_meas_dw);
                break;
            case JIT_ERROR_TOO_EARLY:
                memcpy((void *)(buff_ack + buff_index), (void *)"\"TOO_EARLY\"", 11);
                buff_index += 11;
                /* update stats */
                pthread_mutex_lock(&mx_meas_dw);
                meas_nb_tx_rejected_too_early += 1;
                pthread_mutex_unlock(&mx_meas_dw);
                break;
            case JIT_ERROR_COLLISION_BEACON:
                memcpy((void *)(buff_ack + buff_index), (void *)"\"COLLISION_BEACON\"", 18);
                buff_index += 18;
                /* update stats */
                pthread_mutex_lock(&mx_meas_dw);
                meas_nb_tx_rejected_collision_beacon += 1;
                pthread_mutex_unlock(&mx_meas_dw);
                break;
            case JIT_ERROR_TX_FREQ:
                memcpy((void *)(buff_ack + buff_index), (void *)"\"TX_FREQ\"", 9);
                buff_index += 9;
                break;
            case JIT_ERROR_TX_POWER:
                memcpy((void *)(buff_ack + buff_index), (void *)"\"TX_POWER\"", 10);
                buff_index += 10;
                break;
            case JIT_ERROR_GPS_UNLOCKED:
                memcpy((void *)(buff_ack + buff_index), (void *)"\"GPS_UNLOCKED\"", 14);
                buff_index += 14;
                break;
            default:
                memcpy((void *)(buff_ack + buff_index), (void *)"\"UNKNOWN\"", 9);
                buff_index += 9;
                break;
        }
        /* end of JSON structure */
        memcpy((void *)(buff_ack + buff_index), (void *)"}}", 2);
        buff_index += 2;
    }

    buff_ack[buff_index] = 0; /* add string terminator, for safety */

    /* send datagram to server */
    return send(sock_down, (void *)buff_ack, buff_index, 0);
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int lgwsc_pkt_fwd_init(void)
{
    /* get lgw cfg len */
    int16_t len = lgwsc_get_cfg_len();
    if (len > 0)
    {
        /* get lgw config Parameters */
         if (lgwsc_read_cfg_info(&lgwsc_conf.cfg_flag,len) != len)
         {
             LOG_I("Get LGWSC Radio Parameters Failed!\n");
         }
         else
         {
             LOG_I("Get LGWSC Radio Parameters Successed!\n");
         }
    }
    else
    {
        LOG_I("failed to load lgw parameters from flash!\n");
    }

    if( lgwsc_conf.cfg_flag )
    {
        rt_thread_t tid = rt_thread_create("lgwsc_pkt_fwd", lgwsc_pkt_fwd_thread, NULL, 8192, 15, 5);
        if (tid)
        {
            rt_thread_startup(tid);
            return 0;
        }
        else
        {
            LOG_I("lgwsc_pkt_fwd thread create failed");
            return -1;
        }
    }
    else
    {
        LOG_I("lgwsc_pkt_fwd auto start disable\n");
    }
   
    return 0;
}

void lgwsc_pkt_fwd_thread(void* parameter)
{
    int i; /* loop variable and temporary variable for return value */

    /* configuration file related */

    /* threads */
    pthread_t thrid_up;
    pthread_t thrid_down;
    pthread_t thrid_jit;

    /* network socket creation */
    struct addrinfo hints;
    struct addrinfo *result; /* store result of getaddrinfo */
    struct addrinfo *q; /* pointer to move into *result data */

    /* variables to get local copies of measurements */
    uint32_t cp_nb_rx_rcv;
    uint32_t cp_nb_rx_ok;
    uint32_t cp_nb_rx_bad;
    uint32_t cp_nb_rx_nocrc;
    uint32_t cp_up_pkt_fwd;
    uint32_t cp_up_network_byte;
    uint32_t cp_up_payload_byte;
    uint32_t cp_up_dgram_sent;
    uint32_t cp_up_ack_rcv;
    uint32_t cp_dw_pull_sent;
    uint32_t cp_dw_ack_rcv;
    uint32_t cp_dw_dgram_rcv;
    uint32_t cp_dw_network_byte;
    uint32_t cp_dw_payload_byte;
    uint32_t cp_nb_tx_ok;
    uint32_t cp_nb_tx_fail;
    uint32_t cp_nb_tx_requested = 0;
    uint32_t cp_nb_tx_rejected_collision_packet = 0;
    uint32_t cp_nb_tx_rejected_collision_beacon = 0;
    uint32_t cp_nb_tx_rejected_too_late = 0;
    uint32_t cp_nb_tx_rejected_too_early = 0;
    uint32_t cp_nb_beacon_queued = 0;
    uint32_t cp_nb_beacon_sent = 0;
    uint32_t cp_nb_beacon_rejected = 0;

    /* statistics variable */
    time_t t;
    char stat_timestamp[24];
    float rx_ok_ratio;
    float rx_bad_ratio;
    float rx_nocrc_ratio;
    float up_ack_ratio;
    float dw_ack_ratio;

#ifdef LORAGW_PKT_FWD_USING_OLED_DISPLAY
    #ifdef PKG_USING_U8G2
    lgwsc_oled_init();
    #endif
#endif

    /* display version informations */
    LOG_I("*** LLGWSC Started! *** ");
    /* display host endianness */
    #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        LOG_I("INFO: Little endian host\n");
    #elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        LOG_I("INFO: Big endian host\n");
    #else
        LOG_I("INFO: Host endianness unknown\n");
    #endif
    
    /* process some of the configuration variables */
    net_mac_h = htonl((uint32_t)(0xFFFFFFFF & (lgwsc_conf.gweui[0]<<24)|(lgwsc_conf.gweui[1]<<16)|(lgwsc_conf.gweui[2]<<8)|(lgwsc_conf.gweui[3])));
    net_mac_l = htonl((uint32_t)(0xFFFFFFFF & (lgwsc_conf.gweui[4]<<24)|(lgwsc_conf.gweui[5]<<16)|(lgwsc_conf.gweui[6]<<8)|(lgwsc_conf.gweui[7])));
    LOG_I("LGWSC EUI: %02X%02X%02X%02X%02X%02X%02X%02X", lgwsc_conf.gweui[0],lgwsc_conf.gweui[1],lgwsc_conf.gweui[2],lgwsc_conf.gweui[3],lgwsc_conf.gweui[4],lgwsc_conf.gweui[5],lgwsc_conf.gweui[6],lgwsc_conf.gweui[7]);
 
    struct in_addr addr;
    /* prepare hints to open network sockets */
    memset(&hints, 0, sizeof hints);
    ///memset(&hints, 0, sizeof(struct addrinfo));

    hints.ai_family = AF_INET; /* WA: Forcing IPv4 as AF_UNSPEC makes connection on localhost to fail */
    hints.ai_socktype = SOCK_DGRAM; /* UDP */    
   
    LOG_I("LGWSC serv_addr:%s",lgwsc_conf.serv_addr);
    LOG_I("port up:%s",lgwsc_conf.serv_port_up);
    LOG_I("port dn:%s",lgwsc_conf.serv_port_down);
    /* look for server address w/ upstream port */
    {
        i = getaddrinfo(lgwsc_conf.serv_addr, lgwsc_conf.serv_port_up, &hints, &result);
        if (i != 0) { 
            LOG_I("ERROR: [up] getaddrinfo on address %s (PORT %s) returned %s\n", lgwsc_conf.serv_addr, lgwsc_conf.serv_port_up,strerror(i));
        }
    }
    
    addr.s_addr = ((struct sockaddr_in*)(result->ai_addr))->sin_addr.s_addr;
    LOG_I("[up]ip addresss: %s\n", inet_ntoa(addr));
    
    /* try to open socket for upstream traffic */
    for (q=result; q!=NULL; q=q->ai_next) {
        sock_up = socket(q->ai_family, q->ai_socktype,q->ai_protocol);
        if (sock_up == -1) continue; /* try next field */
        else break; /* success, get out of loop */
    }
 /* connect so we can send/receive packet with the server only */
    i = connect(sock_up, q->ai_addr, q->ai_addrlen);
    if (i != 0) {
        LOG_I("ERROR: [up] connect returned %s\n", strerror(errno));
        //exit(EXIT_FAILURE);
    }
    freeaddrinfo(result);

    /* look for server address w/ downstream port */
    i = getaddrinfo(lgwsc_conf.serv_addr, lgwsc_conf.serv_port_down, &hints, &result);
    if (i != 0) {
        LOG_I("ERROR: [down] getaddrinfo on address %s (port %s) returned %s\n", lgwsc_conf.serv_addr, lgwsc_conf.serv_port_up, strerror(i)); /// lwip 涓嶆敮鎸� gai_strerror
        //exit(EXIT_FAILURE);
    }

    /* try to open socket for downstream traffic */ 
    for (q=result; q!=NULL; q=q->ai_next) {
        sock_down = socket(q->ai_family, q->ai_socktype,q->ai_protocol);
        if (sock_down == -1) continue; /* try next field */
        else break; /* success, get out of loop */
    }
     /* connect so we can send/receive packet with the server only */
    i = connect(sock_down, q->ai_addr, q->ai_addrlen);
    if (i != 0) {
        LOG_I("ERROR: [down] connect returned %s\n", strerror(errno));
        //exit(EXIT_FAILURE);
    }
    freeaddrinfo(result);
    
    /* starting the concentrator */
    i = lgwsc_start();
    if (i == LGW_HAL_SUCCESS) {
        LOG_I("INFO: [main] concentrator started, packet can now be received\n");
    } else {
        LOG_I("ERROR: [main] failed to start the concentrator\n");
        //exit(EXIT_FAILURE);
    }

    /* spawn threads to manage upstream and downstream */
    i = pthread_create( &thrid_up, NULL, (void * (*)(void *))thread_up, NULL);
    if (i != 0) {
        LOG_I("ERROR: [main] impossible to create upstream thread\n");
        //exit(EXIT_FAILURE);
    }
    i = pthread_create( &thrid_down, NULL, (void * (*)(void *))thread_down, NULL);
    if (i != 0) {
        LOG_I("ERROR: [main] impossible to create downstream thread\n");
        exit(EXIT_FAILURE);
    }
    i = pthread_create( &thrid_jit, NULL, (void * (*)(void *))thread_jit, NULL);
    if (i != 0) {
        LOG_I("ERROR: [main] impossible to create JIT thread\n");
        exit(EXIT_FAILURE);
    }
    /* main loop task : statistics collection */
    while (!exit_sig && !quit_sig) {
        /* wait for next reporting interval */
        wait_ms(1000 * lgwsc_conf.stat_interval);

        /* get timestamp for statistics */
        t = time(NULL);
        strftime(stat_timestamp, sizeof stat_timestamp, "%F %T %Z", gmtime(&t));

        /* access upstream statistics, copy and reset them */
        pthread_mutex_lock(&mx_meas_up);
        cp_nb_rx_rcv       = meas_nb_rx_rcv;
        cp_nb_rx_ok        = meas_nb_rx_ok;
        cp_nb_rx_bad       = meas_nb_rx_bad;
        cp_nb_rx_nocrc     = meas_nb_rx_nocrc;
        cp_up_pkt_fwd      = meas_up_pkt_fwd;
        cp_up_network_byte = meas_up_network_byte;
        cp_up_payload_byte = meas_up_payload_byte;
        cp_up_dgram_sent   = meas_up_dgram_sent;
        cp_up_ack_rcv      = meas_up_ack_rcv;
        meas_nb_rx_rcv = 0;
        meas_nb_rx_ok = 0;
        meas_nb_rx_bad = 0;
        meas_nb_rx_nocrc = 0;
        meas_up_pkt_fwd = 0;
        meas_up_network_byte = 0;
        meas_up_payload_byte = 0;
        meas_up_dgram_sent = 0;
        meas_up_ack_rcv = 0;
        pthread_mutex_unlock(&mx_meas_up);
        if (cp_nb_rx_rcv > 0) {
            rx_ok_ratio = (float)cp_nb_rx_ok / (float)cp_nb_rx_rcv;
            rx_bad_ratio = (float)cp_nb_rx_bad / (float)cp_nb_rx_rcv;
            rx_nocrc_ratio = (float)cp_nb_rx_nocrc / (float)cp_nb_rx_rcv;
        } else {
            rx_ok_ratio = 0.0;
            rx_bad_ratio = 0.0;
            rx_nocrc_ratio = 0.0;
        }
        if (cp_up_dgram_sent > 0) {
            up_ack_ratio = (float)cp_up_ack_rcv / (float)cp_up_dgram_sent;
        } else {
            up_ack_ratio = 0.0;
        }

        /* access downstream statistics, copy and reset them */
        pthread_mutex_lock(&mx_meas_dw);
        cp_dw_pull_sent    =  meas_dw_pull_sent;
        cp_dw_ack_rcv      =  meas_dw_ack_rcv;
        cp_dw_dgram_rcv    =  meas_dw_dgram_rcv;
        cp_dw_network_byte =  meas_dw_network_byte;
        cp_dw_payload_byte =  meas_dw_payload_byte;
        cp_nb_tx_ok        =  meas_nb_tx_ok;
        cp_nb_tx_fail      =  meas_nb_tx_fail;
        cp_nb_tx_requested                 +=  meas_nb_tx_requested;
        cp_nb_tx_rejected_collision_packet +=  meas_nb_tx_rejected_collision_packet;
        cp_nb_tx_rejected_collision_beacon +=  meas_nb_tx_rejected_collision_beacon;
        cp_nb_tx_rejected_too_late         +=  meas_nb_tx_rejected_too_late;
        cp_nb_tx_rejected_too_early        +=  meas_nb_tx_rejected_too_early;
        cp_nb_beacon_queued   +=  meas_nb_beacon_queued;
        cp_nb_beacon_sent     +=  meas_nb_beacon_sent;
        cp_nb_beacon_rejected +=  meas_nb_beacon_rejected;
        meas_dw_pull_sent = 0;
        meas_dw_ack_rcv = 0;
        meas_dw_dgram_rcv = 0;
        meas_dw_network_byte = 0;
        meas_dw_payload_byte = 0;
        meas_nb_tx_ok = 0;
        meas_nb_tx_fail = 0;
        meas_nb_tx_requested = 0;
        meas_nb_tx_rejected_collision_packet = 0;
        meas_nb_tx_rejected_collision_beacon = 0;
        meas_nb_tx_rejected_too_late = 0;
        meas_nb_tx_rejected_too_early = 0;
        meas_nb_beacon_queued = 0;
        meas_nb_beacon_sent = 0;
        meas_nb_beacon_rejected = 0;
        pthread_mutex_unlock(&mx_meas_dw);
        if (cp_dw_pull_sent > 0) {
            dw_ack_ratio = (float)cp_dw_ack_rcv / (float)cp_dw_pull_sent;
        } else {
            dw_ack_ratio = 0.0;
        }
        
        /* display a report */
        LOG_D("\n##### %s #####\n", stat_timestamp);
        LOG_D("### [UPSTREAM] ###\n");
        LOG_D("# RF packets received by concentrator: %u\n", cp_nb_rx_rcv);
        LOG_D("# CRC_OK: %.2f%%, CRC_FAIL: %.2f%%, NO_CRC: %.2f%%\n", (double)100.0 * rx_ok_ratio, (double)100.0 * rx_bad_ratio, (double)100.0 * rx_nocrc_ratio);
        LOG_D("# RF packets forwarded: %u (%u bytes)\n", cp_up_pkt_fwd, cp_up_payload_byte);
        LOG_D("# PUSH_DATA datagrams sent: %u (%u bytes)\n", cp_up_dgram_sent, cp_up_network_byte);
        LOG_D("# PUSH_DATA acknowledged: %d%%\n", (double)100.0 * up_ack_ratio);//%.02f
        LOG_D("### [DOWNSTREAM] ###\n");
        LOG_D("# PULL_DATA sent: %u (%.2f%% acknowledged)\n", cp_dw_pull_sent, (double)100.0 * dw_ack_ratio);
        LOG_D("# PULL_RESP(onse) datagrams received: %u (%u bytes)\n", cp_dw_dgram_rcv, cp_dw_network_byte);
        LOG_D("# RF packets sent to concentrator: %u (%u bytes)\n", (cp_nb_tx_ok+cp_nb_tx_fail), cp_dw_payload_byte);
        LOG_D("# TX errors: %u\n", cp_nb_tx_fail);
        if (cp_nb_tx_requested != 0 ) { // %.2f%% -> %d%%
            LOG_D("# TX rejected (collision packet): %.2f%% (req:%u, rej:%u)\n", (double)100.0 * cp_nb_tx_rejected_collision_packet / cp_nb_tx_requested, cp_nb_tx_requested, cp_nb_tx_rejected_collision_packet);
            LOG_D("# TX rejected (collision beacon): %.2f%% (req:%u, rej:%u)\n", (double)100.0 * cp_nb_tx_rejected_collision_beacon / cp_nb_tx_requested, cp_nb_tx_requested, cp_nb_tx_rejected_collision_beacon);
            LOG_D("# TX rejected (too late): %.2f%% (req:%u, rej:%u)\n", (double)100.0 * cp_nb_tx_rejected_too_late / cp_nb_tx_requested, cp_nb_tx_requested, cp_nb_tx_rejected_too_late);
            LOG_D("# TX rejected (too early): %.2f%% (req:%u, rej:%u)\n", (double)100.0 * cp_nb_tx_rejected_too_early / cp_nb_tx_requested, cp_nb_tx_requested, cp_nb_tx_rejected_too_early);
        }
#ifdef LORAGW_PKT_FWD_USING_OLED_DISPLAY
        #ifdef PKG_USING_U8G2
        lgwsc_oled_update(0,cp_nb_rx_rcv);    
        #endif
#endif  
        
        LOG_D("### [JIT] ###\n");
      
        jit_print_queue (&jit_queue, false, DEBUG_LOG);

        /* generate a JSON report (will be sent to server by upstream thread) */
        pthread_mutex_lock(&mx_stat_rep);
   
        {
            snprintf(status_report, STATUS_SIZE, "\"stat\":{\"time\":\"%s\",\"rxnb\":%u,\"rxok\":%u,\"rxfw\":%u,\"ackr\":%.1f,\"dwnb\":%u,\"txnb\":%u}", stat_timestamp, cp_nb_rx_rcv, cp_nb_rx_ok, cp_up_pkt_fwd, (double)100.0 * up_ack_ratio, cp_dw_dgram_rcv, cp_nb_tx_ok);
        }
        ////report_ready = true;
        pthread_mutex_unlock(&mx_stat_rep);
    }

    /* wait for upstream thread to finish (1 fetch cycle max) */
    pthread_join(thrid_up, NULL);
    pthread_cancel(thrid_down); /* don't wait for downstream thread */
    pthread_cancel(thrid_jit); /* don't wait for jit thread */

    LOG_I("INFO: Exiting packet forwarder program\n");
    exit(EXIT_SUCCESS);
}
MSH_CMD_EXPORT(lgwsc_pkt_fwd_thread, LGWSC pkt_fwd_thread );

/* -------------------------------------------------------------------------- */
/* --- THREAD 1: RECEIVING PACKETS AND FORWARDING THEM ---------------------- */

void thread_up(void) {
    int i, j; /* loop variables */
    unsigned pkt_in_dgram; /* nb on Lora packet in the current datagram */

    /* allocate memory for packet fetching and processing */

    struct lgw_pkt_rx_s *p; /* pointer on a RX packet */
    int nb_pkt = 1;

    /* data buffers */
    uint8_t buff_up[TX_BUFF_SIZE]; /* buffer to compose the upstream packet */
    int buff_index;
    uint8_t buff_ack[32]; /* buffer to receive acknowledges */

    /* protocol variables */
    uint8_t token_h; /* random token for acknowledgement matching */
    uint8_t token_l; /* random token for acknowledgement matching */

    /* ping measurement variables */
    struct timespec send_time;
    struct timespec recv_time;

    /* report management variable */
    bool send_report = false;

    /* mote info variables */
    uint32_t mote_addr = 0;
    uint16_t mote_fcnt = 0;

    /* set upstream socket RX timeout */
    i = setsockopt(sock_up, SOL_SOCKET, SO_RCVTIMEO, (void *)&push_timeout_half, sizeof push_timeout_half);
    if (i != 0) {
        LOG_I("ERROR: [up] setsockopt returned %s(%d)\n", strerror(errno),i);
        exit(EXIT_FAILURE);
    }

    /* pre-fill the data buffer with fixed fields */
    buff_up[0] = PROTOCOL_VERSION;
    buff_up[3] = PKT_PUSH_DATA;
    *(uint32_t *)(buff_up + 4) = net_mac_h;
    *(uint32_t *)(buff_up + 8) = net_mac_l;

    rt_uint32_t ev = 0;

    while (!exit_sig && !quit_sig) 
    {
    if (rt_event_recv(&loragw_radio_event, EV_RADIO_ALL,
                                        RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                                        RT_WAITING_FOREVER, &ev) == RT_EOK)
        {
            switch( ev )
            {
                case EV_RADIO_RX_DONE:
                  /* check if there are status report to send */
                        send_report = report_ready; /* copy the variable so it doesn't change mid-function */
                        /* no mutex, we're only reading */

                        /* start composing datagram with the header */
                        token_h = (uint8_t)rand(); /* random token */
                        token_l = (uint8_t)rand(); /* random token */
                        buff_up[1] = token_h;
                        buff_up[2] = token_l;
                        buff_index = 12; /* 12-byte header */

                        /* start of JSON structure */
                        memcpy((void *)(buff_up + buff_index), (void *)"{\"rxpk\":[", 9);
                        buff_index += 9;

                        /* serialize Lora packets metadata and payload */
                        pkt_in_dgram = 0;
                        for (i=0; i < nb_pkt; ++i) {
                            p = &lgwsc_rx_pkt;

                            /* Get mote information from current packet (addr, fcnt) */
                            /* FHDR - DevAddr */
                            mote_addr  = p->payload[1];
                            mote_addr |= p->payload[2] << 8;
                            mote_addr |= p->payload[3] << 16;
                            mote_addr |= p->payload[4] << 24;
                            /* FHDR - FCnt */
                            mote_fcnt  = p->payload[6];
                            mote_fcnt |= p->payload[7] << 8;

                            /* basic packet filtering */
                            pthread_mutex_lock(&mx_meas_up);
                            meas_nb_rx_rcv += 1;
                            switch(p->status) {
                                case STAT_CRC_OK:
                                    meas_nb_rx_ok += 1;
                                    LOG_D( "\nINFO: Received pkt from mote: %08X (fcnt=%u)\n", mote_addr, mote_fcnt );
                                    if (!fwd_valid_pkt) {
                                        pthread_mutex_unlock(&mx_meas_up);
                                        continue; /* skip that packet */
                                    }
                                    break;
                                case STAT_CRC_BAD:
                                    meas_nb_rx_bad += 1;
                                    if (!fwd_error_pkt) {
                                        pthread_mutex_unlock(&mx_meas_up);
                                        continue; /* skip that packet */
                                    }
                                    break;
////                                case STAT_NO_CRC:
////                                    meas_nb_rx_nocrc += 1;
////                                    if (!fwd_nocrc_pkt) {
////                                        pthread_mutex_unlock(&mx_meas_up);
////                                        continue; /* skip that packet */
////                                    }
////                                    break;
                                default:
                                    LOG_I("WARNING: [up] received packet with unknown status %u (size %u, modulation %u, BW %u, DR %u, RSSI %.1f)\n", p->status, p->size, p->params.modulation, p->params.bandwidth, p->params.datarate, p->rssi);
                                    pthread_mutex_unlock(&mx_meas_up);
                                    continue; /* skip that packet */
                                    // exit(EXIT_FAILURE);
                            }
                            meas_up_pkt_fwd += 1;
                            meas_up_payload_byte += p->size;
                            pthread_mutex_unlock(&mx_meas_up);

                            /* Start of packet, add inter-packet separator if necessary */
                            if (pkt_in_dgram == 0) {
                                buff_up[buff_index] = '{';
                                ++buff_index;
                            } else {
                                buff_up[buff_index] = ',';
                                buff_up[buff_index+1] = '{';
                                buff_index += 2;
                            }

                            /* RAW timestamp, 8-17 useful chars */
                            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, "\"tmst\":%u", p->count_us);
                            if (j > 0) {
                                buff_index += j;
                                //for rxpkt timestamp
                                LOG_D("INFO: [up] json.rxpkt.tmst = %u\n", p->count_us);
                            } else {
                                LOG_I("ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                                exit(EXIT_FAILURE);
                            }

                            /* Packet concentrator channel, RF chain & RX frequency, 34-36 useful chars */
                            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"chan\":%1u,\"rfch\":%1u,\"freq\":%.6lf", p->if_chain, p->rf_chain, ((double)p->params.freq_hz / 1e6));
                            if (j > 0) {
                                buff_index += j;
                            } else {
                                LOG_I("ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                                exit(EXIT_FAILURE);
                            }

                            /* Packet status, 9-10 useful chars */
                            switch (p->status) {
                                case STAT_CRC_OK:
                                    memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":1", 9);
                                    buff_index += 9;
                                    break;
                                case STAT_CRC_BAD:
                                    memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":-1", 10);
                                    buff_index += 10;
                                    break;
                                case STAT_NO_CRC:
                                    memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":0", 9);
                                    buff_index += 9;
                                    break;
                                default:
                                    LOG_I("ERROR: [up] received packet with unknown status\n");
                                    memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":?", 9);
                                    buff_index += 9;
                                    exit(EXIT_FAILURE);
                            }

                            /* Packet modulation, 13-14 useful chars */
                            if (p->params.modulation == MOD_LORA) {
                                memcpy((void *)(buff_up + buff_index), (void *)",\"modu\":\"LORA\"", 14);
                                buff_index += 14;

                                /* Lora datarate & bandwidth, 16-19 useful chars */
                                switch (p->params.datarate) {
                                    case DR_LORA_SF7:
                                        memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF7", 12);
                                        buff_index += 12;
                                        break;
                                    case DR_LORA_SF8:
                                        memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF8", 12);
                                        buff_index += 12;
                                        break;
                                    case DR_LORA_SF9:
                                        memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF9", 12);
                                        buff_index += 12;
                                        break;
                                    case DR_LORA_SF10:
                                        memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF10", 13);
                                        buff_index += 13;
                                        break;
                                    case DR_LORA_SF11:
                                        memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF11", 13);
                                        buff_index += 13;
                                        break;
                                    case DR_LORA_SF12:
                                        memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF12", 13);
                                        buff_index += 13;
                                        break;
                                    default:
                                        LOG_I("ERROR: [up] lora packet with unknown datarate\n");
                                        memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF?", 12);
                                        buff_index += 12;
                                        exit(EXIT_FAILURE);
                                }
                                switch (p->params.bandwidth) {
                                    case BW_125KHZ:
                                        memcpy((void *)(buff_up + buff_index), (void *)"BW125\"", 6);
                                        buff_index += 6;
                                        break;
                                    case BW_250KHZ:
                                        memcpy((void *)(buff_up + buff_index), (void *)"BW250\"", 6);
                                        buff_index += 6;
                                        break;
                                    case BW_500KHZ:
                                        memcpy((void *)(buff_up + buff_index), (void *)"BW500\"", 6);
                                        buff_index += 6;
                                        break;
                                    default:
                                        LOG_I("ERROR: [up] lora packet with unknown bandwidth\n");
                                        memcpy((void *)(buff_up + buff_index), (void *)"BW?\"", 4);
                                        buff_index += 4;
                                        exit(EXIT_FAILURE);
                                }

                                /* Packet ECC coding rate, 11-13 useful chars */
                                switch (p->params.coderate) {
                                    case CR_LORA_4_5:
                                        memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/5\"", 13);
                                        buff_index += 13;
                                        break;
                                    case CR_LORA_4_6:
                                        memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/6\"", 13);
                                        buff_index += 13;
                                        break;
                                    case CR_LORA_4_7:
                                        memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/7\"", 13);
                                        buff_index += 13;
                                        break;
                                    case CR_LORA_4_8:
                                        memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/8\"", 13);
                                        buff_index += 13;
                                        break;
                                    case 0: /* treat the CR0 case (mostly false sync) */
                                        memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"OFF\"", 13);
                                        buff_index += 13;
                                        break;
                                    default:
                                        LOG_I("ERROR: [up] lora packet with unknown coderate\n");
                                        memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"?\"", 11);
                                        buff_index += 11;
                                        exit(EXIT_FAILURE);
                                }

                                /* Lora SNR, 11-13 useful chars */
                                j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"lsnr\":%.1f", p->snr);
                                if (j > 0) {
                                    buff_index += j;
                                } else {
                                    LOG_I("ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                                    exit(EXIT_FAILURE);
                                }
                            } else if (p->params.modulation == MOD_FSK) {
                                memcpy((void *)(buff_up + buff_index), (void *)",\"modu\":\"FSK\"", 13);
                                buff_index += 13;

                                /* FSK datarate, 11-14 useful chars */
                                j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"datr\":%u", p->params.datarate);
                                if (j > 0) {
                                    buff_index += j;
                                } else {
                                    LOG_I("ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                                    exit(EXIT_FAILURE);
                                }
                            } else {
                                LOG_I("ERROR: [up] received packet with unknown modulation\n");
                                exit(EXIT_FAILURE);
                            }

                            /* Packet RSSI, payload size, 18-23 useful chars */
                            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"rssi\":%.0f,\"size\":%u", p->rssi, p->size);
                            if (j > 0) {
                                buff_index += j;
                            } else {
                                LOG_I("ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                                exit(EXIT_FAILURE);
                            }

                            /* Packet base64-encoded payload, 14-350 useful chars */
                            memcpy((void *)(buff_up + buff_index), (void *)",\"data\":\"", 9);
                            buff_index += 9;
                            j = bin_to_b64(p->payload, p->size, (char *)(buff_up + buff_index), 341); /* 255 bytes = 340 chars in b64 + null char */
                            if (j>=0) {
                                buff_index += j;
                            } else {
                                LOG_I("ERROR: [up] bin_to_b64 failed line %u\n", (__LINE__ - 5));
                                exit(EXIT_FAILURE);
                            }
                            buff_up[buff_index] = '"';
                            ++buff_index;

                            /* End of packet serialization */
                            buff_up[buff_index] = '}';
                            ++buff_index;
                            ++pkt_in_dgram;
                        }

                        /* restart fetch sequence without sending empty JSON if all packets have been filtered out */
                        if (pkt_in_dgram == 0) {
                            if (send_report == true) {
                                /* need to clean up the beginning of the payload */
                                buff_index -= 8; /* removes "rxpk":[ */
                            } else {
                                /* all packet have been filtered out and no report, restart loop */
                                continue;
                            }
                        } else {
                            /* end of packet array */
                            buff_up[buff_index] = ']';
                            ++buff_index;
                            /* add separator if needed */
                            if (send_report == true) {
                                buff_up[buff_index] = ',';
                                ++buff_index;
                            }
                        }

                        /* add status report if a new one is available */
                        if (send_report == true) {
                            pthread_mutex_lock(&mx_stat_rep);
                            report_ready = false;
                            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, "%s", status_report);
                            pthread_mutex_unlock(&mx_stat_rep);
                            if (j > 0) {
                                buff_index += j;
                            } else {
                                LOG_I("ERROR: [up] snprintf failed line %u\n", (__LINE__ - 5));
                                exit(EXIT_FAILURE);
                            }
                        }

                        /* end of JSON datagram payload */
                        buff_up[buff_index] = '}';
                        ++buff_index;
                        buff_up[buff_index] = 0; /* add string terminator, for safety */

                        LOG_I("\nJSON up: %s\n", (char *)(buff_up + 12)); /* DEBUG: display JSON payload */

                        /* send datagram to server */
                        j = send(sock_up, (void *)buff_up, buff_index, 0);
                        clock_gettime(CLOCK_REALTIME, &send_time); //  clock_gettime(CLOCK_MONOTONIC, &send_time);
                        LOG_I("send datagram to server by udp:%d @ %d\n",j, send_time);
                        
                        pthread_mutex_lock(&mx_meas_up);
                        meas_up_dgram_sent += 1;
                        meas_up_network_byte += buff_index;

                        /* wait for acknowledge (in 2 times, to catch extra packets) */
                        for (i=0; i<2; ++i) {
                            j = recv(sock_up, (void *)buff_ack, sizeof buff_ack, 0);
                            clock_gettime(CLOCK_REALTIME, &recv_time);//clock_gettime(CLOCK_MONOTONIC, &recv_time);
                            if (j == -1) {
                                if (errno == EAGAIN) { /* timeout EWOULDBLOCK */ 
                                    LOG_I("recv timeout:%d\n",j);
                                    continue;
                                } else { /* server connection error */
                                    LOG_I("server connection error:%d,%d\n",j,errno);
                                    break;
                                }
                            } else if ((j < 4) || (buff_ack[0] != PROTOCOL_VERSION) || (buff_ack[3] != PKT_PUSH_ACK)) {
                                LOG_I("WARNING: [up] ignored invalid non-ACL packet\n");
                                continue;
                            } else if ((buff_ack[1] != token_h) || (buff_ack[2] != token_l)) {
                                LOG_I("WARNING: [up] ignored out-of sync ACK packet\n");
                                continue;
                            } else {
                                LOG_I("INFO: [up] PUSH_ACK received in %i ms\n", (int)(1000 * difftimespec(recv_time, send_time)));
                                meas_up_ack_rcv += 1;
                                break;
                            }
                        }
                        pthread_mutex_unlock(&mx_meas_up);
                    //no break//break;
                        
                case EV_RADIO_TX_DONE:
                case EV_RADIO_TX_TIMEOUT:
                case EV_RADIO_RX_TIMEOUT:
                case EV_RADIO_RX_ERROR:
                    // for lorawan downlink with iq inversion
                    Radio.Sleep();
                    Radio.SetChannel( lgwsc_rx_pkt.params.freq_hz );
                    Radio.SetRxConfig( lora_radio_mod_table[lgwsc_rx_pkt.params.modulation >> 5], lora_radio_bw_table[3-lgwsc_rx_pkt.params.bandwidth], lora_radio_dr_table[lora_radio_dr_index],
                                       lgwsc_rx_pkt.params.coderate, 0, LORA_PREAMBLE_LENGTH,
                                       LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_OFF,
                                       0, true, 0, 0, LORA_IQ_INVERSION_OFF, true );
                    Radio.Rx(0);
                    LOG_I("INFO: LGW goto RX,ASAP\n");
                    break;
            }
        }  
    }
    LOG_I("\nINFO: End of upstream thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 2: POLLING SERVER AND ENQUEUING PACKETS IN JIT QUEUE ---------- */

void thread_down(void) {
    int i; /* loop variables */

    /* configuration and metadata for an outbound packet */
    struct lgw_pkt_tx_s txpkt;
    bool sent_immediate = false; /* option to sent the packet immediately */

    /* local timekeeping variables */
    struct timespec send_time; /* time of the pull request */
    struct timespec recv_time; /* time of return from recv socket call */

    /* data buffers */
    uint8_t buff_down[1000]; /* buffer to receive downstream packets */
    uint8_t buff_req[12]; /* buffer to compose pull requests */
    int LOG_I_len;

    /* protocol variables */
    uint8_t token_h; /* random token for acknowledgement matching */
    uint8_t token_l; /* random token for acknowledgement matching */
    bool req_ack = false; /* keep track of whether PULL_DATA was acknowledged or not */

    /* JSON parsing variables */
    JSON_Value *root_val = NULL;
    JSON_Object *txpk_obj = NULL;
    JSON_Value *val = NULL; /* needed to detect the absence of some fields */
    const char *str; /* pointer to sub-strings in the JSON data */
    short x0, x1;

    /* auto-quit variable */
    uint32_t autoquit_cnt = 0; /* count the number of PULL_DATA sent since the latest PULL_ACK */

    /* Just In Time downlink */
    uint32_t current_concentrator_time;

    enum jit_error_e jit_result = JIT_ERROR_OK;
    enum jit_pkt_type_e downlink_type;

    /* set downstream socket RX timeout */
    i = setsockopt(sock_down, SOL_SOCKET, SO_RCVTIMEO, (void *)&pull_timeout, sizeof pull_timeout);
    if (i != 0) {
        LOG_I("ERROR: [down] setsockopt returned %s(%d)\n", strerror(errno),i);
        exit(EXIT_FAILURE);
    }

    /* pre-fill the pull request buffer with fixed fields */
    buff_req[0] = PROTOCOL_VERSION;
    buff_req[3] = PKT_PULL_DATA;
    *(uint32_t *)(buff_req + 4) = net_mac_h;
    *(uint32_t *)(buff_req + 8) = net_mac_l;

    /* JIT queue initialization */
    jit_queue_init(&jit_queue);

    while (!exit_sig && !quit_sig) {

        /* auto-quit if the threshold is crossed */
        if ((autoquit_threshold > 0) && (autoquit_cnt >= autoquit_threshold)) {
            exit_sig = true;
            LOG_I("INFO: [down] the last %u PULL_DATA were not ACKed, exiting application\n", autoquit_threshold);
            break;
        }

        /* generate random token for request */
        token_h = (uint8_t)rand(); /* random token */
        token_l = (uint8_t)rand(); /* random token */
        buff_req[1] = token_h;
        buff_req[2] = token_l;

        /* send PULL request and record time */
        LOG_I_len = send(sock_down, (void *)buff_req, sizeof buff_req, 0);
        clock_gettime(CLOCK_REALTIME, &send_time);//clock_gettime(CLOCK_MONOTONIC, &send_time);
        pthread_mutex_lock(&mx_meas_dw);
        meas_dw_pull_sent += 1;
        pthread_mutex_unlock(&mx_meas_dw);
        req_ack = false;
        autoquit_cnt++;
        
        LOG_D("[udp][up] PULL request(%d):%d @%d", meas_dw_pull_sent, LOG_I_len, send_time);

        /* listen to packets and process them until a new PULL request must be sent */
        recv_time = send_time;
        LOG_D("try to receive a datagram:%d", recv_time);
        while ((int)difftimespec(recv_time, send_time) < lgwsc_conf.keepalive_time) {

            /* try to receive a datagram */
            LOG_I_len = recv(sock_down, (void *)buff_down, (sizeof buff_down)-1, 0);
            clock_gettime(CLOCK_REALTIME, &recv_time); // clock_gettime(CLOCK_MONOTONIC, &recv_time);
            
            /* if no network message was received, got back to listening sock_down socket */
            if (LOG_I_len == -1) {
                //LOG_I("WARNING: [down] recv returned %s\n", strerror(errno)); /* too verbose */
                continue;
            }

            /* if the datagram does not respect protocol, just ignore it */
            if ((LOG_I_len < 4) || (buff_down[0] != PROTOCOL_VERSION) || ((buff_down[3] != PKT_PULL_RESP) && (buff_down[3] != PKT_PULL_ACK))) {
                LOG_I("WARNING: [down] ignoring invalid packet len=%d, protocol_version=%d, id=%d\n",
                        LOG_I_len, buff_down[0], buff_down[3]);
                continue;
            }

            /* if the datagram is an ACK, check token */
            if (buff_down[3] == PKT_PULL_ACK) {
                if ((buff_down[1] == token_h) && (buff_down[2] == token_l)) {
                    if (req_ack) {
                        LOG_I("INFO: [down] duplicate ACK received :)\n");
                    } else { /* if that packet was not already acknowledged */
                        req_ack = true;
                        autoquit_cnt = 0;
                        pthread_mutex_lock(&mx_meas_dw);
                        meas_dw_ack_rcv += 1;
                        pthread_mutex_unlock(&mx_meas_dw);
                        LOG_I("INFO: [down] PULL_ACK received in %i ms\n", (int)(1000 * difftimespec(recv_time, send_time)));
                    }
                } else { /* out-of-sync token */
                    LOG_I("INFO: [down] received out-of-sync ACK\n");
                }
                continue;
            }

            /* the datagram is a PULL_RESP */
            LOG_D(" ### [DOWNSTREAM] ### ");
            buff_down[LOG_I_len] = 0; /* add string terminator, just to be safe */
            LOG_I("INFO: [down] PULL_RESP received  - token[%d:%d] :)\n", buff_down[1], buff_down[2]); /* very verbose */
            LOG_D("\nJSON down: %s\n", (char *)(buff_down + 4)); /* printf DEBUG: display JSON payload */

            /* initialize TX struct and try to parse JSON */
            memset(&txpkt, 0, sizeof txpkt);
            root_val = json_parse_string_with_comments((const char *)(buff_down + 4)); /* JSON offset */
            if (root_val == NULL) {
                LOG_I("WARNING: [down] invalid JSON, TX aborted\n");
                continue;
            }

            /* look for JSON sub-object 'txpk' */
            txpk_obj = json_object_get_object(json_value_get_object(root_val), "txpk");
            if (txpk_obj == NULL) {
                LOG_I("WARNING: [down] no \"txpk\" object in JSON, TX aborted\n");
                json_value_free(root_val);
                continue;
            }

            /* Parse "immediate" tag, or target timestamp, or UTC time to be converted by GPS (mandatory) */
            i = json_object_get_boolean(txpk_obj,"imme"); /* can be 1 if true, 0 if false, or -1 if not a JSON boolean */
            if (i == 1) {
                /* TX procedure: send immediately */
                sent_immediate = true;
                downlink_type = JIT_PKT_TYPE_DOWNLINK_CLASS_C;
                LOG_I("INFO: [down] a packet will be sent in \"immediate\" mode\n");
            } else {
                sent_immediate = false;
                val = json_object_get_value(txpk_obj,"tmst");
                if (val != NULL) {
                    /* TX procedure: send on timestamp value */
                    txpkt.count_us = (uint32_t)json_value_get_number(val);
                    
                    // json.txpkt.count_us 
                    LOG_D("INFO: [udp][down] json.txpkt.count_us:%u us\n",txpkt.count_us);

                    /* Concentrator timestamp is given, we consider it is a Class A downlink */
                    downlink_type = JIT_PKT_TYPE_DOWNLINK_CLASS_A;
                } 
            }

            /* Parse "No CRC" flag (optional field) */
            val = json_object_get_value(txpk_obj,"ncrc");
            if (val != NULL) {
                txpkt.no_crc = (bool)json_value_get_boolean(val);
            }

            /* parse target frequency (mandatory) */
            val = json_object_get_value(txpk_obj,"freq");
            if (val == NULL) {
                LOG_I("WARNING: [down] no mandatory \"txpk.freq\" object in JSON, TX aborted\n");
                json_value_free(root_val);
                continue;
            }
            txpkt.params.freq_hz = (uint32_t)((double)(1.0e6) * json_value_get_number(val));

            /* parse RF chain used for TX (mandatory) */
            val = json_object_get_value(txpk_obj,"rfch");
            if (val == NULL) {
                LOG_I("WARNING: [down] no mandatory \"txpk.rfch\" object in JSON, TX aborted\n");
                json_value_free(root_val);
                continue;
            }
            txpkt.rf_chain = (uint8_t)json_value_get_number(val);

            /* parse TX power (optional field) */
            val = json_object_get_value(txpk_obj,"powe");
            if (val != NULL) {
                txpkt.params.rf_power = (int8_t)json_value_get_number(val) - antenna_gain;
            }

            /* Parse modulation (mandatory) */
            str = json_object_get_string(txpk_obj, "modu");
            if (str == NULL) {
                LOG_I("WARNING: [down] no mandatory \"txpk.modu\" object in JSON, TX aborted\n");
                json_value_free(root_val);
                continue;
            }
            if (strcmp(str, "LORA") == 0) {
                /* Lora modulation */
                txpkt.params.modulation = MOD_LORA;

                /* Parse Lora spreading-factor and modulation bandwidth (mandatory) */
                str = json_object_get_string(txpk_obj, "datr");
                if (str == NULL) {
                    LOG_I("WARNING: [down] no mandatory \"txpk.datr\" object in JSON, TX aborted\n");
                    json_value_free(root_val);
                    continue;
                }
                i = sscanf(str, "SF%2hdBW%3hd", &x0, &x1);
                if (i != 2) {
                    LOG_I("WARNING: [down] format error in \"txpk.datr\", TX aborted\n");
                    json_value_free(root_val);
                    continue;
                }
                switch (x0) {
                    case  7: txpkt.params.datarate = DR_LORA_SF7;  break;
                    case  8: txpkt.params.datarate = DR_LORA_SF8;  break;
                    case  9: txpkt.params.datarate = DR_LORA_SF9;  break;
                    case 10: txpkt.params.datarate = DR_LORA_SF10; break;
                    case 11: txpkt.params.datarate = DR_LORA_SF11; break;
                    case 12: txpkt.params.datarate = DR_LORA_SF12; break;
                    default:
                        LOG_I("WARNING: [down] format error in \"txpk.datr\", invalid SF, TX aborted\n");
                        json_value_free(root_val);
                        continue;
                }
                switch (x1) {
                    case 125: txpkt.params.bandwidth = BW_125KHZ; break;
                    case 250: txpkt.params.bandwidth = BW_250KHZ; break;
                    case 500: txpkt.params.bandwidth = BW_500KHZ; break;
                    default:
                        LOG_I("WARNING: [down] format error in \"txpk.datr\", invalid BW, TX aborted\n");
                        json_value_free(root_val);
                        continue;
                }

                /* Parse ECC coding rate (optional field) */
                str = json_object_get_string(txpk_obj, "codr");
                if (str == NULL) {
                    LOG_I("WARNING: [down] no mandatory \"txpk.codr\" object in json, TX aborted\n");
                    json_value_free(root_val);
                    continue;
                }
                if      (strcmp(str, "4/5") == 0) txpkt.params.coderate  = CR_LORA_4_5;
                else if (strcmp(str, "4/6") == 0) txpkt.params.coderate  = CR_LORA_4_6;
                else if (strcmp(str, "2/3") == 0) txpkt.params.coderate  = CR_LORA_4_6;
                else if (strcmp(str, "4/7") == 0) txpkt.params.coderate  = CR_LORA_4_7;
                else if (strcmp(str, "4/8") == 0) txpkt.params.coderate  = CR_LORA_4_8;
                else if (strcmp(str, "1/2") == 0) txpkt.params.coderate  = CR_LORA_4_8;
                else {
                    LOG_I("WARNING: [down] format error in \"txpk.codr\", TX aborted\n");
                    json_value_free(root_val);
                    continue;
                }

                /* Parse signal polarity switch (optional field) */
                val = json_object_get_value(txpk_obj,"ipol");
                if (val != NULL) {
                    txpkt.params.invert_pol = (bool)json_value_get_boolean(val);
                }

                /* parse Lora preamble length (optional field, optimum min value enforced) */
                val = json_object_get_value(txpk_obj,"prea");
                if (val != NULL) {
                    i = (int)json_value_get_number(val);
                    if (i >= MIN_LORA_PREAMB) {
                        txpkt.params.preamble = (uint16_t)i;
                    } else {
                        txpkt.params.preamble = (uint16_t)MIN_LORA_PREAMB;
                    }
                } else {
                    txpkt.params.preamble = (uint16_t)STD_LORA_PREAMB;
                }

            } else if (strcmp(str, "FSK") == 0) {
                /* FSK modulation */
                txpkt.params.modulation = MOD_FSK;

                /* parse FSK bitrate (mandatory) */
                val = json_object_get_value(txpk_obj,"datr");
                if (val == NULL) {
                    LOG_I("WARNING: [down] no mandatory \"txpk.datr\" object in JSON, TX aborted\n");
                    json_value_free(root_val);
                    continue;
                }
                txpkt.params.datarate = (uint32_t)(json_value_get_number(val));

                /* parse frequency deviation (mandatory) */
                val = json_object_get_value(txpk_obj,"fdev");
                if (val == NULL) {
                    LOG_I("WARNING: [down] no mandatory \"txpk.fdev\" object in JSON, TX aborted\n");
                    json_value_free(root_val);
                    continue;
                }
                txpkt.f_dev = (uint8_t)(json_value_get_number(val) / 1000.0); /* JSON value in Hz, txpkt.f_dev in kHz */

                /* parse FSK preamble length (optional field, optimum min value enforced) */
                val = json_object_get_value(txpk_obj,"prea");
                if (val != NULL) {
                    i = (int)json_value_get_number(val);
                    if (i >= MIN_FSK_PREAMB) {
                        txpkt.params.preamble = (uint16_t)i;
                    } else {
                        txpkt.params.preamble = (uint16_t)MIN_FSK_PREAMB;
                    }
                } else {
                    txpkt.params.preamble = (uint16_t)STD_FSK_PREAMB;
                }

            } else {
                LOG_I("WARNING: [down] invalid modulation in \"txpk.modu\", TX aborted\n");
                json_value_free(root_val);
                continue;
            }

            /* Parse payload length (mandatory) */
            val = json_object_get_value(txpk_obj,"size");
            if (val == NULL) {
                LOG_I("WARNING: [down] no mandatory \"txpk.size\" object in JSON, TX aborted\n");
                json_value_free(root_val);
                continue;
            }
            txpkt.size = (uint16_t)json_value_get_number(val);

            /* Parse payload data (mandatory) */
            str = json_object_get_string(txpk_obj, "data");
            if (str == NULL) {
                LOG_I("WARNING: [down] no mandatory \"txpk.data\" object in JSON, TX aborted\n");
                json_value_free(root_val);
                continue;
            }
            i = b64_to_bin(str, strlen(str), txpkt.payload, sizeof txpkt.payload);
            if (i != txpkt.size) {
                LOG_I("WARNING: [down] mismatch between .size and .data size once converter to binary\n");
            }

            /* free the JSON parse tree from memory */
            json_value_free(root_val);

            /* select TX mode */
            if (sent_immediate) {
                txpkt.tx_mode = IMMEDIATE;
            } else {
                txpkt.tx_mode = TIMESTAMPED;
            }

            /* record measurement data */
            pthread_mutex_lock(&mx_meas_dw);
            meas_dw_dgram_rcv += 1; /* count only datagrams with no JSON errors */
            meas_dw_network_byte += LOG_I_len; /* meas_dw_network_byte */
            meas_dw_payload_byte += txpkt.size;
            pthread_mutex_unlock(&mx_meas_dw);

            /* check TX parameter before trying to queue packet */
            jit_result = JIT_ERROR_OK;
            if ((txpkt.params.freq_hz < tx_freq_min[txpkt.rf_chain]) || (txpkt.params.freq_hz > tx_freq_max[txpkt.rf_chain])) {
                jit_result = JIT_ERROR_TX_FREQ;
                LOG_I("ERROR: Packet REJECTED, unsupported frequency - %u (min:%u,max:%u)\n", txpkt.params.freq_hz, tx_freq_min[txpkt.rf_chain], tx_freq_max[txpkt.rf_chain]);
            }
            if (jit_result == JIT_ERROR_OK) {
                for (i=0; i<txlut.size; i++) {
                    if (txlut.lut[i].rf_power == txpkt.params.rf_power) {
                        /* this RF power is supported, we can continue */
                        break;
                    }
                }
                if (i == txlut.size) {
                    /* this RF power is not supported */
                    jit_result = JIT_ERROR_TX_POWER;
                    LOG_I("ERROR: Packet REJECTED, unsupported RF power for TX - %d\n", txpkt.params.rf_power);
                }
            }

            /* insert packet to be sent into JIT queue */
            if (jit_result == JIT_ERROR_OK) {   
                get_concentrator_time(&current_concentrator_time);
                jit_result = jit_enqueue(&jit_queue, current_concentrator_time, &txpkt, downlink_type);             
                
                if (jit_result != JIT_ERROR_OK) {
                    LOG_I("ERROR: Packet REJECTED (jit error=%d),%d\n", jit_result,TimerGetCurrentTime());//printf
                }
                else
                {
                    LOG_D("JIT OK: Packet Enquene @:%u",current_concentrator_time);
                }
                pthread_mutex_lock(&mx_meas_dw);
                meas_nb_tx_requested += 1;
                pthread_mutex_unlock(&mx_meas_dw);
            }

            /* Send acknoledge datagram to server */
            send_tx_ack(buff_down[1], buff_down[2], jit_result);
        }
    }
    LOG_I("\nINFO: End of downstream thread\n");
}

void print_tx_status(uint8_t tx_status) {
    switch (tx_status) {
        case TX_OFF:
            LOG_I("INFO: [jit] lgwsc_status returned TX_OFF\n");
            break;
        case TX_FREE:
            LOG_I("INFO: [jit] lgwsc_status returned TX_FREE\n");
            break;
        case TX_EMITTING:
            LOG_I("INFO: [jit] lgwsc_status returned TX_EMITTING\n");
            break;
        case TX_SCHEDULED:
            LOG_I("INFO: [jit] lgwsc_status returned TX_SCHEDULED\n");
            break;
        default:
            LOG_I("INFO: [jit] lgwsc_status returned UNKNOWN (%d)\n", tx_status);
            break;
    }
}


/* -------------------------------------------------------------------------- */
/* --- THREAD 3: CHECKING PACKETS TO BE SENT FROM JIT QUEUE AND SEND THEM --- */

void thread_jit(void) {
    int result = LGW_HAL_SUCCESS;
    int pkt_index = -1;
    
    uint32_t current_concentrator_time;

    enum jit_error_e jit_result;
    enum jit_pkt_type_e pkt_type;
    uint8_t tx_status;
    const uint8_t jit_result_string[JIT_ERROR_INVALID+1][21] = 
    {
        "JIT OK",
        "JIT TOO LATE",
        "JIT TOO EARLY",
        "JIT FULL",
        "JIT EMPTY",
        "JIT COLLISION PACKET",
        "JIT COLLISION BEACON",
        "JIT TX FREQ N.S.",
        "JIT TX POWER N.S.",
        "JIT GPS UNLOCKED",
        "JIT INVALID",
    };
    while (!exit_sig && !quit_sig) {
        wait_ms(10);

        /* transfer data and metadata to the concentrator, and schedule TX */   
        get_concentrator_time(&current_concentrator_time);
        jit_result = jit_peek(&jit_queue, current_concentrator_time, &pkt_index);    
        
        if (jit_result != JIT_ERROR_EMPTY ) 
        {
            if (pkt_index > -1)
            {                
                LOG_D("jit_peek: jit_result[%d] = %s,pkt_index = %d\n",jit_result, jit_result_string[jit_result], pkt_index);
            }
        }
        
        if (jit_result == JIT_ERROR_OK) {
            if (pkt_index > -1) {
                jit_result = jit_dequeue(&jit_queue, pkt_index, &lgwsc_tx_pkt, &pkt_type);
                    
                LOG_D("jit_dequeue: jit_result[%d] = %s,pkt_index = %d\n",jit_result, jit_result_string[jit_result], pkt_index);
                          
                if (jit_result == JIT_ERROR_OK) {

                    /* check if concentrator is free for sending new packet */
                    pthread_mutex_lock(&mx_concent); /* may have to wait for a fetch to finish */
                    result = lgwsc_status(TX_STATUS, &tx_status);
                    pthread_mutex_unlock(&mx_concent); /* free concentrator ASAP */
                    if (result == LGW_HAL_ERROR) {
                        LOG_I("WARNING: [jit] lgwsc_status failed\n");
                    } else {
                        if (tx_status == TX_EMITTING) {
                            LOG_I("ERROR: concentrator is currently emitting\n");
                            print_tx_status(tx_status);
                            continue;
                        } else if (tx_status == TX_SCHEDULED) {
                            LOG_I("WARNING: a downlink was already scheduled, overwritting it...\n");
                            print_tx_status(tx_status);
                        } else {
                            /* Nothing to do */
                        }
                    }

                    /* send packet to concentrator */
                    pthread_mutex_lock(&mx_concent); /* may have to wait for a fetch to finish */
                    result = lgwsc_send(lgwsc_tx_pkt);
                    pthread_mutex_unlock(&mx_concent); /* free concentrator ASAP */
                    if (result == LGW_HAL_ERROR) {
                        pthread_mutex_lock(&mx_meas_dw);
                        meas_nb_tx_fail += 1;
                        pthread_mutex_unlock(&mx_meas_dw);
                        LOG_I("WARNING: [jit] lgwsc_send failed\n");
                        continue;
                    } else {
                        pthread_mutex_lock(&mx_meas_dw);
                        meas_nb_tx_ok += 1;
                        pthread_mutex_unlock(&mx_meas_dw);
                    }
                } else {
                    LOG_I("ERROR: jit_dequeue failed with %d\n", jit_result);
                }
            }
        } else if (jit_result == JIT_ERROR_EMPTY) {
            /* Do nothing, it can happen */
        } else {
            LOG_I("ERROR: jit_peek failed with %d\n", jit_result);
        }
    }
}

// lgw
// ms 
uint32_t lgw_time_on_air(struct lgw_pkt_tx_s *packet) 
{
    uint8_t dr = packet->params.datarate;
    uint8_t dr_index = 0;
    
    // SF7~SF12
    for(dr_index = 1; dr_index < 7; dr_index++)
    {
        if( dr & 0x01 )
        {
            break;
        }
        else
        {
            dr = dr >> 1;
        }
    }

    return Radio.TimeOnAir(lora_radio_mod_table[packet->params.modulation >> 5],lora_radio_bw_table[3-packet->params.bandwidth],
                                lora_radio_dr_table[dr_index],packet->params.coderate, packet->params.preamble,  
                                LORA_FIX_LENGTH_PAYLOAD_OFF, packet->size,true);
}

int lgwsc_status(uint8_t select, uint8_t *code) {
    
    if( select == TX_STATUS )
    {
        *code = lora_radio_tx_status;
    }
    return LGW_HAL_SUCCESS;
}

int lgwsc_send(struct lgw_pkt_tx_s pkt_data) 
{  
    if (pkt_data.tx_mode == IMMEDIATE)
    {
        lgwsc_send_packet();
    }
    else if(pkt_data.tx_mode == TIMESTAMPED)
    {
        if(pkt_data.size)
        {
            lora_radio_tx_status = TX_SCHEDULED;
            
#ifdef PKG_USING_MULTI_RTIMER
            TimerSetValueFromNow( &lgwsc_tx_on_timestamp_timer, pkt_data.count_us / 1000 - 15 ); //ms, todo time calib depend on datarate
            TimerStart( &lgwsc_tx_on_timestamp_timer );      
                
            LOG_D("lgwsc_send trigger: current_tmst:%u, pkt.count_us:%u, df:%d, pkt.size:%d\n",(TimerGetCurrentTime() * 1000), lgwsc_tx_pkt.count_us, lgwsc_tx_pkt.count_us - (TimerGetCurrentTime() * 1000), lgwsc_tx_pkt.size);
#else
            TimerSetValueFromNow( &lgwsc_tx_on_timestamp_timer, pkt_data.count_us / 1000 );// ms 
            TimerStart( &lgwsc_tx_on_timestamp_timer );      

            LOG_D("lgwsc_send trigger: current_tmst:%u, pkt.count_us:%u, df:%d, pkt.size:%d\n",(TimerGetCurrentTime() / RT_TICK_PER_SECOND * 1000000), lgwsc_tx_pkt.count_us, lgwsc_tx_pkt.count_us - (TimerGetCurrentTime() / RT_TICK_PER_SECOND * 1000000), lgwsc_tx_pkt.size);
#endif
     }
    }
    
    return LGW_HAL_SUCCESS;
}


/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

void lgwsc_send_packet(void)
{ 
    register rt_base_t level;
    /* disable interrupt */
    level = rt_hw_interrupt_disable();
    
    lora_radio_tx_status = TX_EMITTING;
  
    Radio.SetChannel( lgwsc_tx_pkt.params.freq_hz );
    Radio.SetTxConfig( lora_radio_mod_table[lgwsc_tx_pkt.params.modulation >> 5], lgwsc_tx_pkt.params.rf_power, 0, lora_radio_bw_table[3-lgwsc_tx_pkt.params.bandwidth],
                               lora_radio_dr_table[lora_radio_dr_index], lgwsc_tx_pkt.params.coderate,
                               lgwsc_tx_pkt.params.preamble, LORA_FIX_LENGTH_PAYLOAD_OFF,
                               true, 0, 0, lgwsc_tx_pkt.params.invert_pol, 3000 );

    Radio.Send(lgwsc_tx_pkt.payload,lgwsc_tx_pkt.size);
    
    /* enable interrupt */
    rt_hw_interrupt_enable(level);   
}

static void lgwsc_tx_on_timestamp_irq( void /** context*/ )
{
    TimerStop( &lgwsc_tx_on_timestamp_timer);
    
    lgwsc_send_packet();
#ifdef PKG_USING_MULTI_RTIMER
    uint32_t time = TimerGetCurrentTime() * 1000;// us
#else
    uint32_t time = TimerGetCurrentTime() / RT_TICK_PER_SECOND * 1000; // us
#endif
    LOG_D("lgw start to transmit now,The Current Timestamp - RxDone Timestamp:%d us", (time - Radio.GetRxdoneTimestamp()));
}


int lgwsc_start(void) 
{
    TimerInit( &lgwsc_tx_on_timestamp_timer, lgwsc_tx_on_timestamp_irq );
 
    rt_event_init(&loragw_radio_event, "radio-event", RT_IPC_FLAG_FIFO);
    
    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );
       
    /* radio parameter config */
    lgwsc_radio_config();

    // false - setup private syncword for p2p
    Radio.SetPublicNetwork( true );
        
    Radio.Rx( 0 );
    
    LOG_D("lgw on rx\n");
    
    return 0;
}

int get_concentrator_time(uint32_t *concent_time_us)
{
#if defined PKG_USING_MULTI_RTIMER
    uint32_t temp_time = TimerGetCurrentTime(); // ms
    *concent_time_us = temp_time * 1000; // ms -> us
#else
    uint32_t temp_time = TimerGetCurrentTime(); // ticks
    *concent_time_us = temp_time / RT_TICK_PER_SECOND * 1000000; // us 
#endif

    return 0;
}

void lgwsc_radio_config(void)
{
    lgwsc_tx_pkt.params = lgwsc_conf.txc;
    lgwsc_rx_pkt.params = lgwsc_conf.rxc;
    
    Radio.SetChannel( lgwsc_rx_pkt.params.freq_hz );

    if( lgwsc_rx_pkt.params.modulation == MOD_LORA )
    {
        // [0: 125 kHz,
        //  1: 250 kHz,
        //  2: 500 kHz,
        //  3: Reserved]
        
        uint8_t dr = lgwsc_rx_pkt.params.datarate;
        
        // convert lgwsc_rx_pkt.params.datarate to dr table 
        for(lora_radio_dr_index = 1; lora_radio_dr_index < 7; lora_radio_dr_index++)
        {
            if( dr & 0x01 )
            {
                break;
            }
            else
            {
                dr = dr >> 1;
            }
        }
 
        Radio.SetTxConfig( lora_radio_mod_table[lgwsc_tx_pkt.params.modulation >> 5], lgwsc_tx_pkt.params.rf_power, 0, lora_radio_bw_table[3-lgwsc_tx_pkt.params.bandwidth],
                                       lora_radio_dr_table[lora_radio_dr_index], lgwsc_tx_pkt.params.coderate,
                                       lgwsc_tx_pkt.params.preamble, LORA_FIX_LENGTH_PAYLOAD_OFF,
                                       true, 0, 0, lgwsc_tx_pkt.params.invert_pol, 3000 );

        Radio.SetRxConfig( lora_radio_mod_table[lgwsc_rx_pkt.params.modulation >> 5], lora_radio_bw_table[3-lgwsc_rx_pkt.params.bandwidth], lora_radio_dr_table[lora_radio_dr_index],
                                       lgwsc_rx_pkt.params.coderate, 0, LORA_PREAMBLE_LENGTH,
                                       LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_OFF,
                                       0, true, 0, 0, LORA_IQ_INVERSION_OFF, true );
    }

    
    LOG_I("RX Parameters: Channel=%d, SF%d, BW%s, CR 4/%d",lgwsc_rx_pkt.params.freq_hz,lora_radio_dr_table[lora_radio_dr_index],bw_table_string[lora_radio_bw_table[3-lgwsc_rx_pkt.params.bandwidth]],lgwsc_rx_pkt.params.coderate+4);
    LOG_I("TX Parameters: Channel=%d, SF%d, BW%s, CR 4/%d",lgwsc_tx_pkt.params.freq_hz,lora_radio_dr_table[lora_radio_dr_index],bw_table_string[lora_radio_bw_table[3-lgwsc_tx_pkt.params.bandwidth]],lgwsc_tx_pkt.params.coderate+4);

}
void wait_ms(unsigned long a) 
{
    rt_thread_mdelay(a);
}

static int lgwsc_cfg_save(void *buff, int len)
{
#ifdef PKG_USING_EASYFLASH
    EfErrCode result = EF_NO_ERR;

    /* set and store the wlan config lengths to Env */
    result = ef_set_env_blob("lgwsc_cfg_len", &len, sizeof(len));

    /* set and store the wlan config information to Env */
    result = ef_set_env_blob("lgwsc_cfg_info", buff, len);
    if (result == EF_NO_ERR)
    {
        LOG_I("LGWSC Radio Parameters(%d) Save Successed!\n", len);
    }
    else
    {
        LOG_I("LGWSC Radio Parameters Save Fail,%d!\n", result);
    }
#endif
    return result;
}

static int lgwsc_read_cfg_info(void *buff, int len)
{
#ifdef PKG_USING_EASYFLASH
    size_t saved_len;

    ef_get_env_blob("lgwsc_cfg_info", buff, len, &saved_len);
    if (saved_len == 0)
    {
        return 0;
    }

    return len;
#endif
}

static int lgwsc_get_cfg_len(void)
{
#ifdef PKG_USING_EASYFLASH
    int len;
    size_t saved_len;

    ef_get_env_blob("lgwsc_cfg_len", &len, sizeof(len), &saved_len);
    if (saved_len == 0)
    {
        return 0;
    }

    return len;
#endif
}


/**
 * @brief  get_hex_byte
 * @param **string
 * @returns
 */
uint8_t get_hex_byte(char **hex_string)
{
    char temp[3] = { 0 };
    char *string_ptr = *hex_string;
    uint8_t value;

    for(uint8_t i = 0; i < 2; i++)
    {
        temp[i] = *string_ptr++;
    }
    *hex_string = string_ptr;

    value = strtol(temp,0,16);

    return value;
}

// lgwsc-shell command for finish\msh
#define CMD_LGW_SC_SERVER_ADDR_INDEX    1
#define CMD_LGW_SC_GWEUI_INDEX          2
#define CMD_LGW_SC_MODE_INDEX           3 // lgw mode
#define CMD_LGW_SC_SAVE_INDEX           4 // cfg save
const char* lgwsc_info[] = 
{   
    [CMD_LGW_SC_SERVER_ADDR_INDEX]      = "lgwsc server <addr>,<port_up>,<port_down>            - setup server address and port",
    [CMD_LGW_SC_GWEUI_INDEX]            = "lgwsc gweui <eui>                                    - setup gatway eui",
    [CMD_LGW_SC_MODE_INDEX]             = "lgwsc mode <para>                                    - lgwsc mode setup",
    [CMD_LGW_SC_SAVE_INDEX]             = "lgwsc save                                           - lgwsc save parameters to flash",
    
};

/* LGWSC shell function */
static int lgwsc(int argc, char *argv[])
{
    size_t i = 0;
    
    if (argc < 2) 
    {   // parameter error 
        rt_kprintf("Usage:\n");
        for (i = 0; i < sizeof(lgwsc_info) / sizeof(char*); i++) {
            LOG_I("%s", lgwsc_info[i]);
        }
        LOG_I("\n");
    } 
    else 
    {
        const char *cmd = argv[1];
		
        if (!rt_strcmp("gweui", cmd))
        {
            if( argc > 2 )
            {
                for(uint8_t i = 0;i < 8;i++)
                {
                    lgwsc_conf.gweui[i] = get_hex_byte(&argv[2]);
                }
           }

            rt_kprintf("LGWSC EUI=%02X%02X%02X%02X%02X%02X%02X%02X\n", lgwsc_conf.gweui[0],lgwsc_conf.gweui[1],lgwsc_conf.gweui[2],lgwsc_conf.gweui[3],lgwsc_conf.gweui[4],lgwsc_conf.gweui[5],lgwsc_conf.gweui[6],lgwsc_conf.gweui[7]);
 
        }
        else if (!rt_strcmp(cmd, "server")) 
        {   
            // lorawan lgwsc_conf.serv_addr
            // eg: "192.168.1.101"
            if( argc > 2 )
            {
                //lgwsc_conf.serv_addr = argv[2];
                rt_memcpy(lgwsc_conf.serv_addr,argv[2],rt_strlen(argv[2]));
                // server port up
                if( argc > 3 )
                {
                    //eg: "1700"
                    rt_memcpy(lgwsc_conf.serv_port_up,argv[3],rt_strlen(argv[3]));
                    
                     // server port down
                    if( argc > 4 )
                    {
                        rt_memcpy(lgwsc_conf.serv_port_down,argv[4],rt_strlen(argv[4]));
                    }
                }
            }
            
            LOG_D("lgwsc_conf.serv_addr:%s",lgwsc_conf.serv_addr);
            LOG_D("port up:%s",lgwsc_conf.serv_port_up);
            LOG_D("port dn:%s",lgwsc_conf.serv_port_down);
        }
       else if (!rt_strcmp(cmd, "rx"))
       {
           uint8_t datarate;
           if( argc > 2 )
           {
                lgwsc_conf.rxc.freq_hz  = atol(argv[2]);
                if( argc > 3 )
                {
                    // SF: （5，6），7,8,9,10,11,12
                    uint8_t datarate = atoi(argv[3]);
                    
                    for(uint8_t i = 1; i < 8;i++)
                    {
                        if( lora_radio_dr_table[i] == datarate )
                        {
                            // DR_LORA_SF7     0x02
                            lgwsc_conf.rxc.datarate = 0x01 << (i - 1);
                            break;
                        }
                    }
                    
                }
           }

           rt_kprintf("LGWSC Rx Paramters:\r\n");
           rt_kprintf("Frequence:%d\r\n",lgwsc_conf.rxc.freq_hz, datarate);
        }
        else if(!rt_strcmp(cmd, "mode"))
        {
            /* auto start*/
            if(argc > 2)
            {
                /* 0 - start by hand
                 * 1 - auto start after power up
                 * */
                lgwsc_conf.cfg_flag = atoi(argv[2]);
            }
            rt_kprintf("LGWSC Mode:%d\r\n", lgwsc_conf.cfg_flag);
        }
        else if(!rt_strcmp(cmd,"save"))
        {
            /* save lgw configuration to flash */
            lgwsc_cfg_save(&lgwsc_conf.cfg_flag, LGWSC_CONFIG_SIZE);
        }        
    }
    return 0;
}
MSH_CMD_EXPORT(lgwsc, LGWSC Shell Cmd);
/* --- EOF ------------------------------------------------------------------ */
