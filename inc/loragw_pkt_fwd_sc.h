


#ifndef _LORAGW_PKT_SC_H
#define _LORAGW_PKT_SC_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */

//#include "config.h"     /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

#define IS_LORA_BW(bw)          ((bw == BW_125KHZ) || (bw == BW_250KHZ) || (bw == BW_500KHZ))
#define IS_LORA_STD_DR(dr)      ((dr == DR_LORA_SF7) || (dr == DR_LORA_SF8) || (dr == DR_LORA_SF9) || (dr == DR_LORA_SF10) || (dr == DR_LORA_SF11) || (dr == DR_LORA_SF12))
#define IS_LORA_MULTI_DR(dr)    ((dr & ~DR_LORA_MULTI) == 0) /* ones outside of DR_LORA_MULTI bitmask -> not a combination of LoRa datarates */
#define IS_LORA_CR(cr)          ((cr == CR_LORA_4_5) || (cr == CR_LORA_4_6) || (cr == CR_LORA_4_7) || (cr == CR_LORA_4_8))

#define IS_FSK_BW(bw)           ((bw >= 1) && (bw <= 7))
#define IS_FSK_DR(dr)           ((dr >= DR_FSK_MIN) && (dr <= DR_FSK_MAX))

#define IS_TX_MODE(mode)        ((mode == IMMEDIATE) || (mode == TIMESTAMPED) || (mode == ON_GPS))

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

/* return status code */
#define LGW_HAL_SUCCESS     0
#define LGW_HAL_ERROR       -1
#define LGW_LBT_ISSUE       1

/* radio-specific parameters */
#define LGW_XTAL_FREQU      32000000            /* frequency of the RF reference oscillator */
#define LGW_RF_CHAIN_NB     2                   /* number of RF chains */
#define LGW_RF_RX_BANDWIDTH {1000000, 1000000}  /* bandwidth of the radios */

/* type of if_chain + modem */
#define IF_UNDEFINED        0
#define IF_LORA_STD         0x10    /* if + standard single-SF LoRa modem */
#define IF_LORA_MULTI       0x11    /* if + LoRa receiver with multi-SF capability */
#define IF_FSK_STD          0x20    /* if + standard FSK modem */

/* concentrator chipset-specific parameters */
/* to use array parameters, declare a local const and use 'if_chain' as index */
#define LGW_IF_CHAIN_NB     10    /* number of IF+modem RX chains */
#define LGW_PKT_FIFO_SIZE   16    /* depth of the RX packet FIFO */
#define LGW_DATABUFF_SIZE   1024    /* size in bytes of the RX data buffer (contains payload & metadata) */
#define LGW_REF_BW          125000    /* typical bandwidth of data channel */
#define LGW_MULTI_NB        8    /* number of LoRa 'multi SF' chains */
#define LGW_IFMODEM_CONFIG {\
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_STD, \
        IF_FSK_STD } /* configuration of available IF chains and modems on the hardware */

/* values available for the 'modulation' parameters */
/* NOTE: arbitrary values */
#define MOD_UNDEFINED   0
#define MOD_LORA        0x10
#define MOD_FSK         0x20

/* values available for the 'bandwidth' parameters (LoRa & FSK) */
/* NOTE: directly encode FSK RX bandwidth, do not change */
#define BW_UNDEFINED    0
#define BW_500KHZ       0x01
#define BW_250KHZ       0x02
#define BW_125KHZ       0x03
#define BW_62K5HZ       0x04
#define BW_31K2HZ       0x05
#define BW_15K6HZ       0x06
#define BW_7K8HZ        0x07

/* values available for the 'datarate' parameters */
/* NOTE: LoRa values used directly to code SF bitmask in 'multi' modem, do not change */
#define DR_UNDEFINED    0
#define DR_LORA_SF7     0x02
#define DR_LORA_SF8     0x04
#define DR_LORA_SF9     0x08
#define DR_LORA_SF10    0x10
#define DR_LORA_SF11    0x20
#define DR_LORA_SF12    0x40
#define DR_LORA_MULTI   0x7E
/* NOTE: for FSK directly use baudrate between 500 bauds and 250 kbauds */
#define DR_FSK_MIN      500
#define DR_FSK_MAX      250000

/* values available for the 'coderate' parameters (LoRa only) */
/* NOTE: arbitrary values */
#define CR_UNDEFINED    0
#define CR_LORA_4_5     0x01
#define CR_LORA_4_6     0x02
#define CR_LORA_4_7     0x03
#define CR_LORA_4_8     0x04

/* values available for the 'status' parameter */
/* NOTE: values according to hardware specification */
#define STAT_UNDEFINED  0x00
#define STAT_NO_CRC     0x01
#define STAT_CRC_BAD    0x11
#define STAT_CRC_OK     0x10

/* values available for the 'tx_mode' parameter */
#define IMMEDIATE       0
#define TIMESTAMPED     1
//#define ON_GPS          2
//#define ON_EVENT      3
//#define GPS_DELAYED   4
//#define EVENT_DELAYED 5

/* values available for 'select' in the status function */
#define TX_STATUS       1
#define RX_STATUS       2

/* status code for TX_STATUS */
/* NOTE: arbitrary values */
#define TX_STATUS_UNKNOWN   0
#define TX_OFF              1    /* TX modem disabled, it will ignore commands */
#define TX_FREE             2    /* TX modem is free, ready to receive a command */
#define TX_SCHEDULED        3    /* TX modem is loaded, ready to send the packet after an event and/or delay */
#define TX_EMITTING         4    /* TX modem is emitting */

/* status code for RX_STATUS */
/* NOTE: arbitrary values */
#define RX_STATUS_UNKNOWN   0
#define RX_OFF              1    /* RX modem is disabled, it will ignore commands  */
#define RX_ON               2    /* RX modem is receiving */
#define RX_SUSPENDED        3    /* RX is suspended while a TX is ongoing */

/* Maximum size of Tx gain LUT */
#define TX_GAIN_LUT_SIZE_MAX 16

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */
//ALIGN(RT_ALIGN_SIZE_BYTE)
#pragma pack(push,1)

/**
@struct lgw_conf_rxif_s
@brief Configuration structure for an IF chain
*/
struct lgw_conf_rxif_s {
    bool        enable;         /*!> enable or disable that IF chain */
    uint8_t     rf_chain;       /*!> to which RF chain is that IF chain associated */
    int32_t     freq_hz;        /*!> center frequ of the IF chain, relative to RF chain frequency */
    uint8_t     bandwidth;      /*!> RX bandwidth, 0 for default */
    uint32_t    datarate;       /*!> RX datarate, 0 for default */
    uint8_t     sync_word_size; /*!> size of FSK sync word (number of bytes, 0 for default) */
    uint64_t    sync_word;      /*!> FSK sync word (ALIGN RIGHT, eg. 0xC194C1) */
};


struct lgw_pkt_rx_conf_s {
    uint32_t    freq_hz;        /*!> central frequency of the IF chain */
    uint8_t     modulation;     /*!> modulation used by the packet */
    uint8_t     bandwidth;      /*!> modulation bandwidth (LoRa only) */
    uint32_t    datarate;       /*!> RX datarate of the packet (SF for LoRa) */
    uint8_t     coderate;       /*!> error-correcting code of the packet (LoRa only) */
};

/**
@struct lgw_pkt_rx_s
@brief Structure containing the metadata of a packet that was received and a pointer to the payload
*/
struct lgw_pkt_rx_s {
    struct lgw_pkt_rx_conf_s params;
    
    uint8_t     if_chain;       /*!> by which IF chain was packet received */
    uint8_t     status;         /*!> status of the received packet */
    uint32_t    count_us;       /*!> internal concentrator counter for timestamping, 1 microsecond resolution */
    uint8_t     rf_chain;       /*!> through which RF chain the packet was received */

    float       rssi;           /*!> average packet RSSI in dB */
    float       snr;            /*!> average packet SNR, in dB (LoRa only) */
    float       snr_min;        /*!> minimum packet SNR, in dB (LoRa only) */
    float       snr_max;        /*!> maximum packet SNR, in dB (LoRa only) */
    uint16_t    crc;            /*!> CRC that was received in the payload */
    uint16_t    size;           /*!> payload size in bytes */
    uint8_t     payload[256];   /*!> buffer containing the payload */
};

struct lgw_pkt_tx_conf_s {
    uint32_t    freq_hz;        /*!> center frequency of TX */
    int8_t      rf_power;       /*!> TX power, in dBm */
    uint8_t     modulation;     /*!> modulation to use for the packet */
    uint8_t     bandwidth;      /*!> modulation bandwidth (LoRa only) */
    uint32_t    datarate;       /*!> TX datarate (baudrate for FSK, SF for LoRa) */
    uint8_t     coderate;       /*!> error-correcting code of the packet (LoRa only) */
    uint16_t    preamble;       /*!> set the preamble length, 0 for default */
    bool        invert_pol;     /*!> invert signal polarity, for orthogonal downlinks (LoRa only) */
};

/**
@struct lgw_pkt_tx_s
@brief Structure containing the configuration of a packet to send and a pointer to the payload
*/
struct lgw_pkt_tx_s {
    struct lgw_pkt_tx_conf_s params;
    
    uint8_t     tx_mode;        /*!> select on what event/time the TX is triggered */
    uint32_t    count_us;       /*!> timestamp or delay in microseconds for TX trigger */
    uint8_t     rf_chain;       /*!> through which RF chain will the packet be sent */
    uint8_t     f_dev;          /*!> frequency deviation, in kHz (FSK only) */
    bool        no_crc;         /*!> if true, do not send a CRC in the packet */
    bool        no_header;      /*!> if true, enable implicit header mode (LoRa), fixed length (FSK) */
    uint16_t    size;           /*!> payload size in bytes */
    uint8_t     payload[256];   /*!> buffer containing the payload */
};


#pragma pack(pop)

/**
@struct lgw_tx_gain_s
@brief Structure containing all gains of Tx chain
*/
struct lgw_tx_gain_s {
    int8_t  rf_power;   /*!> measured TX power at the board connector, in dBm */
};

/**
@struct lgw_tx_gain_lut_s
@brief Structure defining the Tx gain LUT
*/
struct lgw_tx_gain_lut_s {
    struct lgw_tx_gain_s    lut[TX_GAIN_LUT_SIZE_MAX];  /*!> Array of Tx gain struct */
    uint8_t                 size;                       /*!> Number of LUT indexes */
};

struct lgwsc_conf_params_s 
{
    uint16_t cfg_flag;

    struct lgw_pkt_tx_conf_s txc;
    struct lgw_pkt_rx_conf_s rxc;
    
    uint8_t gweui[8];

    char serv_addr[64]; /* address of the server (host name or IPv4/IPv6) */
    char serv_port_up[8]; /* server port for upstream traffic */
    char serv_port_down[8]; /* server port for downstream traffic */
    int keepalive_time;
    
    /* statistics collection configuration variables */
    unsigned stat_interval; /* time interval (in sec) at which statistics are collected and displayed */

};
#define LGWSC_CONFIG_SIZE sizeof(struct lgwsc_conf_params_s)

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
@brief Connect to the LoRa concentrator, reset it and configure it according to previously set parameters
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else
*/
int lgwsc_start(void);

/**
@brief Schedule a packet to be send immediately or after a delay depending on tx_mode
@param pkt_data structure containing the data and metadata for the packet to send
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else

/!\ When sending a packet, there is a 1.5 ms delay for the analog circuitry to start and be stable (TX_START_DELAY).
In 'timestamp' mode, this is transparent: the modem is started 1.5ms before the user-set timestamp value is reached, the preamble of the packet start right when the internal timestamp counter reach target value.
In 'immediate' mode, the packet is emitted as soon as possible: transferring the packet (and its parameters) from the host to the concentrator takes some time, then there is the TX_START_DELAY, then the packet is emitted.
In 'triggered' mode (aka PPS/GPS mode), the packet, typically a beacon, is emitted 1.5ms after a rising edge of the trigger signal. Because there is no way to anticipate the triggering event and start the analog circuitry beforehand, that delay must be taken into account in the protocol.
*/
int lgwsc_send(struct lgw_pkt_tx_s pkt_data);

/**
@brief Give the the status of different part of the LoRa concentrator
@param select is used to select what status we want to know
@param code is used to return the status code
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else
*/
int lgwsc_status(uint8_t select, uint8_t *code);

/**
@brief Return time on air of given packet, in milliseconds
@param packet is a pointer to the packet structure
@return the packet time on air in milliseconds
*/
uint32_t lgw_time_on_air(struct lgw_pkt_tx_s *packet);

#endif

/* --- EOF ------------------------------------------------------------------ */
