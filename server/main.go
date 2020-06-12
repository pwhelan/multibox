package main

import (
	"encoding/binary"
	//"fmt"
	"log"
	"time"

	//nrf24 "github.com/galaktor/gorf24"
	"periph.io/x/periph/conn/physic"
	"periph.io/x/periph/conn/spi"
	"periph.io/x/periph/conn/spi/spireg"
	"periph.io/x/periph/host"
)

/*********************************
// Memory Map
#define NRF_CONFIG  0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define NRF_STATUS  0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

// Bit Mnemonics
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5      5
#define DPL_P4      4
#define DPL_P3      3
#define DPL_P2      2
#define DPL_P1      1
#define DPL_P0      0
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

// Instruction Mnemonics
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define RF24_NOP      0xFF

// Non-P omissions
#define LNA_HCURR   0

// P model memory Map
#define RPD                  0x09
#define W_TX_PAYLOAD_NO_ACK  0xB0

// P model bit Mnemonics
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2
******************************/

const (
	// #define EN_ACK_PAY  1
	BitEnAckPay =          1
	// #define ARD         4
	BitArd uint8 = 0x04
	// #define PWR_UP      1
	BitPwrUp = 0x01
	// # define PRIM_RX    0
	BitPrimRx = 0x00
	// #define ARC         0
	BitArc = 0x00
	// #define TX_DS       5
	BitTxDs = 5
	// #define RX_DR       6
	BitRxDr = 0x06
	// #define MAX_RT      4
	BitMaxRt = 0x04
	// #define EN_CRC      3
	BitEnCrc = 0x03
	// #define CRCO        2
	BitCrcO = 0x02
)


const (
	NrfConfig uint8 = 0x00
	EnAA = 0x01
	EnRxAddr = 0x02
	SetupRetr = 0x04
	RfCh = 0x05
	RfSetup = 0x06
	NrfStatus = 0x07
	DynPd = 0x1c
	Feature = 0x1d
	RxAddrP0 = 0x0a
	RxAddrP1 = 0x0b
	RxAddrP2 = 0x0c
	RxAddrP3 = 0x0d
	RxAddrP4 = 0x0e
	RxAddrP5 = 0x0f
	TxAddr   = 0x10
	RxPwP0   = 0x11
	RxPwP1   = 0x12
	RxPwP2   = 0x13
	RxPwP3   = 0x14
	RxPwP4   = 0x15
	RxPwP5   = 0x16
	ErxP0    = 0x00
	ErxP1    = 0x01
	ErxP2    = 0x02
	ErxP3    = 0x03
	ErxP4    = 0x04
	ErxP5    = 0x05
)

const (
	// #define R_REGISTER    0x00
	ReadRegister uint8 = 0x00
	// #define W_REGISTER    0x20
	WriteRegister = 0x20
	// #define REGISTER_MASK 0x1F
	RegisterMask = 0x1f
)

const (
	//#define RF_DR_LOW   5
	RfDrLow = 5
	//#define RF_DR_HIGH  3
	RfDrHigh = 3
)

/***
typedef enum {
    RF24_1MBPS = 0,
    RF24_2MBPS,
    RF24_250KBPS
} rf24_datarate_e;
***/
const (
	Rf1Mbps uint8 = 0
	Rf2Mbps = 1
	Rf250Kbps = 2
)

/***
typedef enum {
    RF24_PA_MIN = 0,
    RF24_PA_LOW,
    RF24_PA_HIGH,
    RF24_PA_MAX,
    RF24_PA_ERROR
} rf24_pa_dbm_e;
***/

const (
	RfPaMin uint8 = 0
	RfPaLow = 1
	RfPaHigh = 2
	RfPaMax = 3
	RfPaError =4
)

/****
typedef enum {
    RF24_CRC_DISABLED = 0,
    RF24_CRC_8,
    RF24_CRC_16
} rf24_crclength_e;
***/
const (
	RfCrcDisabled uint8 = 0
	RfCrc8 = 1
	RfCrc16 = 2
)

func read_register(c spi.Conn, reg uint8) (uint8, error) {
	return 0, nil
}

func write_register(c spi.Conn, reg, val uint8) error {
	tx := []byte{ WriteRegister | ( RegisterMask & reg ), val }
	rx := make([]byte, len(tx))
	if err := c.Tx(tx, rx); err != nil {
		return err
	}
	return nil
}

/********
uint8_t RF24::write_register(uint8_t reg, const uint8_t* buf, uint8_t len)
{
    uint8_t status;

    #if defined(RF24_LINUX)
    beginTransaction();
    uint8_t * prx = spi_rxbuff;
    uint8_t * ptx = spi_txbuff;
    uint8_t size = len + 1; // Add register value to transmit buffer

    *ptx++ = ( W_REGISTER | ( REGISTER_MASK & reg ) );
    while ( len-- )
      *ptx++ = *buf++;

    _SPI.transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);
    status = *prx; // status is 1st byte of receive buffer
    endTransaction();
    #else // !defined(RF24_LINUX)

    beginTransaction();
    status = _SPI.transfer(W_REGISTER | (REGISTER_MASK & reg));
    while (len--) {
        _SPI.transfer(*buf++);
    }
    endTransaction();
    #endif // !defined(RF24_LINUX)

    return status;
}
*********/
func write_register_buf(c spi.Conn, reg uint8, buf []uint8) uint8 {
	txbuff := make([]uint8, len(buf)+1)
	txbuff[0] = WriteRegister | (reg & RegisterMask)
	rxbuff := make([]uint8, 32+1)
	
	
	for i, b := range buf {
		txbuff[i+1] = b
	}
	c.Tx(txbuff, rxbuff)
	return rxbuff[0]
}

func _BV(val uint8) uint8 {
	return 1 << val
}

func set_data_rate(c spi.Conn, speed uint8) bool {
	setup, _ := read_register(c, RfSetup)
	setup &= (_BV(RfDrLow) | _BV(RfDrHigh))
	
	//txDelay := 250
	if speed == Rf250Kbps {
		setup |= _BV(RfDrLow)
		//txDelay = 450
	} else if speed == Rf2Mbps {
		setup |= _BV(RfDrHigh)
		//txDelay = 190
	}
	write_register(c, RfSetup, setup)
	if reg, _ := read_register(c, RfSetup); reg == setup {
		return true
	}
	return false
}

/*
	const uint8_t max_channel = 125;
	write_register(RF_CH, rf24_min(channel, max_channel))
*/
func set_channel(c spi.Conn, channel uint8) {
	if (channel > 125) {
		channel = 125
	}
	write_register(c, RfCh, channel)
}

/*
rf24_datarate_e RF24::getDataRate(void)
{
    rf24_datarate_e result;
    uint8_t dr = read_register(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    // switch uses RAM (evil!)
    // Order matters in our case below
    if (dr == _BV(RF_DR_LOW)) {
        // '10' = 250KBPS
        result = RF24_250KBPS;
    } else if (dr == _BV(RF_DR_HIGH)) {
        // '01' = 2MBPS
        result = RF24_2MBPS;
    } else {
        // '00' = 1MBPS
        result = RF24_1MBPS;
    }
    return result;
}
*/

func set_pa_level(c spi.Conn, level uint8) {
	setup, _ := read_register(c, RfSetup)
	setup &= 0xf8
	
	if level > 3 {
		level = (RfPaMax << 1) + 1
	} else {
		level = (level << 1) + 1
	}
	write_register(c, RfSetup, setup | level)
}

func set_auto_ack(c spi.Conn, autoAck bool) {
	if autoAck == true {
		write_register(c, EnAA, 0x3f)
	} else {
		write_register(c, EnAA, 0)
	}
}

func set_retries(c spi.Conn, delay, count uint8) {
	write_register(c, SetupRetr, (delay & 0xf) << BitArd | (count & 0xf) << BitArc)
}

/****

void RF24::setCRCLength(rf24_crclength_e length)
{
    uint8_t config = read_register(NRF_CONFIG) & ~(_BV(CRCO) | _BV(EN_CRC));

    // switch uses RAM (evil!)
    if (length == RF24_CRC_DISABLED) {
        // Do nothing, we turned it off above.
    } else if (length == RF24_CRC_8) {
        config |= _BV(EN_CRC);
    } else {
        config |= _BV(EN_CRC);
        config |= _BV(CRCO);
    }
    write_register(NRF_CONFIG, config);
}
***/
func set_crc_len(c spi.Conn, len uint8) {
	setup, _ := read_register(c, NrfConfig)
	setup &= ^(_BV(BitCrcO) | _BV(BitEnCrc))
	
	if len == RfCrcDisabled {
	} else if len == RfCrc8 {
		setup |= _BV(BitEnCrc)
	} else {
		setup |= _BV(BitEnCrc)
		setup |= _BV(BitCrcO)
	}
	write_register(c, NrfConfig, setup)
}

/***
void RF24::openWritingPipe(uint64_t value)
{
    // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
    // expects it LSB first too, so we're good.

    write_register(RX_ADDR_P0, reinterpret_cast<uint8_t*>(&value), addr_width);
    write_register(TX_ADDR, reinterpret_cast<uint8_t*>(&value), addr_width);


    //const uint8_t max_payload_size = 32;
    //write_register(RX_PW_P0,rf24_min(payload_size,max_payload_size));
    write_register(RX_PW_P0, payload_size);
}
***/
func open_writing_pipe(c spi.Conn, value uint64, payload_size uint8) {
	buf := make([]uint8, 8)
	binary.PutUvarint(buf, value)
	write_register_buf(c, RxAddrP0, buf)
	write_register_buf(c, TxAddr, buf)
	
	if payload_size > 32 {
		payload_size = 32
	}
	write_register(c, RxPwP0, payload_size)
}

/***
static const uint8_t child_pipe[] PROGMEM =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const uint8_t child_payload_size[] PROGMEM =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};
static const uint8_t child_pipe_enable[] PROGMEM =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

void RF24::openReadingPipe(uint8_t child, uint64_t address)
{
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0) {
        memcpy(pipe0_reading_address, &address, addr_width);
    }

    if (child <= 6) {
        // For pipes 2-5, only write the LSB
        if (child < 2) {
            write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), addr_width);
        } else {
            write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), 1);
        }

        write_register(pgm_read_byte(&child_payload_size[child]), payload_size);

        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        write_register(EN_RXADDR, read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));
    }
}
***/
var child_pipe = []uint8 {
	RxAddrP0,
	RxAddrP1,
	RxAddrP2,
	RxAddrP3,
	RxAddrP4,
	RxAddrP5,
}

var child_payload_size = []uint8 {
	RxPwP0,
	RxPwP1,
	RxPwP2,
	RxPwP3,
	RxPwP4,
	RxPwP5,
}

var child_pipe_enable = []uint8 {
	ErxP0,
	ErxP1,
	ErxP2,
	ErxP3,
	ErxP4,
	ErxP5,
}

func open_reading_pipe(c spi.Conn, child uint8, address uint64) {
	buf := make([]uint8, 8)
	binary.PutUvarint(buf, address)
	
	if child <= 6 {
		if child < 2 {
			write_register_buf(c, child_pipe[child], buf)
		} else {
			write_register(c, child_pipe[child], buf[0])
		}
		
		write_register(c, child_payload_size[child], 32 /* payload_size */)
		
		enaddr, _ := read_register(c, EnRxAddr)
		write_register(c, EnRxAddr, enaddr | _BV(child_pipe_enable[child]))
	}
}

func toggle_features(c spi.Conn) {
	//beginTransaction();
	//_SPI.transfer(ACTIVATE);
	//_SPI.transfer(0x73);
	//endTransaction();
}

/***
//Power up now. Radio will not power down unless instructed by MCU for config changes etc.
void RF24::powerUp(void)
{
    uint8_t cfg = read_register(NRF_CONFIG);

    // if not powered up then power up and wait for the radio to initialize
    if (!(cfg & _BV(PWR_UP))) {
        write_register(NRF_CONFIG, cfg | _BV(PWR_UP));

        // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
        // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
        // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
        delay(5);
    }
}
***/
func power_up(c spi.Conn) {
	setup, _ := read_register(c, NrfConfig)
	if (setup & _BV(BitPwrUp)) == 0 {
		write_register(c, NrfConfig, setup | _BV(BitPwrUp))
		time.Sleep(5 * time.Millisecond)
	}
}

/***
void RF24::startListening(void)
{
    #if !defined(RF24_TINY) && !defined(LITTLEWIRE)
    powerUp();
    #endif
    write_register(NRF_CONFIG, read_register(NRF_CONFIG) | _BV(PRIM_RX));
    write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
    ce(HIGH);
    // Restore the pipe0 adddress, if exists
    if (pipe0_reading_address[0] > 0) {
        write_register(RX_ADDR_P0, pipe0_reading_address, addr_width);
    } else {
        closeReadingPipe(0);
    }

    // Flush buffers
    //flush_rx();
    if (read_register(FEATURE) & _BV(EN_ACK_PAY)) {
        flush_tx();
    }

    // Go!
    //delayMicroseconds(100);
}
***/
func start_listening(c spi.Conn) {
	power_up(c)
	setup, _ := read_register(c, NrfConfig)
	write_register(c, NrfConfig, setup | _BV(BitPrimRx))
	write_register(c, NrfStatus, _BV(BitRxDr) | _BV(BitTxDs) | _BV(BitMaxRt))
}

/***
void RF24::stopListening(void)
{
    ce(LOW);

    delayMicroseconds(txDelay);

    if (read_register(FEATURE) & _BV(EN_ACK_PAY)) {
        delayMicroseconds(txDelay); //200
        flush_tx();
    }
    //flush_rx();
    write_register(NRF_CONFIG, (read_register(NRF_CONFIG)) & ~_BV(PRIM_RX));

    #if defined(RF24_TINY) || defined(LITTLEWIRE)
    // for 3 pins solution TX mode is only left with additonal powerDown/powerUp cycle
    if (ce_pin == csn_pin) {
      powerDown();
      powerUp();
    }
    #endif
    write_register(EN_RXADDR, read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[0]))); // Enable RX on pipe0

    //delayMicroseconds(100);

}
***/
func stop_listening(c spi.Conn) {
	// delay(txDelay)
	feature, _ := read_register(c, Feature)
	if (feature & _BV(BitEnAckPay)) != 0 {
		//delay(txDeleay)
		//flush_tx()
	}
	//flush_rx()
	setup, _ := read_register(c, NrfConfig)
	write_register(c, NrfConfig, setup & ^_BV(BitPrimRx))
	
	enrxaddr, _ := read_register(c, EnRxAddr)
	write_register(c, EnRxAddr, enrxaddr | _BV(child_pipe_enable[0]))
}

/***
void RF24::powerDown(void)
{
    ce(LOW); // Guarantee CE is low on powerDown
    write_register(NRF_CONFIG, read_register(NRF_CONFIG) & ~_BV(PWR_UP));
}
*****/


func main() {
	/*
	var pipe uint64 = 0xE0E0F1F1E0
	rf := nrf24.New("/dev/spidev0.0", 8000000, 25)
	defer rf.Delete()

	rf.Begin()
	rf.SetRetries(15, 15)
	rf.SetAutoAck(true)
	rf.OpenReadingPipe(1, pipe)
	rf.StartListening()
	rf.PrintDetails()

	for {
		if rf.Available() {
			// rf. GetPayloadSize()
			data, := r.Read(5)
			fmt.Printf("data=%v\n", data)
			payload := binary.LittleEndian.Uint32(data)
			fmt.Printf("Received %v\n", payload)
		}
	}
	*/
	if _, err := host.Init(); err != nil {
		log.Fatal(err)
	}

	// Use spireg SPI port registry to find the first available SPI bus.
	p, err := spireg.Open("")
	if err != nil {
		log.Fatal(err)
	}
	defer p.Close()
	
	c, err := p.Connect(physic.MegaHertz, spi.Mode3, 8)
	if err != nil {
		log.Fatal(err)
	}
	// Reset NRF_CONFIG and enable 16-bit CRC.
	write_register(c, NrfConfig, 0x0C)
	//setRetries(5, 15)
	// void RF24::setRetries(uint8_t delay, uint8_t count)
	write_register(c, SetupRetr, (5 & 0xf) << BitArd | (15 & 0xf) << BitArc)
	set_data_rate(c, Rf250Kbps)
	//setup, _ := read_register(c, RfSetup)
	set_data_rate(c, Rf1Mbps)
	toggle_features(c)
	write_register(c, Feature, 0)
	write_register(c, DynPd, 0)
	//dynamic_payloads_enabled = false
	write_register(c, NrfStatus, _BV(BitRxDr) | _BV(BitTxDs) | _BV(BitMaxRt))
	set_channel(c, 76)
	//flush_rx()
	//flush_tx()
	power_up(c)
	setup, _ := read_register(c, NrfConfig)
	write_register(c, NrfConfig, (setup & ^_BV(BitPrimRx)))
	//return (setup != 0 && setup != 0xff)
}
