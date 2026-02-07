#ifndef _NRFLite_h_
#define _NRFLite_h_

#include <avr/io.h>
#include <stdint.h>

#include "nRF24L01.h"

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

#define _BITRATE BITRATE2MBPS

#if _BITRATE == BITRATE2MBPS
#define TRANSMISSION_RETRY_WAIT_MICROS 600  // 100 more than the retry delay
#elif _BITRATE == BITRATE1MBPS
#define TRANSMISSION_RETRY_WAIT_MICROS 600  // 100 more than the retry delay
#else
#define TRANSMISSION_RETRY_WAIT_MICROS 1600  // 100 more than the retry delay
#endif

class NRFLite {
 public:
  // Constructors
  // Optionally pass in an Arduino Serial or SoftwareSerial object for use throughout the library when debugging.
  // Use the debug and debugln DEFINES in NRFLite.cpp to use the serial object.
  NRFLite(PORT_t* cePinPort, uint8_t cePinBm, PORT_t* csnPinPort, uint8_t csnPinBm) : _cePinPort(cePinPort), _cePinBm(cePinBm), _csnPinPort(csnPinPort), _csnPinBm(csnPinBm) {}

  enum Bitrates {
    BITRATE2MBPS,
    BITRATE1MBPS,
    BITRATE250KBPS
  };
  enum SendType {
    REQUIRE_ACK,
    NO_ACK
  };

  const static uint8_t MAX_NRF_CHANNEL = 125;  // Maximum channel number.

  // Methods for receivers and transmitters.
  // init       = Turns the radio on and puts it into receiving mode.  Returns 0 if it cannot communicate with the radio.
  //              Channel can be 0-125 and sets the exact frequency of the radio between 2400 - 2525 MHz.
  // initTwoPin = Same as init but with multiplexed MOSI/MISO and CE/CSN/SCK pins (only works on AVR architectures).
  //              Follow the 2-pin hookup schematic on https://github.com/dparson55/NRFLite
  // readData   = Loads a received data packet or acknowledgment data packet into the specified data parameter.
  // powerDown  = Power down the radio.  Turn the radio back on by calling one of the 'hasData' or 'send' methods.
  // printDetails = Prints many of the radio registers.  Requires a serial object in the constructor, e.g. NRFLite _radio(Serial);
  // scanChannel  = Returns 0-measurementCount to indicate the strength of any existing signal on a channel.  Radio communication will
  //                work best on channels with no existing signals, meaning a 0 is returned.
  uint8_t init(uint8_t* address, uint8_t channel = 100);
  void readData(void* data);
  void powerDown();
  uint8_t scanChannel(uint8_t channel, uint8_t measurementCount = 255);

  // Methods for transmitters.
  // send = Sends a data packet and waits for success or failure.  The default REQUIRE_ACK sendType causes the radio
  //        to attempt sending the packet up to 16 times.  If successful a 1 is returned.  Optionally the NO_ACK sendType
  //        can be used to transmit the packet a single time without any acknowledgement.
  // hasAckData = Checks to see if an ACK data packet was received and returns its length.
  uint8_t send(uint8_t* address, void* data, uint8_t length, SendType sendType = REQUIRE_ACK);
  uint8_t hasAckData();

  // Methods for receivers.
  // hasData     = Checks to see if a data packet has been received and returns its length.  It also puts the radio in RX mode
  //               if it was previously in TX mode.
  // addAckData  = Enqueues an acknowledgment data packet for sending back to a transmitter.  Whenever the transmitter sends the
  //               next data packet, it will get this ACK packet back in the response.  The radio will store up to 3 ACK packets
  //               and will not enqueue more if full, so you can clear any stale packets using the 'removeExistingAcks' parameter.
  // discardData = Removes the current received data packet.  Useful if a packet of an unexpected size is received.
  void addAckData(void* data, uint8_t length, uint8_t removeExistingAcks = 0);
  void discardData(uint8_t unexpectedDataLength);
  uint8_t startRx();
  void whatHappened(uint8_t& txOk, uint8_t& txFail, uint8_t& rxReady);

 private:
  const static uint8_t CONFIG_REG_SETTINGS_FOR_RX_MODE = _BV(PWR_UP) | _BV(PRIM_RX) | _BV(EN_CRC);
  const static uint32_t NRF_SPICLOCK = 4000000;  // Speed to use for SPI communication with the transceiver.

  // Delay used to discharge the radio's CSN pin when operating in 2-pin mode.
  // Determined by measuring time to discharge CSN on a 1MHz ATtiny using 0.1uF capacitor and 1K resistor.
  const static uint16_t CSN_DISCHARGE_MICROS = 500;

  const static uint8_t OFF_TO_POWERDOWN_MILLIS = 100;      // Vcc > 1.9V power on reset time.
  const static uint8_t POWERDOWN_TO_RXTX_MODE_MILLIS = 5;  // 4500uS to Standby + 130uS to RX or TX mode, so 5ms is enough.
  const static uint8_t CE_TRANSMISSION_MICROS = 10;        // Time to initiate data transmission.

  enum SpiTransferType {
    READ_OPERATION,
    WRITE_OPERATION
  };

  PORT_t* _cePinPort;
  uint8_t _cePinBm;
  PORT_t* _csnPinPort;
  uint8_t _csnPinBm;
  volatile uint8_t _resetInterruptFlags;
  uint8_t _usingSeparateCeAndCsnPins;
  uint8_t* _lastToRadioAddress;

  uint8_t getPipeOfFirstRxPacket();
  uint8_t getRxPacketLength();
  uint8_t initRadio(uint8_t* address, uint8_t channel);
  void prepForTx(uint8_t* address, SendType sendType);
  uint8_t waitForTxToComplete();
  uint8_t readRegister(uint8_t regName);
  void readRegister(uint8_t regName, void* data, uint8_t length);
  void writeRegister(uint8_t regName, uint8_t data);
  void writeRegister(uint8_t regName, void* data, uint8_t length);
  uint8_t spiTransferByte(uint8_t data);
  void spiTransfer(SpiTransferType transferType, uint8_t regName, void* data, uint8_t length);
};

#endif
