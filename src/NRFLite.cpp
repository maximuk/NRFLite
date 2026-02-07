#include "NRFLite.h"

#include <avr/interrupt.h>
#include <stddef.h>
#include <util/delay.h>

////////////////////
// Public methods //
////////////////////

uint8_t NRFLite::init(uint8_t* address, uint8_t channel) {
  _usingSeparateCeAndCsnPins = _cePinPort != _csnPinPort || _cePinBm != _csnPinBm;

  _csnPinPort->OUTSET |= _csnPinBm;  // digitalWrite(_csnPin, HIGH);

  // With the microcontroller's pins setup, we can now initialize the radio.
  uint8_t success = initRadio(address, channel);
  return success;
}

void NRFLite::addAckData(void* data, uint8_t length, uint8_t removeExistingAcks) {
  if (removeExistingAcks) {
    spiTransfer(WRITE_OPERATION, FLUSH_TX, NULL, 0);  // Clear the TX buffer.
  }

  // Add the packet to the TX buffer for pipe 1, the pipe used to receive packets from radios that
  // send us data.  When we receive the next transmission from a radio, we'll provide this ACK data in the
  // auto-acknowledgment packet that goes back.
  spiTransfer(WRITE_OPERATION, (W_ACK_PAYLOAD | 1), data, length);
}

void NRFLite::discardData(uint8_t unexpectedDataLength) {
  // Read data from the RX buffer.
  uint8_t data[unexpectedDataLength];
  spiTransfer(READ_OPERATION, R_RX_PAYLOAD, &data, unexpectedDataLength);

  // Clear data received flag.
  uint8_t statusReg = readRegister(STATUS_NRF);
  if (statusReg & _BV(RX_DR)) {
    writeRegister(STATUS_NRF, statusReg | _BV(RX_DR));
  }
}

uint8_t NRFLite::hasAckData() {
  // If we have a pipe 0 packet sitting at the top of the RX buffer, we have auto-acknowledgment data.
  // We receive ACK data from other radios using the pipe 0 address.
  if (getPipeOfFirstRxPacket() == 0) {
    return getRxPacketLength();  // Return the length of the data packet in the RX buffer.
  } else {
    return 0;
  }
}

void NRFLite::readData(void* data) {
  // Determine length of data in the RX buffer and read it.
  uint8_t dataLength;
  spiTransfer(READ_OPERATION, R_RX_PL_WID, &dataLength, 1);
  spiTransfer(READ_OPERATION, R_RX_PAYLOAD, data, dataLength);

  // Clear data received flag.
  uint8_t statusReg = readRegister(STATUS_NRF);
  if (statusReg & _BV(RX_DR)) {
    writeRegister(STATUS_NRF, statusReg | _BV(RX_DR));
  }
}

uint8_t NRFLite::startRx() {
  waitForTxToComplete();

  // Put radio into Standby-I mode in order to transition into RX mode.
  _cePinPort->OUTCLR |= _cePinBm;  // digitalWrite(_cePin, LOW);

  // Configure the radio for receiving.
  writeRegister(CONFIG, CONFIG_REG_SETTINGS_FOR_RX_MODE);

  // Put radio into RX mode.
  _cePinPort->OUTSET |= _cePinBm;  // digitalWrite(_cePin, HIGH);

  // Wait for the transition into RX mode.
  _delay_ms(POWERDOWN_TO_RXTX_MODE_MILLIS);

  uint8_t inRxMode = readRegister(CONFIG) == CONFIG_REG_SETTINGS_FOR_RX_MODE;
  return inRxMode;
}

uint8_t NRFLite::send(uint8_t* address, void* data, uint8_t length, SendType sendType) {
  prepForTx(address, sendType);

  // Clear any previously asserted TX success or max retries flags.
  writeRegister(STATUS_NRF, _BV(TX_DS) | _BV(MAX_RT));

  // Add data to the TX buffer, with or without an ACK request.
  if (sendType == NO_ACK) {
    spiTransfer(WRITE_OPERATION, W_TX_PAYLOAD_NO_ACK, data, length);
  } else {
    spiTransfer(WRITE_OPERATION, W_TX_PAYLOAD, data, length);
  }

  uint8_t result = waitForTxToComplete();
  return result;
}

void NRFLite::whatHappened(uint8_t& txOk, uint8_t& txFail, uint8_t& rxReady) {
  uint8_t statusReg = readRegister(STATUS_NRF);

  txOk = statusReg & _BV(TX_DS);
  txFail = statusReg & _BV(MAX_RT);
  rxReady = statusReg & _BV(RX_DR);

  // When we need to see interrupt flags, we disable the logic here which clears them.
  // Programs that have an interrupt handler for the radio's IRQ pin will use 'whatHappened'
  // and if we don't disable this logic, it's not possible for us to check these flags.
  if (_resetInterruptFlags) {
    writeRegister(STATUS_NRF, _BV(TX_DS) | _BV(MAX_RT) | _BV(RX_DR));
  }
}

void NRFLite::powerDown() {
  // If we have separate CE and CSN pins, we can gracefully transition into Power Down mode by first entering Standby-I mode.
  if (_usingSeparateCeAndCsnPins) {
    _cePinPort->OUTCLR |= _cePinBm;  // digitalWrite(_cePin, LOW);
  }

  // Turn off the radio.
  writeRegister(CONFIG, readRegister(CONFIG) & ~_BV(PWR_UP));
}

uint8_t NRFLite::scanChannel(uint8_t channel, uint8_t measurementCount) {
  uint8_t strength = 0;

  // Put radio into Standby-I mode.
  _cePinPort->OUTCLR |= _cePinBm;  // digitalWrite(_cePin, LOW);

  // Set the channel.
  writeRegister(RF_CH, channel);

  // Take a bunch of measurements.
  do {
    // Put the radio into RX mode and wait a little time for a signal to be received.
    _cePinPort->OUTSET |= _cePinBm;  // digitalWrite(_cePin, HIGH);
    _delay_us(400);
    _cePinPort->OUTCLR |= _cePinBm;  // digitalWrite(_cePin, LOW);

    uint8_t signalWasReceived = readRegister(CD);
    if (signalWasReceived) {
      strength++;
    }
  } while (measurementCount--);

  return strength;
}

/////////////////////
// Private methods //
/////////////////////

uint8_t NRFLite::getPipeOfFirstRxPacket() {
  // The pipe number is bits 3, 2, and 1.  So B1110 masks them and we shift right by 1 to get the pipe number.
  // 000-101 = Data Pipe Number
  //     110 = Not Used
  //     111 = RX FIFO Empty
  return (readRegister(STATUS_NRF) & 0b1110) >> 1;
}

uint8_t NRFLite::getRxPacketLength() {
  // Read the length of the first data packet sitting in the RX buffer.
  uint8_t dataLength;
  spiTransfer(READ_OPERATION, R_RX_PL_WID, &dataLength, 1);

  // Verify the data length is valid. This private method is only called if getPipeOfFirstRxPacket indicates a packet exists,
  // so the datalength should never be 0. Likewise the datalength should never be > 32 since that's the largest possible packet the radio supports.
  if (dataLength > 32 || !dataLength) {
    spiTransfer(WRITE_OPERATION, FLUSH_RX, NULL, 0);  // Clear invalid data in the RX buffer.
    writeRegister(STATUS_NRF, readRegister(STATUS_NRF) | _BV(TX_DS) | _BV(MAX_RT) | _BV(RX_DR));
    return 0;
  } else {
    return dataLength;
  }
}

uint8_t NRFLite::initRadio(uint8_t* address, uint8_t channel) {
  _resetInterruptFlags = 1;

  _delay_ms(OFF_TO_POWERDOWN_MILLIS);

  // Valid channel range is 2400 - 2525 MHz, in 1 MHz increments.
  if (channel > MAX_NRF_CHANNEL) {
    channel = MAX_NRF_CHANNEL;
  }
  writeRegister(RF_CH, channel);

  // Transmission speed, retry times, and output power setup.
  // For 2 Mbps or 1 Mbps operation, a 500 uS retry time is necessary to support the max ACK packet size.
  // For 250 Kbps operation, a 1500 uS retry time is necessary.
  if (_BITRATE == BITRATE2MBPS) {
    writeRegister(RF_SETUP, 0b00001110);    // 2 Mbps, 0 dBm output power
    writeRegister(SETUP_RETR, 0b00011111);  // 0001 =  500 uS between retries, 1111 = 15 retries
  } else if (_BITRATE == BITRATE1MBPS) {
    writeRegister(RF_SETUP, 0b00000110);    // 1 Mbps, 0 dBm output power
    writeRegister(SETUP_RETR, 0b00011111);  // 0001 =  500 uS between retries, 1111 = 15 retries
  } else {
    writeRegister(RF_SETUP, 0b00100110);    // 250 Kbps, 0 dBm output power
    writeRegister(SETUP_RETR, 0b01011111);  // 0101 = 1500 uS between retries, 1111 = 15 retries
  }

  // Assign this radio's address to RX pipe 1.  When another radio sends us data, this is the address
  // it will use.  We use RX pipe 1 to store our address since the address in RX pipe 0 is reserved
  // for use with auto-acknowledgment packets.
  writeRegister(RX_ADDR_P1, address, 5);

  // Enable dynamically sized packets on the 2 RX pipes we use, 0 and 1.
  // RX pipe address 1 is used to for normal packets from radios that send us data.
  // RX pipe address 0 is used to for auto-acknowledgment packets from radios we transmit to.
  writeRegister(DYNPD, _BV(DPL_P0) | _BV(DPL_P1));

  // Enable dynamically sized payloads, ACK payloads, and TX support with or without an ACK request.
  writeRegister(FEATURE, _BV(EN_DPL) | _BV(EN_ACK_PAY) | _BV(EN_DYN_ACK));

  // Ensure RX and TX buffers are empty.  Each buffer can hold 3 packets.
  spiTransfer(WRITE_OPERATION, FLUSH_RX, NULL, 0);
  spiTransfer(WRITE_OPERATION, FLUSH_TX, NULL, 0);

  // Clear any interrupts.
  writeRegister(STATUS_NRF, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

  uint8_t success = startRx();
  return success;
}

void NRFLite::prepForTx(uint8_t* address, SendType sendType) {
  if (_lastToRadioAddress != address) {
    _lastToRadioAddress = address;

    // TX pipe address sets the destination radio for the data.
    // RX pipe 0 is special and needs the same address in order to receive ACK packets from the destination radio.
    writeRegister(TX_ADDR, address, 5);
    writeRegister(RX_ADDR_P0, address, 5);
  }

  // Ensure radio is ready for TX operation.
  uint8_t configReg = readRegister(CONFIG);
  uint8_t readyForTx = configReg == (CONFIG_REG_SETTINGS_FOR_RX_MODE & ~_BV(PRIM_RX));
  if (!readyForTx) {
    // Put radio into Standby-I mode in order to transition into TX mode.
    _cePinPort->OUTCLR |= _cePinBm;  // digitalWrite(_cePin, LOW);
    configReg = CONFIG_REG_SETTINGS_FOR_RX_MODE & ~_BV(PRIM_RX);
    writeRegister(CONFIG, configReg);
    _delay_ms(POWERDOWN_TO_RXTX_MODE_MILLIS);
  }

  uint8_t fifoReg = readRegister(FIFO_STATUS);

  // If RX buffer is full and we require an ACK, clear it so we can receive the ACK response.
  uint8_t rxBufferIsFull = fifoReg & _BV(RX_FULL);
  if (sendType == REQUIRE_ACK && rxBufferIsFull) {
    spiTransfer(WRITE_OPERATION, FLUSH_RX, NULL, 0);
  }

  // If TX buffer is full, wait for all queued packets to be sent.
  uint8_t txBufferIsFull = fifoReg & _BV(FIFO_FULL);
  if (txBufferIsFull) {
    waitForTxToComplete();
  }
}

uint8_t NRFLite::waitForTxToComplete() {
  _resetInterruptFlags = 0;  // Disable interrupt flag reset logic in 'whatHappened'.

  uint8_t fifoReg, statusReg;
  uint8_t txBufferIsEmpty;
  uint8_t packetWasSent, packetCouldNotBeSent;
  uint8_t txAttemptCount = 0;
  uint8_t result = 0;  // Default to indicating a failure.

  // TX buffer can store 3 packets, sends retry up to 15 times, and the retry wait time is about half
  // the time necessary to send a 32 byte packet and receive a 32 byte ACK response.  3 x 15 x 2 = 90
  const static uint8_t MAX_TX_ATTEMPT_COUNT = 90;

  while (txAttemptCount++ < MAX_TX_ATTEMPT_COUNT) {
    fifoReg = readRegister(FIFO_STATUS);
    txBufferIsEmpty = fifoReg & _BV(TX_EMPTY);

    if (txBufferIsEmpty) {
      result = 1;  // Indicate success.
      break;
    }

    // If we have separate pins for CE and CSN, CE will be LOW so we must toggle it to send a packet.
    if (_usingSeparateCeAndCsnPins) {
      _cePinPort->OUTSET |= _cePinBm;  // digitalWrite(_cePin, HIGH);
      _delay_us(CE_TRANSMISSION_MICROS);
      _cePinPort->OUTCLR |= _cePinBm;  // digitalWrite(_cePin, LOW);
    }

    _delay_us(TRANSMISSION_RETRY_WAIT_MICROS);

    statusReg = readRegister(STATUS_NRF);
    packetWasSent = statusReg & _BV(TX_DS);
    packetCouldNotBeSent = statusReg & _BV(MAX_RT);

    if (packetWasSent) {
      writeRegister(STATUS_NRF, _BV(TX_DS));  // Clear TX success flag.
    } else if (packetCouldNotBeSent) {
      spiTransfer(WRITE_OPERATION, FLUSH_TX, NULL, 0);  // Clear TX buffer.
      writeRegister(STATUS_NRF, _BV(MAX_RT));           // Clear max retry flag.
      break;
    }
  }

  _resetInterruptFlags = 1;  // Re-enable interrupt reset logic in 'whatHappened'.

  return result;
}

uint8_t NRFLite::readRegister(uint8_t regName) {
  uint8_t data;
  readRegister(regName, &data, 1);
  return data;
}

void NRFLite::readRegister(uint8_t regName, void* data, uint8_t length) {
  spiTransfer(READ_OPERATION, (R_REGISTER | (REGISTER_MASK & regName)), data, length);
}

void NRFLite::writeRegister(uint8_t regName, uint8_t data) {
  writeRegister(regName, &data, 1);
}

void NRFLite::writeRegister(uint8_t regName, void* data, uint8_t length) {
  spiTransfer(WRITE_OPERATION, (W_REGISTER | (REGISTER_MASK & regName)), data, length);
}

uint8_t NRFLite::spiTransferByte(uint8_t data) {
  SPI0.DATA = data;
  while (!(SPI0.INTFLAGS & SPI_IF_bm));
  return SPI0.DATA;
}

void NRFLite::spiTransfer(SpiTransferType transferType, uint8_t regName, void* data, uint8_t length) {
  uint8_t* intData = reinterpret_cast<uint8_t*>(data);

  cli();  // Prevent an interrupt from interferring with the communication.

  _csnPinPort->OUTCLR |= _csnPinBm;  // digitalWrite(_csnPin, LOW);  // Signal radio to listen to the SPI bus.

  spiTransferByte(regName);
  for (uint8_t i = 0; i < length; ++i) {
    uint8_t newData = spiTransferByte(intData[i]);
    if (transferType == READ_OPERATION) {
      intData[i] = newData;
    }
  }

  _csnPinPort->OUTSET |= _csnPinBm;  // digitalWrite(_csnPin, HIGH);  // Stop radio from listening to the SPI bus.

  sei();
}
