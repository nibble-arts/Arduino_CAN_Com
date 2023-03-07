/*
 * LOCO-CAN can class
 * 
 * @author: Thomas H Winkler
 * @copyright: 2020
 * @lizence: GG0
 */

/*
 * create can communication
 */

#include <CAN.h>
#include <ArduinoUniqueID.h>
#include <RokkitHash.h>

#include "can_com.h"
#include "simpletimeout.h"

#define DEBUG

/*
 * create CAN communication
 * user standard CS (10) and INT (2) ports
 */
CAN_COM::CAN_COM() {
  create_uuid();
}


/*
 * create CAN communication
 * set CS and INT ports
 */
CAN_COM::CAN_COM(uint8_t CS, uint8_t INT) {
  CAN.setPins(CS, INT);
  create_uuid();
}



bool CAN_COM::begin(long speed, uint8_t led_port) {

  // init status LED
  _led.begin(led_port);
  _led.on();

  // start the CAN bus
  #ifdef DEBUG
    Serial.print("Start CAN at ");
    Serial.print(speed);
    Serial.println(" bps");
  #endif
  
  while (!CAN.begin(speed)) {

    _led.on();

    #ifdef DEBUG
      Serial.println("Starting CAN failed!");
    #endif

    delay(250);
    _led.off();
    delay(1000);
  }

  #ifdef DEBUG
    Serial.println("OK!");
  #endif

  clear_filter();
  
  set_alive(CAN_ALIVE_TIMEOUT);

  delay(150);
  _led.off();

  return true;
}


void CAN_COM::create_uuid(void) {
  _uuid = rokkit((char*) UniqueID8, 8) & 0xFFFF; // 0x3FFFF
}


long CAN_COM::uuid(void) {
  return _uuid;
}


/*
 * set alive timeout
 */
void CAN_COM::set_alive(uint16_t alive_timeout) {
  _alive_timeout.begin(alive_timeout);
}


/*
 * check if CAN communication alive
 * timeout 
 */
bool CAN_COM::alive(void) {
  return _alive;
}

/*
 * send data package
 */
bool CAN_COM::send(uint8_t* data, uint8_t length, uint32_t id) {

  uint8_t i = 0;

  // begin packet
  // use 29 bit identifier
  // 11 bit: id
  // 18 bit: board uuid
  CAN.beginExtendedPacket((id << 18) | _uuid);

  // send data with length
  // restrict to 8 uint8_ts
  while (i < length && i < 8) {
    CAN.write(data[i++]);  
  }

  CAN.endPacket();

  return true;
}


/*
 * read data, return true if filter
 */
uint16_t CAN_COM::read(CAN_MESSAGE* message) {

  uint8_t i;
  uint8_t size;
  uint32_t can_id;


  // check and connection and update alive status
  _alive = !_alive_timeout.check();


  // ===============================================
  // check for package
  size = CAN.parsePacket();

  // received a packet
  if (size) {

    _led.on();

    // retrigger connection timeout
    _alive_timeout.retrigger();


    // fetch data
    i = 0;
    while (CAN.available() && i < 8) {
      message->data[i++] = (uint8_t)CAN.read();
    }

    // add size
    message->size = i;
    
    // get packet id
    // is extended
    // split in group id (11 bit) and uuid (18 bit)
    if (CAN.packetExtended()) {

      can_id = CAN.packetId();

      message->id = can_id >> 18;
      message->uuid = can_id & 0x3FFFF;
    }


    else {
      message->id = CAN.packetId();
      message->uuid = 0;
    }
    
    // check for filter criteriy
    if (_filter_count == 0) {
      return message->id;
    }

    // check for registered filters
    i = 0;
    while (i < _filter_count) {

      // filter found
      if ((message->id & _masks[i]) == _filters[i]) {
        return _filters[i];
      }

      i++;
    }

    _led.off();
  }

  return false;
}


bool CAN_COM::clear_filter() {
  _filter_count = 0;
}


/*
 * filter packet id
 true if valid
 */
bool CAN_COM::register_filter(uint16_t mask, uint16_t filter) {

  // has free filter slots
  if (_filter_count < CAN_MAX_FILTER) {

    _masks[_filter_count] = mask;
    _filters[_filter_count] = filter;

    _filter_count++;

  }

  return _filter_count;
}
