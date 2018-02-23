/**
 * @file       TinyGsmCommon.h
 * @author     Volodymyr Shymanskyy
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2016 Volodymyr Shymanskyy
 * @date       Nov 2016
 */

#ifndef TinyGsmCommon_h
#define TinyGsmCommon_h

#if defined(SPARK) || defined(PARTICLE)
  #include "Particle.h"
#elif defined(ARDUINO)
  #if ARDUINO >= 100
    #include "Arduino.h"
  #else
    #include "WProgram.h"
  #endif
#endif

#include <Client.h>
#include "TinyGsmClient.h"
#include "TinyGsmFifo.h"

#ifndef TINY_GSM_YIELD
  #define TINY_GSM_YIELD() { delay(0); }
#endif

#define TINY_GSM_ATTR_NOT_AVAILABLE __attribute__((error("Not available on this modem type")))
#define TINY_GSM_ATTR_NOT_IMPLEMENTED __attribute__((error("Not implemented")))

#if defined(__AVR__)
  #define TINY_GSM_PROGMEM PROGMEM
  typedef const __FlashStringHelper* GsmConstStr;
  #define GFP(x) (reinterpret_cast<GsmConstStr>(x))
  #define GF(x)  F(x)
#else
  #define TINY_GSM_PROGMEM
  typedef const char* GsmConstStr;
  #define GFP(x) x
  #define GF(x)  x
#endif

#ifdef TINY_GSM_DEBUG
namespace {
  template<typename T>
  static void DBG(T last) {
    TINY_GSM_DEBUG.println(last);
  }

  template<typename T, typename... Args>
  static void DBG(T head, Args... tail) {
    TINY_GSM_DEBUG.print(head);
    TINY_GSM_DEBUG.print(' ');
    DBG(tail...);
  }
}
#else
  #define DBG(...)
#endif

template<class T>
const T& TinyGsmMin(const T& a, const T& b)
{
    return (b < a) ? b : a;
}

template<class T>
const T& TinyGsmMax(const T& a, const T& b)
{
    return (b < a) ? a : b;
}

template<class T>
uint32_t TinyGsmAutoBaud(T& SerialAT, uint32_t minimum = 9600, uint32_t maximum = 115200)
{
  static uint32_t rates[] = { 115200, 57600, 38400, 19200, 9600, 74400, 74880, 230400, 460800, 2400, 4800, 14400, 28800 };

  for (unsigned i = 0; i < sizeof(rates)/sizeof(rates[0]); i++) {
    uint32_t rate = rates[i];
    if (rate < minimum || rate > maximum) continue;

    DBG("Trying baud rate", rate, "...");
    SerialAT.begin(rate);
    delay(10);
    for (int i=0; i<3; i++) {
      SerialAT.print("AT\r\n");
      String input = SerialAT.readString();
      if (input.indexOf("OK") >= 0) {
        DBG("Modem responded at rate", rate);
        return rate;
      }
    }
  }
  return 0;
}

static inline
IPAddress TinyGsmIpFromString(const String& strIP) {
  int Parts[4] = {0, };
  int Part = 0;
  for (uint8_t i=0; i<strIP.length(); i++) {
    char c = strIP[i];
    if (c == '.') {
      Part++;
      if (Part > 3) {
        return IPAddress(0,0,0,0);
      }
      continue;
    } else if (c >= '0' && c <= '9') {
      Parts[Part] *= 10;
      Parts[Part] += c - '0';
    } else {
      if (Part == 3) break;
    }
  }
  return IPAddress(Parts[0], Parts[1], Parts[2], Parts[3]);
}

static inline
String TinyGsmDecodeHex7bit(String &instr) {
  String result;
  byte reminder = 0;
  int bitstate = 7;
  for (unsigned i=0; i<instr.length(); i+=2) {
    char buf[4] = { 0, };
    buf[0] = instr[i];
    buf[1] = instr[i+1];
    byte b = strtol(buf, NULL, 16);

    byte bb = b << (7 - bitstate);
    char c = (bb + reminder) & 0x7F;
    result += c;
    reminder = b >> bitstate;
    bitstate--;
    if (bitstate == 0) {
      char c = reminder;
      result += c;
      reminder = 0;
      bitstate = 7;
    }
  }
  return result;
}

static inline
String TinyGsmDecodeHex8bit(String &instr) {
  String result;
  for (unsigned i=0; i<instr.length(); i+=2) {
    char buf[4] = { 0, };
    buf[0] = instr[i];
    buf[1] = instr[i+1];
    char b = strtol(buf, NULL, 16);
    result += b;
  }
  return result;
}

static inline
String TinyGsmDecodeHex16bit(String &instr) {
  String result;
  for (unsigned i=0; i<instr.length(); i+=4) {
    char buf[4] = { 0, };
    buf[0] = instr[i];
    buf[1] = instr[i+1];
    char b = strtol(buf, NULL, 16);
    if (b) { // If high byte is non-zero, we can't handle it ;(
#if defined(TINY_GSM_UNICODE_TO_HEX)
      result += "\\x";
      result += instr.substring(i, i+4);
#else
      result += "?";
#endif
    } else {
      buf[0] = instr[i+2];
      buf[1] = instr[i+3];
      b = strtol(buf, NULL, 16);
      result += b;
    }
  }
  return result;
}

static const char GSM_OK[] TINY_GSM_PROGMEM = "OK" GSM_NL;
static const char GSM_ERROR[] TINY_GSM_PROGMEM = "ERROR" GSM_NL;


//============================================================================//
//============================================================================//
//               Declaration of the TinyGSMModem Parent Class
//============================================================================//
//============================================================================//
class TinyGSMModem
{

public:

//============================================================================//
//============================================================================//
//      Declaration and definition of the GsmClientCommon Parent Class
//============================================================================//
//============================================================================//
class GsmClientCommon : public Client
{
  friend class TinyGSMModem;
  typedef TinyGsmFifo<uint8_t, TINY_GSM_RX_BUFFER> RxFifo;

public:
  GsmClientCommon() {}

  GsmClientCommon(TinyGSMModem& modem, uint8_t mux = 1) {
    init(&modem, mux);
  }

  bool init(TinyGSMModem* modem, uint8_t mux = 1) {
    this->at = modem;
    this->mux = mux;
    sock_available = 0;
    prev_check = 0;
    sock_connected = false;
    got_data = false;

    at->sockets[mux] = this;

    return true;
  }

public:
  virtual int connect(const char *host, uint16_t port) {
    TINY_GSM_YIELD();
    rx.clear();
    sock_connected = at->modemConnect(host, port, mux);
    return sock_connected;
  }

  virtual int connect(IPAddress ip, uint16_t port) {
    String host; host.reserve(16);
    host += ip[0];
    host += ".";
    host += ip[1];
    host += ".";
    host += ip[2];
    host += ".";
    host += ip[3];
    return connect(host.c_str(), port);
  }

  virtual void stop() {
    TINY_GSM_YIELD();
    at->sendAT(GF("+CIPCLOSE="), mux);
    sock_connected = false;
    at->waitResponse();
  }

  virtual size_t write(const uint8_t *buf, size_t size) {
    TINY_GSM_YIELD();
    at->maintain();
    return at->modemSend(buf, size, mux);
  }

  virtual size_t write(uint8_t c) {
    return write(&c, 1);
  }

  virtual int available() {
    TINY_GSM_YIELD();
    if (!rx.size() && sock_connected) {
      // Workaround: sometimes SIM800 forgets to notify about data arrival.
      // TODO: Currently we ping the module periodically,
      // but maybe there's a better indicator that we need to poll
      if (millis() - prev_check > 500) {
        got_data = true;
        prev_check = millis();
      }
      at->maintain();
    }
    return rx.size() + sock_available;
  }

  virtual int read(uint8_t *buf, size_t size) {
    TINY_GSM_YIELD();
    size_t cnt = 0;
    uint32_t _startMillis = millis();
    while (cnt < size && millis() - _startMillis < _timeout) {
      size_t chunk = TinyGsmMin(size-cnt, rx.size());
      if (chunk > 0) {
        rx.get(buf, chunk);
        buf += chunk;
        cnt += chunk;
        continue;
      }
      // TODO: Read directly into user buffer?
      if (!rx.size() && sock_connected) {
        at->maintain();
        //break;
      }
    }
    return cnt;
  }

  virtual int read() {
    uint8_t c;
    if (read(&c, 1) == 1) {
      return c;
    }
    return -1;
  }

  virtual int peek() { return -1; } //TODO
  virtual void flush() { at->stream.flush(); }

  virtual uint8_t connected() {
    if (available()) {
      return true;
    }
    return sock_connected;
  }
  virtual operator bool() { return connected(); }

  /*
   * Extended API
   */

  String remoteIP() TINY_GSM_ATTR_NOT_IMPLEMENTED;

protected:
  TinyGSMModem* at;
  uint8_t       mux;
  uint16_t      sock_available;
  uint32_t      prev_check;
  bool          sock_connected;
  bool          got_data;
  RxFifo        rx;
};

//============================================================================//
//============================================================================//
//                     The secure client goes here, if applicable
//============================================================================//
//============================================================================//


//============================================================================//
//============================================================================//
//              Functions for within the TinyGSMModem Parent Class
//============================================================================//
//============================================================================//

public:

#ifdef GSM_DEFAULT_STREAM
  TinyGSMModem(Stream& stream = GSM_DEFAULT_STREAM)
#else
  TinyGSMModem(Stream& stream)
#endif
    : stream(stream)
  {}

  /*
   * Basic functions
   */
  virtual bool begin() {
    return init();
  }

  virtual bool init() = 0;
  virtual void setBaud(unsigned long baud) = 0;
  virtual bool testAT(unsigned long timeout = 10000L) = 0;
  virtual void maintain() = 0;
  virtual bool factoryDefault() = 0;
  virtual bool hasSSL() {
    #if defined(TINY_GSM_MODEM_HAS_SSL)
    return true;
    #else
    return false;
    #endif
  }

  /*
   * Power functions
   */

  virtual bool restart() = 0;

  /*
   * SIM card functions
   */
#if defined(TINY_GSM_MODEM_HAS_GPRS)
  virtual String getSimCCID() = 0;
  virtual String getIMEI() = 0;
  virtual String getOperator() = 0;
#endif

  /*
   * Generic network functions
   */

  virtual int getRegistrationStatus() = 0;
  virtual int getSignalQuality() = 0;
  virtual bool isNetworkConnected() = 0;
  virtual bool waitForNetwork(unsigned long timeout = 60000L) {
    for (unsigned long start = millis(); millis() - start < timeout; ) {
      if (isNetworkConnected()) {
        return true;
      }
      delay(250);
    }
    return false;
  }
  virtual String getLocalIP() = 0;
  virtual IPAddress localIP() {
    return TinyGsmIpFromString(getLocalIP());
  }

#if defined(TINY_GSM_MODEM_HAS_GPRS) && defined(TINY_GSM_MODEM_HAS_WIFI)
  virtual bool networkConnect(const char* ssid, const char* pwd) = 0;
  virtual bool networkDisconnect() = 0;
  virtual bool gprsConnect(const char* apn, const char* user = "", const char* pwd = "") = 0;
  virtual bool gprsDisconnect() = 0;
#endif

  /*
   * WiFi functions
   */
#if !defined(TINY_GSM_MODEM_HAS_GPRS) && defined(TINY_GSM_MODEM_HAS_WIFI)
  virtual bool networkConnect(const char* ssid, const char* pwd) = 0;
  virtual bool networkDisconnect() = 0;
  bool gprsConnect(const char* apn, const char* user = "", const char* pwd = "") {
      return networkConnect(user, pwd);
  }
  bool gprsDisconnect() { return networkDisconnect();}
#endif

  /*
   * GPRS functions
   */
#if defined(TINY_GSM_MODEM_HAS_GPRS) && !defined(TINY_GSM_MODEM_HAS_WIFI)
  virtual bool gprsConnect(const char* apn, const char* user = "", const char* pwd = "") = 0;
  virtual bool gprsDisconnect() = 0;
  bool networkConnect(const char* ssid, const char* pwd) {
    return gprsConnect(ssid);
  }
  bool networkDisconnect() { return gprsDisconnect();}
#endif

  /*
   * Messaging functions
   */

#if defined(TINY_GSM_MODEM_HAS_GPRS)
  virtual bool sendSMS(const String& number, const String& text) = 0;
#endif


  /*
   * Location functions
   */

  /*
   * Battery functions
   */

public:

  /* Utilities */

  template<typename T>
  void streamWrite(T last) {
    stream.print(last);
  }

  template<typename T, typename... Args>
  void streamWrite(T head, Args... tail) {
    stream.print(head);
    streamWrite(tail...);
  }

  bool streamSkipUntil(char c) {
    const unsigned long timeout = 1000L;
    unsigned long startMillis = millis();
    while (millis() - startMillis < timeout) {
      while (millis() - startMillis < timeout && !stream.available()) {
        TINY_GSM_YIELD();
      }
      if (stream.read() == c)
        return true;
    }
    return false;
  }

  template<typename... Args>
  void sendAT(Args... cmd) {
    streamWrite("AT", cmd..., GSM_NL);
    stream.flush();
    TINY_GSM_YIELD();
    DBG("### AT:", cmd...);
  }

  // TODO: Optimize this!
  virtual uint8_t waitResponse(uint32_t timeout, String& data,
                               GsmConstStr r1=GFP(GSM_OK),
                               GsmConstStr r2=GFP(GSM_ERROR),
                               GsmConstStr r3=NULL,
                               GsmConstStr r4=NULL,
                               GsmConstStr r5=NULL) = 0;



  uint8_t waitResponse(uint32_t timeout,
                       GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
                       GsmConstStr r3=NULL, GsmConstStr r4=NULL, GsmConstStr r5=NULL)
  {
    String data;
    return waitResponse(timeout, data, r1, r2, r3, r4, r5);
  }

  uint8_t waitResponse(GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
                       GsmConstStr r3=NULL, GsmConstStr r4=NULL, GsmConstStr r5=NULL)
  {
    return waitResponse(1000, r1, r2, r3, r4, r5);
  }


protected:

  virtual bool modemConnect(const char* host, uint16_t port, uint8_t mux, bool ssl = false) = 0;
  virtual bool modemGetConnected(uint8_t mux) = 0;
  virtual int modemSend(const void* buff, size_t len, uint8_t mux) = 0;

  Stream&             stream;
  GsmClientCommon*    sockets[TINY_GSM_MUX_COUNT];
};

#endif
