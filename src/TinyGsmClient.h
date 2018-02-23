/**
 * @file       TinyGsmClient.h
 * @author     Volodymyr Shymanskyy
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2016 Volodymyr Shymanskyy
 * @date       Nov 2016
 */

#ifndef TinyGsmClient_h
#define TinyGsmClient_h

#if defined(TINY_GSM_MODEM_SIM800) || defined(TINY_GSM_MODEM_SIM868) || defined(TINY_GSM_MODEM_U201) || defined(TINY_GSM_MODEM_ESP8266)
  #define TINY_GSM_MODEM_HAS_SSL
#endif

#if defined(TINY_GSM_MODEM_SIM808) || defined(TINY_GSM_MODEM_SIM868) || defined(TINY_GSM_MODEM_A7)
  #define TINY_GSM_MODEM_HAS_GPS
#endif

#if defined(TINY_GSM_MODEM_SIM800) || defined(TINY_GSM_MODEM_SIM900)
  #define TINY_GSM_MODEM_HAS_GPRS
  #define TINY_GSM_MODEM_CALLABLE TinyGsmSim800
  #define GSM_NL "\r\n"
  #if !defined(TINY_GSM_RX_BUFFER)
    #define TINY_GSM_RX_BUFFER 64
  #endif
  #define TINY_GSM_MUX_COUNT 5
  #include <TinyGsmClientSIM800.h>
  typedef TinyGsmSim800 TinyGsm;
  typedef TinyGsmSim800::GsmClient TinyGsmClient;
  typedef TinyGsmSim800::GsmClientSecure TinyGsmClientSecure;

#elif defined(TINY_GSM_MODEM_SIM808) || defined(TINY_GSM_MODEM_SIM868)
  #define TINY_GSM_MODEM_HAS_GPRS
  #include <TinyGsmClientSIM808.h>
  typedef TinyGsmSim808 TinyGsm;
  typedef TinyGsmSim808::GsmClient TinyGsmClient;
  typedef TinyGsmSim808::GsmClientSecure TinyGsmClientSecure;

#elif defined(TINY_GSM_MODEM_A6) || defined(TINY_GSM_MODEM_A7)
  #define TINY_GSM_MODEM_HAS_GPRS
  #define TINY_GSM_MODEM_CALLABLE TinyGsmA6
  #define GSM_NL "\r\n"
  #if !defined(TINY_GSM_RX_BUFFER)
    #define TINY_GSM_RX_BUFFER 256
  #endif
  #define TINY_GSM_MUX_COUNT 8
  #include <TinyGsmClientA6.h>
  typedef TinyGsmA6 TinyGsm;
  typedef TinyGsm::GsmClient TinyGsmClient;

#elif defined(TINY_GSM_MODEM_M590)
  #define TINY_GSM_MODEM_HAS_GPRS
  #define TINY_GSM_MODEM_CALLABLE TinyGsmM590
  #define GSM_NL "\r\n"
  #if !defined(TINY_GSM_RX_BUFFER)
    #define TINY_GSM_RX_BUFFER 256
  #endif
  #define TINY_GSM_MUX_COUNT 2
  #include <TinyGsmClientM590.h>
  typedef TinyGsmM590 TinyGsm;
  typedef TinyGsm::GsmClient TinyGsmClient;

#elif defined(TINY_GSM_MODEM_U201)
  #define TINY_GSM_MODEM_HAS_GPRS
  #define TINY_GSM_MODEM_CALLABLE TinyGsmU201
  #define GSM_NL "\r\n"
  #if !defined(TINY_GSM_RX_BUFFER)
    #define TINY_GSM_RX_BUFFER 64
  #endif
  #define TINY_GSM_MUX_COUNT 5
  #include <TinyGsmClientU201.h>
  typedef TinyGsmU201 TinyGsm;
  typedef TinyGsmU201::GsmClient TinyGsmClient;
  typedef TinyGsmU201::GsmClientSecure TinyGsmClientSecure;

#elif defined(TINY_GSM_MODEM_ESP8266)
  #define TINY_GSM_MODEM_HAS_WIFI
  #define TINY_GSM_MODEM_CALLABLE TinyGsmESP8266
  #define GSM_NL "\r\n"
  #if !defined(TINY_GSM_RX_BUFFER)
    #define TINY_GSM_RX_BUFFER 512
  #endif
  #define TINY_GSM_MUX_COUNT 5
  #include <TinyGsmClientESP8266.h>
  typedef TinyGsmESP8266 TinyGsm;
  typedef TinyGsm::GsmClient TinyGsmClient;
  typedef TinyGsm::GsmClientSecure TinyGsmClientSecure;

#elif defined(TINY_GSM_MODEM_XBEE)
  #define TINY_GSM_MODEM_HAS_GPRS
  #define TINY_GSM_MODEM_HAS_WIFI
    #define TINY_GSM_MODEM_CALLABLE TinyGsmXBee
    #define GSM_NL "\r"
    #if !defined(TINY_GSM_RX_BUFFER)
      #define TINY_GSM_RX_BUFFER 0  // XBee doesn't use the FIFO
    #endif
    #define TINY_GSM_MUX_COUNT 1  // Multi-plexing isn't supported using command mode
  #include <TinyGsmClientXBee.h>
  typedef TinyGsmXBee TinyGsm;
  typedef TinyGsmXBee::GsmClient TinyGsmClient;
  typedef TinyGsmXBee::GsmClientSecure TinyGsmClientSecure;

#else
  #error "Please define GSM modem model"
  // Some definitions for myself to help debugging
  #define TINY_GSM_MODEM_CALLABLE TinyGSMModem
  #define GSM_NL "\r\n"
  #if !defined(TINY_GSM_RX_BUFFER)
    #define TINY_GSM_RX_BUFFER 64
  #endif
  #define TINY_GSM_MUX_COUNT 1
  #include <TinyGsmCommon.h>
  typedef TINY_GSM_MODEM_CALLABLE TinyGsm;
  typedef TINY_GSM_MODEM_CALLABLE TinyGsmClient;
  typedef TINY_GSM_MODEM_CALLABLE TinyGsmClientSecure;
#endif

#endif
