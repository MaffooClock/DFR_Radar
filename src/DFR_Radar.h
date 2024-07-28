/**
  * @file       DFR_Radar.h
  * @brief      An Arduino library that makes it easy to configure and use the DFRobot 24GHz millimeter-wave Human Presence Detection sensor (SEN0395)
  * @copyright  Copyright (c) 2010 DFRobot Co. Ltd. (http://www.dfrobot.com)
  *             Copyright (c) 2023 Matthew Clark (https://github.com/MaffooClock)
  * @license    The MIT License (MIT)
  * @authors    huyujie (yujie.hu@dfrobot.com)
  *             Matthew Clark
  * @version    v1.0
  * @date       2023-11-07
  * @url        https://github.com/MaffooClock/DFRobot_Radar
  */


#ifndef __DFR_Radar_H__
#define __DFR_Radar_H__

#include <Arduino.h>


class DFR_Radar
{
  public:

    /**
      * @brief Constructor
      * @param Stream  Software serial port interface
      */
    DFR_Radar( Stream *s );

    /**
     * @brief Not currently implemented
     *
     * @return true
     */
    bool begin( void );

    /**
     * @brief Set the serial port to use for communicating with the sensor
     *
     * @param s  The serial port to use
     */
    void setStream( Stream *s );

    /**
     * @brief Check if the sensor is ready to accept commands
     *
     * @return true if the sensor is ready;
     *         false if the sensor is not ready
     */
    bool isReady();

    /**
     * @brief Configure sensor detection range
     *
     * @note Values are in meters; minimum is 0, maximum is 9.45; end must be greater than start.
     *       Internally, the range is converted to a level between 0-63, each one being ~15cm.  If
     *       the value isn't a multiple of 15cm, the sensor will round down to the nearest 15cm,
     *       e.g. 5m = 5 / 0.15 = 33.3, which will be rounded down to 33, so your effective range
     *       would actually be 4.95m
     *
     * @param rangeStart Factory default is 0
     * @param rangeEnd   Factory default is 6
     *
     * @return false if the range values are invalid (no changes made), true otherwise
     */
    bool setDetectionRange( float rangeStart, float rangeEnd );

    /**
     * @brief Set the sensitivity level
     *
     * @param level  0 = Low, 9 = High, 7 = Factory Default
     *
     * @return false if the level value is invalid (no changes made), true otherwise
     */
    bool setSensitivity( uint8_t level );

    /**
     * @brief Configure delays that translate actual presence activity to sensor assertion of presence
     *
     * @note A longer confirmation delay can reduce false positives.  A longer disappearance delay can bridge gaps between presence events.
     *
     * @param confirmationDelay  Time in seconds of continuous presence activity before the sensor actually asserts presence; factory default is 0.025s
     * @param disappearanceDelay Time in seconds without any presence activity before desserting presence; factory default is 5s
     *
     * @return false if either delay value is invalid (no changes made), true otherwise
     */
    bool setTriggerLatency( float confirmationDelay, float disappearanceDelay );

    /**
     * @brief Configure delays between state changes on output (IO2)
     *
     * @param triggerDelay  Time in seconds after the sensor has been triggered before setting output HIGH; factory default is 2.5s
     * @param resetDelay    Time in seconds after the sensor is no longer triggered before returning the output LOW; factory default is 10s
     *
     * @return false if either delay value is invalid (no changes made), true otherwise
     */
    bool setOutputLatency( float triggerDelay, float resetDelay );

    /**
     * @brief Check if the sensor is detecting presence
     *
     * @return true if presence is currently being detected;
     *         false if no presence or reading sensor failed
     */
    bool checkPresence( void );

    /**
     * @brief Sets a delay between when the presence detection resets and when it can trigger again.
     *
     * @note Used to prevent short-cycling (re-triggering immediately after a reset).
     *
     * @param time  Time in seconds after the presence detection has reset before it can be triggered again.
     *              Range is 0.1 - 255; factory default is 1
     *
     * @return false if the value is invalid (no changes made), true otherwise
     */
    bool setLockout( float time );

    /**
     * @brief Set whether the IO2 pin is HIGH or LOW when triggered.
     *
     * @param triggerLevel  HIGH = Vcc when triggered, Ground when idle (factory default)
     *                      LOW  = Ground when triggered, Vcc when idle
     *
     * @return false if the value is invalid (no changes made), true otherwise
     */
    bool setTriggerLevel( uint8_t triggerLevel );

    /**
     * @brief Start the sensor
     *
     * @return true if sensor started (or was already started);
     *         false if sensor failed to start
     */
    bool start( void );

    /**
     * @brief Stop the sensor
     *
     * @return true if sensor stopped (or was already stopped);
     *         false if sensor failed to stop
     */
    bool stop( void );

    /**
     * @brief Restart the sensor's internal software (safe; configuration is not lost or changed).
     *
     */
    void reboot( void );

    /**
     * @brief Disable the LED
     *
     * @return true if command was successful
     */
    bool disableLED( void );

    /**
     * @brief Enable the LED
     *
     * @return true if command was successful
     */
    bool enableLED( void );

    /**
     * @brief Set whether LED is enabled.
     *
     * @note Called by `disableLED()` and `enableLED()`
     *
     * @param disabled true if LED should be disabled, false for enabled
     *
     * @return true if command was successful
     */
    bool configureLED( bool disabled );

    /**
     * @brief Allows setting multiple configuration options without
     *        stopping/saving/re-starting with each one.  Make sure
     *        to call `configEnd()` after making changes.
     *
     * @return false if the sensor failed to stop (multi-config mode will be disabled), true otherwise
     */
    bool configBegin( void );

    /**
     * @brief Allows setting multiple configuration options without
     *        stopping/saving/re-starting with each one.  Must call
     *        `configBegin()` first.
     *
     * @return false if multi-config mode isn't enabled (forgot to call `configBegin()` first or it failed),
     *         or if saving or re-starting failed; true otherwise
     */
    bool configEnd( void );

    /**
     * @brief Restore the sensor configuration to factory default settings.
     *
     * @return true if command was successful;
     *         false if the sensor failed to stop or if the ecommand failed
     */
    bool factoryReset( void );

  private:

    /**
     * @brief Read a line (or more) from the UART port
     *
     * @param buffer Store the read data
     *
     * @return length of characters captured
     */
    size_t readLines( char *buffer, size_t lineCount = 1 );

    /**
     * @brief Executes a command string after first stopping the sensor, then afterwards
     *        saves the configuration and re-starts the sensor.
     *
     * @details If multi-config mode is enabled (`configBegin()` was called earlier),
     *          this this method only executes the command string, and `configEnd()`
     *          must be called to save the configuration and re-start the sensor.
     *
     * @param command A command string generated by one of the configuration methods
     *
     * @return true if command was successful;
     *         false if sensor failed to stop or re-start, command failed, or save failed
     */
    bool setConfig( const char *command );

    /**
     * @brief Commits configuration data to flash
     *
     * @return true if command was successful
     */
    bool saveConfig( void );

    /**
     * @brief Used to ensure commands are terminated before writing to UART
     *
     * @note Documentation says to terminate all commands with \r\n, although
     *       it seems to work without it (sensor MCU probably catches the \0),
     *       but let's just be sure we're doing everything right.
     */
    size_t serialWrite( const char *command );

    /**
     * @brief Writes a command string to the sensor UART port and waits for response
     *
     * @param command A command string generated by one of the other config/command methods
     *
     * @return true if response was "Done";
     *         false if "Error" or timeout
     */
    bool sendCommand( const char *command );

    /**
     * @brief Writes a command string to the sensor UART port and compares the response to one provided
     *
     * @note This is for functions like `stop` and `start` that might return an "Error" response if
     *       the sensor is already stopped or started (respectively), in which case we still want to
     *       return `true`.
     *
     * @param command        A command string generated by one of the other config/command methods
     * @param acceptResponse The word or phrase to look for that if found will return `true`
     *
     * @return true if response was "Done" or matched `acceptResponse`;
     *         false if timeout or "Error" (and response didn't already match `acceptResponse`)
     */
    bool sendCommand( const char *command, const char *acceptResponse );

    /**
     * @brief The serial port (hardware or software) to use for communicating with the sensor
     *
     */
    Stream *sensorUART;

    // bool isConfigured;
    bool stopped;
    bool multiConfig;

    static const uint16_t readPacketTimeout         =  100;
    static const size_t packetLength                =   64;

    static const unsigned long startupDelay         = 2000;

    static const unsigned long comTimeout           = 1000;
    static constexpr const char *comStop            = "sensorStop";
    static constexpr const char *comStart           = "sensorStart";
    static constexpr const char *comResetSystem     = "resetSystem 0";
    static constexpr const char *comSetSensitivity  = "setSensitivity %u";
    static constexpr const char *comOutputLatency   = "outputLatency -1 %u %u";
    static constexpr const char *comSetGpioMode     = "setGpioMode 1 %u";
    static constexpr const char *comGetOutput       = "getOutput 1";
    static constexpr const char *comSetLedMode      = "setLedMode 1 %u";
    // static constexpr const char *comSetUartOutput   = "setUartOutput 1 1 0 1501";
    static constexpr const char *comSetEcho         = "setEcho 0";
    static constexpr const char *comResponseSuccess = "Done";
    static constexpr const char *comResponseFail    = "Error";
    static constexpr const char *comFailStopped     = "sensor stopped already";
    static constexpr const char *comFailStarted     = "sensor started already";
    static constexpr const char *comSaveCfg         = "saveConfig";
    static constexpr const char *comFactoryReset    = "resetCfg";
    static constexpr const char *comPrompt          = "leapMMW:/>";

    #ifdef __AVR__
      #ifdef _STDLIB_H_
        static constexpr const char *comSetRange    = "setRange %s %s";
        static constexpr const char *comSetLatency  = "setLatency %s %s";
        static constexpr const char *comSetInhibit  = "setInhibit %s";
      #else
        #warning Floats in `sprintf()` will be converted to integers
        static constexpr const char *comSetRange    = "setRange %u %u";
        static constexpr const char *comSetLatency  = "setLatency %u %u";
        static constexpr const char *comSetInhibit  = "setInhibit %u";
      #endif
    #else
      // TODO:
      // #if !defined( ESP32 ) && !defined( ARDUINO_TEENSY ) && ...
        #warning Assuming floats work in `sprintf()` -- it might not!
      //#endif
      static constexpr const char *comSetRange      = "setRange %.3f %.3f";
      static constexpr const char *comSetLatency    = "setLatency %.3f %.3f";
      static constexpr const char *comSetInhibit    = "setInhibit %.3f";
    #endif
};

#endif
