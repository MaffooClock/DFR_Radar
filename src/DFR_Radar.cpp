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

#include <DFR_Radar.h>


DFR_Radar::DFR_Radar( Stream *s )
{
  sensorUART = s;
  // isConfigured = false;
  stopped = false;
  multiConfig = false;
}

bool DFR_Radar::begin()
{
  /* Not sure if I want to impliment this, keeping it for future consideration...

  unsigned long startTime = millis() + startupDelay;

  // Give the sensor time to start up just in case this method is called too soon.
  //
  // There's probably a smarter way to do this.  Factory default configuration will
  // have the sensor dumping out $JYBSS messages once per second, so that could be
  // an easy way to tell that it's "ready".  But if the sensor is configured to send
  // these only when queried, or when an presence event occurs, or if the interval
  // is set too long, then this won't really work.
  //
  // Another way might be to send a `sensorStart` and see if 1) it complains about
  // not being ready, or 2) it responds with "sensor started already" and "Error",
  // or 3) actually starts?

  while( millis() < startTime )
    yield();

  if( !stop() )
    return false;

  // Disable command echoing (less response data that we have to parse through)
  sendCommand( comSetEcho );

  // Disable periodic $JYBSS messages (we will query for them)
  sendCommand( comSetUartOutput );

  if( !saveConfig() )
    return false;

  if( !start() )
    return false;

  isConfigured = true;

  */

  return true;
}

void DFR_Radar::setStream( Stream *s ) {
    sensorUART = s;
}

bool DFR_Radar::isReady() {
    return sensorUART != nullptr;
}

size_t DFR_Radar::readLines( char *buffer, size_t lineCount )
{
  unsigned long timeLimit = millis() + readPacketTimeout;
  size_t offset = 0, linesLeft = lineCount;

  while( linesLeft && millis() < timeLimit )
  {
    if( sensorUART->available() <= 0 )
      continue;

    char c = sensorUART->read();

    if( c == '\r' )
      continue;

    buffer[offset++] = c;

    if( c == '\n' )
      linesLeft--;
  }

  return strlen( buffer );
}

bool DFR_Radar::checkPresence()
{
  char packet[packetLength] = {0};

  // Factory default settings have $JYBSS messages sent once per second,
  // but we won't want to wait; this will prompt for status immediately
  serialWrite( comGetOutput );

  /**
   * Get the response immediately after sending the command.
   *
   * If command echoing is enabled, there should be three lines:
   *   1. the "getOutput 1" echoed back
   *   2. a "Done" status
   *   3. the "leapMMW:/>" response followed by the $JYBSS data we want
   *
   * If command echoing is disabled, there should be two lines:
   *   1. a "Done" status
   *   2. the $JYBSS data we want
   *
   * Factory default is command echoing on (might change this in `begin()`)
   */
  size_t length = readLines( packet, 3 );

  if( !length )
    return false;

  const size_t expectedLength = 16;
  char data[expectedLength] = {0};
  uint8_t offset = 0;
  bool startCharacterFound = false, endCharacterFound = false;

  /**
   * Parse through the packet until we find a "$", and
   * then start capturing characters until we find a "*"
   *
   * We're expecting to get something like: $JYBSS,1, , , *
   */
  for( uint8_t i = 0; i < length; i++ )
  {
    char c = packet[i];

    if( c == '$' )
      startCharacterFound = true;

    if( !startCharacterFound )
      continue;

    if( c == '*' )
      endCharacterFound = true;

    data[offset++] = c;

    if( endCharacterFound || offset == expectedLength )
      break;
  }

  if( !startCharacterFound || !endCharacterFound )
    return false;

  return ( data[7] == '1' );
}

bool DFR_Radar::setLockout( float time )
{
  if( time < 0.1 || time > 255 )
    return false;

  char _comSetInhibit[19] = {0};

  #ifdef __AVR__
    #ifdef _STDLIB_H_
      char _time[8] = {0};
      dtostrf( time, 3, 3, _time );

      sprintf( _comSetInhibit, comSetInhibit, _time );
    #else
      sprintf( _comSetInhibit, comSetInhibit, (uint8_t)time );
    #endif
  #else
    sprintf( _comSetInhibit, comSetInhibit, time );
  #endif

  return setConfig( _comSetInhibit );
}

bool DFR_Radar::setTriggerLevel( uint8_t triggerLevel )
{
  if( triggerLevel != HIGH || triggerLevel != LOW )
    return false;

  char _comSetGpioMode[16] = {0};
  sprintf( _comSetGpioMode, comSetGpioMode, triggerLevel );

  return setConfig( _comSetGpioMode );
}

bool DFR_Radar::setDetectionRange( float rangeStart, float rangeEnd )
{
  if( rangeStart < 0 || rangeStart > 9.45 )
    return false;

  if( rangeEnd < 0 || rangeEnd > 9.45 )
    return false;

  if( rangeEnd < rangeStart )
    return false;

  char _comSetRange[21] = {0};

  #ifdef __AVR__
    #ifdef _STDLIB_H_
      char _rangeStart[6] = {0};
      dtostrf( rangeStart, 1, 3, _rangeStart );

      char _rangeEnd[6] = {0};
      dtostrf( rangeEnd, 1, 3, _rangeEnd );

      sprintf( _comSetRange, comSetRange, _rangeStart, _rangeEnd );
    #else
      sprintf( _comSetRange, comSetRange, (uint8_t)rangeStart, (uint16_t)rangeEnd );
    #endif
  #else
    sprintf( _comSetRange, comSetRange, rangeStart, rangeEnd );
  #endif

  return setConfig( _comSetRange );
}

bool DFR_Radar::setTriggerLatency( float confirmationDelay, float disappearanceDelay )
{
  if( confirmationDelay < 0 || confirmationDelay > 100 )
    return false;

  if( disappearanceDelay < 0 || disappearanceDelay > 1500 )
    return false;

  char _comSetLatency[28] = {0};

  #ifdef __AVR__
    #ifdef _STDLIB_H_
      char _confirmationDelay[8] = {0};
      dtostrf( confirmationDelay, 3, 3, _confirmationDelay );

      char _disappearanceDelay[9] = {0};
      dtostrf( disappearanceDelay, 4, 3, _disappearanceDelay );

      sprintf( _comSetLatency, comSetLatency, _confirmationDelay , _disappearanceDelay );
    #else
      sprintf( _comSetLatency, comSetLatency, (uint8_t)confirmationDelay , (uint16_t)disappearanceDelay );
    #endif
  #else
    sprintf( _comSetLatency, comSetLatency, confirmationDelay , disappearanceDelay );
  #endif

  return setConfig( _comSetLatency );
}

bool DFR_Radar::setOutputLatency( float triggerDelay, float resetDelay )
{
  if( triggerDelay < 0 || resetDelay < 0 )
    return false;

  // Convert seconds into 25ms units
  uint32_t _triggerDelay = triggerDelay * 1000 / 25;
  uint32_t _resetDelay   = resetDelay * 1000 / 25;

  if( _triggerDelay > 65535 || _resetDelay > 65535 )
    return false;

  char _comOutputLatency[29] = {0};
  sprintf( _comOutputLatency, comOutputLatency, (uint16_t)_triggerDelay , (uint16_t)_resetDelay );

  return setConfig( _comOutputLatency );
}

bool DFR_Radar::setSensitivity( uint8_t level )
{
  if( level > 9 )
    return false;

  char _comSetSensitivity[17] = {0};
  sprintf( _comSetSensitivity, comSetSensitivity, level );

  return setConfig( _comSetSensitivity );
}

bool DFR_Radar::disableLED()
{
  return configureLED( true );
}

bool  DFR_Radar::enableLED()
{
  return configureLED( false );
}

bool DFR_Radar::configureLED( bool disabled )
{
  char _comSetLedMode[15] = {0};
  sprintf( _comSetLedMode, comSetLedMode, disabled );

  return setConfig( _comSetLedMode );
}

bool DFR_Radar::factoryReset()
{
  // if( !stop() )
  //   return false;
  stop();

  bool success = sendCommand( comFactoryReset );
  delay( 2000 );

  return success;
}

bool DFR_Radar::configBegin()
{
  if( multiConfig )
    return true;

  if( !stop() )
    return false;

  multiConfig = true;

  return true;
}

bool DFR_Radar::configEnd()
{
  if( !multiConfig )
    return false;

  multiConfig = false;

  if( !saveConfig() )
    return false;

  if( !start() )
    return false;

  return true;
}

bool DFR_Radar::setConfig( const char *command )
{
  if( multiConfig )
  {
    return sendCommand( command );
  }
  else
  {
    // if( !stop() )
    //   return false;
    stop();

    if( !sendCommand( command ) )
      return false;

    bool saved = saveConfig();

    if( !start() )
      return false;

    return saved;
  }
}

bool DFR_Radar::saveConfig()
{
  return sendCommand( comSaveCfg );
}

bool DFR_Radar::start()
{
  if( !stopped )
    return true;

  if( sendCommand( comStart, comFailStarted ) )
  {
    stopped = false;
    return true;
  }

  return false;
}

bool DFR_Radar::stop()
{
  if( stopped )
    return true;

  if( sendCommand( comStop, comFailStopped ) )
  {
    stopped = true;
    return true;
  }

  return false;
}

void DFR_Radar::reboot()
{
  sendCommand( comResetSystem );
}

size_t DFR_Radar::serialWrite( const char *command )
{
  static char _command[64] = {0};
  size_t commandLength = strlen( command ) + 3;

  // Ensure the re-usable buffer is empty
  memset( &_command[0], 0, sizeof( _command ) );

  // Make a properly-terminated copy of the command
  snprintf( _command, commandLength, "%s\r\n", command );

  // Make sure we have exactly enough time
  sensorUART->setTimeout( comTimeout );

  // Clear the receive buffer
  while( sensorUART->available() )
    sensorUART->read();

  // Send the command...
  sensorUART->write( _command );
  sensorUART->flush();

  return commandLength;
}

bool DFR_Radar::sendCommand( const char *command )
{
  return sendCommand( command, NULL );
}

bool DFR_Radar::sendCommand( const char *command, const char *acceptableResponse )
{
  bool errorAcceptable = false;
  char lineBuffer[64] = {0};
  unsigned long timeout = millis() + comTimeout;

  static const size_t successLength = strlen( comResponseSuccess );
  static const size_t failLength = strlen( comResponseFail );
  static const size_t minResponseLength = min( successLength, failLength );

  const size_t commandLength = strlen( command );
  size_t minLength = min( commandLength, minResponseLength );
  const size_t acceptableLength = acceptableResponse == NULL ? minLength : strlen( acceptableResponse );

  if( acceptableResponse != NULL )
    minLength = min( acceptableLength, minLength );

  // Send the command...
  serialWrite( command );

  // ...then wait for a response
  while( millis() < timeout )
  {
    if( sensorUART->available() <= 0 )
      continue;

    // Ensure the buffer is empty
    memset( &lineBuffer[0], 0, sizeof( lineBuffer ) );

    // Read a whole line
    size_t responseLength = sensorUART->readBytesUntil( '\n', lineBuffer, sizeof( lineBuffer ) );

    // The sensor is supposed to terminate lines with <CRLF>, and we stopped at <LF>,
    // so the last character in the line buffer should be a <CR>.  If so, swap it out
    // for a null terminator
    uint8_t lastCharacter = strlen( lineBuffer ) - 1;
    if( lineBuffer[lastCharacter] == '\r' )
      lineBuffer[lastCharacter] = '\0';

    // Update the length and ensure the line we've received is properly terminated
    responseLength = strlen( lineBuffer );
    lineBuffer[responseLength] = '\0';

    // We got something shorter than anything we're expecting, so try again
    if( responseLength < minLength )
      continue;

    // Check if that line is the command prompt
    if( strncmp( comPrompt, lineBuffer, strlen( comPrompt ) ) == 0 )
      continue;

    // ...or if that line is an echo of the original command
    if( strncmp( command, lineBuffer, commandLength ) == 0 )
      continue;

    // ...or if that line contains an expected response
    if( acceptableResponse != NULL && strncmp( acceptableResponse, lineBuffer, acceptableLength ) == 0 )
    {
      errorAcceptable = true;

      // Even though we got what we want, we can't return yet; we need to go one more round
      // so that we get the "Done" or "Error" that follows out of the serial buffer.
      continue;
    }

    // ...or if that line says "Done"
    if( strncmp( comResponseSuccess, lineBuffer, successLength ) == 0 )
      return true;

    // ...or if that line says "Error"
    if( strncmp( comResponseFail, lineBuffer, failLength ) == 0 )
      return errorAcceptable;

    // ...we got nothing we expected, so try again
  }

  // We've timed out
  return errorAcceptable;
}
