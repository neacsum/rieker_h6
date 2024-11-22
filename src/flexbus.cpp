/*!
  \file flexbus.cpp - Implementation of Flexbus interface

  \author Mircea Neacsu
  \copyright Mircea Neacsu 2017. All rights reserved.

  License

  The MIT License (MIT)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*/

#include <rieker/flexbus.h>
#include <stdio.h>

#include <assert.h>

// Maximum size of a message from H6 inclinometer is 17 bytes 
// (doc version from 3/29/16)
#define BUFSIZE 30    ///< size of communication buffer

using namespace std;
static unsigned short crc16 (unsigned char *str, unsigned len);

static void swapb (unsigned char *c1, unsigned char *c2);
static void swaps (unsigned short *val);
static void swapi (long *val);
static void swapf (float *val) { swapi ((long*)val); };

/*!
  \class Flexbus
  This object encapsulates a COM port connected to a number of Rieker sensors.

  <h3>Addressing</h3>
  Each sensor on the bus is identified by it's address. Alternatively you can
  send a command to the broadcast address (FLEX_BROADCAST_ADDRESS) and it will
  be executed by all sensors.
  
  If you need to work with only one sensor connected, you can set the default
  address to the address of that sensor (using the DefaultAddress function) 
  and omit it in all subsequent function calls.

  <h3>Command Response Sequence</h3>
  After sending a command you have to wait for a sensor reply. In the simplest case,
  the reply is just an acknowledge or a NAK reply. If the command was a request
  for data, the reply contains the requested data.

  In any case, you can wait until a reply is received using the WaitReply() function,
  or you can check periodically if a reply has been received using the IsReplyReady()
  function.

  If the command was a request for information (one of the Request_... functions)
  you can call the corresponding function to retrieve the data. For instance after
  calling Request_FirmwareVersion(), when a reply has been received, you can call
  FirmwareVersion() function to retrieve the sensor firmware version string.

*/

/*!
  Constructor.
  \param    comnum    COM port number
  \param    baud      baudrate

  Opens a COM port. If something goes wrong it throws an exception of Flexbus::Error type.
  \note Default baudrate is 125000 (the default sensor baudrate).
  \note Default sensor address is 223.
*/
Flexbus::Flexbus (int comnum, baud_code baud)
  : num (comnum)
  , hcomm (INVALID_HANDLE_VALUE)
  , state (idle)
  , src (0)
  , last_dst (0)
  , default_dst (223)
  , last_error (ERROR_SUCCESS)
  , buf (new unsigned char[BUFSIZE])
  , cbuf (new unsigned char[BUFSIZE])
  , baudrate (baud)
  , ncbuf (0)
{
  evt = CreateEvent (NULL, true, false, nullptr);
  memset (&ioend, 0, sizeof (OVERLAPPED));
  ioend.hEvent = evt;

  wchar_t portstr[20];
  wsprintf (portstr, L"\\\\.\\COM%d", comnum);
  hcomm = CreateFileW (portstr,
                       GENERIC_READ | GENERIC_WRITE,
                       0,                      // comm devices must be opened w/exclusive-access
                       NULL,                   // no security attrs
                       OPEN_EXISTING,          // comm devices must use OPEN_EXISTING
                       FILE_FLAG_OVERLAPPED,   // overlapped  I/O
                       0l                      // comm devices must use NULL for hTemplate
                       );
  if (hcomm == INVALID_HANDLE_VALUE)
  {
    Error err{ GetLastError (), comnum };
    sprintf (err.msg, "CreateFile failed (%d)", GetLastError ());
    throw err;
  }

  DCB dcb;
  memset (&dcb, 0, sizeof (DCB));
  dcb.DCBlength = sizeof (DCB);

  //fill DCB
  dcb.BaudRate = BaudrateValue(baud);
  dcb.fBinary = true;                     // binary mode, no EOF check
  dcb.fDtrControl = DTR_CONTROL_ENABLE;   /* DTR flow control type
                                          Enables the DTR line when the device
                                          is opened and leaves it on. */
  dcb.fRtsControl = RTS_CONTROL_TOGGLE;   /* RTS line will be high if bytes are available for transmission.
                                          After all buffered bytes have been sent, the RTS line will be low */
  dcb.ByteSize = 8;                       // number of bits/byte, 4-8
  dcb.Parity = 0;                         // 0-4=no,odd,even,mark,space
  dcb.StopBits = 0;                       // 0,1,2 = 1, 1.5, 2

  if (!SetCommState (hcomm, &dcb))
  {
    Error err{ GetLastError (), comnum };
    sprintf (err.msg, "SetComState failed (%d)", GetLastError ());
    CloseHandle (hcomm);
    throw err;
  }

  /* Maximum read timeout set for 20 msec. */
  COMMTIMEOUTS to;
  memset (&to, 0, sizeof (COMMTIMEOUTS));
  to.ReadIntervalTimeout = 1;
  to.ReadTotalTimeoutConstant = 10;
  SetCommTimeouts (hcomm, &to);

  PurgeComm (hcomm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
}

/*!
  Destructor.
  Closes the communication port and frees all associated resources
*/
Flexbus::~Flexbus ()
{
  CloseHandle (hcomm);
  CloseHandle (evt);
  delete buf;
  delete cbuf;
}

/*!
  Send a command to a sensor.

  \param    addr    sensor address
  \param    cmd     command code
  \param    data    pointer to additional data bytes
  \param    len     size of additional data bytes
  \return   true if successful, false otherwise
*/
bool Flexbus::SendCommand (unsigned short addr, unsigned short cmd, unsigned char* data, size_t len)
{
  //Throw away any previous leftovers
  PurgeComm (hcomm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

  if (!addr)
    addr = default_dst;

  unsigned char *p = cbuf;
  *p++ = addr >> 8;
  *p++ = addr & 0xff;
  *p++ = src >> 8;
  *p++ = src & 0xff;
  *p++ = cmd >> 8;
  *p++ = cmd & 0xff;
  *p++ = len & 0xff;
  memcpy (p, data, len);
  p += len;
  unsigned short crc = crc16 (cbuf, (unsigned int)(p-cbuf));
  *p++ = crc >> 8;
  *p++ = crc & 0xff;
  ncbuf = (unsigned int)(p - cbuf);
  if (WriteFile (hcomm, cbuf, ncbuf, nullptr, &ioend))
  {
    last_error = ERR_FLEX_UNEXPECTED;
    return false;
  }
  state = Flexbus::write;
  last_dst = addr;
  last_cmd = cmd;
  last_error = ERROR_SUCCESS;

  return true;
}

/*!
  Check if a reply has been received.

  \return   true if a reply has been received (or a timeout occurred)
  \return   false if user program must continue waiting for a reply
*/
bool Flexbus::IsReplyReady ()
{
  DWORD code;

  if (state == Flexbus::idle)
  {
    Error err{ ERR_FLEX_BADSTATE, num, "No command" };
    throw err;
  }
  else if (state == Flexbus::write)
  {
    if (!GetOverlappedResult (hcomm, &ioend, &nc, false))
    {
      if ((code = GetLastError ()) != ERROR_IO_INCOMPLETE)
      {
        Error err{ code, num };
        sprintf (err.msg, "ReplyReady write GetOverlppedResult %d", code);
        throw err;
      }
      last_error = ERROR_IO_PENDING;
      return false;
    }

    state = Flexbus::read;
    memset (buf, 0xAA, BUFSIZE); // for debugging
    nc = 0;
    if (!ReadFile (hcomm, buf, BUFSIZE, &nc, &ioend))
    {
      code = GetLastError ();
      if (code != ERROR_IO_PENDING)
      {
        Error err{ code, num };
        sprintf (err.msg, "ReplyReady ReadFile %d", code);
        throw err;
      }
      last_error = code;
      return false;
    }
  }
  else
  {
    //already in read state
    if (!GetOverlappedResult (hcomm, &ioend, &nc, false))
    {
      if ((code = GetLastError ()) != ERROR_IO_INCOMPLETE)
      {
        Error err{ code, num };
        sprintf (err.msg, "ReplyReady read GetOverlppedResult %d", code);
        throw err;
      }
      last_error = ERROR_IO_PENDING;
      return false;
    }
  }
  //read has finished here
  state = Flexbus::idle;
  analyze_response ();
  return true;
}

//analyze device response
void Flexbus::analyze_response ()
{
  last_error = ERROR_SUCCESS;     //assume all will go well

  if (nc == 0)
  {
    last_error = ERR_FLEX_TIMEOUT; //no reply
    return;
  }
  if (nc < 9)
  {
    last_error = ERR_FLEX_INVREPLY; //reply too short
    return;
  }

  unsigned short rsrc = (buf[0] << 8) | buf[1];
  unsigned short rdst = (buf[2] << 8) | buf[3];
  unsigned char msglen = buf[6] + 7; //length of message not including CRC
  if ((rsrc != src)
   || (last_dst != 0xfffc && rdst != last_dst)
   || (nc < (DWORD)msglen + 2))
  {
    last_error = ERR_FLEX_INVREPLY;   //bad addresses or data length
    return;
  }

  unsigned short rcrc = (buf[msglen] << 8) | buf[msglen + 1]; //received CRC
  if (rcrc != crc16 (buf, msglen))
  {
    last_error = ERR_FLEX_BADCRC;
    return;
  }

  unsigned short func = (buf[4] << 8) | buf[5];
  if (func == FLEXFUNC_NAK)
    last_error = ERR_FLEX_NAK + buf[7];
}

/*!
  Wait for a reply from the sensor

  \return   true if a reply has been received
  \return   false if an error occurs

*/
bool Flexbus::WaitReply ()
{
  DWORD code;
  if (state == Flexbus::idle)
  {
    Error err{ ERR_FLEX_BADSTATE, num, "No command" };
    throw err;
  }
  else if (state == Flexbus::write)
  {
    if (!GetOverlappedResult (hcomm, &ioend, &nc, true))
    {
      Error err{ code = GetLastError (), num };
      sprintf (err.msg, "ReplyReady write GetOverlppedResult %d", code);
      throw err;
    }
    state = Flexbus::read;
    memset (buf, 0xAA, BUFSIZE); // for debugging
    nc = 0;
    if (!ReadFile (hcomm, buf, BUFSIZE, &nc, &ioend))
    {
      code = GetLastError ();
      if (code != ERROR_IO_PENDING)
      {
        Error err{ code, num };
        sprintf (err.msg, "ReplyReady ReadFile %d", code);
        throw err;
      }
    }
  }
  //already in read state
  if (!GetOverlappedResult (hcomm, &ioend, &nc, true))
  {
    Error err{ code = GetLastError (), num };
    sprintf (err.msg, "ReplyReady read GetOverlppedResult %d", code);
    throw err;
  }
  analyze_response ();
  return (last_error == ERROR_SUCCESS);
}


/*!
  Change the sensor address

  \param    new_addr   new sensor address
  \param    addr       current sensor address
  \return   true if successful, false otherwise

  After sending this command, wait 50 milliseconds before issuing further commands.
*/
bool Flexbus::SetAddress (unsigned short new_addr, unsigned short addr)
{
  unsigned char cmd[2];
  cmd[0] = new_addr >> 8;
  cmd[1] = new_addr & 0xff;
  return SendCommand (addr, FLEXFUNC_SET_ADDRESS, cmd, 2);
}

/*!
  Return the address of the sensor on the bus.

  \return   true if successful, false otherwise

  For this function to work there must be only one sensor on the bus. As opposed
  to all the other commands, this command is always sent to the broadcast address
  (it doesn't make much sense to request the address of a sensor for which you know
  the address). Being sent to the broadcast address, if there are more sensors
  on the bus, their answers can collide.
*/
bool Flexbus::Request_SensorAddress ()
{
  return SendCommand (FLEX_BROADCAST_ADDRESS, FLEXFUNC_GET_ADDRESS, 0, 0);
}

/*!
  Return sensor address.

  \param    addr    sensor address
  \return   true if successful, false otherwise

  This function should be called after Request_GetAddress() function
*/
bool Flexbus::SensorAddress (unsigned short& addr)
{
  assert (addr);
  if (last_cmd != FLEXFUNC_GET_ADDRESS)
  {
    last_error = ERR_FLEX_BADCMD;   // wrong command
    return false;
  }
  if (last_error != ERROR_SUCCESS)
    return false;                   //previous operation failed

  addr = (buf[7] << 8) | buf[8];
  return true;
}

/*!
  Check if sensor with given address is connected

  \param    addr    sensor address
  \return   true if successful, false otherwise

  If there is no sensor with this address on the bus the error code
  will be ERR_FLEX_TIMEOUT.
*/
bool Flexbus::CheckAddress (unsigned short addr)
{
  return SendCommand (addr, FLEXFUNC_CHECK_ADDRESS, 0, 0);
}

/*!
  Send a GET ANGLE command to read both sensor angles

  \param    addr sensor address
  \return   true if successful, false otherwise
*/
bool Flexbus::Request_Angle (unsigned short addr, axis_code axis)
{
  unsigned char cmd = (unsigned char)axis + 1;
  return SendCommand (addr, FLEXFUNC_GET_ANGLE, &cmd, 1);
}

/*!
  Get the sensor angles

  \param    x   X axis angle
  \param    y   Y axis angle
  \return   true if successful, false otherwise
  
  This function should be called after Request_Angle() function.
*/
bool Flexbus::Angle (double& x, double& y)
{
  if (last_cmd != FLEXFUNC_GET_ANGLE)
  {
    last_error = ERR_FLEX_BADCMD;   // wrong command
    return false;
  }
  if (last_error != ERROR_SUCCESS)
    return false;                   //previous operation failed

  union {
    float fv;
    unsigned char uc[4];
  };

  for (int i = 0; i < 4; i++)
    uc[3 - i] = buf[i + 7];
  x = fv;

  if (cbuf[7] == 4)
  {
    for (int i = 0; i < 4; i++)
      uc[3 - i] = buf[i + 11];
    y = fv;
  }

  return true;
}

/*!
  Changes the lock state of a sensor

  \param    unlock    true if sensor should be unlocked, false otherwise
  \param    addr      sensor address
  \return   true if successful, false otherwise
*/
bool Flexbus::ChangeLock (bool unlock, unsigned short addr)
{
  unsigned char cmd[5] = { 0, 0x00, 0xbc, 0x61, 0x4e };
  if (unlock)
    cmd[0] = 1;

  return SendCommand (addr, FLEXFUNC_CHANGE_LOCK, cmd, 5);
}

/*!
  Restart the sensor

  \param    addr      sensor address
  \return   true if successful, false otherwise
*/
bool Flexbus::Restart (unsigned short addr)
{
  return SendCommand (addr, FLEXFUNC_RESTART, 0, 0);
}

/*!
  Requests the placement in a holding buffer of an analog parameter

  \param    param     parameter to place in holding buffer
  \param    addr      sensor address
  \return   true if successful, false otherwise

  \note Undocumented
*/
bool Flexbus::GetAnalogParam (analog_param param, unsigned short addr)
{
  unsigned char cmd[2] = { 0, 0 };
  cmd[1] = param;
  return SendCommand (addr, FLEXFUNC_SET_ANALOG_PARAM, cmd, 2);
}

/*!
  Requests the parameter that was placed in a holding buffer by a previous
  GetAnalogParam() command.

  \param    addr      sensor address
  \return   true if successful, false otherwise

  \note Undocumented
*/
bool Flexbus::Request_Parameter (unsigned short addr)
{
  return SendCommand (addr, FLEXFUNC_READ_PARAMETER, 0, 0);
}

/*!
  Return a sensor parameter.

  \param    requested parameter
  \return   true if successful, false otherwise

  This function should be called after Request_Parameter() function.
  \note Undocumented
*/
bool Flexbus::Parameter (double& par)
{
  if (last_cmd != FLEXFUNC_READ_PARAMETER)
  {
    last_error = ERR_FLEX_BADCMD;   // wrong command
    return false;
  }
  if (last_error != ERROR_SUCCESS)
    return false;                   //previous operation failed

  union {
    float fv;
    unsigned char uc[4];
  };

  for (int i = 0; i < 4; i++)
    uc[3 - i] = buf[i + 7];
  par = fv;
  return true;
}

/*!
  Set Analog Output Parameter

  \param    param     parameter to change
  \param    value     new value
  \param    addr      sensor address
  \return   true if successful, false otherwise

  Current values must be in the range 0 to 24mA and angle values can be in the range
  -180 to 180
*/
bool Flexbus::SetAnalogParam (analog_param param, double value, unsigned short addr)
{
  if (param == current_minx || param == current_maxx || param == current_miny || param == current_maxy)
  {
    if (value < 0. || value > 24.)
    {
      last_error = ERR_FLEX_BADPARAM;
      return false;
    }
    value /= 1000.; //send value in amps
  }
  else if (value < -180. || value > 180.)
  {
    last_error = ERR_FLEX_BADPARAM;
    return false;
  }
  unsigned char cmd[6];
  unsigned short *p = (unsigned short*)cmd;
  cmd[0] = 0x01;      //Rieker doc error - first byte of command is 0x01...
  cmd[1] = param;     //...and 2nd is the parameter code
  float *f = (float*)(cmd + 2);
  *f = (float)value;
  swapf (f);
  return SendCommand (addr, FLEXFUNC_SET_ANALOG_PARAM, cmd, 6);
}

/*!
  Changes the zero offset value for an axis.

  \param    axis      sensor axis to be changed
  \param    value     new offset value
  \param    addr      sensor address
  \return   true if successful, false otherwise
  
  The offset value is subtracted from every angle for the axis 
  (causing Offset to become the new zero value). This lasts until a new offset
  is set or the sensor is reset to factory defaults.

  \note This can be used to remove any previous offsets by setting the value to 0.
  \note This function can be used to mount the sensor upside down by setting both values to 180.
*/
bool Flexbus::SetOffset (axis_code axis, double value, unsigned short addr)
{
  unsigned char cmd[5];
  cmd[0] = axis;
  float *f = (float*)(cmd + 1);
  *f = (float)value;
  swapf (f);
  return SendCommand (addr, FLEXFUNC_SET_ZERO, cmd, 5);
}

/*!
  Set sensor offets to current angle readings.

  \param    permanent if true new settings are permanent
  \param    addr      sensor address
  \return   true if successful, false otherwise

  Temporary settings last until the sensor is powered off or reset. 
  Permanent settings last until a new offset is set or the sensor is reset to factory
  defaults.
*/
bool Flexbus::SetZeroHere (bool permanent, unsigned short addr)
{
  unsigned char cmd;
  cmd = permanent? 1 : 0;
  return SendCommand (addr, FLEXFUNC_SET_ZERO_HERE, &cmd, 1);
}

/*!
  Request sensor firmware version string

  \param    addr sensor address
  \return   true if successful, false otherwise
*/
bool Flexbus::Request_FirmwareVersion (unsigned short addr)
{
  return SendCommand (addr, FLEXFUNC_GET_VERSION, 0, 0);
}

/*!
  Return sensor firmware version string.

  \param    version pointer to version string (at least 8 characters)
  \return   true if successful, false otherwise

  This function should be called after Request_FirmwareVersion() function.
  The firmware version string is 8 characters long (including terminating null character).
*/
bool Flexbus::FirmwareVersion (char* version)
{
  if (last_cmd != FLEXFUNC_GET_VERSION)
  {
    last_error = ERR_FLEX_BADCMD;   // wrong command
    return false;
  }
  if (last_error != ERROR_SUCCESS)
    return false;                   //previous operation failed

  memcpy (version, buf + 7, 7);
  version[7] = 0;
  return true;
}

/*!
  Request sensor lock status

  \param    addr sensor address
  \return   true if successful, false otherwise
*/
bool Flexbus::Request_LockStatus (unsigned short addr)
{
  return SendCommand (addr, FLEXFUNC_GET_LOCK, 0, 0);
}

/*!
  Return sensor lock status

  \param    unlocked set to true if sensor is unlocked
  \return   true if successful, false otherwise

  This function should be called after Request_LockStatus() function.
*/
bool Flexbus::LockStatus (bool& unlocked)
{
  if (last_cmd != FLEXFUNC_GET_LOCK)
  {
    last_error = ERR_FLEX_BADCMD;   // wrong command
    return false;
  }
  if (last_error != ERROR_SUCCESS)
    return false;                   //previous operation failed

  unlocked = (buf[7] != 0);
  return true;
}

/*!
  Request sensor serial number string
  \param  addr sensor address

  \return   true if successful, false otherwise
*/
bool Flexbus::Request_SerialNumber (unsigned short addr)
{
  return SendCommand (addr, FLEXFUNC_GET_SERIAL, 0, 0);
}

/*!
  Return sensor serial number string.

  \param    serial_number pointer to version string (at least 11 characters)
  \return   true if successful, false otherwise

  This function should be called after Request_SerialNumber() function.
  The serial number string is 11 characters long (including terminating null character).
*/
bool Flexbus::SerialNumber (char *serial_number)
{
  if (last_cmd != FLEXFUNC_GET_SERIAL)
  {
    last_error = ERR_FLEX_BADCMD;   // wrong command
    return false;
  }
  if (last_error != ERROR_SUCCESS)
    return false;                   //previous operation failed

  memcpy (serial_number, buf + 7, 10);
  serial_number[10] = 0;
  return true;
}

/*!
  Changes sensor baudrate

  \param    baud     new baudrate
  \param    addr      sensor address
  \return   true if successful, false otherwise

  New baudrate is effective after a reset or power off cycle.

  \note Flexbus object does not provide a function to change the baudrate.
  You have to create a new object with the new baudrate
*/
bool Flexbus::SetBaud (baud_code baud, unsigned short addr)
{
  unsigned char cmd;
  cmd = baud + 1;
  return SendCommand (addr, FLEXFUNC_SET_BAUD, &cmd, 1);
}

/*!
  Changes the setting of the sensor low-pass filter 

  \param    filter    new filter setting
  \param    addr      sensor address
  \return   true if successful, false otherwise
*/
bool Flexbus::SetFilter (filter_code filter, unsigned short addr)
{
  unsigned char cmd;
  cmd = filter + 1;
  return SendCommand (addr, FLEXFUNC_SET_FILTER, &cmd, 1);
}

void Flexbus::GetLastCommand (std::vector<BYTE>& last_command)
{
  last_command.clear ();
  for (int i = 0; i < ncbuf; i++)
    last_command.push_back (cbuf[i]);
}

void Flexbus::GetLastReply (std::vector<BYTE>& last_reply)
{
  last_reply.clear ();
  for (DWORD i = 0; i < nc; i++)
    last_reply.push_back (buf[i]);
}

/*=========================================================================*\
    Compute Cyclic Redundancy Check
\*=========================================================================*/

static const unsigned char CRC_MSB[] = {
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

static const unsigned char CRC_LSB[] = {
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
  0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
  0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
  0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
  0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
  0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
  0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
  0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
  0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
  0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
  0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
  0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
  0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
  0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
  0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

unsigned short crc16 (unsigned char *str, unsigned len)
{
  unsigned char cHi, cLo;             //  CRC Accumulators (MSB & LSB)
  unsigned short w;                   //  CRC Shift In Index

  cHi = cLo = 0;                      //  Init CRC
  while (len--)
  {                                   //  For Each Byte
    w = cHi ^ *str++;                 //  Next Table Index
    cHi = cLo ^ CRC_MSB[w];           //  Next CRC
    cLo = CRC_LSB[w];
  }
  /*Merge the bytes and return. Order is reversed because this is a
  little endian platform */
  return ((unsigned)cLo << 8) | cHi;
}

//functions for changing endianness

void swapb (unsigned char *c1, unsigned char *c2)
{
  unsigned char t = *c1;
  *c1 = *c2;
  *c2 = t;
}

void swaps (unsigned short *val)
{
  unsigned char *p = (unsigned char *)val;
  swapb (p, p + 1);
}

void swapi (long *val)
{
  unsigned char *p1 = (unsigned char *)val;
  unsigned char *p2 = p1 + 3;
  swapb (p1++, p2--);
  swapb (p1, p2);
}

