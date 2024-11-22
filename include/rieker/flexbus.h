/*!
  \file flexbus.h - Declaration of Flexbus interface

  (c) Mircea Neacsu 2017-2024. All rights reserved.

*/
#pragma once
#pragma comment (lib, "flex.lib")

/*!
  \name bus_codes Error codes returned by LastError.
  In addition to these codes, LastError can return one of the Windows standard error codes.
  Per [MS documentation](https://learn.microsoft.com/en-us/openspecs/windows_protocols/ms-erref/f9720d9d-59f7-46bf-8273-14039e8c75ff),
  our codes have bit 29 set to avoid conflict with standard error codes.
  \{
*/
#define ERR_FLEX_NAK              0x2000    ///< NAK received from device
#define ERR_FLEX_NAKCRC           0x2000    ///< NAK - CRC failure
#define ERR_FLEX_NAKNOFUNC        0x2001    ///< NAK - Unknown function
#define ERR_FLEX_NAKINVFUNC       0x2002    ///< NAK - Function not valid for this sensor or mode of operation
#define ERR_FLEX_NAKINVDATA       0x2003    ///< NAK - Data invalid for given function
#define ERR_FLEX_NAKLOCK          0x2004    ///< NAK - invalid lock level
#define ERR_FLEX_NAKBUSY          0x2005    ///< NAK - Sensor busy
#define ERR_FLEX_UNEXPECTED       0x2006    ///< Unexpected I/O completion
#define ERR_FLEX_BADSTATE         0x2007    ///< Wrong bus state
#define ERR_FLEX_TIMEOUT          0x2008    ///< Timeout
#define ERR_FLEX_INVREPLY         0x2009    ///< Invalid device reply
#define ERR_FLEX_BADCRC           0x200a    ///< Bad CRC
#define ERR_FLEX_BADCMD           0x200b    ///< Wrong command
#define ERR_FLEX_BADPARAM         0x200c    ///< Invalid parameter
#define ERR_FLEX_LAST             0x200d
///\}

/*!
  \name Functions
  \{
*/
#define FLEXFUNC_ACK              0         ///< Positive acknowledge
#define FLEXFUNC_NAK              1         ///< Negative acknowledge
#define FLEXFUNC_SET_ADDRESS      2         ///< Set sensor address
#define FLEXFUNC_GET_ADDRESS      3         ///< Get sensor's address
#define FLEXFUNC_CHECK_ADDRESS    4         ///< Check for sensor present at an address
#define FLEXFUNC_GET_ANGLE        5         ///< Get the angle of a sensor
#define FLEXFUNC_CHANGE_LOCK      10        ///< Change lock level
#define FLEXFUNC_RESTART          16        ///< Restarts the sensor
#define FLEXFUNC_READ_PARAMETER   20        ///< Read a parameter (undocumented)
#define FLEXFUNC_SET_ANALOG_PARAM 24        ///< Set analog output parameters
#define FLEXFUNC_SET_ZERO         25        ///< Set zero offset to sent value
#define FLEXFUNC_SET_ZERO_HERE    26        ///< Set zero offset to current angle
#define FLEXFUNC_GET_VERSION      28        ///< Get firmware revision
#define FLEXFUNC_GET_LOCK         30        ///< Get current lock level
#define FLEXFUNC_GET_SERIAL       37        ///< Get sensor's serial number
#define FLEXFUNC_SET_BAUD         38        ///< Set RS485 communication baudrate
#define FLEXFUNC_SET_FILTER       41        ///< Set the response and filtering
///\}

/// Broadcast address. Commands sent to this address are executed by all devices.
#define FLEX_BROADCAST_ADDRESS    0xfffc

#include <windows.h>
#include <vector>

/// Rieker Flex&trade; communication protocol
class Flexbus
{
public:

  ///Exception object
  struct Error {
    DWORD code;   ///< Error code 
    int comnum;   ///< COM port number of Flexbus interface
    char msg[80]; ///< Error message
  };

  /// Baudrate codes
  enum baud_code { baud_9600, baud_38400, baud_115200, baud_125000, baud_128000, baud_250000, baud_19200, baud_last };

  /// Codes for analog parameter used in SetAnalogParam function
  enum analog_param {current_minx, current_maxx, angle_minx, angle_maxx,
                     current_miny=5, current_maxy, angle_miny, angle_maxy};

  /// Codes for axis parameter used in RequestAngle and SetZero functions
  enum axis_code {axis_single, axis_x, axis_y, axis_xy};

  /// Filter configuration codes used in SetFilter function
  enum filter_code {filter_low, filter_medium, filter_high, filter_highest};

  Flexbus (int comnum, baud_code baud=baud_125000);
  ~Flexbus ();

  int Port () const;
  baud_code Baudrate () const;

  void SrcAddress (unsigned short addr);
  unsigned short SrcAddress () const;
  void DefaultAddress (unsigned short addr);
  unsigned short DefaultAddress () const;
  DWORD LastError () const;
  bool SendCommand (unsigned short addr, unsigned short cmd, unsigned char *data, size_t len);
  bool IsReplyReady ();
  bool WaitReply ();
  void GetLastCommand (std::vector<BYTE>& last_command);
  void GetLastReply (std::vector<BYTE>& last_reply);

  bool SetAddress (unsigned short new_addr, unsigned short addr=0);

  bool Request_SensorAddress ();
  bool SensorAddress (unsigned short& addr);

  bool CheckAddress (unsigned short addr=0);

  bool Request_Angle (unsigned short addr=0, axis_code axis=axis_xy);
  bool Angle (double& x, double& y);

  bool ChangeLock (bool unlock, unsigned short addr=0);
  bool Restart (unsigned short addr=0);
  bool GetAnalogParam (analog_param param, unsigned short addr = 0);
  bool Request_Parameter (unsigned short addr = 0);
  bool Parameter (double& par);
  bool SetAnalogParam (analog_param param, double value, unsigned short addr=0);
  bool SetOffset (axis_code axis, double value, unsigned short addr=0);
  bool SetZeroHere (bool permanent, unsigned short addr=0);

  bool Request_FirmwareVersion (unsigned short addr=0);
  bool FirmwareVersion (char* version);
  
  bool Request_LockStatus (unsigned short addr=0);
  bool LockStatus (bool& unlocked);

  bool Request_SerialNumber (unsigned short addr = 0);
  bool SerialNumber (char* serial_number);

  bool SetBaud (baud_code baud, unsigned short addr = 0);
  bool SetFilter (filter_code filter, unsigned short addr = 0);

  static int BaudrateValue (baud_code baud);

private:
  void analyze_response ();

  HANDLE hcomm;
  int num;
  baud_code baudrate;
  enum { idle, write, read } state;
  unsigned short src;
  unsigned short default_dst;
  unsigned short last_dst;
  unsigned short last_cmd;
  DWORD last_error;
  unsigned char *buf;
  OVERLAPPED ioend;
  HANDLE evt;
  DWORD nc;
  unsigned char *cbuf;  ///< last command buffer
  int ncbuf;            ///< number of chars in cbuf
};

/// Return serial port number.
inline
int Flexbus::Port () const
{
  return num;
}

/// Return current baudrate setting.
inline
Flexbus::baud_code Flexbus::Baudrate () const
{
  return baudrate;
}

/*!
  Set source address.
  \param addr source address

  All commands include an address (called source address) that identifies
  the device sending the command. This function allows changing the source
  address. The default source address is 0.
*/
inline
void Flexbus::SrcAddress (unsigned short addr)
{
  src = addr;
}

/// Return current source address.
inline
unsigned short Flexbus::SrcAddress () const 
{ 
  return src; 
}

/*!
  Set default sensor address.
  \param addr default sensor address

  Almost all commands take a sensor address parameter. If the sensor address
  is 0 the command is sent to the "default device" whose address is set by
  this function.
  \note When Flexbus object is constructed the default address is
  223 matching the factory default.
*/
inline
void Flexbus::DefaultAddress (unsigned short addr)
{
  default_dst = addr;
}

/// Return current default sensor address.
inline
unsigned short Flexbus::DefaultAddress () const 
{ 
  return default_dst; 
}

/// Return last error code
inline
DWORD Flexbus::LastError () const 
{ 
  return last_error; 
}

///  Return baudrate value corresponding to a baudrate code
inline 
int Flexbus::BaudrateValue (baud_code baud)
{
  static const int baudtab[] = { 9600, 38400, 115200, 125000, 128000, 250000, 19200 };
  return baudtab[baud];
}
