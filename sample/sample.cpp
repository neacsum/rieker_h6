#include <rieker/flexbus.h>
#include <stdio.h>
#include <conio.h>

#define COMPORT_NUMBER 6

/*
  Sample communication with Rieker H6 sensor. The program assumes there is a sensor
  with default address (223) and baudrate (125000) connected to the COM port.
*/
int main (int argc, char **argv)
{
  double x, y;
  unsigned short addr;
  char rev[8];
  char serial[11];

  int ch = 0;


  try {
    Flexbus bus (COMPORT_NUMBER, Flexbus::baud_125000);

    while (ch != 'q')
    {
      bus.Request_Angle ();
      if (!bus.WaitReply ())
        printf ("Request_Angle failed 0x%08x", bus.LastError ());
      else
      {
        bus.Angle (x, y);
        printf ("Sensor angles are x=%.3lf y=%.3lf\n", x, y);
      }

      ch = _getch ();
    }

    //Assume there is only one sensor connected to the bus.
    //Find sensor address
    if (!bus.Request_SensorAddress ())
    {
      printf ("Request_SensorAddress failed 0x%08x", bus.LastError ());
      return 1;
    }
    if (!bus.WaitReply ())
    {
      printf ("WaitReply error 0x%08x", bus.LastError ());
      return 1;
    }
    if (!bus.SensorAddress (addr))
    {
      printf ("SensorAddress failed 0x%08x", bus.LastError ());
      return 1;
    }

    //Set default address
    bus.DefaultAddress (addr);

    //Normally you want to check all function returns for failure as shown above.
    //This however just a sample so we will dispense with error checking for now.

    //Get firmware version
    bus.Request_FirmwareVersion ();
    bus.WaitReply ();
    bus.FirmwareVersion (rev);

    //... and serial number
    bus.Request_SerialNumber ();
    bus.WaitReply ();
    bus.SerialNumber (serial);

    printf ("Sensor address %d Serial# %s rev=%s\n", addr, rev, serial);

    //Read current angles
    bus.Request_Angle ();

    while (!bus.IsReplyReady ())
      Sleep (0);

    bus.Angle (x, y);
    printf ("Sensor angles are x=%.3lf y=%.3lf\n", x, y);
  }
  catch (Flexbus::Error& x)
  {
    printf ("Flexbus exception while talking to port COM%d %s (code=0x%08x)", x.comnum, x.msg, x.code);
    return 2;
  }
  return 0;

}