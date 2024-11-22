#include <utpp/utpp.h>
#include <rieker/flexbus.h>
#include <fstream>
#include <conio.h>

#define COMPORT_NUMBER  6

TEST_MAIN (int argc, char **argv)
{
  std::ofstream os ("flex_tests.xml");
  UnitTest::ReporterXml xml (os);
  return UnitTest::RunAllTests (xml);
}

TEST (GetFirmwareVersion)
{
  Flexbus bus (COMPORT_NUMBER);
  char version[8];

  CHECK (bus.Request_FirmwareVersion ());
  CHECK (bus.WaitReply ());
  CHECK (bus.FirmwareVersion (version));
  printf ("Firmware version: %s\n", version);
}

TEST (SerialNumber)
{
  Flexbus bus (COMPORT_NUMBER);
  char serial[11];

  CHECK (bus.Request_SerialNumber ());
  CHECK (bus.WaitReply ());
  CHECK (bus.SerialNumber (serial));
  printf ("Serial number: %s\n", serial);
}

TEST (ReadAngles)
{
  Flexbus bus (COMPORT_NUMBER);
  double x, y;

  CHECK (bus.Request_Angle ());
  CHECK (bus.WaitReply ());
  CHECK_EQUAL (ERROR_SUCCESS, bus.LastError ());
  CHECK (bus.Angle (x, y));
  printf ("Angles x=%.3lf y=%.3lf\n", x, y);
}

TEST (GetAddress)
{
  Flexbus bus (COMPORT_NUMBER);

  unsigned short addr;
  CHECK (bus.Request_SensorAddress ());
  CHECK (bus.WaitReply ());
  CHECK_EQUAL (ERROR_SUCCESS, bus.LastError ());
  CHECK (bus.SensorAddress (addr));
  CHECK_EQUAL (bus.DefaultAddress (), addr);
  printf ("Sensor address: %hd\n", addr);
}

TEST (Lock_Change_Get)
{
  Flexbus bus (COMPORT_NUMBER);
  bool is_unlocked;

  CHECK (bus.ChangeLock (true));
  CHECK (bus.WaitReply ());
  CHECK_EQUAL (ERROR_SUCCESS, bus.LastError ());

  CHECK (bus.Request_LockStatus ());
  CHECK (bus.WaitReply ());
  CHECK_EQUAL (ERROR_SUCCESS, bus.LastError ());
  CHECK (bus.LockStatus (is_unlocked));

  CHECK_EQUAL (true, is_unlocked);
}

TEST (Restart)
{
  Flexbus bus (COMPORT_NUMBER);

  CHECK (bus.Restart ());
  /*
  Restart cycle takes about 200 ms. Here we wait until we got a reply to
  GetAddress command or 500 msec have elapsed.
  */
  bool waiting = true;
  UnitTest::Timer t;
  t.Start ();

  while (waiting)
  {
    unsigned short addr;
    bus.Request_SensorAddress ();
    bus.WaitReply ();
    if (bus.SensorAddress (addr) || t.GetTimeInMs () > 500)
      waiting = false;
  }
  CHECK (t.GetTimeInMs () < 500);
}

TEST (SetAnalogParam)
{
  Flexbus bus (COMPORT_NUMBER);
  bus.ChangeLock (true);
  bus.WaitReply ();

  CHECK (bus.SetAnalogParam (Flexbus::current_minx, 0.005));
}

TEST (SetOffset)
{
  Flexbus bus (COMPORT_NUMBER);
  double x, y;
  bus.ChangeLock (true);
  bus.WaitReply ();

  CHECK (bus.SetOffset (Flexbus::axis_x, 10));
  CHECK (bus.WaitReply ());

  CHECK (bus.SetOffset (Flexbus::axis_y, 20));
  CHECK (bus.WaitReply ());

  bus.Request_Angle ();
  bus.WaitReply ();
  bus.Angle (x, y);
  CHECK_CLOSE (-10, x, 1.0);
  CHECK_CLOSE (-20, y, 1.0);

  printf ("Angles after SetOffset x=%.3lf y=%.3lf\n", x, y);

  CHECK (bus.SetOffset (Flexbus::axis_x, 0));
  CHECK (bus.WaitReply ());

  CHECK (bus.WaitReply ());
  CHECK (bus.SetOffset (Flexbus::axis_y, 0));
}

TEST (SetZeroHere)
{
  Flexbus bus (COMPORT_NUMBER);
  double x, y;
  bus.ChangeLock (true);
  bus.WaitReply ();

  CHECK (bus.SetZeroHere (false));
  CHECK (bus.WaitReply ());

  //Verify that angles are close to 0 now.
  bus.Request_Angle ();
  bus.WaitReply ();
  bus.Angle (x, y);
  CHECK_CLOSE (0., x, 0.2);
  CHECK_CLOSE (0., y, 0.2);

  printf ("Angles after SetZeroHere x=%.3lf y=%.3lf\n", x, y);
}

TEST (SetFilterResponse)
{
  Flexbus bus (COMPORT_NUMBER);
  bus.ChangeLock (true);
  bus.WaitReply ();

  CHECK (bus.SetFilter (Flexbus::filter_highest));
  CHECK (bus.WaitReply ());
}

