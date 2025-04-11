@0xddd34693532bf10e;

struct TeardownFromConsole
{
  consoleId @0 :Text;
  vehicleId @1 :Text;
}

enum TeardownResult
{
  notApplicable @0;
  success @1;
  fail @2;
}

struct TeardownFromConsoleAck
{
  consoleId @0 :Text;
  vehicleId @1 :Text;
  result @2 :TeardownResult;
}

struct Teardown {
  command :union {
    teardownFromConsole @0: TeardownFromConsole;
    teardownFromConsoleAck @1: TeardownFromConsoleAck;
  }
}