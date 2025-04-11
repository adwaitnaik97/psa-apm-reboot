@0x94d8a3b25981c78e;

enum BreakLinkReason
{
  notApplicable @0;
  timeout @1;
  hardwareDisconnect @2;
  teardownTimeout @3;
  maxLinkupAttempts @4;
  webuiHeartbeatLost @5;
}

struct BreakLinkFromConsole
{
  consoleId @0 :Text;
  vehicleId @1 :Text;
  breakLinkReason @2: BreakLinkReason;
}

struct BreakLinkFromConsoleAck
{
  consoleId @0 :Text;
  vehicleId @1 :Text;
}

struct BreakLink {
  command :union {
    breakLinkFromConsole @0: BreakLinkFromConsole;
    breakLinkFromConsoleAck @1: BreakLinkFromConsoleAck;
  }
}