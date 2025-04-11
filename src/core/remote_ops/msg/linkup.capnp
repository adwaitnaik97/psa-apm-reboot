@0x85bf30ef5dc73322;

enum ConsoleViewType
{
  control @0;
  view @1;
}

enum ConsoleMode
{
  test @0;
  production @1;
}

enum LinkUpResult
{
  notApplicable @0;
  success @1;
  failNotInValidState @2;
  failAlreadyControlled @3;
  failNotExpectingLinkup @4;
}

struct LinkUpFromConsole
{
  consoleId @0 :Text;
  vehicleId @1 :Text;
  viewType @2 :ConsoleViewType;
  consoleMode @3 :ConsoleMode;
}

struct LinkUpFromConsoleAck
{
  consoleId @0 :Text;
  vehicleId @1 :Text;
  viewType @2 :ConsoleViewType;
  result @3 :LinkUpResult;
}

struct LinkUp {
  command :union {
    linkUpFromConsole @0: LinkUpFromConsole;
    linkUpFromConsoleAck @1: LinkUpFromConsoleAck;
  }
}