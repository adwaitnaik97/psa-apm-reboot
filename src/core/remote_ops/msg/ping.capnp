@0xbd533bc48ecb23b1;

struct Ping {}

struct PingAck
{
    requestLatency @0: UInt64; # Time taken for the ping request to be received
}

struct PingMessage {
  sender @0: Text;
  receiver @1: Text;
  seq @2: UInt8;
  type :union {
    ping @3: Ping;
    pingAck @4: PingAck;
  }
}