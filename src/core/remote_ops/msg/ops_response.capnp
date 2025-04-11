@0xbe7930e675ce716c;

struct OpsResponse {
    # response message for any operation request from remote ops system

    reqReceivedTime @0 :UInt64;
    operationStatus @1 :Bool;
    operationType @2 :UInt8;
    reason @3 :Text;
}