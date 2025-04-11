@0xdec7f74b80337371;

struct SetDestinationCmd {
  blockName @0 :Text;
  blockId @1 :UInt8;
  laneId @2 :UInt8;
  slotId @3 :UInt8;
  containerType @4 :UInt8;
  jobType @5 :UInt8;
}