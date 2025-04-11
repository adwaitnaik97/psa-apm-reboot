@0xc26d75f8175ce741;

struct NudgeForwardCmd {
  engage @0 :UInt8; # [1 - Vehicle should nudge forward]
  nudgeInstanceId @1 :UInt8; 
}