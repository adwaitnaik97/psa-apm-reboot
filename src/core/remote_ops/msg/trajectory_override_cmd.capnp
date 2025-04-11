@0x9e0611acb030daa7;

using P = import "path.capnp";

struct TrajectoryOverrideCmd {
  path @0 :P.Path; # Path to override with
}