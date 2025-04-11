@0xf9a1654bb2bfbda7;

struct AdjustPositionByDistanceCmd {
  commandType :union {
  distance @0 :Float64; # Signed distance to adjust vehicle by in meters
  safetyOverride @1 :Bool;
  }
}