@0x880e3b3ff58d24b0;

struct Path {
    timestamp @0 :UInt64;

    segmentId @1 :List(UInt32);

    discretizedPathPointsX @2 :List(Float32);
    discretizedPathPointsY @3 :List(Float32);

    discretizedPathPointsYaw @4 :List(Float32);
}