@0xf3540b4b55bb25a7;

struct PCD
{
#schema for point clouds

    stamp @0 :UInt64;
    #timestamp

    includesZ @1 :Bool;
    #indicates whether points in the payload includes values

    includesIntensity @2 :Bool;
    #indicates whether points in the payload includes values

    payload @3 :List(Float32);
    #direct list of raw floats forming the pointcloud
}