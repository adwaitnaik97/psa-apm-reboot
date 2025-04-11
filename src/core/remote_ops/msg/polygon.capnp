@0x8ff102e2f323f748;

struct Polygon
{
#schema for polygon

    stamp @0 :UInt64;
    #timestamp

    includesZ @1 :Bool;
    #indicates whether points in the payload includes values

    payload @2 :List(Float32);
    #direct list of raw floats forming the polygon
}