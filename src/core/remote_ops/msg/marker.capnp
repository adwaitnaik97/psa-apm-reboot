@0xc5742ab6e2b51977;

struct Marker {
    # roughly corresponds to ros visualization_msgs::Marker type

    stamp @0 :UInt64;
    id @1 :Int32;
    type @2 :UInt8;
    action @3 :UInt8;
    
    positionX @4 :Float32;
    positionY @5 :Float32;
    positionZ @6 :Float32;

    orientationX @7 :Float32;
    orientationY @8 :Float32;
    orientationZ @9 :Float32;
    orientationW @10 :Float32;

    scaleX @11 :Float32;
    scaleY @12 :Float32;
    scaleZ @13 :Float32;

    colorR @14 :Float32;
    colorG @15 :Float32;
    colorB @16 :Float32;
    colorA @17 :Float32;

    points @18 :List(Float32); # list of x1,y1,z1,x2,y2,z2,x3,y3...

    text @19 :Text;

    ns @20 :Text;
}