@0x90e17369ff836d66;

struct Transform {
    stamp @0 :UInt64;

    translationX @1 :Float32;
    translationY @2 :Float32;
    translationZ @3 :Float32;

    rotationX @4 :Float32;
    rotationY @5 :Float32;
    rotationZ @6 :Float32;
    rotationW @7 :Float32;

    frameId @8 :Text;
    childFrameId @9 :Text;
}