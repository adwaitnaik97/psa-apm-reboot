@0x9a8b5b4ce0c590a7;

struct TrafficLight
{
    const green :UInt8 = 0;
    const amber :UInt8 = 1;
    const red :UInt8 = 2;
    const rightArrow :UInt8 = 3;
    const unknown :UInt8 = 99;
    color @0 :UInt8;
}