@0xd85fe77bd549d23e;
using import "marker.capnp".Marker;

struct MarkerArray {
    markers @0 :List(Marker);
}