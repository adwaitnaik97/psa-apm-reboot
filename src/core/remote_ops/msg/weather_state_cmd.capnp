@0xebd22b2b23bcd810;

struct WeatherStateCmd {
  state @0 :UInt8; # [0 - dry] [1 - Wet] [2- Drizzle Rain][3 - Heavy Rain] [4 - Heavy Fog]
}