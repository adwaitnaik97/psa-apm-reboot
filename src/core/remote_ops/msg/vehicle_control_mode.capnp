@0xe1e8b9720a932810;

struct VehicleControlMode {
    const autoMode :UInt8 = 0;
    const manualMode :UInt8 = 1;
    const interventionMode :UInt8 = 2;
    const emergencyMode :UInt8 = 3;
    const physicalPauseMode :UInt8 = 4;
    mode @0 :UInt8; # 0: auto, 1: manual, 2: intervention, 3: emergency, 4: physicalPause

    pauseResume @1 :Bool;
}