@0xf21670dc9dc1e632;

struct EgoState
{
    const signalLightBothOff :UInt8 = 0;
    const signalLightRightOn :UInt8 = 1;
    const signalLightLeftOn :UInt8 = 2;
    const signalLightBothOn :UInt8 = 3; #hazard 
    signalLight @0 :UInt8; #can be any of the above signalLight* constants

    vehPositionX @1 :Float32;
    vehPositionY @2 :Float32;
    vehYaw @3 :Float32;

    trailerPositionX @4 :Float32;
    trailerPositionY @5 :Float32;
    trailerYaw @6 :Float32;

    const gearStatusReverse :UInt8 = 2;
    const gearStatusNeutral :UInt8 = 3;
    const gearStatusDrive :UInt8 = 4;
    gearStatus @7 :UInt8; #can be any of the above gear* constants

    parkingBrakeStatus @8 :Bool;

    brakePercentage @9 :Float32; #brake pressure %

    velocity @10 :Float32; #m/s

    steeringAngle @11 :Float32; #degrees

    remoteEmergencyButtonStatus @12 :Bool;

    const localizationStatusNormal :UInt8 = 0;
    const localizationStatusDegraded :UInt8 = 1;
    const localizationStatusCritical :UInt8 = 2;
    localizationStatus @13 :UInt8; #can be any of the above localizationStatus* constants

    const headLightsOff :UInt8 = 0;
    const headLightsLowBeam :UInt8 = 1;
    const headLightsHighBeam :UInt8 = 2;
    headLights @14 :UInt8; #can be any of the above headLights* constants

    observedSpeedLimit @15 :Float32; #m/s
    
    inLaneChange @16 :Bool;

    throttlePercentage @17 :Float32; #throttle pressure %

    hornCmdFeedback @18 :Bool; # horn command state

    inSSA @19 :Bool; # in ssa state
    
    gpsInMapX @20 :Float32; # gps x-coordinate from vehicle
    gpsInMapY @21 :Float32; # gps y-coordinate from vehicle

    precedenceOverrideInProgress @22 :Bool; # Override is in progress, used for feedback
    passedPointOfNoReturn @23 :Bool; # For feedback that vehicle will no longer consider stopline at intersection
}
