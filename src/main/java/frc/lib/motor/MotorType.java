package frc.lib.motor;

public enum MotorType {
    kNeoVortex(true, ControllerBrand.kSparkFlex),
    kNeoV1(true, ControllerBrand.kSparkMax),
    kKrakenX60(true, ControllerBrand.kTalonFX),
    kKrakenX40(true, ControllerBrand.kTalonFX),
    kRedline(false, ControllerBrand.kSparkMax),
    kCim(false, ControllerBrand.kVictorSPX);


    public final boolean isBrushless;
    public final ControllerBrand controllerBrand;

    MotorType(boolean brushless, ControllerBrand controller) {
        isBrushless = brushless;
        controllerBrand = controller;
    }
}
