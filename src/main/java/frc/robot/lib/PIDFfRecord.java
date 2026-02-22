package frc.robot.lib;

/**
 * @param kP Proportion
 * @param kI Integral
 * @param kD Derivative
 * @param kFf Feedforward
 * @param kFfS Feedforward Static gain (Volts)
 * @param kFfV Feedforward Velogity gain (Volts/v, default V/RPM)
 * @param kFfA Feedforward Acceleration gain (Volts/v/s, default V/RPM/s)
 */
public record PIDFfRecord(
    double kP, double kI, double kD, double kFf,
    double kFfS, double kFfV, double kFfA
) {}