package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    /** NEO Vortex on SparkFlex */
    private final SparkFlex m_upperIntake;
    private double upperIntakeSpeedReference = 0;
    /** Redline on SparkMax */
    private final SparkMax m_lowerIntake;
    private double lowerIntakeSpeedReference = 0;

    private final IntakeArm m_arm = IntakeArm.getInstance();

    public IntakeSubsystem() {
        m_upperIntake = new SparkFlex(Constants.CanId.INTAKE_UPPER, MotorType.kBrushless);
        m_upperIntake.configure(
            new SparkFlexConfig()
                .apply(new LimitSwitchConfig()
                    .forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
                    .reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
                )
                .apply(new ClosedLoopConfig()
                    // TODO: Tune PIDs and Feedforward
                    .pid(0, 0, 0)
                    .apply(new FeedForwardConfig()
                        .sva(0, 0, 0)
                    )
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                )
                .idleMode(IdleMode.kBrake),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        // Do we need PID for redline?
        m_lowerIntake = new SparkMax(Constants.CanId.INTAKE_LOWER, MotorType.kBrushed);
        m_lowerIntake.configure(
            new SparkFlexConfig()
                .apply(new LimitSwitchConfig()
                    .forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
                    .reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
                )
                // I don't think we need a Closed Loop controller because this
                // is just movement, no setpoints.
                .inverted(true)
                .idleMode(IdleMode.kBrake),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    @Override
    public void periodic() {
        if (Constants.DebugLevel.isOrAll(Constants.DebugLevel.Intake))
            SmartDashboard.putData(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(this.getClass().getName());
        builder.addDoubleProperty("ArmPositionDeg", () -> m_arm.getAngle().in(Units.Degrees), null);
        builder.addDoubleProperty("ArmSetpointDeg", () -> m_arm.getSetpoint().in(Units.Degrees), null);
        builder.addDoubleProperty("ArmLeftCurrentAmps", () -> m_arm.getMasterOutputCurrent().in(Units.Amps), null);
        builder.addDoubleProperty("ArmRightCurrentAmps", () -> m_arm.getSlaveOutputCurrent().in(Units.Amps), null);
        builder.addDoubleProperty("IntakeUpperSpeedPercent", () -> upperIntakeSpeedReference, null);
        builder.addDoubleProperty("IntakeLowerSpeedPercent", () -> lowerIntakeSpeedReference, null);
    }

    private static double calculateLowerReferenceInRelationToTop(double setpoint) {
        AngularVelocity upperVelocity = Constants.Motors.NEO_MAX_VELOCITY.times(setpoint);
        AngularVelocity otherSide = upperVelocity.div(Constants.Intake.UPPER_GEAR_RATIO);
        double linearSpeedSurfaceSpeedInchesPerMinute = otherSide.in(Units.RPM) * 
            Constants.Intake.UPPER_WHEEL_CURCUMFERENCE.in(Units.Inches);
        double lowerMotorRpm = linearSpeedSurfaceSpeedInchesPerMinute *
            Constants.Intake.LOWER_WHEEL_CURCUMFERENCE.in(Units.Inches);
        double lowerRpmWithReducer = lowerMotorRpm * Constants.Intake.LOWER_GEAR_RATIO;
        double lowerReference = lowerRpmWithReducer / Constants.Motors.REDLINE_MAX_VELOCITY.in(Units.RPM);
        return lowerReference;
    }

    public void extendArm() {
        m_arm.rotateOut();
    }

    public void retractArm() {
        m_arm.rotateIn();
    }

    public void enableIntake() {
        final double speed = Constants.Intake.SPEED.in(Units.Value); // Value gives n/100
        upperIntakeSpeedReference = speed;
        lowerIntakeSpeedReference = calculateLowerReferenceInRelationToTop(speed);
        m_upperIntake.set(upperIntakeSpeedReference);
        m_lowerIntake.set(lowerIntakeSpeedReference);
    }

    public void enableReverse() {
        final double speed = Constants.Intake.SPEED.times(-1.0).in(Units.Value);
        upperIntakeSpeedReference = speed;
        lowerIntakeSpeedReference = calculateLowerReferenceInRelationToTop(speed);
        m_upperIntake.set(upperIntakeSpeedReference);
        m_lowerIntake.set(lowerIntakeSpeedReference);
    }

    public void disableIntake() {
        upperIntakeSpeedReference = lowerIntakeSpeedReference = 0;
        m_upperIntake.set(upperIntakeSpeedReference);
        m_lowerIntake.set(lowerIntakeSpeedReference);
    }

    public Command toggleIntake() {
        return Commands.startEnd(this::enableIntake, this::disableIntake, this);
    }

    public Command toggleArm() {
        return Commands.startEnd(this::extendArm, this::retractArm, this);
    }

    public Command toggleAll() {
        return Commands.startEnd(
            () -> { extendArm(); enableIntake(); },
            () -> { disableIntake(); retractArm(); },
            this 
        );
    }
}
