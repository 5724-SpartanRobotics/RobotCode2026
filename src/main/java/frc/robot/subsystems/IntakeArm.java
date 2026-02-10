package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;

public class IntakeArm {
    private static IntakeArm instance = null;

    public static Angle kRotateBy = Units.Degrees.of(5.0);

    /** NEO Vortex on SparkFlex */
    private final SparkFlex m_masterLeft;
    /** NEO Vortex on SparkFlex */
    private final SparkFlex m_masterRight;
    private final SparkClosedLoopController m_masterLeftPID;
    private final RelativeEncoder m_masterLeftEncoder;
    private final RelativeEncoder m_slaveRightEncoder;

    private Angle setpoint = Units.Degrees.of(0);

    private IntakeArm() {
        m_masterLeft = new SparkFlex(Constants.CanId.ARM_LEFT_MASTER, MotorType.kBrushless);
        m_masterLeft.configure(
            new SparkFlexConfig()
                .apply(new LimitSwitchConfig()
                    .forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
                    .reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
                )
                .apply(new ClosedLoopConfig()
                    // TODO: Tune PIDs and Feedforward
                    .pid(0, 0, 0).iMaxAccum(0.1)
                    .apply(new FeedForwardConfig()
                        // There's no good way to convert a Feedforward constant to
                        // a kS or kA, you can do kV with some math, though.
                        .sva(0, 0, 0)
                    )
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                )
                .inverted(false)
                .idleMode(IdleMode.kBrake),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
        m_masterLeftPID = m_masterLeft.getClosedLoopController();
        m_masterLeftEncoder = m_masterLeft.getEncoder();

        m_masterRight = new SparkFlex(Constants.CanId.ARM_RIGHT_SLAVE, MotorType.kBrushless);
        m_masterRight.configure(
            new SparkFlexConfig()
                .apply(new LimitSwitchConfig()
                    .forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
                    .reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
                )
                .apply(new ClosedLoopConfig()
                    // TODO: Tune PIDs and Feedforward
                    .pid(0, 0, 0).iMaxAccum(0.1)
                    .apply(new FeedForwardConfig()
                        // There's no good way to convert a Feedforward constant to
                        // a kS or kA, you can do kV with some math, though.
                        .sva(0, 0, 0)
                    )
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                )
                .idleMode(IdleMode.kBrake)
                // TODO: IDK if this is correct
                .follow(m_masterLeft, true),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
        m_slaveRightEncoder = m_masterRight.getEncoder();
    }

    public static IntakeArm getInstance() {
        if (instance == null) instance = new IntakeArm();
        return instance;
    }

    public boolean rotateIn() {
        setpoint = Constants.Intake.Arm.MIN_ROTATION;
        double reference = setpoint.in(Units.Degrees) * Constants.Motors.NEO_COUNTS_PER_REVOLUTION *
            Constants.Intake.Arm.GEAR_RATIO / 360.0;
        return m_masterLeftPID.setSetpoint(reference, ControlType.kMAXMotionPositionControl).equals(REVLibError.kOk);
    }

    public boolean rotateOut() {
        setpoint = Constants.Intake.Arm.MAX_ROTATION;
        double reference = setpoint.in(Units.Degrees) * Constants.Motors.NEO_COUNTS_PER_REVOLUTION *
            Constants.Intake.Arm.GEAR_RATIO / 360.0;
        return m_masterLeftPID.setSetpoint(reference, ControlType.kMAXMotionPositionControl).equals(REVLibError.kOk);
    }

    public boolean rotateTo(Angle setpoint) {
        double setpointDeg = clampAngle(
            setpoint,
            Constants.Intake.Arm.MIN_ROTATION,
            Constants.Intake.Arm.MAX_ROTATION
        ).in(Units.Degrees);
        double reference = setpointDeg * Constants.Motors.NEO_COUNTS_PER_REVOLUTION *
            Constants.Intake.Arm.GEAR_RATIO / 360.0;
        return m_masterLeftPID.setSetpoint(reference, ControlType.kMAXMotionPositionControl).equals(REVLibError.kOk);
    }
    
    public void rotateAt(double speed) {
        stop();
        m_masterLeft.set(MathUtil.clamp(speed, -1.0, 1.0));
    }

    public boolean increment() {
        Angle newSetpoint = clampAngle(
            setpoint.plus(kRotateBy),
            Constants.Intake.Arm.MIN_ROTATION,
            Constants.Intake.Arm.MAX_ROTATION
        );
        setpoint = newSetpoint;
        double reference = setpoint.in(Units.Degrees) * Constants.Motors.NEO_COUNTS_PER_REVOLUTION *
            Constants.Intake.Arm.GEAR_RATIO / 360.0;
        return m_masterLeftPID.setSetpoint(reference, ControlType.kMAXMotionPositionControl).equals(REVLibError.kOk);
    }

    public boolean decrement() {
        Angle newSetpoint = clampAngle(
            setpoint.minus(kRotateBy),
            Constants.Intake.Arm.MIN_ROTATION,
            Constants.Intake.Arm.MAX_ROTATION
        );
        setpoint = newSetpoint;
        double reference = setpoint.in(Units.Degrees) * Constants.Motors.NEO_COUNTS_PER_REVOLUTION *
            Constants.Intake.Arm.GEAR_RATIO / 360.0;
        return m_masterLeftPID.setSetpoint(reference, ControlType.kMAXMotionPositionControl).equals(REVLibError.kOk);
    }
    
    public boolean stop() {
        double reference = m_masterLeftPID.getSetpoint();
        return m_masterLeftPID.setSetpoint(reference, ControlType.kMAXMotionPositionControl).equals(REVLibError.kOk);
    }

    /** Get the actual angle of the master (left) motor. */
    public Angle getAngle() {
        return getMasterAngle();
    }

    public Angle getMasterAngle() {
        double rotations = m_masterLeftEncoder.getPosition();
        return Units.Degrees.of(rotations / Constants.Intake.Arm.GEAR_RATIO * 360.0);
    }

    public Angle getSlaveAngle() {
        double rotations = m_slaveRightEncoder.getPosition();
        return Units.Degrees.of(rotations / Constants.Intake.Arm.GEAR_RATIO * 360.0);
    }

    public Current getMasterOutputCurrent() {
        return Units.Amps.of(m_masterLeft.getOutputCurrent());
    }

    public Current getSlaveOutputCurrent() {
        return Units.Amps.of(m_masterRight.getOutputCurrent());
    }

    public Angle getSetpoint() {
        return setpoint;
    }

    private Angle clampAngle(Angle a, Angle min, Angle max) {
        double clampedDeg = MathUtil.clamp(
            a.in(Units.Degrees),
            min.in(Units.Degrees),
            max.in(Units.Degrees)
        );
        return Units.Degrees.of(clampedDeg);
    }
}
