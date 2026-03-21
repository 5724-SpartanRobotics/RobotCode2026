package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
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
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.LoggedSlewRateLimiter;
import frc.lib.spark.SparkIO_SparkFlex;
import frc.robot.info.Motors;
import frc.robot.info.constants.CanIdConstants;
import frc.robot.info.constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeArmIO.IntakeArmIOInputs;

public class IntakeArm {
	private static final double kIMaxAccum = 0.1;
	private static final Current kMaxCurrent = Units.Amps.of(60);
	private static IntakeArm instance = null;

	public static Angle kRotateBy = Units.Degrees.of(5.0);

	/** NEO Vortex on SparkFlex */
	private final SparkIO_SparkFlex m_masterLeft;
	/** NEO Vortex on SparkFlex */
	private final SparkIO_SparkFlex m_slaveRight;
	private final RelativeEncoder m_masterLeftEncoder;
	private final RelativeEncoder m_slaveRightEncoder;
	private final LoggedSlewRateLimiter m_ramp;

	private Angle setpoint = Units.Degrees.of(0);
	private double rampedSetpointDegrees = 0;
	private double motorSetpointRotations = 0;
	private double lastPositionDeg = 0;

	private final IntakeArmIOInputs inputs = new IntakeArmIOInputs();

	private IntakeArm() {
		m_masterLeft = new SparkIO_SparkFlex(CanIdConstants.ARM_LEFT_MASTER, MotorType.kBrushless);
		m_masterLeft.configure(
			new SparkFlexConfig()
				.apply(new LimitSwitchConfig()
					.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
					.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor))
				.apply(new ClosedLoopConfig()
					// TODO: Tune PIDs and Feedforward
					.pidf(
						IntakeConstants.Arm.PIDF.kP(),
						IntakeConstants.Arm.PIDF.kI(),
						IntakeConstants.Arm.PIDF.kD(),
						IntakeConstants.Arm.PIDF.kFf())
					.iMaxAccum(kIMaxAccum)
					.apply(new FeedForwardConfig()
						// There's no good way to convert a Feedforward constant to
						// a kS or kA, you can do kV with some math, though.
						.sva(
							IntakeConstants.Arm.PIDF.kFfS(),
							IntakeConstants.Arm.PIDF.kFfV(),
							IntakeConstants.Arm.PIDF.kFfA()))
					.feedbackSensor(FeedbackSensor.kPrimaryEncoder))
				.inverted(false)
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit((int) kMaxCurrent.in(Units.Amps)),
			ResetMode.kResetSafeParameters,
			PersistMode.kNoPersistParameters);
		m_masterLeftEncoder = m_masterLeft.getEncoder();

		m_slaveRight = new SparkIO_SparkFlex(CanIdConstants.ARM_RIGHT_SLAVE, MotorType.kBrushless);
		m_slaveRight.configure(
			new SparkFlexConfig()
				.apply(new LimitSwitchConfig()
					.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
					.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor))
				.apply(new ClosedLoopConfig()
					// TODO: Tune PIDs and Feedforward
					.pidf(
						IntakeConstants.Arm.PIDF.kP(),
						IntakeConstants.Arm.PIDF.kI(),
						IntakeConstants.Arm.PIDF.kD(),
						IntakeConstants.Arm.PIDF.kFf())
					.iMaxAccum(kIMaxAccum)
					.apply(new FeedForwardConfig()
						// There's no good way to convert a Feedforward constant to
						// a kS or kA, you can do kV with some math, though.
						.sva(
							IntakeConstants.Arm.PIDF.kFfS(),
							IntakeConstants.Arm.PIDF.kFfV(),
							IntakeConstants.Arm.PIDF.kFfA()))
					.feedbackSensor(FeedbackSensor.kPrimaryEncoder))
				.idleMode(IdleMode.kBrake)
				// TODO: IDK if this is correct
				.follow(m_masterLeft, true)
				.smartCurrentLimit((int) kMaxCurrent.in(Units.Amps)),
			ResetMode.kResetSafeParameters,
			PersistMode.kNoPersistParameters);
		m_slaveRightEncoder = m_slaveRight.getEncoder();

		m_ramp = new LoggedSlewRateLimiter(this.getClass().getName(),
			Units.DegreesPerSecond.of(45).in(Units.DegreesPerSecond));
	}

	public static synchronized IntakeArm getInstance() {
		if (instance == null)
			instance = new IntakeArm();
		return instance;
	}

	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Arm Setpoint Degrees", () -> setpoint.in(Units.Degrees), null);
		builder.addDoubleProperty("Arm Ramped Setpoint Degrees", () -> rampedSetpointDegrees, null);
		builder.addDoubleProperty("Arm Ramped Setpoint Motor Rotations",
			() -> motorSetpointRotations, null);
	}

	public void periodic() {
		double limitedDegrees = m_ramp.calculate(setpoint.in(Units.Degrees));
		rampedSetpointDegrees = limitedDegrees;
		double reference = limitedDegrees *
			Motors.NEO_COUNTS_PER_REVOLUTION *
			IntakeConstants.Arm.GEAR_RATIO / 360.0;
		motorSetpointRotations = reference;
		m_masterLeft.setPosition(reference, true, false);

		log();
		Logger.recordOutput(
			"Intake/Arm/Stalled",
			Math.abs(inputs.errorDeg) > 5 && inputs.masterCurrentAmps > 40);
		double velocityDegPerSec = (inputs.positionDeg - lastPositionDeg) / 0.02;

		Logger.recordOutput("Intake/Arm/VelocityDegPerSec", velocityDegPerSec);
		lastPositionDeg = inputs.positionDeg;
		Logger.recordOutput(
			"Intake/Arm/LeftRightErrorDeg",
			inputs.positionDeg - inputs.slavePositionDeg);
	}

	public void log() {
		inputs.positionDeg = getMasterAngle().in(Units.Degrees);
		inputs.slavePositionDeg = getSlaveAngle().in(Units.Degrees);

		inputs.setpointDeg = setpoint.in(Units.Degrees);
		inputs.rampedSetpointDeg = rampedSetpointDegrees;

		inputs.motorSetpointRotations = motorSetpointRotations;

		inputs.masterCurrentAmps = getMasterOutputCurrent().in(Units.Amps);
		inputs.slaveCurrentAmps = getSlaveOutputCurrent().in(Units.Amps);

		inputs.errorDeg = inputs.setpointDeg - inputs.positionDeg;
		inputs.atSetpoint = Math.abs(inputs.errorDeg) < 2.0;
	}

	public IntakeArmIOInputs getInputs() {
		return inputs;
	}

	public void rotateIn() {
		setpoint = IntakeConstants.Arm.MIN_ROTATION;
	}

	public void rotateOut() {
		setpoint = IntakeConstants.Arm.MAX_ROTATION;
	}

	public void rotateTo(Angle setpoint) {
		double setpointDeg = clampAngle(
			setpoint,
			IntakeConstants.Arm.MIN_ROTATION,
			IntakeConstants.Arm.MAX_ROTATION).in(Units.Degrees);
		setpoint = Units.Degrees.of(setpointDeg);
	}

	public void rotateAt(double speed) {
		stop();
		m_masterLeft.set(MathUtil.clamp(speed, -1.0, 1.0));
	}

	public void increment() {
		Angle newSetpoint = clampAngle(
			setpoint.plus(kRotateBy),
			IntakeConstants.Arm.MIN_ROTATION,
			IntakeConstants.Arm.MAX_ROTATION);
		setpoint = newSetpoint;
	}

	public void decrement() {
		Angle newSetpoint = clampAngle(
			setpoint.minus(kRotateBy),
			IntakeConstants.Arm.MIN_ROTATION,
			IntakeConstants.Arm.MAX_ROTATION);
		setpoint = newSetpoint;
	}

	public void stop() {
		m_masterLeft.stopMotor();
	}

	/** Get the actual angle of the master (left) motor. */
	public Angle getAngle() {
		return getMasterAngle();
	}

	public Angle getMasterAngle() {
		double rotations = m_masterLeftEncoder.getPosition();
		return Units.Degrees.of(rotations / IntakeConstants.Arm.GEAR_RATIO * 360.0);
	}

	public Angle getSlaveAngle() {
		double rotations = m_slaveRightEncoder.getPosition();
		return Units.Degrees.of(rotations / IntakeConstants.Arm.GEAR_RATIO * 360.0);
	}

	public Current getMasterOutputCurrent() {
		return Units.Amps.of(m_masterLeft.getOutputCurrent());
	}

	public Current getSlaveOutputCurrent() {
		return Units.Amps.of(m_slaveRight.getOutputCurrent());
	}

	public Angle getSetpoint() {
		return setpoint;
	}

	private Angle clampAngle(Angle a, Angle min, Angle max) {
		double clampedDeg = MathUtil.clamp(
			a.in(Units.Degrees),
			min.in(Units.Degrees),
			max.in(Units.Degrees));
		return Units.Degrees.of(clampedDeg);
	}
}
