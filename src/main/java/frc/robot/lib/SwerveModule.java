package frc.robot.lib;

import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiConsumer;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Robot;

public class SwerveModule {
	public enum MotorType {
		VortexSparkFlex,
		Falcon500
	}

	public enum Report {
		All(0x111),
		AbsolutePosition(0x001),
		MotorEncoderPosition(0x010),
		MotorCurrent(0x100);

		public final int value;
		Report(int v) { this.value = v; }
	}

	private static int moduleIndex = 0;

	private String _moduleName;
	private MotorType _motorType;
	private SwerveModuleConstantsRecord _constants;
	/** -1.0 if inverted, 1 if not inverted */
	private double _inverted = 0;
	private double _driveSpeed = 0.0;
	private double _driveAngle = 0.0;
	private int _moduleNumber;

	private ReentrantLock _debugLock = new ReentrantLock();
	private AtomicBoolean _debug = new AtomicBoolean(true);
	private double _createdTimestamp;

	private CANcoder m_encoder;

	private TalonFX m_driveFalcon;
	private TalonFX m_turnFalcon;
	private TalonFXConfiguration m_falconConfig;
	private PositionVoltage m_falconTurnPositionVoltage;

	private SparkFlex m_driveSparkFlex;
	private SparkFlex m_turnSparkFlex;
	private SparkBaseConfig m_sparkConfig;
	private SparkClosedLoopController m_sparkTurnPID;

	/**
	 * NT4 Raw Absolute Angle publisher for the absolute encoder. Radians.
	 */
	private final DoublePublisher m_rawAbsoluteAnglePublisher;
	/**
	 * NT4 Adjusted Absolute angle publisher for the absolute encoder. Radians.
	 */
	private final DoublePublisher m_adjAbsoluteAnglePublisher;
	/**
	 * NT4 raw angle motor. Degrees.
	 */
	private final DoublePublisher m_rawAnglePublisher;
	/**
	 * NT4 Raw drive motor. Degrees.
	 */
	private final DoublePublisher m_rawDriveEncoderPublisher;
	/**
	 * NT4 Raw drive motor. RPM.
	 */
	private final DoublePublisher m_rawDriveVelocityPublisher;
	/**
	 * Speed setpoint publisher for the module motor-controller PID.
	 */
	private final DoublePublisher m_speedSetpointPublisher;
	/**
	 * Angle setpoint publisher for the module motor-controller PID.
	 */
	private final DoublePublisher m_angleSetpointPublisher;

	/**
	 * Create a new SwerveModule. Debugging is enabled by default; use <code>setDebug(false)</code> to disable.
	 * @param name Name of the module, eg., FL.
	 * @param motorType {@link MotorType} that will be used for <b>both</b> turn and drive.
	 */
	public SwerveModule(
		String name,
		MotorType motorType,
		SwerveModuleConstantsRecord constants
	)
	throws Exception
	{
		this._createdTimestamp = Timer.getFPGATimestamp();
		this._moduleName = name;
		this._motorType = motorType;
		this._constants = constants;
		this._inverted = this._constants.invert() ? -1.0 : 1.0;
		this._moduleNumber = moduleIndex;
		moduleIndex++;

		this.m_encoder = new CANcoder(this._constants.encoderId());

		switch (this._motorType) {
			case VortexSparkFlex:
				this.setupVortex();
				break;
			case Falcon500:
				this.setupFalcon();
				break;
			default:
				throw new Exception("That motor type isn't supported!");
		}

		this.applyTurnConfiguration();
		if (isVortex()) this.m_sparkTurnPID = this.m_turnSparkFlex.getClosedLoopController();

		this.resetEncoders();
		this.resetTurnToAbsolute();

		m_rawAbsoluteAnglePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
			"swerve/modules/" + this._moduleName + "/Raw Absolute Encoder").publish();
		m_adjAbsoluteAnglePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
			"swerve/modules/" + this._moduleName + "/Adjusted Absolute Encoder").publish();
		m_rawAnglePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
			"swerve/modules/" + this._moduleName + "/Raw Angle Encoder").publish();
		m_rawDriveEncoderPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
			"swerve/modules/" + this._moduleName + "/Raw Drive Encoder").publish();
		m_rawDriveVelocityPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
			"swerve/modules/" + this._moduleName + "/Raw Drive Velocity").publish();
		m_speedSetpointPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
			"swerve/modules/" + this._moduleName + "/Speed Setpoint").publish();
		m_angleSetpointPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
			"swerve/modules/" + this._moduleName + "/Angle Setpoint").publish();
	}

	private boolean isVortex() {
		return this._motorType == MotorType.VortexSparkFlex;
	}

	private void setupVortex() {
		this.m_driveSparkFlex = new SparkFlex(
			_constants.driveId(),
			com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
		);
		this.m_turnSparkFlex = new SparkFlex(
			_constants.turnId(),
			com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
		);
		this.m_sparkConfig = new SparkFlexConfig()
			.inverted(false)
			.apply(new SoftLimitConfig()
				.forwardSoftLimitEnabled(false)
				.reverseSoftLimitEnabled(false)
			)
			.apply(new LimitSwitchConfig()
				.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
				.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
			)
			.apply(new ClosedLoopConfig()
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pid(_constants.pid().getP(), _constants.pid().getI(), _constants.pid().getD())
			)
			.idleMode(IdleMode.kBrake);
		this.m_driveSparkFlex.configure(this.m_sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	private void setupFalcon() {
		this.m_turnFalcon = new TalonFX(_constants.turnId());
		this.m_falconTurnPositionVoltage = new PositionVoltage(0);
		this.m_driveFalcon = new TalonFX(_constants.driveId());
		this.m_driveFalcon.getConfigurator().apply(new TalonFXConfiguration());
		this.m_driveFalcon.setNeutralMode(NeutralModeValue.Brake);
		this.m_falconConfig = new TalonFXConfiguration()
			.withFeedback(new FeedbackConfigs()
				.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
			)
			.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
				.withForwardSoftLimitEnable(false)
				.withReverseSoftLimitEnable(false)
			)
			.withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
				.withForwardLimitEnable(false)
				.withReverseLimitEnable(false)
			)
			.withSlot0(new Slot0Configs()
				.withKV(0.0)
				.withKP(_constants.pid().getP()).withKI(_constants.pid().getI()).withKD(_constants.pid().getD())
			)
			.withVoltage(new VoltageConfigs()
				.withPeakForwardVoltage(Units.Volts.of(10.0))
				.withPeakReverseVoltage(Units.Volts.of(10.0).unaryMinus())
			);
	}

	private void resetEncoders() {
		if (isVortex()) {
			m_driveSparkFlex.getEncoder().setPosition(0);
			double absoluteEncoderAngle =
				(m_encoder.getAbsolutePosition().refresh().getValueAsDouble() -
					_constants.encoderOffset()) *
				Constants.TWO_PI * 1.0; // encoder not reversed
			m_turnSparkFlex.getEncoder().setPosition(
				absoluteEncoderAngle / (Constants.TWO_PI / Drive.SWERVE_TURN_GEAR_RATIO)
			);
		}
	}

	private void applyTurnConfiguration() {
		if (isVortex()) {
			REVLibError status;
			for (int i = 0; i < 6; i++) {
				status = m_turnSparkFlex.configure(
					m_sparkConfig,
					ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters
				);

				if (status.equals(REVLibError.kOk)) break;
				else SmartDashboard.putString(
					nameSmartDashboardTopicKey("Configure Error"),
					status.toString()
				);
			}
		} else {
			StatusCode status1, status2;
			for (int i = 0; i < 6; ++i) {
				status1 = this.m_turnFalcon.getConfigurator().apply(m_falconConfig);
				status2 = this.m_driveFalcon.setNeutralMode(NeutralModeValue.Brake);

				if (status1.isOK() && status2.isOK()) break;
				if(!status1.isOK()) {
					SmartDashboard.putString(
						nameSmartDashboardTopicKey("Configure Error (application)"),
						status1.toString()
					);
				}
				if (!status2.isOK()) {
					SmartDashboard.putString(
						nameSmartDashboardTopicKey("Configure Error (neutral mode)"),
						status2.toString()
					);
				}
			}
		}
	}

	private String nameSmartDashboardTopicKey(String name) {
		return String.format("Swerve Modules/%s/%s", this._moduleName, name);
	}

	private void reportAbsolutePosition() {
		if (!this._debug.get()) return;
		SmartDashboard.putNumber(
			nameSmartDashboardTopicKey("Encoder Position (Degrees)"),
			m_encoder.getAbsolutePosition().refresh().getValue().in(Units.Degrees)
		);
	}

	private void reportMotorEncoderPosition() {
		if (!this._debug.get()) return;
		double drivePosition = (isVortex() ?
			ConversionLib.vortexToMeters(m_driveSparkFlex.getEncoder().getPosition()) :
			ConversionLib.falconToMeters(m_driveFalcon.getPosition().getValueAsDouble())
		) * _inverted;
		double turnPosition = isVortex() ?
			m_turnSparkFlex.getEncoder().getPosition() :
			m_turnFalcon.getPosition().getValueAsDouble();

		SmartDashboard.putNumber(
			nameSmartDashboardTopicKey("Drive Motor Encoder (Rotations)"),
			drivePosition
		);
		SmartDashboard.putNumber(
			nameSmartDashboardTopicKey("Turn Motor Encoder (Rotations)"),
			turnPosition
		);
	}

	private void reportMotorCurrent() {
		if (!this._debug.get()) return;
		SmartDashboard.putNumber(
			nameSmartDashboardTopicKey("Drive Motor Current (Amps)"),
			isVortex() ?
				m_driveSparkFlex.getOutputCurrent() :
				m_driveFalcon.getTorqueCurrent().getValue().in(Units.Amps)
		);
		SmartDashboard.putNumber(
			nameSmartDashboardTopicKey("Turn Motor Current (Amps)"),
			isVortex() ?
				m_turnSparkFlex.getOutputCurrent() :
				m_turnFalcon.getTorqueCurrent().getValue().in(Units.Amps)
		);
	}

	public void report(EnumSet<Report> reportSet) {
		for ( Report r : reportSet ) {
			switch (r) {
				case All:
					reportAbsolutePosition();
					reportMotorEncoderPosition();
					reportMotorCurrent();
					break;
				case AbsolutePosition: reportAbsolutePosition(); break;
				case MotorEncoderPosition: reportMotorEncoderPosition(); break;
				case MotorCurrent: reportMotorCurrent(); break;
				default: break;
			}
		}
	}

	public void report(EnumSet<Report> reportSet, boolean force) {
		boolean debugState = this._debug.get();
		this.setDebug(true);
		report(reportSet);
		this.setDebug(debugState);
	}

	public void report(Report report) {
		report(EnumSet.of(report));
	}

	public void report(Report report, boolean force) {
		report(EnumSet.of(report), force);
	}
	
	public void resetTurnToAbsolute() {
		double absolutePosition = m_encoder.getAbsolutePosition().refresh().getValue().in(Units.Radians);
		double toZero = absolutePosition - _constants.encoderOffset();
		double absolutePosition_MotorUnits = isVortex() ?
			ConversionLib.radiansToVortex(toZero) :
			ConversionLib.radiansToFalcon(toZero);
		
		if (Constants.DebugLevel.isOrAll(Constants.DebugLevel.Drive) || this._debug.get()) {
			SmartDashboard.putNumber(
				nameSmartDashboardTopicKey("Initial Absolute Position (Motor counts)"),
				absolutePosition_MotorUnits
			);
		}
		
		if (isVortex()) m_turnSparkFlex.getEncoder().setPosition(-1.0 * absolutePosition_MotorUnits);
		else m_turnFalcon.setPosition(1.0 * absolutePosition_MotorUnits);
	}

	/**
	 * @param desiredState
	 * @param telemetryCallback - <b><i>BE SURE TO END THE CONTROL CYCLE IN THIS CALLBACK<i></b>
	 */
	public void setDesiredState(SwerveModuleState desiredState, BiConsumer<Integer, SwerveModuleState> telemetryCallback) {
		final double maxRobotSpeedmps = Robot.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond);
		desiredState = CTREModuleState.optimize(desiredState, getState().angle);
		double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxRobotSpeedmps * 0.01)) ?
			_driveAngle : desiredState.angle.getRadians();
		_driveSpeed = desiredState.speedMetersPerSecond / maxRobotSpeedmps;
		_driveAngle = Units.Radians.of(angle).baseUnitMagnitude();

		if (Constants.DebugLevel.isOrAll(Constants.DebugLevel.Drive) || this._debug.get()) {
			SmartDashboard.putNumber(nameSmartDashboardTopicKey("Drive Motor Reference"), _driveSpeed);
			SmartDashboard.putNumber(nameSmartDashboardTopicKey("Turn Motor Reference"), _driveAngle);
		}

		if (isVortex()) {
			m_driveSparkFlex.set(_driveSpeed * _inverted);
			m_sparkTurnPID.setSetpoint(-1.0 * ConversionLib.radiansToVortex(angle), ControlType.kPosition);
		} else {
			m_driveFalcon.set(_driveSpeed);
			m_turnFalcon.setControl(m_falconTurnPositionVoltage.withPosition(
				-1.0 * ConversionLib.radiansToFalcon(angle)
			));
		}

		m_speedSetpointPublisher.set(desiredState.speedMetersPerSecond);
		m_angleSetpointPublisher.set(desiredState.angle.getDegrees());
		telemetryCallback.accept(this._moduleNumber, desiredState);
	}

	public double getCreatedTimestamp() {
		return this._createdTimestamp;
	}

	public SwerveModuleConstantsRecord getModuleConstants() {
		return this._constants;
	}

	/**
	 * Gets the module's number. Module 0 should correspond to FL, and 3 to FR (move counterclockwise).
	 * If it would help, see <a href="https://web.archive.org/web/20260127232043/https://docs.yagsl.com/~gitbook/image?url=https%3A%2F%2F567506766-files.gitbook.io%2F%7E%2Ffiles%2Fv0%2Fb%2Fgitbook-x-prod.appspot.com%2Fo%2Fspaces%252F754c0Fpq8fBi6k4ByS1k%252Fuploads%252FP8kIHNspDzTMqBiHw1U1%252Fimage.png%3Falt%3Dmedia%26token%3D9923865e-47c7-4b1e-90a5-9da9610c0764&width=400&dpr=3&quality=100&sign=6a519e95&sv=2">
	 * this diagram from YAGSL</a>.
	 * @return The module number, 0 index.
	 */
	public int getModuleNumber() {
		return this._moduleNumber;
	}

	public SwerveModuleState getState() {
		double velocity = isVortex() ? m_driveSparkFlex.get() : m_driveFalcon.get();
		Rotation2d angle = Rotation2d.fromDegrees(
			isVortex() ?
				ConversionLib.vortexToDegrees(-1.0 * m_turnSparkFlex.getEncoder().getPosition()) :
				ConversionLib.falconToDegrees(-1.0 * m_turnFalcon.getPosition().getValueAsDouble())
		);
		return new SwerveModuleState(velocity, angle);
	}

	public SwerveModulePosition getPosition() {
		double drivePosition = (isVortex() ?
			ConversionLib.vortexToMeters(m_driveSparkFlex.getEncoder().getPosition()) :
			ConversionLib.falconToMeters(m_driveFalcon.getPosition().getValueAsDouble())
		) * _inverted;
		double turnPosition = isVortex() ?
			m_turnSparkFlex.getEncoder().getPosition() :
			m_turnFalcon.getPosition().getValueAsDouble();

		Rotation2d angle = Rotation2d.fromDegrees(
			isVortex() ?
				ConversionLib.vortexToDegrees(-1.0 * turnPosition) :
				ConversionLib.falconToDegrees(-1.0 * turnPosition)
		);
		return new SwerveModulePosition(drivePosition, angle);
	}

	public void updateTelemetry() {
		m_rawAbsoluteAnglePublisher.set(
			this.m_encoder.getAbsolutePosition().getValue().in(Units.Radians)
		);
		m_rawAnglePublisher.set(
			isVortex() ?
				m_turnSparkFlex.getEncoder().getPosition() * 360.0 :
				m_turnFalcon.getPosition().getValue().in(Units.Degrees)
		);
		m_rawDriveEncoderPublisher.set(
			isVortex() ?
				m_driveSparkFlex.getEncoder().getPosition() :
				m_driveFalcon.getPosition().getValueAsDouble()
		);
		m_rawDriveVelocityPublisher.set(
			isVortex() ?
				m_driveSparkFlex.getEncoder().getVelocity() :
				m_driveFalcon.getVelocity().getValue().in(Units.RPM)
		);
		m_adjAbsoluteAnglePublisher.set(
			this.m_encoder.getAbsolutePosition().getValue().in(Units.Radians) - this._constants.encoderOffset()
		);
	}

	public SwerveModule setDebug(boolean debug) {
		_debugLock.lock();
		try {
			this._debug.set(debug);
		} finally {
			_debugLock.unlock();
		}
		return this;
	}

	protected class CTREModuleState {
		/**
		 * Minimize the change in heading the desired swerve module state would require by potentially
		 * reversing the direction the wheel spins. Customized from WPILib's version to include placing
		 * in appropriate scope for CTRE onboard control.
		 *
		 * @param desiredState The desired state.
		 * @param currentAngle The current module angle.
		 */
		public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
			double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
			double targetSpeed = desiredState.speedMetersPerSecond;
			double delta = targetAngle - currentAngle.getDegrees();
			if (Math.abs(delta) > 90){
				targetSpeed = -targetSpeed;
				targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
			}		
			return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
		}

		/**
		* @param scopeReference Current Angle
		* @param newAngle Target Angle
		* @return Closest angle within scope
		*/
		private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
			double lowerBound;
			double upperBound;
			double lowerOffset = scopeReference % 360;
			if (lowerOffset >= 0) {
				lowerBound = scopeReference - lowerOffset;
				upperBound = scopeReference + (360 - lowerOffset);
			} else {
				upperBound = scopeReference - lowerOffset;
				lowerBound = scopeReference - (360 + lowerOffset);
			}
			while (newAngle < lowerBound) {
				newAngle += 360;
			}
			while (newAngle > upperBound) {
				newAngle -= 360;
			}
			if (newAngle - scopeReference > 180) {
				newAngle -= 360;
			} else if (newAngle - scopeReference < -180) {
				newAngle += 360;
			}
			return newAngle;
		}
	}
}
