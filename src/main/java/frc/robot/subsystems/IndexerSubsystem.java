package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
    private static IndexerSubsystem instance = null;

    private final SparkFlex m_motor;
    private double setpoint = 0;

    private IndexerSubsystem() {
        m_motor = new SparkFlex(Constants.CanId.INDEXER, MotorType.kBrushless);
        m_motor.configure(
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
    }

    public static void createInstance() {
        getInstance();
    }

    public static IndexerSubsystem getInstance() {
        if (instance == null) instance = new IndexerSubsystem();
        return instance;
    }

    @Override
    public void periodic() {
        if (Constants.DebugLevel.isOrAll(Constants.DebugLevel.Indexer))
            SmartDashboard.putData(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(this.getClass().getName());
        builder.addDoubleProperty("Duty Cycle Setpoint", () -> setpoint, null);
    }

    public void enable() {
        setpoint = Constants.Indexer.RUN_SETPOINT;
        m_motor.set(setpoint);
    }

    public void disable() {
        setpoint = 0;
        m_motor.stopMotor();
    }

    public Command toggle() {
        return Commands.startEnd(
            () -> enable(),
            () -> disable(),
            this
        );
    }
}
