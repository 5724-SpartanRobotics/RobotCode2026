package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedSubsystem2 extends SubsystemBase {
    /** Default colors (public so commands or RobotContainer can reference them). */
    public static Color kDefaultActiveColor = Color.kGreen;
    public static Color kDefaultInactiveColor = Color.kBlack;
    public static Color kDefaultNotificationColor = Color.kBeige;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
    private final Timer m_timer = new Timer();
    private boolean isBlinking = false;

    public LedSubsystem2() {
        m_led = new AddressableLED(Constants.LED.PORT);
        // The actual strip is GRBW; setColorOrder to match physical wiring.
        m_led.setColorOrder(ColorOrder.kRGB); // keep as you had if that maps correctly
        m_buffer = new AddressableLEDBuffer(Constants.LED.STRIP_LENGTH);
        m_led.setLength(m_buffer.getLength());
        m_led.setData(m_buffer);
        m_led.start();
        setInactive(); // start off
        m_timer.start();
    }

    @Override
    public void periodic() {
        // No internal timing or blinking here. Commands handle dynamic behavior.
        if (isBlinking) {
            if (m_timer.get() % 2 == 0)
                setNotification();
            else
                setInactive();
        }
    }

    /** Utility: clamp 0..1 to 0..255 */
    private static int percentTo255(double percent) {
        return Math.max(0, Math.min(255, (int) (percent * 255.0)));
    }

    /**
     * Set every LED to the given Color immediately.
     * This method writes to the buffer and pushes to the hardware.
     */
    public synchronized void setColor(Color color) {
        if (color == null) {
            setInactive();
            return;
        }

        for (int i = 0; i < m_buffer.getLength(); i++) {
            // Keep your GRBW mapping logic if required by your strip.
            // The mapping below preserves your previous byte-order hack but is clearer.
            int r = percentTo255(color.red);
            int g = percentTo255(color.green);
            int b = percentTo255(color.blue);
            switch (i % 4) {
                case 0:
                    m_buffer.setRGB(i, g, r, b);
                    break;
                case 1:
                    m_buffer.setRGB(i, 0, g, r);
                    break;
                case 2:
                    m_buffer.setRGB(i, b, 0, g);
                    break;
                case 3:
                    m_buffer.setRGB(i, r, b, 0);
                    break;
                default:
                    m_buffer.setRGB(i, r, g, b);
                    break;
            }
        }
        m_led.setData(m_buffer);
    }

    /** Turn the strip off (inactive color). */
    public synchronized void setInactive() {
        isBlinking = false;
        setColor(kDefaultInactiveColor);
    }

    /** Convenience: set to the default active color. */
    public synchronized void setActive() {
        isBlinking = false;
        setColor(kDefaultActiveColor);
    }

    public synchronized void setNotification() {
        isBlinking = false;
        setColor(kDefaultNotificationColor);
    }

    public synchronized void setNotification(boolean blink) {
        isBlinking = true;
        m_timer.reset();
        setNotification();
    }

    public synchronized void endNotification() {
        isBlinking = false;
    }

    /** Return the buffer length (useful for patterns). */
    public int getLength() {
        return m_buffer.getLength();
    }

    public Command setColorCommand(Color color) {
        var ledSubsystem = this;
        return new InstantCommand() {
            {
                addRequirements(ledSubsystem);
            }

            @Override
            public void initialize() {
                ledSubsystem.setColor(color);
            }
        };
    }

    public Command setColorForDurationCommand(Color color, Time duration) {
        var ledSubsystem = this;
        return new Command() {
            private final double durationSeconds;
            private final Timer m_timer = new Timer();

            {
                this.durationSeconds = Math.max(0.0, duration.in(Units.Seconds));
                addRequirements(ledSubsystem);
            }

            @Override
            public void initialize() {
                m_timer.reset();
                m_timer.start();
                ledSubsystem.setColor(color);
            }

            @Override
            public boolean isFinished() {
                return m_timer.hasElapsed(durationSeconds);
            }

            @Override
            public void end(boolean interrupted) {
                m_timer.stop();
                ledSubsystem.setInactive();
            }
        };
    }

    public Command blinkLedCommand(Color blinkColor, Time onFor, Time offFor, int blinkCount) {
        var ledSubsystem = this;
        return new Command() {
            private final double onTime;
            private final double offTime;
            private final int blinkCountTarget;

            private final Timer m_timer = new Timer();
            private int blinkCount = 0;
            private boolean isOn = false;

            {
                this.onTime = Math.max(0.0, onFor.in(Units.Seconds));
                this.offTime = Math.max(0.0, offFor.in(Units.Seconds));
                this.blinkCountTarget = Math.max(0, blinkCount);
                addRequirements(ledSubsystem);
            }

            @Override
            public void initialize() {
                blinkCount = 0;
                isOn = false;
                m_timer.reset();
                m_timer.start();
                // start with off state; first execute will flip to on after onTime if desired
                ledSubsystem.setInactive();
            }

            @Override
            public void execute() {
                double elapsed = m_timer.get();
                double currentPeriod = isOn ? onTime : offTime;

                if (elapsed >= currentPeriod) {
                    // toggle
                    isOn = !isOn;
                    m_timer.reset();
                    m_timer.start();

                    if (isOn) {
                        blinkCount++;
                        ledSubsystem.setColor(blinkColor);
                    } else {
                        ledSubsystem.setInactive();
                    }
                }
            }

            @Override
            public boolean isFinished() {
                return blinkCountTarget > 0 && blinkCount >= blinkCountTarget && !isOn;
            }

            @Override
            public void end(boolean interrupted) {
                m_timer.stop();
                ledSubsystem.setInactive();
            }
        };
    }
}
