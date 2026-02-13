package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Deque;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedSubsystem3 extends SubsystemBase {
    // Bing AI told me that WPILib uses 24 bits/pixel for LEDs (cannot corroborate this).
    private static final int WPILIB_BITS_PER_PIXEL = 24;

    private static LedSubsystem3 instance = null;

    /** Default: 10 seconds */
    public static Time kDefaultStandbyDelay = Constants.Drive.WHEEL_LOCK_TIME;
    /** Default: #7f796c (gray) */
    public static Color kDisabledColor = new Color("#5f5a5a");
    /** Default: Black */
    public static Color kInactiveColor = Color.kBlack;
    /** Default: Green */
    public static Color kNotification1Color = Color.kGreen;
    /** Default: Yellow */
    public static Color kNotification2Color = Color.kYellow;
    /** Default: Purple */
    public static Color kNotification3Color = Color.kPurple;

    private static final boolean kIsRGB_Only = false;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;

    private boolean _hasEverEnabled = false;
    private boolean _lastDisabled = false;
    private double _disabledStartTimeSec = 0;

    private enum BaseState { RAINBOW, ACTIVE_OFF, DISABLED_STANDBY }
    private BaseState _baseState = BaseState.RAINBOW;

    private static class LedNotification {
        final Color color;
        final double expiresAt;
        final boolean persistent;
        LedNotification(Color c, Time duration) {
            color = c;
            expiresAt = Timer.getFPGATimestamp() + duration.in(Units.Seconds);
            persistent = false;
        }
        LedNotification(Color c) {
            color = c;
            expiresAt = Double.POSITIVE_INFINITY;
            persistent = true;
        }
    }
    private final Deque<LedNotification> _notifications = new ArrayDeque<>();

    private long _rainbowOffset = 0;
    private Color _currentRenderedColor = kInactiveColor;

    private Color[] m_simStrip;

    private LedSubsystem3() {
        m_led = new AddressableLED(Constants.LED.PORT);
        m_led.setColorOrder(ColorOrder.kRGB);
        // The magic of floating point numbers does something to this value, but I'm not sure
        // exactly what. This is calculated somehow, and this calculation yields correct results,
        // but I'm not exactly sure how I came to it.
        double countCoeff = (WPILIB_BITS_PER_PIXEL / Constants.LED.STRIP_BITS_PER_PIXEL_0) *
            ( Constants.LED.STRIP_BITS_PER_PIXEL_0 / Constants.LED.STRIP_BITS_PER_PIXEL_1 );
        // This magic value of 9 is to reduce the length of the buffer so we aren't trying to index
        // too many LEDs out of the strip. Not calculated, I just counted on the actual strip.
        int adjustedStripLength = (int)(Constants.LED.LED_COUNT * countCoeff) - 9;
        m_buffer = new AddressableLEDBuffer(adjustedStripLength);
        m_led.setLength(adjustedStripLength);
        m_led.start();

        if (RobotBase.isSimulation()) {
            m_simStrip = new Color[m_buffer.getLength()];
        }
    }

    public static void createInstance() {
        getInstance();
    }

    public static LedSubsystem3 getInstance() {
        if (instance == null) instance = new LedSubsystem3();
        return instance;
    }

    /** Push the {@link #kNotification1Color} (default green) to the notification stack. */
    public void notifyColor1(Time duration) {
        pushNotification(kNotification1Color, duration);
    }

    /** Push the {@link #kNotification2Color} (default yellow) to the notification stack. */
    public void notifyColor2(Time duration) {
        pushNotification(kNotification2Color, duration);
    }

    /** Push the {@link #kNotification3Color} (default purple) to the notification stack. */
    public void notifyColor3(Time duration) {
        pushNotification(kNotification3Color, duration);
    }

    private void pushNotification(Color c, Time d) {
        _notifications.push(new LedNotification(c, d));
    }

    public void setPersistentNotify(Color c) {
        _notifications.push(new LedNotification(c));
    }

    public void clearPersistentNotify(Color c) {
        _notifications.removeIf(n -> n.persistent && n.color.equals(c));
    }

    public void clearAllPersistentNotify() {
        _notifications.removeIf(n -> n.persistent);
    }

    public Command togglePersistentNotificationCommand(Color c) {
        return new InstantCommand(() -> {
            for ( LedNotification n : _notifications ) {
                if (n.persistent && n.color.equals(c)) {
                    clearPersistentNotify(c);
                    return;
                }
            }
            setPersistentNotify(c);
        }, this);
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        final boolean dsEnabled = DriverStation.isEnabled();
        final boolean dsDisabled = DriverStation.isDisabled();
        final boolean dsAttached = DriverStation.isDSAttached();

        if (dsEnabled) _hasEverEnabled = true;
        if (dsDisabled && !_lastDisabled) _disabledStartTimeSec = now;
        _lastDisabled = dsDisabled;

        while (!_notifications.isEmpty()) {
            LedNotification top = _notifications.peek();
            if (!top.persistent && top.expiresAt <= now) {
                _notifications.pop();
            } else {
                break;
            }
        }

        if (!_hasEverEnabled || !dsAttached) _baseState = BaseState.RAINBOW;
        else if (dsEnabled) _baseState = BaseState.ACTIVE_OFF;
        else {
            if (now - _disabledStartTimeSec > kDefaultStandbyDelay.in(Units.Seconds)) _baseState = BaseState.DISABLED_STANDBY;
            else _baseState = BaseState.ACTIVE_OFF;
        }

        if (!_notifications.isEmpty()) {
            renderSolid(_notifications.peek().color);
        } else {
            switch (_baseState) {
                case RAINBOW: renderRainbow(); break;
                case DISABLED_STANDBY:
                    clearAllPersistentNotify(); // this doesn't work here?
                    renderSolid(kDisabledColor);
                    break;
                case ACTIVE_OFF:
                default:
                    renderSolid(kInactiveColor); break;
            }
        }

        m_led.setData(m_buffer);

        if (Constants.DebugLevel.isOrAll(Constants.DebugLevel.LED))
            SmartDashboard.putData(this);
    }

    private void setPixelGRBW(int i, Color c)  {
        // Max is so that we don't burn out the LEDs and the VRM.
        final int max = 100;
        int r = (int)(c.red * max) / 2,
            g = (int)(c.green * max) / 2,
            b = (int)(c.blue * max) / 2;
        if (kIsRGB_Only) { m_buffer.setRGB(i, r, g, b); return; }
        switch (i % 4) {
            // This says "setRGB", but we're just using it as a hack to set the bytes in a specific order.
            // The LED strip we have has color order GRBW, so that's why for the first component we pass
            // green, then the second X, etc., etc.
            case 0: m_buffer.setRGB(i, g, r, b); break;
            case 1: m_buffer.setRGB(i, 0, g, r); break;
            case 2: m_buffer.setRGB(i, b, 0, g); break;
            case 3: m_buffer.setRGB(i, r, b, 0); break;
            default: break;
        }
    }

    private void renderSolid(Color c) {
        _currentRenderedColor = c;
        final int length = m_buffer.getLength();
        for (int i = 0; i < length; i++) {
            setPixelGRBW(i, c);
            if (RobotBase.isSimulation()) m_simStrip[i] = c;
        }
    }

    private void renderRainbow() {
        final int length = m_buffer.getLength();
        _rainbowOffset = (_rainbowOffset + 2) % 180;

        int hue = (int)(_rainbowOffset % 180);
        _currentRenderedColor = Color.fromHSV(hue, 255, 255);

        for (int i = 0; i < length; i++) {
            int h = (int)( (i * 180.0 / length + _rainbowOffset) % 180 );
            Color c = Color.fromHSV(h, 255, 255);
            setPixelGRBW(i, c);
            if (RobotBase.isSimulation()) m_simStrip[i] = c;
        }
    }

    private Color getRenderedColorAt(int index) {
        return _currentRenderedColor;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(this.getClass().getName());
        builder.addBooleanProperty("HasEverEnabled", () -> _hasEverEnabled, null);
        builder.addStringProperty("BaseState", () -> _baseState.toString(), null);
        builder.addIntegerProperty("NotificationCount", () -> _notifications.size(), null);
        builder.addStringProperty("TopNotificationColor", () -> _notifications.isEmpty() ? "None" : _notifications.peek().color.toString(), null);
        builder.addDoubleProperty("DisabledTime", () -> Timer.getFPGATimestamp() - _disabledStartTimeSec, null);
        builder.addDoubleProperty("RainbowOffset", () -> _rainbowOffset, null);
        builder.addDoubleArrayProperty(
            "CurrentColorRGB",
            () -> new double[] {
                _currentRenderedColor.red,
                _currentRenderedColor.green,
                _currentRenderedColor.blue
            },
            null
        );
        builder.addStringProperty(
            "CurrentColorHex",
            () -> String.format(
                "#%02X%02X%02X",
                (int)(_currentRenderedColor.red * 255),
                (int)(_currentRenderedColor.green * 255),
                (int)(_currentRenderedColor.blue * 255)
            ),
            null
        );
        builder.addDoubleArrayProperty(
            "LEDStripRGB",
            () -> {
                double[] arr = new double[m_buffer.getLength() * 3];
                for (int i = 0; i < m_buffer.getLength(); i++) {
                    Color c = getRenderedColorAt(i);
                    arr[i * 3]     = c.red;
                    arr[i * 3 + 1] = c.green;
                    arr[i * 3 + 2] = c.blue;
                }
                return arr;
            },
            null
        );

    }
}
