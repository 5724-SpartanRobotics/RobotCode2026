package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedSubsystem extends SubsystemBase {
	/** you can set default colors to whatever you want. */
	public static Color kDefaultActiveColor = Color.kGreen;
	public static Color kDefaultInactiveColor = Color.kBlack;
	public static Color kDefaultNotificationColor = Color.kBeige;

	public static final Color kWhiteWhiteGreen = Color.fromHSV(217, 73, 96);
	public static int percentTo255(double percent) { return Math.min((int)(percent * 255), 255); }
	public static final class RGBW {
		public static final int R = 0;
		public static final int G = 0;
		public static final int B = 0;
		public static final int W = 0;
	}

	private final AddressableLED m_led;
	private final AddressableLEDBuffer m_ledBuffer;
	private final Timer m_timer = new Timer();
	private boolean _useDuration = false;
	private double _duration = 0;
	private double _sleepDuration = 0;
	private boolean _lock = false;

	private boolean _isBlinkOn = false;
	private int _blinkCount = 0;
	private int _targetBlinkCount = 0;
	private Color _blinkColor = Color.kBlack;

	public LedSubsystem() {
		this.m_led = new AddressableLED(Constants.LED.PORT);
		// The actual color order is GRBW, but we have it set to RGB so it's easier to visualize the
		// color order in the code below.
		this.m_led.setColorOrder(ColorOrder.kRGB);
		this.m_ledBuffer = new AddressableLEDBuffer(Constants.LED.STRIP_LENGTH);
		this.m_led.setLength(this.m_ledBuffer.getLength());
		this.m_led.setData(this.m_ledBuffer);
		this.m_led.start();
		this.m_timer.reset();
		this.m_timer.start();
	}

	@Override
	public void periodic() {
		super.periodic();

		if (_useDuration && m_timer.get() >= _duration && _sleepDuration <= 0) {
			_useDuration = false;
			_duration = 0.0;
			reset();
		}
		if (_useDuration && _sleepDuration > 0) {
			double elapsed = m_timer.get();
			double cycleTime = _isBlinkOn ? _duration : _sleepDuration;
	
			if (elapsed >= cycleTime) {
				m_timer.restart();
				_isBlinkOn = !_isBlinkOn;
				_blinkCount++;
	
				if (_blinkCount >= _targetBlinkCount) {
					_useDuration = false;
					_lock = false;
					_blinkCount = 0;
					_targetBlinkCount = 0;
					SetColor(kDefaultInactiveColor);
				} else {
					SetColor(_isBlinkOn ? _blinkColor : kDefaultInactiveColor);
				}
			}
		}

		if (!_lock) reset();
	}

	private void SetColor(Color color) {
		for (int i = 0; i < m_ledBuffer.getLength(); i++) {
			switch (i % 4) {
				// This says "setRGB", but we're just using it as a hack to set the bytes in a specific order.
				// The LED strip we have has color order GRBW, so that's why for the first component we pass
				// green, then the second X, etc., etc.
				case 0: m_ledBuffer.setRGB(i, percentTo255(color.green), percentTo255(color.red), percentTo255(color.blue)); break;
				case 1: m_ledBuffer.setRGB(i, RGBW.W, percentTo255(color.green), percentTo255(color.red)); break;
				case 2: m_ledBuffer.setRGB(i, percentTo255(color.blue), RGBW.W, percentTo255(color.green)); break;
				case 3: m_ledBuffer.setRGB(i, percentTo255(color.red), percentTo255(color.blue), RGBW.W); break;
				default: break;
			}
		}
		m_led.setData(m_ledBuffer);
	}

	public LedSubsystem setColor(Color color) {
		_lock = false;
		SetColor(color);
		return this;
	}

	public LedSubsystem setColor(Color color, boolean lock) {
		_lock = true;
		SetColor(color);
		return this;
	}

	public LedSubsystem reset() {
		_lock = false;
		SetColor(kDefaultInactiveColor);
		m_timer.restart();
		return this;
	}

	public LedSubsystem off() {
		LEDPattern.solid(kDefaultInactiveColor).applyTo(m_ledBuffer);
		m_led.setData(m_ledBuffer);
		return this;
	}

	/**
	 * Runs the LED strip for a set color and duration.
	 * @param color
	 * @param duration Seconds
	 * @return
	 */
	public LedSubsystem setColorForDuration(Color color, double duration) {
		m_timer.restart();
		_useDuration = true;
		_duration = Math.max(duration, 0);
		setColor(color, true);
		return this;
	}	
}