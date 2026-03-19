package frc.lib;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class LoggedSlewRateLimiter {
	private final SlewRateLimiter limiter;
	private final String key;

	private double lastOutput = 0.0;

	public LoggedSlewRateLimiter(String key, double rateLimit) {
		this.key = key;
		this.limiter = new SlewRateLimiter(rateLimit);
	}

	public LoggedSlewRateLimiter(String key, double positiveRateLimit, double negativeRateLimit) {
		this.key = key;
		this.limiter = new SlewRateLimiter(positiveRateLimit, negativeRateLimit, 0.0);
	}

	public double calculate(double input) {
		double output = limiter.calculate(input);

		// Logging
		Logger.recordOutput(key + "/Input", input);
		Logger.recordOutput(key + "/Output", output);
		Logger.recordOutput(key + "/Delta", output - lastOutput);

		lastOutput = output;
		return output;
	}

	public void reset(double value) {
		limiter.reset(value);
		lastOutput = value;

		Logger.recordOutput(key + "/ResetValue", value);
	}

	public double getLastOutput() {
		return lastOutput;
	}
}