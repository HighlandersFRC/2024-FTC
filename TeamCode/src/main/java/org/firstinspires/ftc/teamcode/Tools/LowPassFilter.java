package org.firstinspires.ftc.teamcode.Tools;

public class LowPassFilter {
    private double alpha;  // smoothing factor (0 < alpha < 1)
    private double previousOutput;  // stores the previous output of the filter
    private boolean initialized;  // keeps track of whether the filter has been initialized

    // Constructor for the low pass filter
    public LowPassFilter(double alpha) {
        if (alpha < 0.0 || alpha > 1.0) {
            throw new IllegalArgumentException("Alpha must be between 0 and 1");
        }
        this.alpha = alpha;
        this.initialized = false;
    }

    // Apply the filter to the input value
    public double filter(double input) {
        if (!initialized) {
            // The first time, we initialize the filter with the input value
            previousOutput = input;
            initialized = true;
        }
        // Apply the filter equation: output = alpha * input + (1 - alpha) * previousOutput
        previousOutput = alpha * input + (1 - alpha) * previousOutput;
        return previousOutput;
    }

    // Reset the filter
    public void reset() {
        initialized = false;
    }

    // Optionally set a new alpha value
    public void setAlpha(double alpha) {
        if (alpha < 0.0 || alpha > 1.0) {
            throw new IllegalArgumentException("Alpha must be between 0 and 1");
        }
        this.alpha = alpha;
    }
}
