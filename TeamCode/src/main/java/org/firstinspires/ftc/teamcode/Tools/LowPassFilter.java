package org.firstinspires.ftc.teamcode.Tools;

public class LowPassFilter {
    private double alpha;  // Smoothing factor
    private Double filteredValue = null;  // Filtered value (null initially)

    // Constructor with alpha value (0 < alpha <= 1)
    public LowPassFilter(double alpha) {
        this.alpha = alpha;
    }

    // Method to update the filter with a new value
    public double update(double newValue) {
        if (filteredValue == null) {
            // Initialize the filtered value on the first update
            filteredValue = newValue;
        } else {
            // Apply the low-pass filter formula
            filteredValue = (alpha * newValue) + ((1 - alpha) * filteredValue);
        }
        return filteredValue;
    }

    // Setter to adjust alpha if needed
    public void setAlpha(double alpha) {
        this.alpha = alpha;
    }

    // Getter to retrieve the current filtered value
    public double getFilteredValue() {
        return filteredValue != null ? filteredValue : 0.0;
    }
}
