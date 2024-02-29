package frc.robot.utils;

public class EncoderBasedLimits {
    private double lowerLimit;
    private double upperLimit;
    public EncoderBasedLimits(double lower, double upper) {
            this.lowerLimit = lower;
            this.upperLimit = upper;
    }

    public double getUpperLimit() {
            return this.upperLimit;
    }

    public double getLowerLimit() {
            return this.lowerLimit;
    }

    public void setUpperLimit(double limit) {
            this.upperLimit = limit;
    }

    public void setLowerLimit(double limit) {
            this.lowerLimit = limit;
    }
}