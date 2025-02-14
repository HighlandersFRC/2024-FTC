package org.firstinspires.ftc.teamcode.Tools;

public class PIDF {
    private double error;
    private double totalError;
    private double prevError;

    private double PValue;
    private double IValue;
    private double DValue;
    private double FValue;

    private double maxInput;
    private double minInput;
    private double maxOutput = 1.0;
    private double minOutput = -1.0;

    private boolean continuous = false;
    private double setPoint;
    private double result;

    public PIDF(double kp, double ki, double kd, double kf) {
        PValue = kp;
        IValue = ki;
        DValue = kd;
        FValue = kf;
        totalError = 0.0;
    }

    public double updatePIDF(double value, double gravityEffect) {
        error = setPoint - value;

        if (continuous) {
            if (Math.abs(error) > (maxInput - minInput) / 2) {
                error = error > 0 ? error - (maxInput - minInput) : error + (maxInput - minInput);
            }
        }

        if (Math.abs(error) < 0.1) {
            totalError += error;
            totalError = clamp(totalError);
        }

        result = PValue * error + IValue * totalError + DValue * (error - prevError) + (FValue * gravityEffect);
        prevError = error;
        result = clamp(result);
        return result;
    }

    public void setPIDF(double p, double i, double d, double f) {
        PValue = p;
        IValue = i;
        DValue = d;
        FValue = f;
    }

    public void setSetPoint(double target) {
        setPoint = target;
        totalError = 0;
    }

    public double getSetPoint() {
        return setPoint;
    }

    public double getResult() {
        return result;
    }

    public void setMaxOutput(double output) {
        maxOutput = output;
    }

    public void setMinOutput(double output) {
        minOutput = output;
    }

    public void setMinInput(double input) {
        minInput = input;
    }

    public void setMaxInput(double input) {
        maxInput = input;
    }

    public void setContinuous(boolean value) {
        continuous = value;
    }

    public double clamp(double input) {
        if (input > maxOutput) {
            return maxOutput;
        }
        if (input < minOutput) {
            return minOutput;
        }
        return input;
    }

    public double getError() {
        return error;
    }
}
