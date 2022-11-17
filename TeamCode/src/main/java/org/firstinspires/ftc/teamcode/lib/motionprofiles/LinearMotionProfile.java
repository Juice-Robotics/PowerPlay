package org.firstinspires.ftc.teamcode.lib.motionprofiles;

public class LinearMotionProfile {
    public double start;
    public double target;
    public double slope;
    public double step;
    public double startTime;
    public boolean targetMet;

    public LinearMotionProfile(double start, double target, double slope, double step, double startTime) {
        this.start = start;
        this.target = target;
        this.slope = slope;
        this.step = step;
        this.startTime = startTime;
        this.targetMet = false;
    }

    public double[] calculateAllValues() {
        double[] result = new double[(int) ((target/slope) * (1/step))];
        double currentValue = start;
        int i = 0;
        double time = 0;

        while (currentValue != target) {
            currentValue = calculate(time - startTime);
            result[i] = currentValue;
            i += 1;
            time += step;
        }

        return result;
    }

    public double calculate(double time) {
        if (targetMet) {
            return target;
        }

        time = time - startTime;

        if (start < target) {
            double result = ensureRange((slope * time) + start, start, target);
            if (result == target) {
                targetMet = true;
            }
            return result;
        } else {
            double result = ensureRange((-slope * time) + start, start, target);
            if (result == target) {
                targetMet = true;
            }
            return result;
        }
    }

    public double getDuration() {
        return (target - start) / slope;
    }

    public double calculateSlope() {
        return (target - start) / start;
    }

    public void setTarget(double start, double target, double slope, double step, double startTime) {
        this.start = start;
        this.target = target;
        this.slope = slope;
        this.step = step;
        this.startTime = startTime;
        this.targetMet = false;
    }

    private double ensureRange(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
}
