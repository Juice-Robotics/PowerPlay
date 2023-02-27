package org.firstinspires.ftc.teamcode.subsystems.relocalization;

public class LowPassFilter {
    public double a; // 0 < a < 1
    double previousEstimate = 0;
    double currentEstimate = 0;

    public LowPassFilter(double a) {
        // low values of a are more responsive, higher values have more smoothing
        this.a = a;
    }

    public void update(double measurement) {
        currentEstimate = (a * previousEstimate) + (1 - a) * measurement;
        previousEstimate = currentEstimate;
    }

    public double returnValue() {
        return currentEstimate;
    }
}

