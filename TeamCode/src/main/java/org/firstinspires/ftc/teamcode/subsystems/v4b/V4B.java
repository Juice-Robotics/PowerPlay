package org.firstinspires.ftc.teamcode.subsystems.v4b;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class V4B {
    public StepperServo v4b1;
    public StepperServo v4b2;

    public float currentAngle;

    // TARGETS
    public float zeroTarget = (float) 10;
    public float groundTarget = (float) 20;
    public float lowTarget = (float) 30;
    public float midTarget = (float) 35;
    public float highTarget = (float) 40;

    public V4B(StepperServo servo1, StepperServo servo2) {
        this.v4b1 = servo1;
        this.v4b2 = servo2;
    }

    public void setAngle(float angleInDegrees) {
        this.v4b1.setAngle(angleInDegrees);
        this.v4b2.setAngle(angleInDegrees);
        this.currentAngle = angleInDegrees;
    }

    public float getAngle() {
        return currentAngle;
    }

    public void runToPreset(Levels level) {
        switch (level) {
            case ZERO:
                this.setAngle(zeroTarget);
            case GROUND:
                this.setAngle(groundTarget);
            case LOW:
                this.setAngle(lowTarget);
            case MEDIUM:
                this.setAngle(midTarget);
            case HIGH:
                this.setAngle(highTarget);
        }
    }
}
