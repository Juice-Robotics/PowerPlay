package org.firstinspires.ftc.teamcode.subsystems.v4b;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class V4B {
    public StepperServo v4b1;
    public StepperServo v4b2;

    public double currentAngle;

    // TARGETS
    public double zeroTarget = 10;
    public double groundTarget = 10;
    public double lowTarget = 50;
    public double midTarget = 220;
    public double highTarget = 190;

    public V4B(StepperServo servo1, StepperServo servo2) {
        this.v4b1 = servo1;
        this.v4b2 = servo2;
//        v4b1.servo.setDirection(Servo.Direction.REVERSE);
        v4b2.servo.setDirection(Servo.Direction.REVERSE);
    }

    public void setAngle(double angle) {
        this.v4b1.setAngle((float) angle);
        this.v4b2.setAngle((float) angle);
        this.currentAngle = angle;
    }

    public double getAngle() {
        return currentAngle;
    }

    public void runToPreset(Levels level) {
//        switch (level) {
//            case ZERO:
//                this.setAngle(zeroTarget);
//            case GROUND:
//                this.setAngle(groundTarget);
//            case LOW:
//                this.setAngle(lowTarget);
//            case MEDIUM:
//                this.setAngle(midTarget);
//            case HIGH:
//                this.setAngle(highTarget);
//        }
        if (level == Levels.ZERO) {
            this.setAngle(zeroTarget);
        } else if (level == Levels.GROUND) {
            this.setAngle(groundTarget);
        } else if (level == Levels.LOW) {
            this.setAngle(lowTarget);
        } else if (level == Levels.MEDIUM) {
            this.setAngle(midTarget);
        } else if (level == Levels.HIGH) {
            this.setAngle(highTarget);
        }
    }
}
