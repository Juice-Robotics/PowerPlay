package org.firstinspires.ftc.teamcode.claw;

import org.firstinspires.ftc.teamcode.lib.StepperServo;
import org.firstinspires.ftc.teamcode.lib.Axis;

public class Claw {
    public StepperServo claw;
    public StepperServo clawX1;
    public StepperServo clawX2;
    public StepperServo clawY;

    public boolean state; // if open, true

    // CONSTANTS
    public float clawOpen = 90;
    public float clawClose = 0;
    public float clawXReset = 0;
    public float clawYReset = 0;
    public double clawXStep = 0.1;
    public double clawYStep = 0.1;

    public Claw(StepperServo claw, StepperServo clawX1, StepperServo clawX2, StepperServo clawY) {
        this.claw = claw;
        this.clawX1 = clawX1;
        this.clawX2 = clawX2;
        this.clawY = clawY;
    }

    public void toggle() {
        // Close Claw
        if (this.state) {
            this.claw.setAngle(clawClose);
            this.state = false;
        }
        // Open Claw
        else {
            this.claw.setAngle(clawOpen);
            this.state = true;
        }
    }

    public void setXRotation(float rotation) {
        this.clawX1.setAngle(rotation);
        this.clawX2.setAngle(rotation);
    }

    public void setYRotation(float rotation) {
        this.clawY.setAngle(rotation);
    }

    public void startXRotation(boolean direction) {
        if (direction) {
            this.clawX1.servo.setPosition(this.clawX1.servo.getPosition() + clawXStep);
            this.clawX2.servo.setPosition(this.clawX2.servo.getPosition() + clawXStep);
        }
        else {
            this.clawX1.servo.setPosition(this.clawX1.servo.getPosition() - clawXStep);
            this.clawX2.servo.setPosition(this.clawX2.servo.getPosition() - clawXStep);
        }
    }

    public void startYRotation(boolean direction) {
        if (direction) {
            this.clawY.servo.setPosition(this.clawY.servo.getPosition() + clawYStep);
        }
        else {
            this.clawY.servo.setPosition(this.clawY.servo.getPosition() - clawYStep);
        }
    }

    public void resetRotation(Axis axis) {
        switch (axis) {
            case X:
                this.clawX1.setAngle(clawXReset);
                this.clawX2.setAngle(clawXReset);
            case Y:
                this.clawY.setAngle(clawYReset);
            case ALL:
                this.clawX1.setAngle(clawXReset);
                this.clawX2.setAngle(clawXReset);
                this.clawY.setAngle(clawYReset);
        }
    }

}
