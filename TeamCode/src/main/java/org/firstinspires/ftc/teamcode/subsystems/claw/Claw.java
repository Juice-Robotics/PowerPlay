package org.firstinspires.ftc.teamcode.subsystems.claw;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.StepperServo;
import org.firstinspires.ftc.teamcode.lib.Axis;
import com.qualcomm.robotcore.hardware.ColorSensor;


public class Claw {
    public StepperServo claw;
    public StepperServo clawX1;
    public StepperServo clawX2;
    public StepperServo clawY;
    public ClawSensor sensor;


    public boolean isOpen = false; // if open, true

    // CONSTANTS
    public double clawOpen = 0.4;
    public double clawClose = 0.9;
    public float clawXReset = 25;
    public float clawYReset = 0;
    public double clawXStep = 0.1;
    public double clawYStep = 0.1;

    public Claw(StepperServo claw, StepperServo clawX1, StepperServo clawX2, StepperServo clawY, ColorSensor sensor) {
        this.claw = claw;
        this.clawX1 = clawX1;
        this.clawX2 = clawX2;
        this.clawY = clawY;
        this.clawX2.servo.setDirection(Servo.Direction.REVERSE);
        this.clawY.servo.setDirection(Servo.Direction.REVERSE);
        this.sensor = new ClawSensor(sensor);
    }

    public void setPositionClaw(double angle) {
        // Close Claw
        this.claw.servo.setPosition(angle);
    }

    public void toggle() {
        // Close Claw
        if (this.isOpen) {
            this.claw.servo.setPosition(clawClose);
            this.isOpen = false;
        }
        // Open Claw
        else {
            this.claw.servo.setPosition(clawOpen);
            this.isOpen = true;
        }
    }

    public void setClawOpen() {
        this.claw.servo.setPosition(clawOpen);
    }

    public void setClawClose() {
        this.claw.servo.setPosition(clawClose);
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