package org.firstinspires.ftc.teamcode.subsystems.v4b;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class V4B {
    public StepperServo v4b1;
    public StepperServo v4b2;

    public double currentAngle;

    private MotionProfile profile;
    private ElapsedTime timer;
    double maxvel = 0.0;
    double maxaccel = 0.0;

    // TARGETS
    public double zeroTarget = 10;
    public double groundTarget = 10;
    public double lowTarget = 255;
    public double midTarget = 190;
    public double highTarget = 220;

    public V4B(StepperServo servo1, StepperServo servo2) {
        this.v4b1 = servo1;
        this.v4b2 = servo2;
        v4b2.servo.setDirection(Servo.Direction.REVERSE);

        timer = new ElapsedTime();
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), maxvel, maxaccel);
    }

    public void setAngle(double angle) {
//        this.v4b1.setAngle((float) angle);
//        this.v4b2.setAngle((float) angle);
        timer = new ElapsedTime();
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), maxvel, maxaccel);
        this.currentAngle = angle;
    }

    public double getAngle() {
        return currentAngle;
    }

    public void runToPreset(Levels level) {
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

    public void update() {
        double angle = profile.get(timer.time()).getX();
        this.v4b1.setAngle((float) angle);
        this.v4b2.setAngle((float) angle);
    }
}