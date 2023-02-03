package org.firstinspires.ftc.teamcode.subsystems.guide;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.StepperServo;
import org.firstinspires.ftc.teamcode.lib.Axis;

public class Guide {
    public StepperServo guideServo;

    // CONSTANTS
    public double guideUp = 0;
    public double guideDown = 0.49;

    public boolean state = false;

    public Guide(StepperServo guideServo) {
        this.guideServo = guideServo;
    }

    public void toggle() {
        // Guide Down
        if (this.state) {
            this.guideServo.servo.setPosition(guideUp);
            this.state = false;
        }
        // Guide Up
        else {
            this.guideServo.servo.setPosition(guideDown);
            this.state = true;
        }

    }

    public void setGuideDown() {
        this.guideServo.servo.setPosition(guideDown);
    }

    public void setGuideUp() {
        this.guideServo.servo.setPosition(guideUp);
    }

    public void setGuideRotation(float rotation) {
        this.guideServo.setAngle(rotation);
    }

}