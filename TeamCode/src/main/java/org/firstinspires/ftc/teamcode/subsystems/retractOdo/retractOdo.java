package org.firstinspires.ftc.teamcode.subsystems.retractOdo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.StepperServo;
import org.firstinspires.ftc.teamcode.lib.Axis;

public class retractOdo {
    public StepperServo retractServo;

    // CONSTANTS
    public double servoUp = 0;
    public double servoDown = 0.42;

    public boolean state = false;

    public retractOdo(StepperServo retractServo) {
        this.retractServo = retractServo;
    }

    public void setRetractServoRotation(float rotation) {
        this.retractServo.setAngle(rotation);
    }

    public void setRetractDown() {
        this.retractServo.servo.setPosition(servoDown);
    }

    public void setRetractUp() {
        this.retractServo.servo.setPosition(servoUp);
    }


    public void toggle() {
        // Guide Down
        if (this.state) {
            this.retractServo.servo.setPosition(servoUp);
            this.state = false;
        }
        // Guide Up
        else {
            this.retractServo.servo.setPosition(servoDown);
            this.state = true;
        }

    }
}