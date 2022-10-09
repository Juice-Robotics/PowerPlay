package org.firstinspires.ftc.teamcode.slides;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.Motor;

public class Slides {
    public Motor slides1;
    public Motor slides2;

    public int zeroTarget = 0;
    public int groundTarget = 0;
    public int lowTarget = 0;
    public int midTarget = 0;
    public int highTarget = 0;

    public Slides(Motor slides1, Motor slides2) {
        this.slides1 = slides1;
        this.slides2 = slides2;

        this.slides1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slides2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runToPreset(Levels level) {
        switch (level) {
            case ZERO:
                this.slides1.setTarget(zeroTarget);
                this.slides2.setTarget(zeroTarget);
            case GROUND:
                this.slides1.setTarget(groundTarget);
                this.slides2.setTarget(groundTarget);
            case LOW:
                this.slides1.setTarget(lowTarget);
                this.slides2.setTarget(lowTarget);
            case MEDIUM:
                this.slides1.setTarget(midTarget);
                this.slides2.setTarget(midTarget);
            case HIGH:
                this.slides1.setTarget(highTarget);
                this.slides2.setTarget(highTarget);
        }
    }
}
