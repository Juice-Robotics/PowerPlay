package org.firstinspires.ftc.teamcode.subsystems.slides;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.Motor;

public class Slides {
    private PIDController controller1;
    private PIDController controller2;

    public double p = 0.06, i = 0, d = 0.002;
    public double f = 0.15;

    public int target = -50;
    private final double ticks_in_degrees = 700 / 180.0;

    public Motor slides1;
    public Motor slides2;

    // TARGETS IN NEGATIVE
    public int zeroTarget = -50;
    public int groundTarget = -100;
    public int lowTarget = -200;
    public int midTarget = -250;
    public int highTarget = -300;


    public Slides(Motor slides1, Motor slides2) {
        this.slides1 = slides1;
        this.slides2 = slides2;

        this.slides1.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        controller1 = new PIDController(p, i , d);
        controller2 = new PIDController(p, i , d);
    }


    public void update() {
        int slides1Pos = slides1.motor.getCurrentPosition();
        int slides2Pos = slides2.motor.getCurrentPosition();

        double pid1 = controller1.calculate(slides1Pos, target);
        double pid2 = controller2.calculate(slides2Pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double power1 = pid1 + ff;
        double power2 = pid2 + ff;

        slides1.motor.setPower(power1);
        slides2.motor.setPower(power2);
    }

    public void runToPosition(int ticks) {
        target = ticks;
    }

    public void runToPreset(Levels level) {
        switch (level) {
            case ZERO:
                target = zeroTarget;
            case GROUND:
                target = groundTarget;
            case LOW:
                target = lowTarget;
            case MEDIUM:
                target = midTarget;
            case HIGH:
                target = highTarget;
        }
    }


    public void resetAllEncoders(){
        slides1.resetEncoder();
        slides2.resetEncoder();
    }

}
