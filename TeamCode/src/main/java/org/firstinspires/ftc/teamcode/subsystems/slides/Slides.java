package org.firstinspires.ftc.teamcode.subsystems.slides;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.Motor;


public class Slides {
    private PIDController controller1;

    private MotionProfile profile;
    private ElapsedTime timer;
    double maxvel = 0.0;
    double maxaccel = 0.0;

    public double p = 0.009, i = 0.01, d = 0.0005;
    public double f = -0.003;
    double voltageCompensation;

    public double target = 0;
    public Levels currentLevel = Levels.ZERO;
    private final double ticks_in_degrees = 700 / 180.0;
    public double power1;
    public double power2;

    public Motor slides1;
    public Motor slides2;
    public VoltageSensor voltageSensor;

    // TARGETS IN NEGATIVE
    public int zeroTarget = -10;
    public int groundTarget = -10;
    public int lowTarget = -250;
    public int midTarget = -700;
    public int highTarget = -1090;

    private boolean threadState = false;


    public Slides(Motor slides1, Motor slides2, VoltageSensor voltageSensor) {
        this.slides1 = slides1;
        this.slides2 = slides2;
        this.voltageSensor = voltageSensor;

        controller1 = new PIDController(p, i , d);
        slides1.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        timer = new ElapsedTime();
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), maxvel, maxaccel);
    }


    public void update() {
        MotionState state = profile.get(timer.time());
        target = state.getX();

        int slides1Pos = slides1.motor.getCurrentPosition();

        double pid1 = controller1.calculate(slides1Pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        voltageCompensation = 13.2 / voltageSensor.getVoltage();
        power1 = (pid1 + ff) * voltageCompensation;

        if (target == groundTarget){
            slides1.motor.setPower(power1*0.3);
            slides2.motor.setPower(-power1*0.3);
        }
        else {
            slides1.motor.setPower(power1);
            slides2.motor.setPower(-power1);
        }
    }

    public void runToPosition(int ticks) {
        target = ticks;
    }

    public void runToPreset(Levels level) {
        if (level == Levels.ZERO) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(zeroTarget, 0), maxvel, maxaccel);
            timer.reset();
            currentLevel = level;
        } else if (level == Levels.GROUND) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(groundTarget, 0), maxvel, maxaccel);
            timer.reset();
            currentLevel = level;
        } else if (level == Levels.LOW) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(lowTarget, 0), maxvel, maxaccel);
            timer.reset();
            currentLevel = level;
        } else if (level == Levels.MEDIUM) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(midTarget, 0), maxvel, maxaccel);
            timer.reset();
            currentLevel = level;
        } else if (level == Levels.HIGH) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(highTarget, 0), maxvel, maxaccel);
            timer.reset();
            currentLevel = level;
        }
    }

    public void launchAsThread(Telemetry telemetry) {
        threadState = true;
        telemetry.addData("Slides Threads Status:", "STARTING");
        telemetry.update();
        Thread t1 = new Thread(() -> {
            telemetry.addData("Slides Threads Status:", "STARTED");
            telemetry.update();
            while (threadState == true) {
                update();
            }
            telemetry.addData("Slides Threads Status:", "STOPPED");
            telemetry.update();
        });
        t1.start();
    }

    public void destroyThreads(Telemetry telemetry) {
        telemetry.addData("Slides Threads Status:", "STOPPING");
        telemetry.update();
        threadState = false;
    }


    public void resetAllEncoders(){
        slides1.resetEncoder();
        slides2.resetEncoder();
    }

    private int getPos() {
        return slides1.motor.getCurrentPosition();
    }

}