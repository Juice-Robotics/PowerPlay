package org.firstinspires.ftc.teamcode.subsystems.slides;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.Motor;

import java.util.function.Function;

public class Slides {
    private PIDController controller1;
    private PIDController controller2;

    private MotionProfile profile;
    public MotionState curState;
    private ElapsedTime timer;
    double maxvel = 5000;
    double maxaccel = 2700;

    public double p = 0.012, i = 0.00, d = 0.00;
    public double f = 0;
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
    public int lowTarget = -10;
    public int midTarget = -550;
    public int highTarget = -1200;
    public int highAuto = -1400;
    public int starterStack1 = -360;

    private boolean threadState = false;


    public Slides(Motor slides1, Motor slides2, VoltageSensor voltageSensor) {
        this.slides1 = slides1;
        this.slides2 = slides2;
        this.voltageSensor = voltageSensor;

        controller1 = new PIDController(p, i , d);
        controller2 = new PIDController(p, i , d);
        slides1.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        timer = new ElapsedTime();
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), maxvel, maxaccel);
    }


    public void update() {
        MotionState state = profile.get(timer.time());
        target = state.getX();

        int slides1Pos = slides1.motor.getCurrentPosition();
//        int slides2Pos = slides2.motor.getCurrentPosition();

        double pid1 = controller1.calculate(slides1Pos, target);
//        double pid2 = controller2.calculate(slides2Pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        voltageCompensation = 13.2 / voltageSensor.getVoltage();
        power1 = (pid1 + ff) * voltageCompensation;
//        power2 = pid2 + ff;

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
//        target = ticks;
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(ticks, 0), maxvel, maxaccel);
        timer.reset();
    }

    public void runToPreset(Levels level) {
//        switch (level) {
//            case ZERO:
//                target = zeroTarget;
//                currentLevel = level;
//            case GROUND:
//                target = groundTarget;
//                currentLevel = level;
//            case LOW:
//                target = lowTarget;
//                currentLevel = level;
//            case MEDIUM:
//                target = midTarget;
//                currentLevel = level;
//            case HIGH:
               /*target = lowTarget;
               currentLevel = level;*/
//        }
        if (level == Levels.ZERO) {
//            target = zeroTarget;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(zeroTarget, 0), maxvel, maxaccel);
            timer.reset();
            currentLevel = level;
        } else if (level == Levels.GROUND) {
//            target = groundTarget;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(groundTarget, 0), maxvel, maxaccel);
            timer.reset();
            currentLevel = level;
        } else if (level == Levels.LOW) {
//            target = lowTarget;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(lowTarget, 0), maxvel, maxaccel);
            timer.reset();
            currentLevel = level;
        } else if (level == Levels.MEDIUM) {
//            target = midTarget;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(midTarget, 0), maxvel, maxaccel);
            timer.reset();
            currentLevel = level;
        } else if (level == Levels.HIGH) {
//            target = highTarget;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(highTarget, 0), maxvel, maxaccel);
            timer.reset();
            currentLevel = level;
        } else if (level == Levels.STARTSTACK1) {
//            target = highTarget;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(starterStack1, 0), maxvel, maxaccel);
            timer.reset();
            currentLevel = level;
    } else if (level == Levels.AUTOHIGH) {
//            target = highTarget;
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(highAuto, 0), maxvel, maxaccel);
        timer.reset();
        currentLevel = level;
    }
    }

    public void launchAsThread(Telemetry telemetry) {
        threadState = true;
        telemetry.addData("Slides Threads State:", "STARTING");
        telemetry.update();
        Thread t1 = new Thread(() -> {
            telemetry.addData("Slides Threads State:", "STARTED");
            telemetry.update();
            while (threadState == true) {
                update();
            }
            telemetry.addData("Slides Threads State:", "STOPPED");
            telemetry.update();
        });
        t1.start();
    }

    public void destroyThreads(Telemetry telemetry) {
        telemetry.addData("Slides Threads State:", "STOPPING");
        telemetry.update();
        target = 0;
        threadState = false;
    }

    public void launchAsThreadBasic() {
        threadState = true;
        Thread t1 = new Thread(() -> {
            while (threadState == true) {
                update();
            }
        });
        t1.start();
    }

    public void destroyThreadsBasic() {
        threadState = false;
    }
    public void resetAllEncoders(){
        slides1.resetEncoder();
        slides2.resetEncoder();
    }

    public int getPos() {
        return slides1.motor.getCurrentPosition();
    }

}