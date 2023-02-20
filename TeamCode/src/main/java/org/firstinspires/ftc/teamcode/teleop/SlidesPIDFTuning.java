package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp
public class SlidesPIDFTuning extends OpMode {
    private PIDController controller1;
    private PIDController controller2;
    private MotionProfile profile;
    public MotionState curState;

    double maxvel = 5000;
    double maxaccel = 2700;
    private ElapsedTime timer;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;
    private int previousTarget = 0;
    private final double ticks_in_degrees = 700 / 180.0;

    private DcMotorEx slides1;
    private DcMotorEx slides2;

    private double duration = 0;
    private boolean targetReached = false;
    private double totalCD1 = 0;
    private double averageCD1 = 0;
    private double totalCD2 = 0;
    private double averageCD2 = 0;

    @Override
    public void init() {
        controller1 = new PIDController(p, i , d);
        controller2 = new PIDController(p, i , d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slides1 = hardwareMap.get(DcMotorEx.class, "slides1");
        slides2 = hardwareMap.get(DcMotorEx.class, "slides2");
        slides1.setDirection(DcMotorSimple.Direction.REVERSE);
        slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer = new ElapsedTime();
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), maxvel, maxaccel);
    }

    @Override
    public void loop() {
        controller1.setPID(p, i, d);
        controller2.setPID(p, i, d);
        int slides1Pos = slides1.getCurrentPosition();
        int slides2Pos = slides2.getCurrentPosition();

        if (target != previousTarget) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(slides1Pos, 0), new MotionState(target, 0), maxvel, maxaccel);
            duration = 0;
            targetReached = false;
            totalCD1 = 0;
            averageCD1 = 0;
            totalCD2 = 0;
            averageCD2 = 0;
            timer.reset();
        }

        if (slides1Pos == target && !targetReached) {
            duration = timer.time();
            targetReached = true;
        }

        MotionState state = profile.get(timer.time());

        double pid1 = controller1.calculate(slides1Pos, state.getX());
        double pid2 = controller2.calculate(slides2Pos, state.getX());
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double power1 = pid1 + ff;
        double power2 = pid2 + ff;

        slides1.setPower(power1);
        slides2.setPower(power2);

        totalCD1 += slides1.getCurrent(CurrentUnit.AMPS);
        averageCD1 = totalCD1 / timer.time();

        totalCD2 += slides2.getCurrent(CurrentUnit.AMPS);
        averageCD2 = totalCD2 / timer.time();

        telemetry.addData("pos1 ", slides1Pos);
        telemetry.addData("pos2 ", -slides1Pos);
        telemetry.addData("target ", target);
        telemetry.addData("motor 1 current", slides1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("motor 2 current", slides2.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("duration to target ", duration);
        telemetry.addData("average current draw 1", averageCD1);
        telemetry.addData("average current draw 2", averageCD2);
        telemetry.update();
    }
}