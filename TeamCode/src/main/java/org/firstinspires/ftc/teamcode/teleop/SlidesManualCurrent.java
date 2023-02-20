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
public class SlidesManualCurrent extends OpMode {
    private ElapsedTime timer;

    public static double power = 0;
    private double previousPower = 0;

    private DcMotorEx slides1;
    private DcMotorEx slides2;

    private double totalCD1 = 0;
    private double averageCD1 = 0;
    private double totalCD2 = 0;
    private double averageCD2 = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slides1 = hardwareMap.get(DcMotorEx.class, "slides1");
        slides2 = hardwareMap.get(DcMotorEx.class, "slides2");
        slides1.setDirection(DcMotorSimple.Direction.REVERSE);
        slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer = new ElapsedTime();
    }

    @Override
    public void loop() {

        if (gamepad1.cross) {
            timer.reset();
        }

        if (previousPower != power) {
            timer.reset();
        }

        slides1.setPower(power);
        slides2.setPower(power);

        totalCD1 += slides1.getCurrent(CurrentUnit.AMPS);
        averageCD1 = totalCD1 / timer.time();

        totalCD2 += slides2.getCurrent(CurrentUnit.AMPS);
        averageCD2 = totalCD2 / timer.time();


        telemetry.addData("motor 1 current", slides1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("motor 2 current", slides2.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("average current draw 1", averageCD1);
        telemetry.addData("average current draw 2", averageCD2);
        telemetry.update();

        previousPower = power;
    }
}