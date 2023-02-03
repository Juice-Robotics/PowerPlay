package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp
public class SlidesPIDFTuning extends OpMode {
    private PIDController controller1;
    private PIDController controller2;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;
    private final double ticks_in_degrees = 700 / 180.0;

    private DcMotorEx slides1;
    private DcMotorEx slides2;

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
    }

    @Override
    public void loop() {
        controller1.setPID(p, i, d);
        controller2.setPID(p, i, d);
        int slides1Pos = slides1.getCurrentPosition();
        int slides2Pos = slides2.getCurrentPosition();

        double pid1 = controller1.calculate(slides1Pos, target);
        double pid2 = controller2.calculate(slides2Pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double power1 = pid1 + ff;
        double power2 = pid2 + ff;

        slides1.setPower(power1);
        slides2.setPower(power2);

        telemetry.addData("pos1 ", slides1Pos);
        telemetry.addData("pos2 ", -slides1Pos);
        telemetry.addData("target ", target);
        telemetry.addData("motor 1 current", slides1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("motor 2 current", slides2.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}