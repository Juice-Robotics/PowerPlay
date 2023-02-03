package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;

@TeleOp(group = "competition")
public class TeleOpMain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your own robot class
        Robot robot = new Robot(hardwareMap,false);

        double x;
        double y;
        double rx;

        boolean previousClawSensorState = false;
        ElapsedTime clawSensorTimeout = new ElapsedTime();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        ElapsedTime timer;
        timer = new ElapsedTime();

        boolean autoCloseEnabled = true;
        boolean autoClosePreviousState = false;
        boolean previousClawState = false;
        boolean previousGuideState = false;
        boolean previousRetractState = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.dpad_up) {
                robot.slides.resetAllEncoders();
            }

            //DRIVE
            if (gamepad1.right_trigger > 0.1){
                x = -gamepad1.left_stick_x*(1-0.66*gamepad1.right_trigger);
                y = -gamepad1.left_stick_y*(1-0.66*gamepad1.right_trigger);
                rx = gamepad1.right_stick_x*(1-0.66*gamepad1.right_trigger);

            } else{
                x = -gamepad1.left_stick_x;
                y = -gamepad1.left_stick_y;
                rx = gamepad1.right_stick_x;
            }
            robot.setDrivePower(-x, y, rx);

            //PRESETS
            if (gamepad1.right_bumper)
                robot.groundPreset(gamepad1.dpad_down);
            if (gamepad1.dpad_left)
                robot.lowPreset(gamepad1.dpad_left);
            if (gamepad1.dpad_right)
                robot.mediumPreset(gamepad1.dpad_right);
            if (gamepad1.left_bumper)
                robot.highPreset(gamepad1.left_bumper);
            if (gamepad1.dpad_down)
                robot.sidewaysPickup(gamepad1.dpad_down);

            //CLAW
            if (gamepad1.circle && !autoClosePreviousState) {
                autoCloseEnabled = !autoCloseEnabled;
            }

            if (gamepad2.left_trigger > 0.1) {
                robot.slides.runToPosition((int) (robot.slides.slides1.motor.getCurrentPosition() + (0.1*gamepad2.left_trigger)));
            } else if (gamepad2.right_trigger > 0.1) {
                robot.slides.runToPosition((int) (robot.slides.slides1.motor.getCurrentPosition() - (10*gamepad2.left_trigger)));
            }

            if (gamepad2.left_bumper) {
                robot.v4b.setAngle(robot.v4b.currentAngle - 1);
            } else if (gamepad2.right_bumper) {
                robot.v4b.setAngle(robot.v4b.currentAngle + 1);
            }

            if (gamepad2.dpad_up) {
                robot.claw.setXRotation((float) (robot.claw.clawX1.getAngle() + 1));
            } else if (gamepad2.dpad_down) {
                robot.claw.setXRotation((float) (robot.claw.clawX1.getAngle() - 1));
            }

            if (gamepad2.dpad_right) {
                robot.claw.clawY.setAngle((float) (robot.claw.clawY.getAngle() + 1));
            } else if (gamepad2.dpad_right) {
                robot.claw.clawY.setAngle((float) (robot.claw.clawY.getAngle() - 1));
            }

            boolean isPressed = gamepad1.cross;
            if (isPressed && !previousClawState) {
                robot.advancedToggleClaw();
            }
            previousClawState = isPressed;

            if (!previousClawSensorState && robot.claw.sensor.conePresent() && clawSensorTimeout.milliseconds() >= 50) {
                gamepad1.rumble(500);
                robot.claw.toggle();
            }

            if (previousClawSensorState && !robot.claw.sensor.conePresent() && clawSensorTimeout.milliseconds() >= 50) {
                clawSensorTimeout.reset();
            }
//
//            if (gamepad1.circle) {
//                robot.resetAllServos();
//            }

            if (gamepad2.right_stick_x > 0.2) {
                robot.startClawY(true);
            } else if (gamepad2.right_stick_x < -0.2) {
                robot.startClawY(false);
            }

            //GUIDE AND RETRACT
            boolean isPressed1 = gamepad1.triangle;
            if (gamepad1.triangle && !previousGuideState) {
                robot.toggleGuide();
            }
            previousGuideState = isPressed1;

            boolean isPressed2 = gamepad1.square;
            if (gamepad1.square && !previousRetractState) {
                robot.toggleRetract();
            }
            previousRetractState = isPressed2;

            previousClawSensorState = robot.claw.sensor.conePresent();
            autoClosePreviousState = gamepad1.circle;
            robot.slides.update();
            telemetry.addData("claw sensor: ", robot.claw.sensor.getRange());
            telemetry.addData("v4b position target: ", robot.v4b.getAngle());
            telemetry.addData("v4b1 position: ", (robot.v4b.v4b1.servo.getPosition()*180));
            telemetry.addData("slides target: ", robot.slides.target);
            telemetry.addData("slides pos: ", robot.slides.slides1.motor.getCurrentPosition());
            telemetry.addData("slides power", robot.slides.power1);
            telemetry.addData("slides level: ", robot.slides.currentLevel);
            telemetry.addData("voltage: ", robot.slides.voltageSensor.getVoltage());
            telemetry.addData("retractpos", robot.retractodo.state);
            telemetry.addData("guidepos", robot.guide.state);
            telemetry.update();

        }
    }
}