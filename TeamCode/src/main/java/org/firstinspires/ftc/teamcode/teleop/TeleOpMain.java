package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;

import java.util.concurrent.TimeUnit;

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

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(4);
        PhotonCore.enable();

        ElapsedTime timer;
        ElapsedTime matchTimer;
        timer = new ElapsedTime();

        int buzzers = 0;

        boolean autoCloseEnabled = true;
        boolean autoClosePreviousState = false;
        boolean previousClawState = false;
        boolean previousRetractState = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;
        matchTimer = new ElapsedTime();

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

            if (gamepad2.left_trigger > 0.1) {
                robot.slides.runToPosition((int) (robot.slides.slides1.motor.getCurrentPosition() + 70));
            } else if (gamepad2.right_trigger > 0.1) {
                robot.slides.runToPosition((int) (robot.slides.slides1.motor.getCurrentPosition() - 70));
            }

            if (gamepad2.right_stick_x > 0.2) {
                robot.startClawY(true);
            } else if (gamepad2.right_stick_x < -0.2) {
                robot.startClawY(false);
            }

            boolean isPressed2 = gamepad1.square;
            if (gamepad1.square && !previousRetractState) {
                robot.toggleRetract();
            }
            previousRetractState = isPressed2;

            // TIME ALERTS
            if (buzzers == 0 && matchTimer.time(TimeUnit.SECONDS) >= 75) {
                gamepad1.rumble(500);
                gamepad2.rumble(500);
                buzzers = 1;
            } else if (buzzers == 1 && matchTimer.time(TimeUnit.SECONDS) >= 90) {
                gamepad1.rumble(500);
                gamepad2.rumble(500);
                buzzers = 2;
            }

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
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();

        }
    }
} //test