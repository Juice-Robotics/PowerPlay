package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;

@TeleOp(group = "competition")
public class TeleOpMain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize robot class
        Robot robot = new Robot(hardwareMap,false);

        double x;
        double y;
        double rx;

        boolean previousClawState = false;
        boolean previousClawSensorState = false;
        boolean autoCloseEnabled = true;
        boolean autoClosePreviousState = false;
        ElapsedTime clawSensorTimeout = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            //DRIVE
            if (gamepad1.right_bumper){
                x = gamepad1.right_stick_x*0.25;
                y = -gamepad1.left_stick_y*0.25;
                rx = gamepad1.left_stick_x*0.25;

            } else {
                x = gamepad1.right_stick_x;
                y = -gamepad1.left_stick_y;
                rx = gamepad1.left_stick_x;
            }
            robot.setDrivePower(-x, y, rx);

            //PRESETS
            if (gamepad1.dpad_down)
                robot.groundPreset(gamepad1.dpad_down);
            if (gamepad1.dpad_left)
                robot.lowPreset(gamepad1.dpad_left);
            if (gamepad1.dpad_right)
                robot.mediumPreset(gamepad1.dpad_right);
            if (gamepad1.left_bumper)
                robot.highPreset(gamepad1.left_bumper);

            //CLAW
            if (gamepad1.cross && !previousClawState)
                robot.advancedToggleClaw(gamepad1.x);
            if (gamepad2.left_stick_y > 0.2) {
                robot.startClawX(true);
            } else if (gamepad2.left_stick_y < -0.2) {
                robot.startClawX(false);
            }

            if (gamepad1.circle && !autoClosePreviousState) {
                autoCloseEnabled = !autoCloseEnabled;
            }

            if (!previousClawSensorState && robot.claw.sensor.conePresent() && clawSensorTimeout.time() >= 2000 && autoCloseEnabled) {
                gamepad1.rumble(500);
                robot.claw.toggle();
            }

            if (previousClawSensorState && !robot.claw.sensor.conePresent() && clawSensorTimeout.time() >= 2000 && autoCloseEnabled) {
                clawSensorTimeout.reset();
            }

//            if (gamepad1.circle) {
//                robot.resetAllServos();
//            }

            if (gamepad2.left_trigger > 0.1) {
                robot.slides.runToPosition((int) (robot.slides.slides1.motor.getCurrentPosition() + (0.1*gamepad2.left_trigger)));
            } else if (gamepad2.right_trigger > 0.1) {
                robot.slides.runToPosition((int) (robot.slides.slides1.motor.getCurrentPosition() - (0.1*gamepad2.left_trigger)));
            }

            if (gamepad2.left_bumper) {
                robot.v4b.setAngle(robot.v4b.currentAngle - 0.1);
            } else if (gamepad2.right_bumper) {
                robot.v4b.setAngle(robot.v4b.currentAngle + 0.1);
            }

            if (gamepad2.dpad_up) {
                robot.claw.setXRotation((float) (robot.claw.clawX1.getAngle() + 0.1));
            } else if (gamepad2.dpad_down) {
                robot.claw.setXRotation((float) (robot.claw.clawX1.getAngle() - 0.1));
            }

            if (gamepad2.dpad_right) {
                robot.claw.clawY.setAngle((float) (robot.claw.clawY.getAngle() + 0.1));
            } else if (gamepad2.dpad_right) {
                robot.claw.clawY.setAngle((float) (robot.claw.clawY.getAngle() - 0.1));
            }


            robot.slides.update();
            telemetry.addData("v4b position target: ", robot.v4b.getAngle());
            telemetry.addData("v4b1 position: ", robot.v4b.v4b1.servo.getPosition());
            telemetry.addData("v4b2 position: ", robot.v4b.v4b2.servo.getPosition());
            telemetry.addData("x pressed?", gamepad1.cross);
            telemetry.addData("claw status: ", robot.claw.state);
            telemetry.addData("claw sensor: ", robot.claw.sensor.getRawARGB());
            telemetry.addData("d left? ", gamepad1.dpad_left);
            telemetry.addData("slides target: ", robot.slides.target);
            telemetry.addData("slides pos: ", robot.slides.slides1.motor.getCurrentPosition());
            telemetry.addData("slides power", robot.slides.power1);
            telemetry.addData("slides level: ", robot.slides.currentLevel);
            telemetry.addData("voltage: ", robot.slides.voltageSensor.getVoltage());
            telemetry.update();

            previousClawState = gamepad1.cross;
            previousClawSensorState = robot.claw.sensor.conePresent();
            autoClosePreviousState = gamepad1.circle;
        }
    }
}
