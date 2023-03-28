package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(group = "competition")
public class TeleOpMain extends LinearOpMode {

    enum MODE {
        MANUAL,
        PIDF
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your own robot class
        Robot robot = new Robot(hardwareMap,false);
        SampleMecanumDriveCancelable autonDrive = new SampleMecanumDriveCancelable(hardwareMap);
        MODE slidesMode = MODE.PIDF;
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
        autonDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(0, 0, 0);
        boolean stackMode = false;

        double x;
        double y;
        double rx;

        boolean previousClawSensorState = false;
        ElapsedTime clawSensorTimeout = new ElapsedTime();

        TrajectorySequence autonCycle = autonDrive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    boolean coneSeen = false;
                    while (!coneSeen) {
                        if (robot.claw.sensor.conePresent()) {
                            coneSeen = true;
                        }
                    }
                    robot.claw.setClawClose();
                    robot.highPreset(true);
                })
                .back(10)
                .addTemporalMarker(5, () -> {
                    robot.advancedToggleClaw();
                    robot.groundPreset(true);
                })
                .waitSeconds(2)
                .forward(10)
                .build();

        boolean autoCloseEnabled = true;
        boolean autoClosePreviousState = false;
        boolean previousClawState = false;
        boolean previousRetractState = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {


                    if (gamepad1.dpad_up) {
                        robot.slides.resetAllEncoders();
                    }

                    //DRIVE
                    if (gamepad1.right_trigger > 0.1) {
                        x = -gamepad1.left_stick_x * (1 - 0.66 * gamepad1.right_trigger);
                        y = -gamepad1.left_stick_y * (1 - 0.66 * gamepad1.right_trigger);
                        rx = gamepad1.right_stick_x * (1 - 0.66 * gamepad1.right_trigger);

                    } else {
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
                    if (gamepad1.square) {
                        stackMode = !stackMode;
                    }

                    //CLAW
                    if (gamepad1.circle && !autoClosePreviousState) {
                        autoCloseEnabled = !autoCloseEnabled;
                    }

                    if (gamepad2.left_trigger > 0.1) {
                        slidesMode = MODE.MANUAL;
                        robot.slides.slides1.motor.setPower(gamepad2.left_trigger * 0.75);
                        robot.slides.slides2.motor.setPower(gamepad2.left_trigger * 0.75);
                    } else if (gamepad2.right_trigger > 0.1) {
                        slidesMode = MODE.MANUAL;
                        robot.slides.slides1.motor.setPower(gamepad2.left_trigger * 0.75);
                        robot.slides.slides2.motor.setPower(gamepad2.left_trigger * 0.75);
                    } else if (slidesMode == MODE.MANUAL) {
                        slidesMode = MODE.PIDF;
                        robot.slides.runToPosition(robot.slides.slides1.motor.getCurrentPosition());
                    }

                    if (gamepad2.left_bumper) {
                        robot.slides.runToPosition((int) (robot.slides.slides1.motor.getCurrentPosition() + 60));
                    } else if (gamepad2.right_bumper) {
                        robot.slides.runToPosition((int) (robot.slides.slides1.motor.getCurrentPosition() - 60));
                    }

                    if (gamepad2.x) {
                      robot.slides.runToPreset(Levels.AUTOINIT);
                    }

                    if (gamepad2.dpad_right) {
                        robot.claw.clawY.setAngle((float) (robot.claw.clawY.getAngle() + 1));
                    } else if (gamepad2.dpad_right) {
                        robot.claw.clawY.setAngle((float) (robot.claw.clawY.getAngle() - 1));
                    }

                    boolean isPressed = gamepad1.cross;
                    if (isPressed && !previousClawState && !stackMode) {
                        robot.advancedToggleClaw();
                    } else if (isPressed && !previousClawState) {
                        robot.advancedToggleClawStack();
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

                    boolean isPressed2 = gamepad1.square;
                    if (gamepad1.square && !previousRetractState) {
                        robot.toggleRetract();
                    }
                    previousRetractState = isPressed2;

                    previousClawSensorState = robot.claw.sensor.conePresent();
                    autoClosePreviousState = gamepad1.circle;




            if (slidesMode == MODE.PIDF) robot.slides.update();
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
}