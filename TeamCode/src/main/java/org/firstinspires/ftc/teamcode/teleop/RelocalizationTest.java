package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.relocalization.Relocalization;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class RelocalizationTest extends LinearOpMode {
    public static int REFRESH_RATE_MS = 500;
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        Relocalization relocalizer = new Relocalization(hardwareMap, false);
        Pose2d poseEstimate;
        timer = new ElapsedTime();

        waitForStart();

        while (!isStopRequested()) {
            if (timer.time(TimeUnit.MILLISECONDS) >= REFRESH_RATE_MS) {
                poseEstimate = relocalizer.relocalize();

                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.addData("sensorFrontLeft", relocalizer.getDistance(Relocalization.DistanceSensor.FRONT_LEFT));
                telemetry.addData("sensorFrontRight", relocalizer.getDistance(Relocalization.DistanceSensor.FRONT_LEFT));
                telemetry.addData("sensorLeft", relocalizer.getDistance(Relocalization.DistanceSensor.FRONT_LEFT));
                telemetry.addData("sensorRight", relocalizer.getDistance(Relocalization.DistanceSensor.FRONT_LEFT));
                telemetry.update();

                timer.reset();
            }
        }
    }
}