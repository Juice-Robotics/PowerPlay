package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.relocalization.Relocalization;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoAlign;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class RelocalizationTest extends LinearOpMode {
    public static int REFRESH_RATE_MS = 500;
    public static double a = 0.99;
    public static boolean DISTANCE_SENSORS = false;
    public static boolean CAMERA_ALIGN = true;
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d poseEstimate;
        timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Relocalization relocalizer = new Relocalization(hardwareMap, drive.gyro,false);
        AutoAlign autoAlign = new AutoAlign(drive);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (DISTANCE_SENSORS) {
                relocalizer.a = a;
                if (timer.time(TimeUnit.MILLISECONDS) >= REFRESH_RATE_MS) {
                    poseEstimate = relocalizer.relocalize();
                    drive.setPoseEstimate(poseEstimate);

                    telemetry.addData("x", poseEstimate.getX());
                    telemetry.addData("y", poseEstimate.getY());
                    telemetry.addData("heading", poseEstimate.getHeading());
                    telemetry.addData("sensorFrontLeft", relocalizer.getCachedDistance(Relocalization.DistanceSensor.FRONT_LEFT));
                    telemetry.addData("sensorFrontRight", relocalizer.getCachedDistance(Relocalization.DistanceSensor.FRONT_RIGHT));
                    telemetry.addData("sensorLeft", relocalizer.getCachedDistance(Relocalization.DistanceSensor.LEFT));
                    telemetry.addData("sensorRight", relocalizer.getCachedDistance(Relocalization.DistanceSensor.RIGHT));
                    telemetry.update();

                    timer.reset();
                }
            }

            if (CAMERA_ALIGN) {

            }
            drive.update();
        }
    }
}