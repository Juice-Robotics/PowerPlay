package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.CycleCommand;
import org.firstinspires.ftc.teamcode.commands.HighJunctionCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.subsystems.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "LeftAuto")
@Config
public class LeftCommandCycles extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1, 2, 3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    boolean start = true;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        Robot robot = new Robot(hardwareMap, true);
        Pose2d startPose = new Pose2d(-36, 58, Math.toRadians(-90));
        robot.drive.setPoseEstimate(startPose);
        robot.claw.claw.servo.setPosition(0.8);



//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(8);
//        PhotonCore.enable();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        camera.stopStreaming();

        TrajectorySequence toPreload = robot.drive.trajectorySequenceBuilder(startPose)
                .back(47)
                .setReversed(true)
                .splineTo(new Vector2d(30.5,8.5), 179.7)
                .build();

        TrajectorySequence toStarterStack = robot.drive.trajectorySequenceBuilder(toPreload.end())
                .setReversed(false)
                .splineTo(new Vector2d(60,11), 0)
                .build();

        TrajectorySequence parkTrajectory = null;

        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            // insert trajectory code
            parkTrajectory = robot.drive.trajectorySequenceBuilder(toStarterStack.end())
                    .setReversed(false)
                    .splineTo(new Vector2d(35,13), 0)
                    .forward(22)
                    .build();
        } else if (tagOfInterest.id == MIDDLE) {
            // insert trajectory code
            parkTrajectory = robot.drive.trajectorySequenceBuilder(toStarterStack.end())
                    .setReversed(false)
                    .splineTo(new Vector2d(35,13), 0)
                    .build();
        } else if (tagOfInterest.id == RIGHT) {
            // insert trajectory code
            parkTrajectory = robot.drive.trajectorySequenceBuilder(new Pose2d(30, 10, 180))
                    .setReversed(false)
                    .splineTo(new Vector2d(35,13), 0)
                    .back(22)
                    .build();
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new TrajectorySequenceCommand(toPreload),
                        new HighJunctionCommand(robot),
                        new TrajectorySequenceCommand(toStarterStack),
                        new CycleCommand(robot, toStarterStack.end(), Robot.StarterStack.FIVE),
                        new CycleCommand(robot, new Pose2d(30, 10, 180), Robot.StarterStack.FOUR),
                        new CycleCommand(robot, new Pose2d(30, 10, 180), Robot.StarterStack.THREE),
                        new CycleCommand(robot, new Pose2d(30, 10, 180), Robot.StarterStack.TWO),
                        new TrajectorySequenceCommand(parkTrajectory)
                )
        );

        while (opModeIsActive()) {
            if (start) {
                start = false;
                robot.resetEncoders();
            }
            CommandScheduler.getInstance().run();
//            PhotonCore.CONTROL_HUB.clearBulkCache();
//            PhotonCore.EXPANSION_HUB.clearBulkCache();
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
