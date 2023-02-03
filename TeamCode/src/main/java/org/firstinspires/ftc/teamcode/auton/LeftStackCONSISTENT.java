package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.subsystems.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(group = "drive")

public class LeftStackCONSISTENT extends LinearOpMode {

    Robot robot;
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

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot = new Robot(hardwareMap, true);
        Pose2d startPose = new Pose2d(in(92), in(160), rad(90));
        drive.setPoseEstimate(startPose);
        robot.claw.setPositionClaw(0.8);
        robot.autoInit(true);
        robot.retractodo.setRetractUp();

        TrajectorySequence preloadTrajectory = drive.trajectorySequenceBuilder(startPose)
                .back(50)
                .addDisplacementMarker(1, ()-> {
                    robot.autoHigh(true);
                    robot.guide.setGuideDown();
                })
                .setReversed(true)
                .addDisplacementMarker(32, ()-> {
                    robot.slides.runToPreset(Levels.AUTOHIGH);
                })
                .splineTo(new Vector2d(30.5,7), Math.toRadians(221.780999988))
                .addDisplacementMarker(54,()->{
                    robot.slides.runToPosition(-310);
                })
                .addTemporalMarker(2.3, ()->{
                    robot.claw.setClawOpen();
                })
                .build();

        TrajectorySequence poleToStackTrajectory1 = drive.trajectorySequenceBuilder(preloadTrajectory.end())
                .addTemporalMarker(0, ()->{
                    robot.autoLow(true);
                })
                .waitSeconds(0.5)
                .setReversed(false)
                .splineTo(new Vector2d(56.5,8.3), Math.toRadians(0))
                .addTemporalMarker(1.4, ()->{
                    robot.claw.setClawClose();
                })
                .addTemporalMarker(2.6, ()->{
                    robot.slides.runToPreset(Levels.AUTOHIGH);
                })
                .build();

        TrajectorySequence stackToHighTrajectory1 = drive.trajectorySequenceBuilder(poleToStackTrajectory1.end())
                .setReversed(true)
                .splineTo(new Vector2d(31,6), Math.toRadians(221.780999988))
                .addTemporalMarker(1, ()->{
                    robot.autoHigh(true);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(1.3, ()->{
                    robot.slides.runToPosition(-220);
                })
                .addTemporalMarker(1.7,()->{
                    robot.claw.setClawOpen();
                })
                .addTemporalMarker(1.5, ()->{
                    robot.guide.setGuideUp();
                })
                .waitSeconds(0.2)
                .build();

        TrajectorySequence poleToStackTrajectory2 = drive.trajectorySequenceBuilder(stackToHighTrajectory1.end())
                .addTemporalMarker(0, ()->{
                    robot.autoLow(true);
                })
                .addTemporalMarker(0.7, ()->{
                    robot.guide.setGuideDown();
                })
                .waitSeconds(0.5)
                .setReversed(false)
                .splineTo(new Vector2d(56.5,8.3), Math.toRadians(0))
                .addTemporalMarker(1.4, ()->{
                    robot.claw.setClawClose();
                })
                .addTemporalMarker(2.6, ()->{
                    robot.slides.runToPreset(Levels.AUTOHIGH);
                })
                .build();

        TrajectorySequence stackToHighTrajectory2 = drive.trajectorySequenceBuilder(poleToStackTrajectory2.end())
                .setReversed(true)
                .splineTo(new Vector2d(31,6), Math.toRadians(221.780999988))
                .addTemporalMarker(1, ()->{
                    robot.autoHigh(true);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(1.3, ()->{
                    robot.slides.runToPosition(-140);
                })
                .addTemporalMarker(1.7,()->{
                    robot.claw.setClawOpen();
                })
                .addTemporalMarker(1.5, ()->{
                    robot.guide.setGuideUp();
                })
                .waitSeconds(0.2)
                .build();

        TrajectorySequence poleToStackTrajectory3 = drive.trajectorySequenceBuilder(stackToHighTrajectory2.end())
                .addTemporalMarker(0, ()->{
                    robot.autoLow(true);
                })
                .addTemporalMarker(0.7, ()->{
                    robot.guide.setGuideDown();
                })
                .waitSeconds(0.5)
                .setReversed(false)
                .splineTo(new Vector2d(56.5,8.3), Math.toRadians(0))
                .addTemporalMarker(1.4, ()->{
                    robot.claw.setClawClose();
                })
                .addTemporalMarker(2.6, ()->{
                    robot.slides.runToPreset(Levels.AUTOHIGH);
                })
                .build();
        TrajectorySequence stackToHighTrajectory3 = drive.trajectorySequenceBuilder(poleToStackTrajectory3.end())
                .setReversed(true)
                .splineTo(new Vector2d(31,6), Math.toRadians(221.780999988))
                .addTemporalMarker(1, ()->{
                    robot.autoHigh(true);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(1.3, ()->{
                    robot.slides.runToPosition(-20);
                })
                .addTemporalMarker(1.7,()->{
                    robot.claw.setClawOpen();
                })
                .addTemporalMarker(1.5, ()->{
                    robot.guide.setGuideUp();
                })
                .waitSeconds(0.2)
                .build();

        TrajectorySequence poleToStackTrajectory4 = drive.trajectorySequenceBuilder(stackToHighTrajectory3.end())
                .addTemporalMarker(0, ()->{
                    robot.autoLow(true);
                })
                .addTemporalMarker(0.7, ()->{
                    robot.guide.setGuideDown();
                })
                .waitSeconds(0.5)
                .setReversed(false)
                .splineTo(new Vector2d(56.5,8.3), Math.toRadians(0))
                .addTemporalMarker(1.4, ()->{
                    robot.claw.setClawClose();
                })
                .addTemporalMarker(2.6, ()->{
                    robot.slides.runToPreset(Levels.AUTOHIGH);
                })
                .build();

        TrajectorySequence stackToHighTrajectory4 = drive.trajectorySequenceBuilder(poleToStackTrajectory4.end())
                .setReversed(true)
                .splineTo(new Vector2d(31,6), Math.toRadians(221.780999988))
                .addTemporalMarker(1, ()->{
                    robot.autoHigh(true);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(1.3, ()->{
                    robot.slides.runToPosition(0);
                })
                .addTemporalMarker(1.7,()->{
                    robot.claw.setClawOpen();
                })
                .addTemporalMarker(1.5, ()->{
                    robot.guide.setGuideUp();
                })
                .waitSeconds(0.2)
                .build();

        TrajectorySequence poleToStackTrajectory5 = drive.trajectorySequenceBuilder(stackToHighTrajectory4.end())
                .addTemporalMarker(0, ()->{
                    robot.autoLow(true);
                })
                .addTemporalMarker(0.7, ()->{
                    robot.guide.setGuideDown();
                })
                .waitSeconds(0.5)
                .setReversed(false)
                .splineTo(new Vector2d(56.5,8.3), Math.toRadians(0))
                .addTemporalMarker(1.4, ()->{
                    robot.claw.setClawClose();
                })
                .addTemporalMarker(2.6, ()->{
                    robot.slides.runToPreset(Levels.AUTOHIGH);
                })
                .build();

        TrajectorySequence stackToHighTrajectory5 = drive.trajectorySequenceBuilder(poleToStackTrajectory5.end())
                .setReversed(true)
                .splineTo(new Vector2d(31,6), Math.toRadians(221.780999988))
                .addTemporalMarker(1, ()->{
                    robot.autoHigh(true);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(1.3, ()->{
                    robot.slides.runToPosition(0);
                })
                .addTemporalMarker(1.7,()->{
                    robot.claw.setClawOpen();
                })
                .addTemporalMarker(1.5, ()->{
                    robot.guide.setGuideUp();
                })
                .waitSeconds(0.2)
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
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
                    telemetry.addData("Location: ", tagOfInterest.id);
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
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        robot.slides.launchAsThread(telemetry);
        robot.guide.setGuideDown();
        drive.followTrajectorySequence(preloadTrajectory);
        drive.followTrajectorySequence(poleToStackTrajectory1);
        drive.followTrajectorySequence(stackToHighTrajectory1);
        drive.followTrajectorySequence(poleToStackTrajectory2);
        drive.followTrajectorySequence(stackToHighTrajectory2);
        drive.followTrajectorySequence(poleToStackTrajectory3);
        drive.followTrajectorySequence(stackToHighTrajectory3);
        drive.followTrajectorySequence(poleToStackTrajectory4);
        drive.followTrajectorySequence(stackToHighTrajectory4);
        drive.followTrajectorySequence(poleToStackTrajectory5);
        drive.followTrajectorySequence(stackToHighTrajectory5);

        TrajectorySequence parkTrajectory = null;
        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            // insert trajectory code
            parkTrajectory = robot.drive.trajectorySequenceBuilder(stackToHighTrajectory5.end())
                    .waitSeconds(0.5)
                    .setReversed(false)
                    .splineTo(new Vector2d(60,9), 0)
                    .build();
        } else if (tagOfInterest.id == MIDDLE) {
            // insert trajectory code
            parkTrajectory = robot.drive.trajectorySequenceBuilder(stackToHighTrajectory5.end())
                    .waitSeconds(0.5)
                    .setReversed(false)
                    .splineTo(new Vector2d(36,12), 0)
                    .build();
        } else if (tagOfInterest.id == RIGHT) {
            // insert trajectory code
            parkTrajectory = robot.drive.trajectorySequenceBuilder(stackToHighTrajectory5.end())
                    .waitSeconds(0.5)
                    .setReversed(false)
                    .splineTo(new Vector2d(35,13), 0)
                    .back(22)
                    .build();
        }

        drive.followTrajectorySequence(parkTrajectory);
        robot.guide.setGuideUp();

        robot.slides.destroyThreads(telemetry);
        while (!isStopRequested() && opModeIsActive()) ;
    }

    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }

}