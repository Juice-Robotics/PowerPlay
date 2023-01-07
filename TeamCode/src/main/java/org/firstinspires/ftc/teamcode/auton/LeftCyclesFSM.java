package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class LeftCyclesFSM extends LinearOpMode {
    enum AutonStates {
        INIT,
        PRELOAD,
        CYCLE,
        PARK,
        COMPLETE
    }

    AutonStates currentState = AutonStates.INIT;

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


    Robot robot;
    Pose2d startPose = new Pose2d(-36, 58, Math.toRadians(-90));

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, true);
        robot.slides.slides1.resetEncoder();
        robot.slides.slides2.resetEncoder();
        robot.drive.setPoseEstimate(startPose);

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

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        robot.slides.launchAsThread(telemetry);

        TrajectorySequence toPreloadJunction = robot.drive.trajectorySequenceBuilder(startPose)
                .back(47)
                .setReversed(true)
                .splineTo(new Vector2d(30.5, 8.5), 179.7)
                .build();

        TrajectorySequence preloadToStack = robot.drive.trajectorySequenceBuilder(toPreloadJunction.end())
                .setReversed(false)
                .splineTo(new Vector2d(60, 11), 0)
                .build();

        TrajectorySequence stackToHighJunction = robot.drive.trajectorySequenceBuilder(preloadToStack.end())
                .setReversed(true)
                .splineTo(new Vector2d(30, 10), 180)
                .build();

        TrajectorySequence highJunctionToStack = robot.drive.trajectorySequenceBuilder(stackToHighJunction.end())
                .setReversed(false)
                .splineTo(new Vector2d(60, 11), 0)
                .build();


        TrajectorySequence parkTrajectory = null;
        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            // insert trajectory code
            parkTrajectory = robot.drive.trajectorySequenceBuilder(highJunctionToStack.end())
                    .setReversed(false)
                    .splineTo(new Vector2d(35, 13), 0)
                    .forward(22)
                    .build();
        } else if (tagOfInterest.id == MIDDLE) {
            // insert trajectory code
            parkTrajectory = robot.drive.trajectorySequenceBuilder(highJunctionToStack.end())
                    .setReversed(false)
                    .splineTo(new Vector2d(35, 13), 0)
                    .build();
        } else if (tagOfInterest.id == RIGHT) {
            // insert trajectory code
            parkTrajectory = robot.drive.trajectorySequenceBuilder(highJunctionToStack.end())
                    .setReversed(false)
                    .splineTo(new Vector2d(35, 13), 0)
                    .back(22)
                    .build();
        }

        // Start loop, stops when opmode is stopped or it reaches complete state
        while (!isStopRequested() && currentState != AutonStates.COMPLETE) {
            switch (currentState) {
                case INIT:
                    // some init idk
                    currentState = AutonStates.PRELOAD;
                    break;
                case PRELOAD:
                    robot.drive.followTrajectorySequence(toPreloadJunction);

                    robot.highPreset(false);
                    robot.v4b.setAngle(218);

                    sleep(2000);
                    robot.claw.toggle();
                    sleep(1000);

                    robot.starterStackPreset(Robot.StarterStack.FIVE);

                    currentState = AutonStates.CYCLE;
                    break;
                case CYCLE:
                    // CYCLE 1
                    // Uses special path since it starts from the end of preload
                    robot.drive.followTrajectorySequence(preloadToStack);
                    robot.claw.toggle();
                    robot.slides.runToPosition(-700);

                    robot.drive.followTrajectorySequence(stackToHighJunction);
                    robot.highPreset(false);
                    robot.v4b.setAngle(218);

                    robot.claw.toggle();
                    robot.robotOff(true, -290);

                    // CYCLES 2 - n
                    for (int cycle = 2; cycle <= 4; cycle++) {
                        robot.drive.followTrajectorySequence(highJunctionToStack);
                        robot.claw.toggle();
                        robot.slides.runToPosition(-700);

                        robot.drive.followTrajectorySequence(stackToHighJunction);
                        robot.highPreset(false);
                        robot.v4b.setAngle(218);

                        robot.claw.toggle();
                        robot.robotOff(true, (-290 - (200 * cycle)));
                    }
                    currentState = AutonStates.PARK;
                    break;
                case PARK:
                    robot.drive.followTrajectorySequence(parkTrajectory);
                    currentState = AutonStates.COMPLETE;
                    break;
            }
        }

        robot.slides.destroyThreads(telemetry);
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
