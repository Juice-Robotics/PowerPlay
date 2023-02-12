package org.firstinspires.ftc.teamcode.subsystems.relocalization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Relocalization {
    private MB1242 frontLeftSensor;
    private MB1242 frontRightSensor;
    private MB1242 leftSensor;
    private MB1242 rightSensor;
    private boolean reversed;

    public Pose2d poseEstimate;
    public double x;
    public double y;
    public double heading;

    private double frontLeftRaw;
    private double frontRightRaw;
    private double leftRaw;
    private double rightRaw;

    private DistanceUnit unit = DistanceUnit.CM;

    // CONSTANTS
    private double DISTANCE_BETWEEN_FRONT = 325.848;
    private double FRONT_TO_CENTER_OFFSET = 0;
    private double LEFT_TO_CENTER_OFFSET = 0;
    private double RIGHT_TO_CENTER_OFFSET = 0;

    private double STARTER_STACK_WALL_X = 72;
    private double ADJACENT_WALL_Y = 72;

    public enum DistanceSensor {
        FRONT_LEFT,
        FRONT_RIGHT,
        LEFT,
        RIGHT
    }

    public Relocalization(HardwareMap hardwareMap, Pose2d initialPose, boolean reversed) {
        frontLeftSensor = new MB1242(hardwareMap, "frontLeftDistanceSensor");
        frontRightSensor = new MB1242(hardwareMap, "frontRightDistanceSensor");
        leftSensor = new MB1242(hardwareMap, "leftDistanceSensor");
        rightSensor = new MB1242(hardwareMap, "rightDistanceSensor");
        this.reversed = reversed;

        poseEstimate = initialPose;
        x = initialPose.getX();
        y = initialPose.getY();
        heading = initialPose.getHeading();
    }

    public Relocalization(HardwareMap hardwareMap, boolean reversed) {
        frontLeftSensor = new MB1242(hardwareMap, "flDistanceSensor");
        frontRightSensor = new MB1242(hardwareMap, "frDistanceSensor");
        leftSensor = new MB1242(hardwareMap, "lSensor");
        rightSensor = new MB1242(hardwareMap, "rDistanceSensor");
        this.reversed = reversed;

        poseEstimate = new Pose2d(0, 0, 0);
        x = 0;
        y = 0;
        heading = 0;
    }

    public Pose2d relocalize() {
        double angleToWall = 0;

        // GET SENSOR VALUES
        // get 3 bursts of detections so the low-pass will be as accurate as possible
        frontLeftRaw = frontLeftSensor.getDistance(unit);
        frontRightRaw = frontRightSensor.getDistance(unit);
        leftRaw = leftSensor.getDistance(unit);
        rightRaw = rightSensor.getDistance(unit);

        frontLeftRaw = frontLeftSensor.getDistance(unit);
        frontRightRaw = frontRightSensor.getDistance(unit);
        leftRaw = leftSensor.getDistance(unit);
        rightRaw = rightSensor.getDistance(unit);

        frontLeftRaw = frontLeftSensor.getDistance(unit);
        frontRightRaw = frontRightSensor.getDistance(unit);
        leftRaw = leftSensor.getDistance(unit);
        rightRaw = rightSensor.getDistance(unit);

        // CALCULATE HEADING
        if (frontLeftRaw == frontRightRaw) {
            // PARALLEL TO WALL
            heading = 0;
        } else if (frontLeftRaw < frontRightRaw) {
            // TILTED RIGHT
            angleToWall = Math.atan((frontRightRaw - frontLeftRaw) / DISTANCE_BETWEEN_FRONT);

            heading = Math.toRadians(90 - angleToWall);
        } else if (frontLeftRaw > frontRightRaw) {
            // TILTED RIGHT
            angleToWall = Math.atan((frontLeftRaw - frontRightRaw) / DISTANCE_BETWEEN_FRONT);

            heading = Math.toRadians(270 + angleToWall);
        }

        // CALCULATE X
        if (frontLeftRaw == frontRightRaw) {
            x = STARTER_STACK_WALL_X - ((frontRightRaw + frontLeftRaw) / 2);
        } else {
            x = STARTER_STACK_WALL_X - (Math.sin(90 - angleToWall) * (((frontLeftRaw + frontRightRaw) / 2) + FRONT_TO_CENTER_OFFSET));
        }

        // CALCULATE Y
        y = ADJACENT_WALL_Y - Math.cos(90 -angleToWall) * (frontLeftRaw + LEFT_TO_CENTER_OFFSET);

        // CLEANUP
        if (reversed) {
            x = -x;
            heading = Math.toRadians(90 - Math.toDegrees(heading));
        }

        poseEstimate = new Pose2d(x, y, heading);

        return poseEstimate;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public Pose2d getPoseEstimate () {
        return poseEstimate;
    }

    public double getDistance(DistanceSensor sensor) {
        switch (sensor) {
            case FRONT_LEFT:
                return frontLeftRaw;
            case FRONT_RIGHT:
                return frontRightRaw;
            case LEFT:
                return leftRaw;
            case RIGHT:
                return rightRaw;
        }
        return -1;
    }
}
