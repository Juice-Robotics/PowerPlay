package org.firstinspires.ftc.teamcode.subsystems.relocalization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.outoftheboxrobotics.photoncore.Neutrino.MB1242.MB1242Ex;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;

public class Relocalization {
    private MB1242Ex frontLeftSensor;
    private MB1242Ex frontRightSensor;
    private MB1242Ex leftSensor;
    private MB1242Ex rightSensor;
    private final boolean reversed;

    public Pose2d poseEstimate;
    public double x;
    public double y;
    public double heading;

    public double a = 0.8;

    private double frontLeftRaw;
    private double frontRightRaw;
    private double leftRaw;
    private double rightRaw;
    private double sideRaw;

    private double previousFrontLeft;
    private double previousFrontRight;
    private double previousSide;

    private DistanceUnit unit = DistanceUnit.CM;

    // CONSTANTS
    private double DISTANCE_BETWEEN_FRONT = 325.848;
    private double FRONT_TO_ORIGIN_OFFSET = 0;
    private double OFFSET_SIDE_X = 0;
    private double OFFSET_SIDE_Y = 0;

    private double STARTER_STACK_WALL_X = 72;
    private double ADJACENT_WALL_Y = 72;

    public enum DistanceSensor {
        FRONT_LEFT,
        FRONT_RIGHT,
        LEFT,
        RIGHT,
        SIDE
    }

//    public Relocalization(HardwareMap hardwareMap, Pose2d initialPose, boolean reversed) {
//        frontLeftSensor = new MB1242Ex((I2cDeviceSynch) hardwareMap.get("frontRightDistanceSensor"));
//        frontRightSensor = new MB1242Ex((I2cDeviceSynch) hardwareMap.i2cDevice.get("frontRightDistanceSensor"));
//        leftSensor = new MB1242Ex((I2cDeviceSynch) hardwareMap.i2cDevice.get("leftDistanceSensor"));
//        rightSensor = new MB1242Ex((I2cDeviceSynch) hardwareMap.i2cDevice.get("rightDistanceSensor"));
//        this.reversed = reversed;
//
//        poseEstimate = initialPose;
//        x = initialPose.getX();
//        y = initialPose.getY();
//        heading = initialPose.getHeading();
//    }

    public Relocalization(HardwareMap hardwareMap, boolean reversed) {
        frontLeftSensor = hardwareMap.get(MB1242Ex.class, "frontLeftDistanceSensor");
        frontRightSensor = hardwareMap.get(MB1242Ex.class, "frontRightDistanceSensor");
        leftSensor = hardwareMap.get(MB1242Ex.class, "leftDistanceSensor");
        rightSensor = hardwareMap.get(MB1242Ex.class, "rightDistanceSensor");
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
        frontLeftRaw = lowPassFilter(frontLeftSensor.getDistance(unit), previousFrontLeft);
        frontRightRaw = lowPassFilter(frontRightSensor.getDistance(unit), previousFrontRight);
        sideRaw = getSideMeasurement();
        previousFrontLeft = frontLeftRaw;
        previousFrontRight = frontRightRaw;
        previousSide = sideRaw;

        frontLeftRaw = lowPassFilter(frontLeftSensor.getDistance(unit), previousFrontLeft);
        frontRightRaw = lowPassFilter(frontRightSensor.getDistance(unit), previousFrontRight);
        sideRaw = getSideMeasurement();
        previousFrontLeft = frontLeftRaw;
        previousFrontRight = frontRightRaw;
        previousSide = sideRaw;

        frontLeftRaw = lowPassFilter(frontLeftSensor.getDistance(unit), previousFrontLeft);
        frontRightRaw = lowPassFilter(frontRightSensor.getDistance(unit), previousFrontRight);
        sideRaw = getSideMeasurement();
        previousFrontLeft = frontLeftRaw;
        previousFrontRight = frontRightRaw;
        previousSide = sideRaw;

        // CALCULATE HEADING
        if (frontLeftRaw == frontRightRaw) {
            // PARALLEL TO WALL
            heading = 0;
        } else if (frontLeftRaw < frontRightRaw) {
            // TILTED RIGHT
            angleToWall = Math.atan((frontRightRaw - frontLeftRaw) / DISTANCE_BETWEEN_FRONT);

            heading = Math.toRadians(90 - angleToWall);
        } else {
            // TILTED LEFT
            angleToWall = Math.atan((frontLeftRaw - frontRightRaw) / DISTANCE_BETWEEN_FRONT);

            heading = Math.toRadians(270 + angleToWall);
        }

        // CALCULATE X
        if (frontLeftRaw == frontRightRaw) {
            x = STARTER_STACK_WALL_X - toInch(((frontRightRaw + frontLeftRaw) / 2));
        } else {
            x = STARTER_STACK_WALL_X - toInch((Math.cos(90 - angleToWall) * (((frontLeftRaw + frontRightRaw) / 2) + FRONT_TO_ORIGIN_OFFSET)));
        }

        // CALCULATE Y
        y = ADJACENT_WALL_Y - toInch(Math.hypot((sideRaw + OFFSET_SIDE_X), (OFFSET_SIDE_Y)) * Math.cos(90 - Math.atan(heading / (sideRaw + OFFSET_SIDE_X)) - heading));

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

    public double getCachedDistance(DistanceSensor sensor) {
        switch (sensor) {
            case FRONT_LEFT:
                return frontLeftRaw;
            case FRONT_RIGHT:
                return frontRightRaw;
            case LEFT:
                return leftRaw;
            case RIGHT:
                return rightRaw;
            case SIDE:
                return sideRaw;
        }
        return -1;
    }

    private double lowPassFilter(double measurement, double previousEstimate) {
        return (a * previousEstimate) + (1 - a) * measurement;
    }

    private double getSideMeasurement() {
        if (reversed) {
            rightRaw = lowPassFilter(rightSensor.getDistance(unit), previousSide);
            return rightRaw;
        } else {
            leftRaw = lowPassFilter(leftSensor.getDistance(unit), previousSide);
            return leftRaw;
        }
    }

    private double toInch(double mm) {
        return mm / 25.4;
    }
}
