package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

// IMPORT SUBSYSTEMS
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.relocalization.Relocalization;
import org.firstinspires.ftc.teamcode.subsystems.retractOdo.retractOdo;
import org.firstinspires.ftc.teamcode.subsystems.slides.Slides;
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4B;


public class Robot {

    // SUBSYSTEM DECLARATIONS
    public Component[] components;
    public Claw claw;
    public Slides slides;
    public V4B v4b;
    public retractOdo retractodo;
    public Relocalization relocalizer;


    // STATE VARS
    // example: clawToggled = false;
    Levels currentPosition = Levels.GROUND;


    // POSE
//    public Pose robotPose = new Pose(
//        Math.PI/2, Math.PI/2, 0.0,
//        7.41830709, -7.41830709, 0.5748031,
//        0.5748031, 0.5748031, 3.75,
//        0.6968503935
//    );

    // AUTON CONSTANTS
    public SampleMecanumDrive drive;
    boolean auton;

    public enum StarterStack {
        FIVE,
        FOUR,
        THREE,
        TWO,
        ONE
    }

    public Robot(HardwareMap map, boolean auton){
        this.auton = auton;

        this.drive = new SampleMecanumDrive(map);

        this.components = new Component[]{
                new Motor(3, "leftRear", map, true),          //0 left odometer
                new Motor(2, "rightRear", map, false),        //1 right odometer
                new Motor(1, "leftFront", map, true),         //2 middle odometer
                new Motor(0, "rightFront", map, false),       //3

                new Motor(1, "slides1", map, true),           //4
                new Motor(2, "slides2", map, true),           //5

                new StepperServo(0, "v4bServo1", map),        //6
                new StepperServo(1, "v4bServo2", map),        //7

                new StepperServo(0, "clawYServo", map),       //10

                new StepperServo(1, "clawServo", map),        //11

                new StepperServo(1, "retractOdo", map),       //12
        };

        VoltageSensor voltageSensor = map.voltageSensor.iterator().next();

        // INIT SUBSYSTEMS
        this.claw = new Claw((StepperServo) components[11], (StepperServo) components[10], map.colorSensor.get("colorSensor"));
        this.slides = new Slides((Motor) components[4], (Motor) components[5], voltageSensor);
        this.v4b = new V4B((StepperServo) components[6], (StepperServo) components[7]);
        this.retractodo = new retractOdo((StepperServo) components[12]);
        this.relocalizer = new Relocalization(map, false);
    }


    // CONTROL FUNCTIONS

    //RETRACTODO
    public void toggleRetract() {
        this.retractodo.toggle();
    }

    //CLAW
    public void advancedToggleClaw() {
        if (currentPosition == Levels.GROUND) {
            this.claw.toggle();
        } else {
            this.slides.runToPreset(Levels.GROUND);
            this.claw.toggle();
//            try {
//                this.slides.launchAsThreadBasic();
//                Thread.sleep(400);
//                this.slides.destroyThreadsBasic();
//            } catch (Exception e) {}
//            autoLow(true);
            Thread thread = new Thread(new Runnable() {
                public void run() {
                    try {
                        Thread.sleep(400);
                    } catch (Exception e) {
                    }
                    autoLow(true);
                }});
            thread.start();
        }
    }

    public void startClawY(boolean direction) {
        this.claw.startYRotation(direction);
    }

    public void resetClawRotation(boolean b) {
        this.claw.resetRotation(Axis.ALL);
    }

    public void update() {
        slides.update();
    }

    // SLIDES + V4B + CLAW PRESETS
    public void groundPreset(boolean pad_down) {
        this.slides.runToPreset(Levels.GROUND);
        try {
            Thread.sleep(100);
        } catch (Exception e) {}
        this.v4b.runToPreset(Levels.GROUND);
        this.claw.setYRotation(2);
        currentPosition = Levels.GROUND;
    }

    public void lowPreset(boolean pad_left) {
        this.slides.runToPreset(Levels.LOW);
        this.v4b.runToPreset(Levels.LOW);
        this.claw.setYRotation(142);
        currentPosition = Levels.LOW;
    }

    public void mediumPreset(boolean pad_right) {
        this.slides.runToPreset(Levels.MEDIUM);
        this.v4b.runToPreset(Levels.MEDIUM);
        this.claw.setYRotation(142);
        currentPosition = Levels.MEDIUM;
    }

    public void highPreset(boolean pad_up) {
        this.slides.runToPreset(Levels.HIGH);
        try {
            Thread.sleep(100);
        } catch (Exception e) {}
        this.v4b.runToPreset(Levels.HIGH);
        this.claw.setYRotation(142);
        currentPosition = Levels.HIGH;
    }

    public void sidewaysPickup(boolean pad_up) {
        this.slides.runToPreset(Levels.HIGH);
        try {
            Thread.sleep(100);
        } catch (Exception e) {}
        this.v4b.runToPreset(Levels.HIGH);
        this.claw.setYRotation(62);
        currentPosition = Levels.HIGH;
    }

    public void starterStack1(boolean pad_left) {
        this.slides.runToPreset(Levels.STARTSTACK1);
        try {
            Thread.sleep(100);
        } catch (Exception e) {}
        this.v4b.runToPreset(Levels.GROUND);
        this.claw.setYRotation(0);
    }

    public void autoHigh(boolean pad_up) {
        this.v4b.runToPreset(Levels.AUTOHIGH);
        this.claw.setYRotation(142);
    }

    public void autoLow(boolean pad_down) {
        try {
            Thread.sleep(100);
        } catch (Exception ignored) {}
        this.v4b.runToPreset(Levels.GROUND);
        this.claw.setYRotation(2);
    }

    public void autoInit(boolean pad_left) {
        this.v4b.runToPreset(Levels.AUTOINIT);
        this.claw.setYRotation(142);
    }


//    public void robotOff(boolean pad_left, int ticks) {
//        this.slides.runToPosition(ticks);
//        this.claw.setXRotation(0);
//        try {
//            Thread.sleep(100);
//        } catch (Exception e) {}
//        this.v4b.runToPreset(Levels.GROUND);
//        this.claw.setYRotation(0);
//        this.guide.toggle();
//    }

    public void resetAllServos() {
        this.v4b.setAngle(0);
        this.claw.clawY.servo.setPosition(0);
        this.retractodo.setRetractServoRotation(0);
    }


    //DRIVE
    public void setDrivePower(double x, double y, double rx) {
        double powerFrontLeft = y + x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackLeft = (y - x + rx) * -1;
        double powerBackRight = (y + x - rx) * -1;

        if (Math.abs(powerFrontLeft) > 1 || Math.abs(powerBackLeft) > 1 ||
                Math.abs(powerFrontRight) > 1 || Math.abs(powerBackRight) > 1) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(powerFrontLeft), Math.abs(powerBackLeft));
            max = Math.max(Math.abs(powerFrontRight), max);
            max = Math.max(Math.abs(powerBackRight), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            powerFrontLeft /= max;
            powerBackLeft /= max;
            powerFrontRight /= max;
            powerBackRight /= max;
        }
        Motor backLeft = (Motor) components[0];
        Motor backRight = (Motor) components[1];
        Motor frontLeft = (Motor) components[2];
        Motor frontRight = (Motor) components[3];
        frontLeft.setSpeed((float)powerFrontLeft);
        frontRight.setSpeed((float)powerFrontRight);
        backLeft.setSpeed(-(float)powerBackLeft);
        backRight.setSpeed(-(float)powerBackRight);
    }

    public void relocalize() {
        Pose2d newPoseEstimate = relocalizer.relocalize();
        drive.setPoseEstimate(newPoseEstimate);
    }

    public void safeRelocalize() {
        Pose2d newPoseEstimate = relocalizer.relocalize();
        if (Math.abs((newPoseEstimate.getX() - drive.getPoseEstimate().getX())) < 5 || Math.abs((newPoseEstimate.getY() - drive.getPoseEstimate().getY())) < 5 || Math.abs((newPoseEstimate.getHeading() - drive.getPoseEstimate().getHeading())) < Math.toRadians(10)) {
            drive.setPoseEstimate(newPoseEstimate);
        }
    }
}