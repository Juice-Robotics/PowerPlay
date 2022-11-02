package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

// IMPORT SUBSYSTEMS
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.slides.Slides;


public class Robot {

    // SUBSYSTEM DECLARATIONS
    public Component[] components;
    public Claw claw;
    public Slides slides;

    // STATE VARS
    // example: clawToggled = false;


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

            new StepperServo(0, "clawXServo1", map),      //8
            new StepperServo(1, "clawXServo2", map),      //9

            new StepperServo(0, "clawYServo", map),       //10

            new StepperServo(1, "clawServo", map),        //11
        };

        // INIT SUBSYSTEMS
        this.claw = new Claw((StepperServo) components[11], (StepperServo) components[8], (StepperServo) components[9], (StepperServo) components[10]);
        this.slides = new Slides((Motor) components[4], (Motor) components[5]);
    }


    // CONTROL FUNCTIONS

    //CLAW
    public void toggleClaw(boolean x) {
        this.claw.toggle();
    }

    public void startClawX(boolean direction) {
        this.claw.startXRotation(direction);
    }

    public void startClawY(boolean direction) {
        this.claw.startYRotation(direction);
    }

    public void resetClawRotation(boolean b) {
        this.claw.resetRotation(Axis.ALL);
    }


    // SLIDES
    public void slidesGroundPreset(boolean pad_down) {
        this.slides.runToPreset(Levels.GROUND);
    }

    public void slidesLowPreset(boolean pad_left) {
        this.slides.runToPreset(Levels.LOW);
    }

    public void slidesMediumPreset(boolean pad_right) {
        this.slides.runToPreset(Levels.MEDIUM);
    }

    public void slidesHighPreset(boolean pad_up) {
        this.slides.runToPreset(Levels.HIGH);
    }


    //DRIVE
    public void setDrivePower(double x, double y, double rx) {
        double powerFrontLeft = y + x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackLeft = y - x + rx;
        double powerBackRight = y + x - rx;

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
        backLeft.setSpeed((float)powerBackLeft);
        backRight.setSpeed((float)powerBackRight);
    }
}