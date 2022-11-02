package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.lib.Component;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.Motor;

@TeleOp(group = "competition")
public class SlidesAndDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Component[] components = new Component[]{
                new Motor(3, "leftRear", hardwareMap, true),          //0 left odometer
                new Motor(2, "rightRear", hardwareMap, false),        //1 right odometer
                new Motor(1, "leftFront", hardwareMap, true),         //2 middle odometer
                new Motor(0, "rightFront", hardwareMap, false),       //3

                new Motor(1, "slides1", hardwareMap, true),           //4
                new Motor(2, "slides2", hardwareMap, true),           //5
        };

        Motor slides1 = (Motor) components[4];
        Motor slides2 = (Motor) components[5];
//        slides1.setTarget(1);
//        slides1.motor.setTargetPosition(1);
//        slides2.setTarget(1);
//        slides2.motor.setTargetPosition(1);
//        slides1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slides2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides2.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        double x;
        double y;
        double rx;

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            //DRIVE
            if (gamepad1.right_bumper){
                x = -gamepad1.left_stick_x*0.25;
                y = -gamepad1.left_stick_y*0.25;
                rx = gamepad1.right_stick_x*0.25;

            } else{
                x = -gamepad1.left_stick_x;
                y = -gamepad1.left_stick_y;
                rx = gamepad1.right_stick_x;
            }


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

//        //SLIDES
//        slides.updateDistance();
//        if (gamepad1.dpad_down && !previousGamepad1.dpad_down)
//            slides.runToPreset(Levels.GROUND);
//        else if (gamepad1.dpad_left && !previousGamepad1.dpad_left)
//            slides.runToPreset(Levels.LOW);
//        else if (gamepad1.dpad_right && !previousGamepad1.dpad_right)
//            slides.runToPreset(Levels.MEDIUM);
//        else if (gamepad1.left_bumper && !previousGamepad1.left_bumper)
//            slides.runToPreset(Levels.HIGH);
        if (gamepad1.left_trigger > 0.5) {
            slides1.setSpeed(1);
            slides2.setSpeed(1);
        } else if (gamepad1.left_trigger < 0.5) {
            slides1.setSpeed(0);
            slides2.setSpeed(0);
        }

        try {
            previousGamepad1.copy(gamepad1);
            previousGamepad2.copy(gamepad2);
        } catch (RobotCoreException e) {

        }
    }
}

