package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Component;
import org.firstinspires.ftc.teamcode.lib.Motor;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp(group = "competition")
public class DriveOnly extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Component[] components = new Component[]{
                new Motor(3, "leftRear", hardwareMap, true),          //0 left odometer
                new Motor(2, "rightRear", hardwareMap, false),        //1 right odometer
                new Motor(1, "leftFront", hardwareMap, true),         //2 middle odometer
                new Motor(0, "rightFront", hardwareMap, false),       //3
        };

        double x;
        double y;
        double rx;

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
        }
    }

