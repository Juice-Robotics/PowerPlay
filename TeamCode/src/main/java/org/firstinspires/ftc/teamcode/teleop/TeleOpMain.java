package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(group = "competition")
public class TeleOpMain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your own robot class
        Robot robot = new Robot(hardwareMap,false);

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
            robot.setDrivePower(-x, y, rx);

            //SLIDES
            if (gamepad1.dpad_down && !previousGamepad1.dpad_down)
                robot.slidesGroundPreset(gamepad1.dpad_down);
            else if (gamepad1.dpad_left && !previousGamepad1.dpad_left)
                robot.slidesLowPreset(gamepad1.dpad_left);
            else if (gamepad1.dpad_right && !previousGamepad1.dpad_right)
                robot.slidesMediumPreset(gamepad1.dpad_right);
            else if (gamepad1.left_bumper && !previousGamepad1.left_bumper)
                robot.slidesHighPreset(gamepad1.left_bumper);

            //CLAW
            if (gamepad1.x && !previousGamepad1.x)
                robot.toggleClaw(gamepad1.x);
            if (gamepad2.left_stick_y > 0.2) {
                robot.startClawX(true);
            } else if (gamepad2.left_stick_y < -0.2) {
                robot.startClawX(false);
            }

            if (gamepad2.right_stick_x > 0.2) {
                robot.startClawY(true);
            } else if (gamepad2.right_stick_x < -0.2) {
                robot.startClawY(false);
            }

//            if (gamepad2.b && !previousGamepad2.b)
//                robot.resetClawRotation(gamepad2.b);
//            if (gamepad1.)

            try {
                previousGamepad1.copy(gamepad1);
                previousGamepad2.copy(gamepad2);
            } catch (RobotCoreException e) {
                
            }
        }
    }
}
