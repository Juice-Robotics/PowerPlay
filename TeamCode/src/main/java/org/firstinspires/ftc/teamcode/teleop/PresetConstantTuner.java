package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.lib.Component;
import org.firstinspires.ftc.teamcode.lib.Motor;
import org.firstinspires.ftc.teamcode.lib.StepperServo;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.slides.Slides;
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4B;

@Config
@TeleOp
public class PresetConstantTuner extends OpMode {

    public static double CLAW_Y = 0;
    public static double V4B_1 = 0;
    public static boolean SLIDES_ENABLED = true;
    public static int SLIDES = 0;
    Component[] components;
    Claw claw;
    V4B v4b;
    Slides slides;

    @Override
    public void init() {
        components = new Component[]{
                new StepperServo(0, "v4bServo1", hardwareMap),        //0
                new StepperServo(1, "v4bServo2", hardwareMap),        //1

                new StepperServo(0, "clawYServo", hardwareMap),      //2
                new StepperServo(1, "clawYServo", hardwareMap),      //3

                new StepperServo(0, "clawYServo", hardwareMap),       //4

                new StepperServo(1, "clawServo", hardwareMap),        //5

                new Motor(1, "slides1", hardwareMap, false), //6
                new Motor(1, "slides2", hardwareMap, false), //7
        };
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // INIT SUBSYSTEMS
        claw = new Claw((StepperServo) components[5], (StepperServo) components[2], (StepperServo) components[3], (StepperServo) components[4], hardwareMap.get(ColorSensor.class, "clawSensor"));
        v4b = new V4B((StepperServo) components[0], (StepperServo) components[1]);
        slides = new Slides((Motor) components[6], (Motor) components[7], voltageSensor);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
//        claw.clawX1.setAngle((float) CLAW_X_1*180);
//        claw.clawX2.setAngle((float) CLAW_X_2*180);
        claw.clawY.servo.setPosition(CLAW_Y);

        v4b.v4b1.servo.setPosition(V4B_1);
        v4b.v4b2.servo.setPosition(1-V4B_1);


        if (SLIDES_ENABLED) {
            slides.runToPosition(SLIDES);
            slides.update();
        }

        telemetry.addData("claw y target", CLAW_Y);
        telemetry.addData("claw y", claw.clawY.servo.getPosition());
        telemetry.addData("v4b1 target", V4B_1);
        telemetry.addData("v4b1", v4b.v4b1.servo.getPosition());
        telemetry.addData("v4b2 target", V4B_1);
        telemetry.addData("v4b2", v4b.v4b2.servo.getPosition());
        telemetry.addData("slides pos1 ", slides.slides1.motor.getCurrentPosition());
        telemetry.addData("slides pos2 ", slides.slides2.motor.getCurrentPosition());
        telemetry.addData("slides power 1", slides.power1);
        telemetry.addData("slides power 2", slides.power2);
        telemetry.addData("slides target ", slides.target);
        telemetry.update();
        telemetry.update();
    }
}