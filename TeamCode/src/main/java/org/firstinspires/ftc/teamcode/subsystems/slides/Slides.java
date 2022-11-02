package org.firstinspires.ftc.teamcode.subsystems.slides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.Motor;
import org.firstinspires.ftc.teamcode.lib.PIDController;

public class Slides {
    public Motor slides1;
    public Motor slides2;

    public PIDController slidesPID;
    public  PIDController slidesPID2;

    public double slidesMotor1Distance = 0.0;
    public double slidesMotor2Distance = 0.0;

    public double lastTarget = 5300;
    private long lastTime;
    public static double KP = 0.0005, KI = 0.00, KD = 0.00;

    public int zeroTarget = 0;
    public int groundTarget = 100;
    public int lowTarget = 200;
    public int midTarget = 300;
    public int highTarget = 300;
    public int targetDistance = 0;
    private double slidesMotorFF = 0.7;
    public double currentDistance = 0;
    public double currentDistance2 = 0;

    boolean armShift = false;
    boolean reverse = false;
    boolean armMove = false;
    boolean gravity = false;

    public Slides(Motor slides1, Motor slides2) {
        this.slides1 = slides1;
        this.slides2 = slides2;

        this.slides1.setTarget(0);
        this.slides2.setTarget(0);
        this.slides1.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.slides2.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.slides2.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesMotor1Distance = 1;
        slidesMotor2Distance = 1;

        lastTime = System.currentTimeMillis();
    }

    public void updateDistance() {

        if (lastTarget != targetDistance) {
            slidesPID = new PIDController(targetDistance, KP, KI, KD, false);
            slidesPID2 = new PIDController(targetDistance, KP, KI, KD, false);
        }


        lastTarget = targetDistance;

        currentDistance = slides1.getEncoderValue();
        currentDistance2 = slides2.getEncoderValue();


        double correction1 = slidesPID.update(currentDistance);
        double correction2 = slidesPID2.update(currentDistance2);

        slidesMotor1Distance = correction1 + slidesMotorFF;
        slidesMotor2Distance = correction2 + slidesMotorFF;

        if (armShift) {

            //0.45
            //0.3
            slides1.setSpeed((float) Range.clip(slidesMotor1Distance, -1, 1)*0.35f);
            slides2.setSpeed((float) Range.clip(slidesMotor1Distance, -1, 1)*0.35f);

        }
        else if (reverse){
            slides1.setSpeed(((float) Range.clip(slidesMotor1Distance, -1, 1)*-1)*0.75f);
            slides2.setSpeed(((float) Range.clip(slidesMotor1Distance, -1, 1)*-1)*0.75f);

        }
      /*  else if (gravity){
            slidesMotor1.setSpeed(((float) Range.clip(slidesMotor1Distance, -1, 1)*-1)*0.60f);
            slidesMotor2.setSpeed(((float) Range.clip(slidesMotor1Distance, -1, 1)*-1)*0.60f);*/

        //}
        else {
            slides1.setSpeed(0);
            slides2.setSpeed(0);
        }



    }

    public void runToPosition(int ticks) {
//        this.slides1.setTarget(ticks);
//        this.slides2.setTarget(ticks);
    }

    public void runToPreset(Levels level) {
        switch (level) {
            case ZERO:
//                this.slides1.setTarget(zeroTarget);
//                this.slides2.setTarget(zeroTarget);
                  targetDistance = zeroTarget;
            case GROUND:
//                this.slides1.setTarget(groundTarget);
//                this.slides2.setTarget(groundTarget);
                targetDistance = groundTarget;
            case LOW:
//                this.slides1.setTarget(lowTarget);
//                this.slides2.setTarget(lowTarget);
                targetDistance = lowTarget;
            case MEDIUM:
//                this.slides1.setTarget(midTarget);
//                this.slides2.setTarget(midTarget);
                targetDistance = midTarget;
            case HIGH:
//                this.slides1.setTarget(highTarget);
//                this.slides2.setTarget(highTarget);
                targetDistance = highTarget;
        }
    }

    public void run (float speed){

        slides1.setSpeed(speed);
        slides2.setSpeed(speed);


    }
    public void start (double distance) {
        armShift = true;
        targetDistance = (int) distance;


        PIDController armPID = new PIDController(targetDistance, KP, KI, KD, false);
        PIDController armPID2 = new PIDController(targetDistance, KP, KI, KD, false);


        //  FtcDashboard dashboard = FtcDashboard.getInstance();


    }

    public void work(float speed,int distance){

        slides1.setTarget(distance);
        slides2.setTarget(distance);
        slides1.setSpeed(speed);
        slides2.setSpeed(speed);

    }
    public void reverse () {
        slides1.setSpeed(-0.4f);
        slides2.setSpeed(-0.4f);

    }
    public void resetMotorSpeeds () {
        slides1.setSpeed(0.7f);
        slides2.setSpeed(0.7f);
    }

    public void enable(float speed){
        armMove = true;
        slides1.setSpeed(speed);
        slides2.setSpeed(speed);


    }

    public void brake(){
        armMove = false;
        slides1.setSpeed(0);
        slides2.setSpeed(0);


    }

    public void extend(double distance){


        armMove = true;
        slides1.setSpeed(1f);
        slides2.setSpeed(1f);

    }

    public void retract(float speed){

        armMove = true;
        slides1.setSpeed(speed*-1);
        slides2.setSpeed(speed*-1);
        //start(-distance);

    }

    public void resetAllEncoders(){
        slides1.resetEncoder();
        slides2.resetEncoder();

    }
    public void stop () {
        armShift =false;
        reverse = false;
        //gravity = false;
        armMove = false;

        slides1.setSpeed(0);
        slides2.setSpeed(0);



    }
}
