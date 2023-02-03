package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class StepperServo extends Component{

    private float angle;
    public Servo servo;

    public StepperServo(int port, String name, HardwareMap map){
        super(port, name);
        servo = map.servo.get(name);
    }

    public void setAngle(float angle) {
        this.angle = angle;
        servo.setPosition(angle/255);
    }

    public float getAngle(){
        return (float) servo.getPosition();
    }

}