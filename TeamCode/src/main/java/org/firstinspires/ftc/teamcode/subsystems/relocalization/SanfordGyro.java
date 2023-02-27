package org.firstinspires.ftc.teamcode.subsystems.relocalization;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SanfordGyro {
    HardwareMap hardwareMap;
    public AnalogInput angleOut;
    private boolean firstUpdateLoop;
    private double angle;

    private double minVoltage = 0.002;
    private double maxVoltage = 3.293;

    public LowPassFilter lowPassFilter;
    public double a = 0.1;

    public double startingVoltage;
    boolean measuringLoop = true;

    private double angleOffset = 0;

    ElapsedTime bootup;

    public SanfordGyro(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        angleOut = hardwareMap.get(AnalogInput.class,"gyro");
        firstUpdateLoop = true;
        angle = 0;
        lowPassFilter = new LowPassFilter(a);
        bootup = new ElapsedTime();
    }

    public void update() {
        angle = angleOut.getVoltage() - startingVoltage;
        angle -= minVoltage;
        angle /= (maxVoltage - minVoltage);
        angle *= Math.PI * 2;
        angle = MathFunctions.angleWrap(angle + angleOffset);
        if(bootup.milliseconds() >= 1000 && measuringLoop && Math.abs(angleOut.getVoltage()) > 0.05) {
            startingVoltage = angleOut.getVoltage();
            measuringLoop = false;
        }
        if(bootup.milliseconds() >= 1000)
            lowPassFilter.update(angle);
    }

    public double getAngle(){
        update();
        return MathFunctions.angleWrap(angle);
    }

    public double getLowPassEstimate() {
        update();
        return lowPassFilter.returnValue();
    }

    public void setStartingAngle(double angle) {
        angleOffset = angle;
    }
}