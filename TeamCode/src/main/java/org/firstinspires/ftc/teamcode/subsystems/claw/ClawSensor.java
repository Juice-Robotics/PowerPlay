package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;

public class ClawSensor {
    ColorSensor sensor;

    // Constants of the cone colors
    int redColor = 0;
    int blueColor = 0;

    DistanceUnit unit = DistanceUnit.CM;

    public ClawSensor(ColorSensor sensor) {
        this.sensor = sensor;
    }

    public int getRawARGB() {
        return sensor.argb();
    }

    public boolean conePresent(AllianceColor color) {
        int sensorValue = sensor.argb();

        switch (color) {
            case RED:
                if (sensorValue == redColor) {
                    return true;
                }
                break;
            case BLUE:
                if (sensorValue == blueColor) {
                    return true;
                }
                break;
        }

        // Return false if color provided is BOTH or if the color was not detected
        return false;
    }

    public double getRange() {
        return ((DistanceSensor) sensor).getDistance(unit);
    }


}
