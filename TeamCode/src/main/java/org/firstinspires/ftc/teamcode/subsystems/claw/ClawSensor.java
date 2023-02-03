package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;

public class ClawSensor {
    ColorSensor sensor;

    // Constants of the cone colors

    DistanceUnit unit = DistanceUnit.MM;

    public ClawSensor(ColorSensor sensor) {
        this.sensor = sensor;
    }

    public int getRawARGB() {
        return sensor.argb();
    }

    public boolean conePresent() {
        double sensorValue = getRange();

        if (sensorValue >= 1.0 && sensorValue <= 45.0) {
            return true;
        }

        // Return false if color provided is BOTH or if the color was not detected
        return false;
    }

    public double getRange() {
        return ((DistanceSensor) sensor).getDistance(unit);
    }


}