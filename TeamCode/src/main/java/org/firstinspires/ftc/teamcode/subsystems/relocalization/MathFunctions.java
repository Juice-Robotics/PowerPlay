package org.firstinspires.ftc.teamcode.subsystems.relocalization;

public class MathFunctions {
    // normalizes the angle to -pi to pi
    public static double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
}
