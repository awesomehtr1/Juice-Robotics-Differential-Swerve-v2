package org.firstinspires.ftc.teamcode.helperfunctions;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lamprey {
    HardwareMap hardwareMap;
    AnalogInput angleOut; // analog output
    double angle; // stores current angle
    double correction; // converts voltage to radians

    public Lamprey(HardwareMap hardwareMap, String deviceName) {
        this.hardwareMap = hardwareMap;
        angleOut = hardwareMap.get(AnalogInput.class, deviceName);
        update();
    }

    public double getAngle() {
        return MathFunctions.angleWrap(angle);
    }

    public void update() {
         angle = angleOut.getVoltage() * correction;
    }
}
