package org.firstinspires.ftc.teamcode.helperfunctions;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SanfordGyro {
    HardwareMap hardwareMap;
    AnalogInput angleOut;
    double corectionCoeff = -360/358.0*7200/7319.98 * 1.002087682672234 * 360/360.3990130404954;
    boolean firstUpdateLoop;
    double prevAngle, cumulativeAngle;

    public SanfordGyro(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        angleOut = hardwareMap.get(AnalogInput.class,"gyro");
        firstUpdateLoop = true;
        cumulativeAngle = Math.PI;
    }

    public double getAngleRaw(){
        return angleOut.getVoltage() / 3.3 * 360 / 354 * 360 / 360.22187981510024 * 2 * Math.PI;
    }

    public void update(){
        if(firstUpdateLoop){
            prevAngle = getAngleRaw();
            firstUpdateLoop = false;
        }
        double currentAngle = getAngleRaw();
        cumulativeAngle += MathFunctions.angleWrap(currentAngle-prevAngle) * corectionCoeff;
        prevAngle = currentAngle;
    }

    public double getAngleCorrected(){
        return MathFunctions.angleWrap(cumulativeAngle);
    }
}