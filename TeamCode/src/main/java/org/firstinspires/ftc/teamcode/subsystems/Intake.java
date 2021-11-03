package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class Intake implements Subsystem{
    Robot robot;
    CRServo servo1, servo2;

    // stores current state of intake
    enum STATE {
        OFF,
        ON,
    }
    Intake.STATE state;

    public Intake(Robot robot){
        this.robot = robot;
        servo1 = robot.hardwareMap.get(CRServo.class, ""); // TODO: update device name
        servo2 = robot.hardwareMap.get(CRServo.class, "");
        state = STATE.OFF;
    }

    @Override
    public void update() {
        if(state == STATE.OFF)
            setPower(0);
        if(state == STATE.ON)
            setPower(1);
    }

    // use these methods to set the state of the intake
    public void off(){
        state = STATE.OFF;
    }

    public void on(){
        state = STATE.ON;
    }

    public void setPower(double power){
        // TODO: change power direction if needed
        servo1.setPower(power);
        servo2.setPower(power);
    }
}
