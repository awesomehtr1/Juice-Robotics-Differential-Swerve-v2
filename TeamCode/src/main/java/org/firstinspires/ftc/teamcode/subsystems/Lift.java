package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class Lift implements Subsystem{
    Robot robot;
    Servo servo1, servo2;
    final double rest = 0; // TODO: update positions
    final double low = 0;
    final double mid = 0;
    final double high = 0;

    // stores current state of lift
    enum STATE {
        REST,
        LOW,
        MID,
        HIGH
    }
    STATE state;

    public Lift(Robot robot){
        this.robot = robot;
        servo1 = robot.hardwareMap.get(Servo.class, ""); // TODO: update device name
        servo2 = robot.hardwareMap.get(Servo.class, "");
        state = STATE.REST;
    }

    @Override
    public void update() {
        if (state == STATE.REST)
            setPos(rest);
        else if (state == STATE.LOW)
            setPos(low);
        else if (state == STATE.MID)
            setPos(mid);
        else if (state == STATE.HIGH)
            setPos(high);
    }

    // use the setXXX methods to set the position/state of the lift
    public void setRest(){
        state = STATE.REST;
    }

    public void setLow(){
        state = STATE.LOW;
    }

    public void setMid(){
        state = STATE.MID;
    }

    public void setHigh(){
        state = STATE.HIGH;
    }

    public void setPos(double pos){
        // TODO: change relative position setting if needed for servo2
        servo1.setPosition(pos);
        servo2.setPosition(1 - pos);
    }
}
