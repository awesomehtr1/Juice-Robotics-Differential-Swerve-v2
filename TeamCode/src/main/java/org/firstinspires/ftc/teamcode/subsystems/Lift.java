package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class Lift implements Subsystem{
    Robot robot;
    Servo servo1, servo2, servo3, servo4;
    final double rest = 0; // TODO: update positions
    final double low = 0;
    final double mid = 0;
    final double high = 0;
    final double offset = 0;

    // stores current state of lift
    enum State {
        REST,
        LOW,
        MID,
        HIGH
    }
    State state;

    public Lift(Robot robot){
        this.robot = robot;
        servo1 = robot.hardwareMap.get(Servo.class, ""); // TODO: update device name
        servo2 = robot.hardwareMap.get(Servo.class, "");
        servo3 = robot.hardwareMap.get(Servo.class, "");
        servo4 = robot.hardwareMap.get(Servo.class, "");
        state = State.REST;
    }

    @Override
    public void update() {
        if (state == State.REST)
            setPos(rest);
        else if (state == State.LOW)
            setPos(low);
        else if (state == State.MID)
            setPos(mid);
        else if (state == State.HIGH)
            setPos(high);
    }

    // use the setXXX methods to set the position/state of the lift
    public void rest(){ state = State.REST; }
    public void low(){
        state = State.LOW;
    }
    public void mid(){
        state = State.MID;
    }
    public void high(){
        state = State.HIGH;
    }

    public void setPos(double pos){
        // TODO: change relative position setting if needed for servo2/3
        servo1.setPosition(pos);
        servo2.setPosition(1 - pos + offset);
        servo3.setPosition(1 - pos + offset);
        servo4.setPosition(pos);
    }
}
