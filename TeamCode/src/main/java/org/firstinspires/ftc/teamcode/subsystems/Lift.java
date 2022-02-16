package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

public class Lift implements Subsystem{
    Robot robot;
    Servo R1, R2, L1, L2;
    final double rest = 0.15;
    final double mid = 0.47;
    final double high = 0.8;

    boolean move;
    double delay;
    ElapsedTime time;

    // stores current state of lift
    enum State {
        REST,
        MID,
        HIGH
    }
    State state;

    public Lift(Robot robot){
        this.robot = robot;
        R1 = robot.hardwareMap.get(Servo.class, "liftR1");
        R2 = robot.hardwareMap.get(Servo.class, "liftR2");
        L1 = robot.hardwareMap.get(Servo.class, "liftL1");
        L2 = robot.hardwareMap.get(Servo.class, "liftL2");
        state = State.REST;
    }

    @Override
    public void update() {
        move = time.milliseconds() > delay ? true : false;

        if (state == State.REST && move && move)
            setPos(rest);
        else if (state == State.MID && move)
            setPos(mid);
        else if (state == State.HIGH && move)
            setPos(high);
    }

    // use the setXXX methods to set the position/state of the lift
    public void rest(){ state = State.REST; }
    public void mid(){
        state = State.MID;
    }
    public void high(){
        state = State.HIGH;
    }

    public void setPos(double pos){
        L1.setPosition(1 - pos);
        L2.setPosition(1 - pos);
        R1.setPosition(pos);
        R2.setPosition(pos);
    }

    public void delayAction(double delay) {
        this.delay = delay;
        if(time.milliseconds() > delay)
            time.reset();
    }
}
