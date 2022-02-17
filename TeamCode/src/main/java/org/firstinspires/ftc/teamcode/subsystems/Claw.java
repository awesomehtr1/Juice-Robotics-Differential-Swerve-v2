package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

public class Claw implements Subsystem{
    Robot robot;
    Servo clawServo;
    static double grip = 0.55;
    static double intake = 0.432;
    static double deposit = 0.35;

    boolean move;
    double delay;
    ElapsedTime time;

    // stores current state of claw
    public enum State{
        GRIP,
        INTAKE,
        DEPOSIT
    }
    Claw.State state;

    public Claw(Robot robot) {
        this.robot = robot;
        clawServo = robot.hardwareMap.get(Servo.class, "claw"); // TODO: update device name
        state = State.INTAKE;
        time = new ElapsedTime();
        move = true;
    }

    @Override
    public void update() {
        move = time.milliseconds() > delay ? true : false;

        if (state == State.GRIP)
            clawServo.setPosition(grip);
        if (state == State.INTAKE)
            clawServo.setPosition(intake);
        if(state == State.DEPOSIT)
            clawServo.setPosition(deposit);
    }

    // use these methods to change state of claw
    public void grip() {state = State.GRIP;}
    public void intake() {state = State.INTAKE;}
    public void deposit() {state = State.DEPOSIT;}

    public void delayAction(double delay) {
        this.delay = delay;
        if(time.milliseconds() > delay)
            time.reset();
    }
}
