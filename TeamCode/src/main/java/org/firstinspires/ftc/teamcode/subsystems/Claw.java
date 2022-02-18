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

    boolean retract;
    ElapsedTime retractTimer;

    // stores current state of claw
    public enum State{
        GRIP,
        INTAKE,
        DEPOSIT
    }
    Claw.State state;

    public Claw(Robot robot) {
        this.robot = robot;
        clawServo = robot.hardwareMap.get(Servo.class, "claw");
        state = State.INTAKE;
        time = new ElapsedTime();
        move = true;
        retract = false;
        retractTimer = new ElapsedTime();
    }

    @Override
    public void update() {
        move = time.milliseconds() > delay ? true : false;

        if(retract) {
            if(retractTimer.milliseconds() < 500)
                grip();
            else if(retractTimer.milliseconds() >= 500) {
                intake();
                retract = false;
            }
        }

        if (state == State.GRIP && move)
            clawServo.setPosition(grip);
        if (state == State.INTAKE && move)
            clawServo.setPosition(intake);
        if(state == State.DEPOSIT && move)
            clawServo.setPosition(deposit);
    }

    // use these methods to change state of claw
    public void grip() {state = State.GRIP;}
    public void intake() {state = State.INTAKE;}
    public void deposit() {state = State.DEPOSIT;}

    public void toggleGrip() { state = state == State.GRIP ? State.DEPOSIT : State.GRIP; }

    public void timedRetract() {
        retractTimer.reset();
        retract = true;
    }

    public void delayAction(double delay) {
        this.delay = delay;
        if(time.milliseconds() > delay)
            time.reset();
    }
}
