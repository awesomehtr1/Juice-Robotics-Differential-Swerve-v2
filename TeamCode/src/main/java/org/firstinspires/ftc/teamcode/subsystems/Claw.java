package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class Claw implements Subsystem{
    Robot robot;
    Servo clawServo;
    static double rest = 0.0; // TODO: tune values
    static double grip = 0.0;
    static double intake = 0.0;
    static double release = 0.0;

    // stores current state of claw
    public enum State{
        REST,
        GRIP,
        INTAKE,
        RELEASE
    }
    Claw.State state;

    public Claw(Robot robot) {
        this.robot = robot;
        clawServo = robot.hardwareMap.get(Servo.class, ""); // TODO: update device name
        state = State.REST;
    }

    @Override
    public void update() {
        if (state == State.REST)
            clawServo.setPosition(rest);
        if (state == State.GRIP)
            clawServo.setPosition(grip);
        if (state == State.INTAKE)
            clawServo.setPosition(intake);
        if (state == State.RELEASE)
            clawServo.setPosition(release);
    }

    // use these methods to change state of claw
    public void rest() {state = State.REST;}
    public void grip() {state = State.GRIP;}
    public void intake() {state = State.INTAKE;}
    public void release() {state = State.RELEASE;}
}
