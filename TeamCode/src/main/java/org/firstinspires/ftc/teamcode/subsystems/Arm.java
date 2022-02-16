package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

public class Arm implements Subsystem{
    Robot robot;
    Servo armServo1, armServo2;
    final double intakePos = 0.04;
    final double depositLowPos = 0.97;
    final double depositMidPos = 0.95;
    final double depositHighPos = 0.87;

    boolean move;
    double delay;
    ElapsedTime time;

    // stores current state of arm
    public enum State{
        INTAKE,
        DEPOSITLOW,
        DEPOSITMID,
        DEPOSITHIGH
    }
    Arm.State state;

    public Arm(Robot robot) {
        this.robot = robot;
        armServo1 = robot.hardwareMap.get(Servo.class, "armR"); // TODO: update device name
        armServo2 = robot.hardwareMap.get(Servo.class, "armL");
        state = State.INTAKE;
        time = new ElapsedTime();
        move = true;
    }

    @Override
    public void update() {
        move = time.milliseconds() > delay ? true : false;

        if(state == State.INTAKE && move)
            setPos(intakePos);
        if(state == State.DEPOSITLOW && move)
            setPos(depositLowPos);
        if(state == State.DEPOSITMID && move)
            setPos(depositLowPos);
        if(state == State.DEPOSITHIGH && move)
            setPos(depositLowPos);
    }

    // use these methods to change state of arm
    public void intake() { state = State.INTAKE; }
    public void low() { state = State.DEPOSITLOW;}
    public void mid() { state = State.DEPOSITMID;}
    public void high() { state = State.DEPOSITHIGH;}

    public void setPos(double pos) {
        armServo1.setPosition(pos);
        armServo2.setPosition(1 - pos);
    }

    public void delayAction(double delay) {
        this.delay = delay;
        if(time.milliseconds() > delay)
            time.reset();
    }
}
