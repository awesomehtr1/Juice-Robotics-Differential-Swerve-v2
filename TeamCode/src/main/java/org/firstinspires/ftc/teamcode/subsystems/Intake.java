package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class Intake implements Subsystem{
    Robot robot;
    CRServo servoR1, servoR2, servoL1, servoL2;

    double power;

    // stores current state of intake
    enum State {
        OFF,
        ON,
        REVERSE
    }
    Intake.State state, prevState;

    public Intake(Robot robot){
        this.robot = robot;
        servoR1 = robot.hardwareMap.get(CRServo.class, "intakeR1"); // TODO: update device name
        servoR2 = robot.hardwareMap.get(CRServo.class, "intakeR2");
        servoL1 = robot.hardwareMap.get(CRServo.class, "intakeL1");
        servoL2 = robot.hardwareMap.get(CRServo.class, "intakeL2");
        state = State.OFF;
        prevState = state;
        power = 1;
    }

    @Override
    public void update() {
        if(state == State.OFF)
            setPower(0);
        if(state == State.ON)
            setPower(-power);
        if(state == State.REVERSE)
            setPower(power);
    }

    // use these methods to set the state of the intake
    public void off(){
        state = State.OFF;
    }
    public void on(){ state = State.ON; }
    public void reverse() { state = State.REVERSE; }

    public void toggleIntake() { state = state == State.OFF ? State.ON : State.OFF; }

    public void queryReverse() {
        if(state != State.REVERSE)
            prevState = state;
        state = State.REVERSE;
    }

    public void restoreState() {
        state = prevState;
    }

    public void changePower(double power) { this.power = power;}

    public void setPower(double power){
        servoL1.setPower(power);
        servoL2.setPower(-power);
        servoR1.setPower(-power);
        servoR2.setPower(power);
    }
}
