package org.firstinspires.ftc.teamcode.auto.autocontrol;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.BasicPID;
import org.firstinspires.ftc.teamcode.helperfunctions.SanfordGyro;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;

public class Drive {
    private SwerveDrive swerveDrive;
    private SanfordGyro gyro;

    private Robot robot;

    private ElapsedTime currentTime, PIDtime;
    private double time;

    private double rotation, strafe, forward;
    private double angle;

    private BasicPID rotationPID;

    private double power;
    private double rotPower;

    private boolean moving;

    private VoltageSensor voltageSensor;

    public Drive(Robot robot, HardwareMap hardwareMap) {
        swerveDrive = new SwerveDrive(hardwareMap);
        this.robot = robot;
        gyro = new SanfordGyro(hardwareMap);
        currentTime = new ElapsedTime();
        time = 0.0;
        PIDtime = new ElapsedTime();
        rotationPID = new BasicPID(1.2, 0.25, 0.245, 0, PIDtime);
        power = 0.5;
        rotPower = 1.0;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void update() {
        rotation = rotationPID.updatePID(getAngle()) * rotPower;
        if(currentTime.seconds() >= time) {
            forward = 0;
            strafe = 0;
        }
        if(forward != 0 || strafe != 0)
            moving = true;
        else
            moving = false;

        rotation = rotation * 13.6 / voltageSensor.getVoltage();
        strafe = strafe * 13.6 / voltageSensor.getVoltage();
        forward = forward * 13.6 / voltageSensor.getVoltage();
        swerveDrive.setMotorPowers(rotation, strafe, forward);
        robot.update();
    }

    public void forward(boolean forward, double time) {
        currentTime.reset();
        this.time = time;
        this.forward = forward ? power : -power;
    }

    public void strafe(boolean right, double time) {
        currentTime.reset();
        this.time = time;
        strafe = right ? power : -power;
    }

    public void rotateTo(double angle) {
        this.angle = angle;
        rotationPID.setState(angle);
    }

    public double getAngle() { return gyro.getLowPassEstimate(); }

    public boolean isMoving() { return moving; }

    public void stopDrive() { swerveDrive.setMotorPowers(0, 0, 0); }
}
