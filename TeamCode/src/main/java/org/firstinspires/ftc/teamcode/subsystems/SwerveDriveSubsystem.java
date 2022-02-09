package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;

public class SwerveDriveSubsystem implements Subsystem{
    private SwerveDrive drive;
    private Robot robot;
    private HardwareMap hardwareMap;
    private double strafe, forward, rotation;
    private double power;
    private boolean slowmode, fieldCentric;

    public SwerveDriveSubsystem(Robot robot) {
        this.robot = robot;
        hardwareMap = robot.hardwareMap;
        drive = new SwerveDrive(hardwareMap);
        slowmode = false;
    }
    public void update() {
        if(!slowmode) power = 1.0;
        else power = 0.3;
        rotation *= power;
        strafe *= power;
        forward *= power;

        if(fieldCentric) {
            Vector2d inputs = new Vector2d(strafe, forward);
            inputs.rotated(robot.getHeading());
            strafe = inputs.getX();
            forward = inputs.getY();
        }

        drive.setMotorPowers(rotation, strafe, forward);
    }
    public void gamepadInput(double strafe, double forward, double rotation) {
        this.strafe = strafe;
        this.forward = forward;
        this.rotation = rotation;
    }
    public void setSlowmode(boolean slowmode) { this.slowmode = slowmode; }
    public void setFieldCentric(boolean fieldCentric) { this.fieldCentric = fieldCentric; }
}
