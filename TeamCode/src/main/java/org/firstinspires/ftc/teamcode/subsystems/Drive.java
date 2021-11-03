package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SwerveDriveRR;

public class Drive implements Subsystem {
    Robot robot;
    SwerveDriveRR drive;

    public Drive(Robot robot){
        this.robot = robot;
        drive = new SwerveDriveRR(robot.hardwareMap);
    }

    @Override
    public void update() {
        drive.update();
    }
}