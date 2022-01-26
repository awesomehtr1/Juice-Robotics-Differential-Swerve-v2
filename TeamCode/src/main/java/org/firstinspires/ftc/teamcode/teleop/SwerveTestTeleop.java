package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;

public class SwerveTestTeleop extends LinearOpMode {
    public void runOpMode(){
        SwerveDrive swerveDrive = new SwerveDrive(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            double rotation = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            double forward = gamepad1.left_stick_y;
            swerveDrive.setMotorPowers(rotation, strafe, forward);
        }
    }
}