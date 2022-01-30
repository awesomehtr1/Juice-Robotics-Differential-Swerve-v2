package org.firstinspires.ftc.teamcode.testopmodes.swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.swerve.SwerveModule;

@TeleOp(name = "Drive Direction Test", group = "TestOpModes")
public class DriveMotorDirectionTest extends LinearOpMode {
    SwerveDrive swerveDrive;
    public void runOpMode() throws InterruptedException {
        swerveDrive = new SwerveDrive(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            for(SwerveModule swerveModule : swerveDrive.swerveModules) {
                swerveModule.setDrive(0.3);
            }
        }
    }
}
