package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helperfunctions.SanfordGyro;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;

@TeleOp(name = "Drive Only", group = "Teleop")
public class SwerveTestTeleop extends LinearOpMode {
    public void runOpMode(){
        SwerveDrive swerveDrive = new SwerveDrive(hardwareMap);
        SanfordGyro gyro = new SanfordGyro(hardwareMap);
        gyro.setStartingAngle(0);

        waitForStart();
        while(opModeIsActive()) {
            double rotation = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            swerveDrive.setMotorPowers(rotation, strafe, forward);
            swerveDrive.setHeading(gyro.getAngle());

            if(gamepad1.a)
                swerveDrive.setSlowmode(true);
            if(gamepad1.b)
                swerveDrive.setSlowmode(false);
            if(gamepad1.y)
                swerveDrive.setFieldCentric(true);
            if(gamepad1.x)
                swerveDrive.setFieldCentric(false);
            if(gamepad1.dpad_up)
                swerveDrive.fieldCentricRed();
            if(gamepad1.dpad_down)
                swerveDrive.fieldCentricBlue();

            telemetry.addData("field centric: ", swerveDrive.fieldCentric);
            telemetry.addData("heading", gyro.getAngle());
            telemetry.update();
        }
    }
}