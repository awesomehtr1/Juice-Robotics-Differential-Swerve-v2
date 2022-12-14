package org.firstinspires.ftc.teamcode.testopmodes.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.AS5600;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.swerve.SwerveModule;

@TeleOp(name = "Swerve Homing Test", group = "TestOpModes")
public class SwerveHomingTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        SwerveDrive swerveDrive = new SwerveDrive(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            for(SwerveModule swerveModule : swerveDrive.swerveModules) {
                swerveModule.setAngle(0);
                double pos = swerveModule.getAngle();
                double power = swerveModule.updatePID(pos);
                swerveModule.setRot(power);
            }
        }
    }
}
