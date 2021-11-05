package org.firstinspires.ftc.teamcode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.helperfunctions.Lamprey;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;

@Config
@TeleOp(name = "Swerve Module PID Tuner", group = "TestOpModes")
public class SwerveModulePIDTuner extends LinearOpMode {
    public static double kP, kI, kD, kS;
    ElapsedTime time = new ElapsedTime();

    public void runOpMode(){
        SwerveRotationPID pid = new SwerveRotationPID(kP, kI, kD, kS, time);
        DcMotor rotationMotor = hardwareMap.get(DcMotor.class, "");
        Lamprey lamprey = new Lamprey(hardwareMap, null); // TODO: change device name

        TelemetryPacket packet = new TelemetryPacket();

        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            double angle = lamprey.getAngle();
            if (gamepad1.a)
                pid.setState(Math.PI/2);
            if (gamepad1.b)
                pid.setState(0);
            rotationMotor.setPower(pid.updatePID(angle));

            packet.put("Set State", pid.desiredState);
            packet.put("Current State", angle);
            telemetry.update();
        }
    }
}
