package org.firstinspires.ftc.teamcode.testopmodes.swerve;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.AS5600;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;

@Config
@TeleOp(name = "Swerve Module PID Tuner", group = "TestOpModes")
public class SwerveModulePIDTuner extends LinearOpMode {
    public static double kP, kI, kD, kS;
    ElapsedTime time = new ElapsedTime();

    public void runOpMode(){
        SwerveRotationPID pid = new SwerveRotationPID(kP, kI, kD, kS, time);
        DcMotor rotationMotor = hardwareMap.get(DcMotor.class, "");
        AS5600 AS5600 = new AS5600(hardwareMap, null, 0.0); // TODO: change device name

        TelemetryPacket packet = new TelemetryPacket();

        boolean rotate = false;
        ElapsedTime time = new ElapsedTime();

        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            if(gamepad1.a) {
                rotate = true;
                time.reset();
            }
            if(gamepad1.b)
                rotate = false;

            double angle = AS5600.getAngle();
            if(rotate) {
                if(time.seconds() > 1.0) {
                    pid.setState(Math.PI);
                }
                if(time.seconds() > 1.5)
                    time.reset();
                else {
                    pid.setState(0);
                }
                    rotationMotor.setPower(pid.updatePID(angle));
            }

            packet.put("Set State", pid.desiredState);
            packet.put("Current State", angle);
            telemetry.update();
        }
    }
}
