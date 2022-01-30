package org.firstinspires.ftc.teamcode.testopmodes.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.AS5600;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.swerve.SwerveModule;

@Config
@TeleOp(name = "Swerve Module PID Tuner", group = "TestOpModes")
public class SwerveModulePIDTuner extends LinearOpMode {
    public static double kP, kI, kD, kS;
    ElapsedTime time = new ElapsedTime();
    VoltageSensor voltageSensor;

    public void runOpMode(){
        SwerveDrive drive = new SwerveDrive(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        boolean rotate = false;
        double setState = 0;
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

            for(SwerveModule module : drive.swerveModules) {
                double angle = module.getAngle();
                module.pid.kP = kP;
                module.pid.kI = kI;
                module.pid.kD = kD;
                module.pid.kS = kS;
                if (rotate) {
                    if (time.seconds() < 0.75) {
                        setState = Math.PI / 2;
                        module.setAngle(Math.PI / 2);
                    }
                    else if (time.seconds() >= 0.75 && time.seconds() < 1.5) {
                        setState = 0;
                        module.setAngle(0);
                    }
                    else if (time.seconds() >= 1.5)
                        time.reset();
                    double power = module.updatePID(angle);
                    power = power * 12 / voltageSensor.getVoltage();
                    module.setRot(power);
                }
            }
            packet.put("Set State", setState);
            packet.put("RF", drive.swerveModules[0].getAngle());
            packet.put("LF", drive.swerveModules[1].getAngle());
            packet.put("LB", drive.swerveModules[2].getAngle());
            packet.put("RB", drive.swerveModules[3].getAngle());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
