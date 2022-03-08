package org.firstinspires.ftc.teamcode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.PID.BasicPID;
import org.firstinspires.ftc.teamcode.helperfunctions.SanfordGyro;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;

@Config
@TeleOp(name = "Drive Rot PID Tuner", group = "TestOpModes")
public class DriveRotationPIDTuner extends LinearOpMode {
    SwerveDrive drive;
    BasicPID pid;
    SanfordGyro gyro;

    public static double kP, kI, kD, kS;
    public static double angle;
    ElapsedTime time, movementTimer;

    VoltageSensor voltageSensor;

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    double target;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SwerveDrive(hardwareMap);
        drive.setSlowmode(false);
        time = new ElapsedTime();
        movementTimer = new ElapsedTime();
        pid = new BasicPID(kP, kI, kD, kS, time);
        gyro = new SanfordGyro(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        target = 0;
        angle = 0;

        waitForStart();
        while(opModeIsActive()) {
            pid.kP = kP;
            pid.kI = kI;
            pid.kD = kD;
            pid.kS = kS;

            if (movementTimer.seconds() < 1.5) {
                target = angle;
            }
            else if (movementTimer.seconds() >= 1.5 && movementTimer.seconds() < 3) {
                target = 0;
            }
            else if (movementTimer.seconds() >= 3)
                movementTimer.reset();
            pid.setState(target);

            double power = pid.updatePID(gyro.getLowPassEstimate());
            double voltageCompensation = 13.6 / voltageSensor.getVoltage();

            drive.setMotorPowers(power * voltageCompensation, 0, 0);

            packet.put("integral", Math.abs(pid.integral * kI) * Math.signum(pid.error));
            packet.put("starting voltage", gyro.startingVoltage);
            packet.put("set state", target);
            packet.put("angle", gyro.getLowPassEstimate());
//            dashboard.sendTelemetryPacket(packet);
        }
    }
}
