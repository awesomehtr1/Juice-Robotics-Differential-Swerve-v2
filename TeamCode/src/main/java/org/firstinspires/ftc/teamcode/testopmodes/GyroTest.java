package org.firstinspires.ftc.teamcode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helperfunctions.SanfordGyro;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;

@Config
@TeleOp(name = "Gyro Test", group = "TestOpModes")
public class GyroTest extends LinearOpMode {
    public static double a = 0.0;
    public static double power = 0;
    SanfordGyro gyro;

    SwerveDrive drive;

    double minVoltage = 3.3;
    double maxVoltage = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        gyro = new SanfordGyro(hardwareMap);
        drive = new SwerveDrive(hardwareMap);

        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        while(opModeIsActive()) {
            drive.setMotorPowers(power, 0, 0);
            drive.setSlowmode(false);

            double voltage = gyro.angleOut.getVoltage();
            if(voltage < minVoltage)
                minVoltage = voltage;
            if(voltage > maxVoltage)
                maxVoltage = voltage;

            gyro.lowPassFilter.a = a;
            packet.put("raw", gyro.getAngle());
            packet.put("low pass estimate", gyro.getLowPassEstimate());
            packet.put("voltage", voltage);
            packet.put("min", minVoltage);
            packet.put("max", maxVoltage);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
