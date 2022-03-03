package org.firstinspires.ftc.teamcode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.helperfunctions.LowPassFilter;

@TeleOp(name = "Low Pass Demonstration", group = "TestOpModes")
@Config
public class LowPassDemonstration extends LinearOpMode {
    public static double a = 0;
    public static double power = 0;
    public static double graphLower = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        LowPassFilter lowPassFilter = new LowPassFilter(a);
        DcMotorEx RB = hardwareMap.get(DcMotorEx.class, "RB");

        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        while(opModeIsActive()) {
            lowPassFilter.a = a;
            lowPassFilter.update(RB.getVelocity());
            RB.setPower(power);
            packet.put("graph lower", graphLower);
            packet.put("raw velocity", RB.getVelocity());
            packet.put("filtered velocity", lowPassFilter.returnValue());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
