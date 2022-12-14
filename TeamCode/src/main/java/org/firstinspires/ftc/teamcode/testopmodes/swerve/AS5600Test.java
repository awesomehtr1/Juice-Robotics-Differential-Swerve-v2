package org.firstinspires.ftc.teamcode.testopmodes.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.helperfunctions.AS5600;

@TeleOp(name = "Analog Encoder Test", group = "TestOpModes")
public class AS5600Test extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        AS5600 as5600 = new AS5600(hardwareMap, "LBanalog", 2.940, 3.29, 0);
        DcMotor RFrot = hardwareMap.get(DcMotor.class, "RFrot");
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        double minVoltage = 3.0;
        double maxVoltage = 0.0;
        double startingVoltage = 0.0;
        waitForStart();

        while(opModeIsActive()) {
            RFrot.setPower(0.0);
            double voltage = as5600.getVoltage();
            if(startingVoltage == 0.0)
                startingVoltage = voltage;
            if(voltage < minVoltage)
                minVoltage = voltage;
            if(voltage > maxVoltage)
                maxVoltage = voltage;

            packet.put("Current Heading (rad)", as5600.getAngle());
            packet.put("Current Voltage", voltage);
            packet.put("Max Voltage", maxVoltage);
            packet.put("Min Voltage", minVoltage);
            packet.put("Starting Voltage", startingVoltage);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
