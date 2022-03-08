package org.firstinspires.ftc.teamcode.testopmodes.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helperfunctions.AS5600;

import java.util.ArrayList;

@TeleOp(name = "Zero Tuner", group = "TestOpModes")
public class ZeroTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        AS5600 RF = new AS5600(hardwareMap, "RFanalog", 0, 0, 0);
        AS5600 LF = new AS5600(hardwareMap, "LFanalog", 0, 0, 0);
        AS5600 LB = new AS5600(hardwareMap, "LBanalog", 0, 0, 0);
        AS5600 RB = new AS5600(hardwareMap, "RBanalog", 0, 0, 0);
        AS5600[] analogs = {RF, LF, LB, RB};

        double RFstart = 0;
        double LFstart = 0;
        double LBstart = 0;
        double RBstart = 0;
        double[] starts = {RFstart, LFstart, LBstart, RBstart};

        double RFmax = 0;
        double LFmax = 0;
        double LBmax = 0;
        double RBmax = 0;
        double[] max = {RFmax, LFmax, LBmax, RBmax};

        double RFmin = 3.3;
        double LFmin = 3.3;
        double LBmin = 3.3;
        double RBmin = 3.3;
        double[] min = {RFmin, LFmin, LBmin, RBmin};

        waitForStart();
        while(opModeIsActive()) {
            for(int i = 0; i < 4; i++) {
                double voltage = analogs[i].getVoltage();
                if(starts[i] == 0)
                    starts[i] = voltage;
                if(voltage > max[i])
                    max[i] = voltage;
                if(voltage < min[i])
                    min[i] = voltage;
            }

            packet.put("RF start", starts[0]);
            packet.put("LF start", starts[1]);
            packet.put("LB start", starts[2]);
            packet.put("RB start", starts[3]);

            packet.put("RF max", max[0]);
            packet.put("LF max", max[1]);
            packet.put("LB max", max[2]);
            packet.put("RB max", max[3]);

            packet.put("RF min", min[0]);
            packet.put("LF min", min[1]);
            packet.put("LB min", min[2]);
            packet.put("RB min", min[3]);

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
