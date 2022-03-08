package org.firstinspires.ftc.teamcode.testopmodes.swerve;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.AS5600;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;

@Config
@TeleOp(name = "PID Test", group = "TestOpModes")
public class PIDTest extends LinearOpMode {
    public static double pos = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime time = new ElapsedTime();
        SwerveRotationPID pid = new SwerveRotationPID(0.8, 0.08, 0.011, 0.08, time);
        DcMotor rot = hardwareMap.get(DcMotor.class, "RFrot");
        AS5600 analog = new AS5600(hardwareMap, "RFanalog", 2.523, 3.29, 0);
        waitForStart();
        while (opModeIsActive()) {
            pid.setState(pos);
            rot.setPower(pid.updatePID(analog.getAngle()));
        }
    }
}
