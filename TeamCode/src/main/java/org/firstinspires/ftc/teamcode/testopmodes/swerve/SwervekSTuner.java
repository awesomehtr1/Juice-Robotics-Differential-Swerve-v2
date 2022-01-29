package org.firstinspires.ftc.teamcode.testopmodes.swerve;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Swerve kS Tuner", group = "TestOpModes")
public class SwervekSTuner extends LinearOpMode {
    public static double kS = 0.0;
    public void runOpMode() {
        DcMotor RF = hardwareMap.get(DcMotor.class, "RFrot");
        DcMotor LF = hardwareMap.get(DcMotor.class, "LFrot");
        DcMotor LB = hardwareMap.get(DcMotor.class, "LBrot");
        DcMotor RB = hardwareMap.get(DcMotor.class, "RBrot");
        RB.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            RF.setPower(kS);
            LF.setPower(kS);
            LB.setPower(kS);
            RB.setPower(kS);
        }
    }
}
