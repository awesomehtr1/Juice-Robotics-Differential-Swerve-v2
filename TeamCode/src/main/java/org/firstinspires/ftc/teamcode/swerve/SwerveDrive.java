package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.AS5600;
import org.firstinspires.ftc.teamcode.helperfunctions.MathFunctions;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

// high level swerve drive class which utilizes gamepad controls to control the swerve modules
// wraps SwerveKinematics controls and implements SwerveModule
public class SwerveDrive {
    public SwerveKinematics swerveKinematics;
    public SwerveOdometry swerveOdometry;

    private HardwareMap hardwareMap;

    private DcMotorEx RFdrive, LFdrive, LBdrive, RBdrive;
    private DcMotor RFrot, LFrot, LBrot, RBrot;

    // pid constants
    ElapsedTime RFtime = new ElapsedTime();
    ElapsedTime LFtime = new ElapsedTime();
    ElapsedTime LBtime = new ElapsedTime();
    ElapsedTime RBtime = new ElapsedTime();
    public SwerveRotationPID RFPID, LFPID, LBPID, RBPID;

    private AS5600 RFas5600, LFas5600, LBas5600, RBas5600;

    // array of swerve module objects
    private SwerveModule RF, LF, LB, RB;
    public SwerveModule[] swerveModules = new SwerveModule[4];

    private double drivePower = 0.3;

    VoltageSensor voltageSensor;

    private enum ZEROBEHAVIOR {
        RESPONSIVE,
        LOCK
    }
    ZEROBEHAVIOR zerobehavior;

    public SwerveDrive(HardwareMap hardwareMap) {
        swerveKinematics = new SwerveKinematics();
        swerveKinematics.setTrackwidth(SwerveConstants.trackwidth);
        swerveKinematics.setWheelbase(SwerveConstants.wheelbase);

        swerveOdometry = new SwerveOdometry();
        swerveOdometry.setTrackwidth(SwerveConstants.trackwidth);
        swerveOdometry.setWheelbase(SwerveConstants.wheelbase);

        this.hardwareMap = hardwareMap;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        RFdrive = hardwareMap.get(DcMotorEx.class, "RF");
        LFdrive = hardwareMap.get(DcMotorEx.class, "LF");
        LBdrive = hardwareMap.get(DcMotorEx.class, "LB");
        RBdrive = hardwareMap.get(DcMotorEx.class, "RB");

        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        RFrot = hardwareMap.get(DcMotor.class, "RFrot");
        LFrot = hardwareMap.get(DcMotor.class, "LFrot");
        LBrot = hardwareMap.get(DcMotor.class, "LBrot");
        RBrot = hardwareMap.get(DcMotor.class, "RBrot");

        RFrot.setDirection(DcMotorSimple.Direction.REVERSE);
        LFrot.setDirection(DcMotorSimple.Direction.REVERSE);
        LBrot.setDirection(DcMotorSimple.Direction.REVERSE);
        RBrot.setDirection(DcMotorSimple.Direction.REVERSE);

        RFas5600 = new AS5600(hardwareMap, "RFanalog", 2.621, 3.291, 0.001);
        LFas5600 = new AS5600(hardwareMap, "LFanalog", 0.125, 3.286, 0.004);
        LBas5600 = new AS5600(hardwareMap, "LBanalog", 3.251, 3.282, 0.001);
        RBas5600 = new AS5600(hardwareMap, "RBanalog", 2.971, 3.291, 0.0);

        RFPID = new SwerveRotationPID(0.7, 0.0, 0.013, 0.03, RFtime);
        LFPID = new SwerveRotationPID(0.67, 0.0, 0.013, 0.02, LFtime);
        LBPID = new SwerveRotationPID(0.7, 0.0, 0.01, 0.05, LBtime);
        RBPID = new SwerveRotationPID(0.7, 0.0, 0.006, 0.04, RBtime);

        RF = new SwerveModule(RFrot, RFdrive, RFPID, RFas5600);
        LF = new SwerveModule(LFrot, LFdrive, LFPID, LFas5600);
        LB = new SwerveModule(LBrot, LBdrive, LBPID, LBas5600);
        RB = new SwerveModule(RBrot, RBdrive, RBPID, RBas5600);

        // add swerve modules to array
        swerveModules[0] = RF;
        swerveModules[1] = LF;
        swerveModules[2] = LB;
        swerveModules[3] = RB;

        // stop and reset drive for odo
        for(SwerveModule swerveModule : swerveModules) {
            swerveModule.reset();
            swerveModule.runWithoutEncoder();
        }

        zerobehavior = ZEROBEHAVIOR.RESPONSIVE;
    }

    // sets rotation and drive powers using inputs from gamepad
    public void setMotorPowers(double rotation, double strafe, double forward) {
        double deadzone = 0.1;
        if(drivePower == 1.0)
            rotation *= 0.5;
        swerveKinematics.calculateKinematics(
                rotation * drivePower,
                strafe * drivePower,
                forward * drivePower);
        double[] rotAngleArray = swerveKinematics.getWheelAngles();
        double[] drivePowerArray = swerveKinematics.getWheelVelocities();
        if(Math.abs(strafe) < deadzone && Math.abs(forward) < deadzone && Math.abs(rotation) < deadzone) {
            drivePowerArray[0] = 0;
            drivePowerArray[1] = 0;
            drivePowerArray[2] = 0;
            drivePowerArray[3] = 0;
        }
        setRotPowerArray(rotAngleArray);
        setDrivePowerArray(drivePowerArray);
    }

    // method for setting pid target for individual swerve module with angle optimization
    public void setRotationPower(SwerveModule module, double angle) {
        angle = angleOptimization(module, angle);
        module.setAngle(angle);
        double power = module.updatePID(module.getAngle());
        double voltageCompensation = 13.2 / voltageSensor.getVoltage();
        module.setRot(power * voltageCompensation);
    }

    public void setDrivePower(SwerveModule module, double power) {
        module.setDrive(power);
    }

    // make pod rotate no more than 90 degrees
    public double angleOptimization(SwerveModule module, double angle) {
        if(Math.abs(angle - module.getAngle()) > Math.PI/2) {
            angle += Math.PI;
            MathFunctions.angleWrap(angle);
            module.setReverseDrive(true);
        }
        else
            module.setReverseDrive(false);
        return angle;
    }

    // compensate wheel rotation when pod rotates
    public double compensateRotation(SwerveModule module) {
        double power = module.getRotPower();
        power *= SwerveConstants.wheelRotPerPodRot;
        power /= SwerveConstants.driveRotRatio;
        return power;
    }

    // sets rotation power for all modules in the array
    // takes 1 dimensional array of rotation angles from SwerveKinematics
    public void setRotPowerArray(double[] rotAngleArray) {
        for (int i = 0; i < 4; i++)
            setRotationPower(swerveModules[i], rotAngleArray[i]);
    }

    // sets drive power for all modules in the array
    // takes 1 dimensional array of drive powers from SwerveKinematics
    public void setDrivePowerArray(double[] drivePowerArray) {
        for (int i = 0; i < 4; i++)
            setDrivePower(swerveModules[i], drivePowerArray[i]);
    }

    public void updatePose(double heading, double elapsedTime) {
        double[] wheelVelocities = getWheelVelocities(elapsedTime);
        double[] moduleOrientations = getModuleOrientations();
        swerveOdometry.updatePose(wheelVelocities, moduleOrientations, heading, elapsedTime);
    }

    public void updatePoseExponential(double heading, double elapsedTime) {
        double[] wheelVelocities = getWheelVelocities(elapsedTime);
        double[] moduleOrientations = getModuleOrientations();
        swerveOdometry.updatePoseExponential(wheelVelocities, moduleOrientations, heading, elapsedTime);
    }

    public double getX() { return swerveOdometry.getX(); }
    public double getY() { return swerveOdometry.getY(); }
    public double getHeading() { return swerveOdometry.getHeading(); }

    public double getVelocity() { return swerveOdometry.getV(); }

    public double[] getWheelVelocities(double elapsedTime) {
        double[] wheelVelocities = new double[4];
        for(int i = 0; i < 4; i++)
            wheelVelocities[i] = swerveModules[i].getWheelVelocity(elapsedTime);
        return wheelVelocities;
    }

    public double[] getModuleOrientations() {
        double[] moduleOrientations = new double[4];
        for(int i = 0; i < 4; i++)
            moduleOrientations[i] = swerveModules[i].getAngle();
        return moduleOrientations;
    }

    // set/reset pose
    public void setPose(double x, double y, double heading) {
        swerveOdometry.setPose(x, y, heading);
    }

    public void setSlowmode(boolean slowmode) {
        drivePower = slowmode ? 0.3 : 1.0;
    }

    public void setBrake() {
        for(SwerveModule module : swerveModules)
            module.setBrake();
    }

    public void setZeroBehavior(SwerveDrive.ZEROBEHAVIOR behavior) {
        zerobehavior = behavior;
    }
}
