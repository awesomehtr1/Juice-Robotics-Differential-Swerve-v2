package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Spinner;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.teleopmanager.TeleOpManager;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    public ArrayList<Subsystem> Subsystems; // list of subsystems
    public ArrayList<TeleOpManager> TeleOpManagers; // list of teleopmanagers
    public HardwareMap hardwareMap; // stores hardware map

    // robot subsystems
    public Drive drive;
    public Lift lift;
    public Intake intake;
    public Arm arm;
    public Claw claw;
    public Spinner spinner;

    // gamepads
    public Gamepad gamepad1, gamepad2;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap; // init passed-in hardwaremap

        this.gamepad1 = gamepad1; // init passed-in gamepads
        this.gamepad2 = gamepad2;

        // bulk read
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // init subsystems
        drive = new Drive(this);
        lift = new Lift(this);
        intake = new Intake(this);
        arm = new Arm(this);
        claw = new Claw(this);
        spinner = new Spinner(this);

        Subsystems = new ArrayList<>();
        TeleOpManagers = new ArrayList<>();

        // add subsystems to list
        Subsystems.add(drive);
        Subsystems.add(lift);
        Subsystems.add(intake);
        Subsystems.add(arm);
        Subsystems.add(claw);
        Subsystems.add(spinner);
    }

    // updates robot
    public void update() {
        for(Subsystem subsystem : Subsystems)
            subsystem.update(); // calls update method of every subsystem
        for(TeleOpManager manager: TeleOpManagers)
            manager.update(); // calls update method of every teleopmanager
    }

    // creates teleopmanager and adds it to list of teleopmanagers
    public void createTeleOpManager(TeleOpManager teleOpManager) {
        TeleOpManagers.add(teleOpManager);
    }
}