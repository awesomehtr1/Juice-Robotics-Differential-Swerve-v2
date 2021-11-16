package org.firstinspires.ftc.teamcode.teleopmanager;

import android.app.job.JobInfo;

import org.firstinspires.ftc.teamcode.teleop.Teleop;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleOpManager {
    public BooleanSupplier trigger; // gamepad boolean input
    public RobotAction action, action2; // passed in robot actions
    public ArrayList<RobotAction> actions; // passed in robot action list

    public boolean prevState, state = false; // booleans for toggle/trigger

    public enum MODE {
        TRIGGER,
        TOGGLE,
        LISTTRIGGER
    };
    public MODE mode;

    // basic constructor (1 boolean trigger, 1 action)
    public TeleOpManager(BooleanSupplier trigger, RobotAction action) {
        this.trigger = trigger;
        this.action = action;
        mode = MODE.TRIGGER;
    }

    // toggle constructor (1 boolean toggle, 2 actions)
    public TeleOpManager(BooleanSupplier trigger, RobotAction action, RobotAction action2) {
        this.trigger = trigger;
        this.action = action;
        this.action2 = action2;
        mode = MODE.TOGGLE;
    }

    // list constructor (1 boolean trigger, list of actions)
    public TeleOpManager(BooleanSupplier trigger, ArrayList<RobotAction> robotActions) {
        this.trigger = trigger;
        actions = robotActions;
        mode = MODE.LISTTRIGGER;
    }

    // updates state and calls run()
    public void update() {
        // trigger mode
        if (mode == MODE.TRIGGER) {
            if (trigger.getAsBoolean())
                action.run();
        }

        // toggle mode
        else if (mode == MODE.TOGGLE) {
            if (trigger.getAsBoolean()) {
                if (!prevState)
                    state = !state;
                prevState = true;
            }
            else {
                if (prevState)
                    prevState = false;
            }
            if(state)
                action.run();
            else
                action2.run();
        }

        // list mode
        else if (mode == MODE.LISTTRIGGER) {
            if (trigger.getAsBoolean()) {
                for (RobotAction action : actions)
                    action.run();
            }
        }
    }
}