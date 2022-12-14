package org.firstinspires.ftc.teamcode.teleopmanager;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class TeleOpManager {
    public BooleanSupplier trigger; // gamepad boolean input
    public RobotAction action, action2; // passed in robot actions
    public ArrayList<RobotAction> actions; // passed in robot action list

    public boolean prevState, state = false; // booleans for toggle/trigger

    public enum MODE {
        TOGGLE,
        TRIGGER,
        FALLINGEDGE,
        HOLD
    }
    public MODE mode;

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
        mode = MODE.TRIGGER;
    }

    // falling edge constructor (1 boolean falling edge, list of actions)
    public TeleOpManager(BooleanSupplier trigger, ArrayList<RobotAction> robotActions, boolean falling) {
        this.trigger = trigger;
        actions = robotActions;
        mode = MODE.FALLINGEDGE;
    }

    // hold constructor (1 boolean hold, 2 actions)
    public TeleOpManager(BooleanSupplier trigger, RobotAction action1, RobotAction action2, boolean hold) {
        this.trigger = trigger;
        this.action = action1;
        this.action2 = action2;
        mode = MODE.HOLD;
    }

    // updates state and calls run()
    public void update() {
        // trigger mode
        if (mode == MODE.TRIGGER) {
            if (trigger.getAsBoolean()) {
                for (RobotAction action : actions)
                    action.run();
            }
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

        // falling edge mode
        else if(mode == MODE.FALLINGEDGE) {
            if(trigger.getAsBoolean()) {
                if(!prevState) {
                    for (RobotAction action : actions)
                        action.run();
                }
                prevState = true;
            }
            else
                prevState = false;
        }

        // hold mode
        else if(mode == MODE.HOLD) {
            (trigger.getAsBoolean() ? action : action2).run();
        }
    }
}