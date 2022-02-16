package org.firstinspires.ftc.teamcode.teleopmanager;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class TeleOpManagerBuilder {
    private BooleanSupplier trigger; // gamepad boolean input
    private ArrayList<RobotAction> actions; // passed in robot action list

    private enum MODE {
        TOGGLE,
        TRIGGER
    }
    private MODE mode;

    public TeleOpManagerBuilder typeTrigger(BooleanSupplier trigger) {
        actions = new ArrayList<RobotAction>();
        this.trigger = trigger;
        mode = MODE.TRIGGER;
        return this;
    }

    public TeleOpManagerBuilder typeToggle(BooleanSupplier trigger) {
        actions = new ArrayList<RobotAction>();
        this.trigger = trigger;
        mode = MODE.TOGGLE;
        return this;
    }

    public TeleOpManagerBuilder addAction(RobotAction action) {
        actions.add(action);
        return this;
    }

    public TeleOpManager build(){
        TeleOpManager manager;
        manager = mode == MODE.TRIGGER ?
                new TeleOpManager(trigger, actions) :
                new TeleOpManager(trigger, actions.get(0), actions.get(1));
        return manager;
    }
}
