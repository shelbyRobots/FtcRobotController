package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.EnumMap;

import hallib.HalDashboard;

public class ManagedGamepad
{
    private static final String TAG = "SJH_MGP";
    private static int cycle = 0;
    private EnumMap<Button, Boolean> current       = new EnumMap<>(Button.class);
    private EnumMap<Button, Boolean> previous      = new EnumMap<>(Button.class);
    private EnumMap<Button, Boolean> just_pressed  = new EnumMap<>(Button.class);
    private EnumMap<Button, Boolean> just_released = new EnumMap<>(Button.class);
    private EnumMap<AnalogInput, Double>  scale    = new EnumMap<>(AnalogInput.class);

    private CommonUtil com;
    private HalDashboard dashboard;

    private final Gamepad gamepad;

    public ManagedGamepad(Gamepad gamepad)
    {
        this.gamepad = gamepad;
        com = CommonUtil.getInstance();
        dashboard = com.getDashboard();
        init();
    }

    private void init()
    {
        for(Button b : Button.values())
        {
            current.put(b, false);
            previous.put(b, false);
            just_pressed.put(b, false);
        }

        for(AnalogInput a : AnalogInput.values())
        {
            scale.put(a, 0.0);
        }
    }

    private void update_just_pressed()
    {
        for(int i = 0; i < Button.values().length ; i++)
        {
            Button b = Button.values()[i];
            just_pressed.put(b,   current.get(b) && !previous.get(b));
            just_released.put(b, !current.get(b) &&  previous.get(b));
            previous.put(b, current.get(b));
        }
    }

    public void update()
    {
        if(gamepad == null)
        {
            if(cycle == 0)
            {
                RobotLog.ee(TAG, "ERROR: No gamepad");
            }
            cycle++;
            return;
        }

        current.put(Button.A,         gamepad.a);
        current.put(Button.B,         gamepad.b);
        current.put(Button.X,         gamepad.x);
        current.put(Button.Y,         gamepad.y);
        current.put(Button.D_UP,      gamepad.dpad_up);
        current.put(Button.D_DOWN,    gamepad.dpad_down);
        current.put(Button.D_LEFT,    gamepad.dpad_left);
        current.put(Button.D_RIGHT,   gamepad.dpad_right);
        current.put(Button.L_BUMP,    gamepad.left_bumper);
        current.put(Button.R_BUMP,    gamepad.right_bumper);
        current.put(Button.L_TRIGGER, gamepad.left_trigger  > 0.1);
        current.put(Button.R_TRIGGER, gamepad.right_trigger > 0.1);
        current.put(Button.L_STICK_BUTTON, gamepad.left_stick_button);
        current.put(Button.R_STICK_BUTTON, gamepad.right_stick_button);
        scale.put(AnalogInput.L_STICK_X,     (double)gamepad.left_stick_x);
        scale.put(AnalogInput.L_STICK_Y,     (double)gamepad.left_stick_y);
        scale.put(AnalogInput.R_STICK_X,     (double)gamepad.right_stick_x);
        scale.put(AnalogInput.R_STICK_Y,     (double)gamepad.right_stick_y);
        scale.put(AnalogInput.L_TRIGGER_VAL, (double)gamepad.left_trigger);
        scale.put(AnalogInput.R_TRIGGER_VAL, (double)gamepad.right_trigger);

        boolean aaa = gamepad.left_stick_button;

        update_just_pressed();
    }

    public boolean just_pressed(Button b)
    {
        return just_pressed.get(b);
    }

    public boolean just_released(Button b)
    {
        return just_released.get(b);
    }

    public boolean pressed(Button b)
    {
        return current.get(b);
    }

    public double value(AnalogInput a)
    {
        return scale.get(a);
    }

    public void log(int idx)
    {
        String pressed = "1_";
        for(Button b : Button.values())
        {
            if (pressed(b))
            {
                pressed += b;
            }
        }
        dashboard.displayPrintf(idx, "%s", pressed);
        for(AnalogInput a : AnalogInput.values())
        {
            idx +=2;
            dashboard.displayPrintf(idx, "%s %4.3f", a, scale.get(a));
        }
    }

    public enum Button
    {
        A,
        B,
        X,
        Y,
        D_UP,
        D_DOWN,
        D_LEFT,
        D_RIGHT,
        L_BUMP,
        R_BUMP,
        L_TRIGGER,
        R_TRIGGER,
        L_STICK_BUTTON,
        R_STICK_BUTTON
    }

    public enum AnalogInput
    {
        L_STICK_X,
        L_STICK_Y,
        R_STICK_X,
        R_STICK_Y,
        L_TRIGGER_VAL,
        R_TRIGGER_VAL
    }
}
