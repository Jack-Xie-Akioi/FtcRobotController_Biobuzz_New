package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class constants {

    //General Constants

    //Intake Constants
    @Config
    public static class intake {
        public static double Servo1_up=0.5;
    }

    public enum INTAKE_PRESETS {
        on,
        off
    }

    @Config
    public enum INTAKE_EXTENSION {
        ON;
    }

    //Shooter Constants
    @Config
    public static class shooter {

    }

    public enum HOOD {

    }

    public enum FLYWHEEL {

    }
}