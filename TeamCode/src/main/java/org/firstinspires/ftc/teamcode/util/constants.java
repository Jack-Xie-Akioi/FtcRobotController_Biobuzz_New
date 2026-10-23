package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class constants {

    //General Constants

    //Intake Constants
    @Config
    public static class intake {

    }

    public enum INTAKE_PRESETS {
       //figure out how to add NO and Yes States
        ON,OFF;//work out how enum works
        public static final int servoAngleDown = 0;//I'm not super used to using enum, lmk if variables like this should go here
        public static final int servoAngleIdle=90;//these values are my best guess, idk where the servos are

    }

    @Config
    public enum INTAKE_EXTENSION {

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