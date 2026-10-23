package org.firstinspires.ftc.teamcode.util;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;


@Configurable
public class constants {

    //General Constants

    //Intake Constants
    @Configurable
    public static class intake {
        public static double Servo1_up=0.5;
    }

    public enum INTAKE_PRESETS {
       //figure out how to add NO and Yes States
        ON,OFF;//work out how enum works
        public static final int servoAngleDown = 0;//I'm not super used to using enum, lmk if variables like this should go here
        public static final int servoAngleIdle=90;//these values are my best guess, idk where the servos are

    }

    @Configurable
    public enum INTAKE_EXTENSION {
        ON;
    }

    //Shooter Constants
    @Configurable
    public static class shooter {

    }

    public enum HOOD {

    }

    public enum FLYWHEEL {

    }
}