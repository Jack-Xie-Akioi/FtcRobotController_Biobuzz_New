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
        on,
        off
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