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
        public static double servo_up=0.29;//I'm not super used to using enum, lmk if variables like this should go here
        public static double servo_down=0.5;
        public static double servo_offset=0.02;//left and right servo offset
    }

    public enum INTAKE_PRESETS {
       //figure out how to add NO and Yes States
        ON,OFF,OUT;//work out how enum works
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