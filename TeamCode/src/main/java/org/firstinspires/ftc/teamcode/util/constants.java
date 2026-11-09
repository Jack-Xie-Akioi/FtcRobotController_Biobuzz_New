package org.firstinspires.ftc.teamcode.util;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;


@Configurable
public class constants {

    //General Constants
    public static double NOMINAL_VOLTAGE = 12.0;

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
        // Flywheel targets (set at runtime by shooter.setTarget / far())
        public static double TARGET_RPM = 0;      // actually ticks/sec, since getVelocity() returns ticks/sec
        public static double Target_Hood = 0;  // where the hood should go
        public static double Hood_pos = 0;     // where the hood is currently commanded

        // Flywheel control gains
        public static double kS = 0.0;              // TUNE: static friction power
        public static double kV = 0.000435;         // TUNE: power per tick/sec (starting guess = 1 / 2800 for a 6000 rpm, 28 tick/rev motor)
        public static double kP = 0.001;           // TUNE: proportional gain on velocity error
        public static double CLAMP_NEG_POWER = 0.0; // lowest power allowed (0 = never reverse-brake the flywheel)

        // Hood servo
        public static double MIN_ANGLE = 0.1;
        public static double MAX_ANGLE = 1;

        // Stopper servo
        public static double STOP = 0.0;            // TUNE: stopper blocking the ball
        public static double PASSTHROUGH = 0.5;     // TUNE: stopper open, ball can feed into the flywheel
    }

    public enum HOOD {
        RESET, MANUAL, AUTO
    }

    public enum FLYWHEEL {
        ON, OFF
    }
}