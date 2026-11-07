package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.constants;

@TeleOp(name = "Shooter Test", group = "Test")
@Configurable
public class ShooterTest extends OpMode {
    private DcMotorEx motor;
    private Shooter shooter;

    // TUNE: encoder ticks per flywheel revolution
    private static final double TICKS_PER_REV = 28;

    // Change these live from Panels
    public static int mRPM = 0;
    public static double mpower = 0;
    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        motor = hardwareMap.get(DcMotorEx.class, "Shooter");
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooter = new Shooter();
        shooter.init(hardwareMap);

    }

    @Override
    public void start() {
        // runs once when you press START
    }

    @Override
    public void loop() {

        motor.setPower(mpower);
        //shooter.update(mRPM);
        double ticksPerSec = motor.getVelocity();


        panelsTelemetry.debug("Target RPM", constants.shooter.TARGET_RPM);
        panelsTelemetry.debug("Acutual RPM", ticksPerSec);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        motor.setPower(0);
        shooter.flywheelPreset(constants.FLYWHEEL.OFF);
    }
}