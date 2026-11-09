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
    public static double mpower = 0;
    public static int mode=0;
    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        shooter = new Shooter();
        shooter.init(hardwareMap);

    }
    @Override
    public void start() {
        shooter.flywheelPreset(constants.FLYWHEEL.ON);
    }
    @Override
    public void loop() {
        if (mode==0){
            shooter.update();
        }
        else if (mode==1){
            shooter.setPower(mpower);
        }

        panelsTelemetry.debug("Target RPM", constants.shooter.TARGET_RPM);
        panelsTelemetry.debug("Acutual RPM", shooter.getRPM());
        panelsTelemetry.debug("Caculated Power", shooter.calculate());

        panelsTelemetry.addData("Target", constants.shooter.TARGET_RPM);
        panelsTelemetry.addData("Actual", shooter.getRPM());
        panelsTelemetry.addData("Error", constants.shooter.TARGET_RPM - shooter.getRPM());

        panelsTelemetry.update(telemetry);
    }
}