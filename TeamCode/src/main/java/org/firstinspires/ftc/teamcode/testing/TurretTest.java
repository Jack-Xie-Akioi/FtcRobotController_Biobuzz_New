package org.firstinspires.ftc.teamcode.testing;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Configurable
@TeleOp(name = "Turret Test", group = "Test")
public class TurretTest extends OpMode{
    private Turret turret;
    private TelemetryManager panelsTelemetry;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        turret = new Turret();
        turret.init(hardwareMap);
    }

    @Override
    public void loop() {
        turret.periodic();
    }
}
