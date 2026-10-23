package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.constants;

@TeleOp(name = "Intake Test")
public class IntakeTest extends OpMode {
    private Intake intake;
    public void init(){
        intake = new Intake();
        intake.init(hardwareMap);
    }
    @Override
    public void start(){

    }
    public void loop(){
        intake.setIntake(constants.INTAKE_PRESETS.on);

    }

}