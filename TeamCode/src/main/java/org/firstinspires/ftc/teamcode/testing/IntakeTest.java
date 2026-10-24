package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.constants;

@TeleOp(name = "Intake Test")
public class IntakeTest extends OpMode {
    private Intake intake;
    public void init(){
        intake = new Intake();
    }
    @Override
    public void start(){

    }
    public void loop(){
        intake.setIntake(constants.INTAKE_PRESETS.ON);

    }

}