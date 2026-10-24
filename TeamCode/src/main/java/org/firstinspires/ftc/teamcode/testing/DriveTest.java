package org.firstinspires.ftc.teamcode.testing;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.constants;

@TeleOp(name = "Eric's Training Session")
public class DriveTest extends OpMode {
    private Follower follower;
    private DcMotorEx intakeleft;
    private DcMotorEx intakeright;
    Intake intake;
    @Override
    public void init() {
        intake = new Intake();
        intake.init(hardwareMap);
        follower = Constants.create(hardwareMap);
    }
    @Override
    public void loop() {

        //Teleop with Auto Hold
        ManualDrive.driveOrHold(
                follower,
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        if (gamepad1.right_trigger>0)
        {
            intake.setIntake(constants.INTAKE_PRESETS.ON);
        }
        else if (gamepad1.left_trigger>0){
            intake.setIntake(constants.INTAKE_PRESETS.OUT);
        }
        else{
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
        }
        follower.update();
    }
}