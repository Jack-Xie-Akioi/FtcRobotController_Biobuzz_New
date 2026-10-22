package org.firstinspires.ftc.teamcode.testing;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedro.Constants;
@TeleOp(name = "Eric's Training Session")
public class DriveTest extends OpMode {
    private Follower follower;
    private DcMotorEx intakeleft;
    private DcMotorEx intakeright;
    @Override
    public void init() {

        follower = Constants.create(hardwareMap);
        intakeleft = hardwareMap.get(DcMotorEx.class, "IntakeLeft");
        intakeright = hardwareMap.get(DcMotorEx.class, "IntakeRight");

        intakeleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeright.setDirection(DcMotorSimple.Direction.REVERSE);
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
            intakeleft.setPower(1);
            intakeright.setPower(1);
        }
        else if (gamepad1.left_trigger>0){
            intakeleft.setPower(-1);
            intakeright.setPower(-1);
        }
        else{
            intakeleft.setPower(0);
            intakeright.setPower(0);
        }
        follower.update();
    }
}