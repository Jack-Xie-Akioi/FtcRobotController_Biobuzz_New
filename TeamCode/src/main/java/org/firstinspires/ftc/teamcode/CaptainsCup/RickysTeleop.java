package org.firstinspires.ftc.teamcode.CaptainsCup;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.constants;

@TeleOp(name = "CaptainsCup")
@Configurable
public class RickysTeleop extends OpMode{
    public double y;
    public double x;
    public double rx;

    private DcMotorEx FrontLeft;
    private DcMotorEx FrontRight;
    private DcMotorEx BackLeft;
    private DcMotorEx BackRight;

    private DcMotorEx Intake1;
    private DcMotorEx Intake2;
    private DcMotorEx Shooter;

    public void init(){
        FrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        FrontRight = hardwareMap.get(DcMotorEx.class, "FR");
        BackLeft = hardwareMap.get(DcMotorEx.class, "BL");
        BackRight = hardwareMap.get(DcMotorEx.class, "BR");
        Intake1 = hardwareMap.get(DcMotorEx.class, "Intake1");
        Intake2 = hardwareMap.get(DcMotorEx.class, "Intake2");
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(gamepad1.circle){
            Shooter.setPower(0.8);
        }
        else if(gamepad1.cross){
            Shooter.setPower(0.9);
        }
        else if(gamepad1.triangle){
            Shooter.setPower(0.75);
        }
        else if(gamepad1.square){
            Shooter.setPower(0.7);
        }
        else{
            Shooter.setPower(0.7);
        }

        x = gamepad1.right_stick_x;
        y = gamepad1.left_stick_y;
        rx = gamepad1.left_stick_x;

        if(gamepad1.left_trigger > 0){
            Intake1.setPower(-1);
        }
        else if(gamepad1.left_bumper){
            Intake1.setPower(1);
        }
        else{
            Intake1.setPower(0);
        }
        if(gamepad1.right_trigger > 0){
            Intake2.setPower(1);
            Intake1.setPower(-1);
        }
        else{
            Intake2.setPower(0);
        }

        FrontLeft.setPower((x+y+rx)/3);
        FrontRight.setPower((y-x-rx)/3);
        BackLeft.setPower((y-x+rx)/3);
        BackRight.setPower((y+x-rx)/3);

    }

}
