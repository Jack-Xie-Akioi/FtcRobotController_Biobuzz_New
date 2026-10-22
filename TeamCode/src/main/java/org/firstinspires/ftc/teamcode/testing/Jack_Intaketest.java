package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Jack Intake Test")
@Config
public class Jack_Intaketest extends OpMode {
    public static int mode=0;
    public static double Servo_pos=0;

    private DcMotorEx intakeleft;
    private DcMotorEx intakeright;

    private Servo servoL;
    private Servo servoR;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeleft = hardwareMap.get(DcMotorEx.class, "IntakeLeft");
        intakeright = hardwareMap.get(DcMotorEx.class, "IntakeRight");

        intakeleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeright.setDirection(DcMotorSimple.Direction.REVERSE);


        servoL = hardwareMap.get(Servo.class, "IntakeServoLeft");
        servoR = hardwareMap.get(Servo.class, "IntakeServoRight");
        servoR.setDirection(Servo.Direction.REVERSE);

        telemetry.update();
    }

    @Override
    public void loop() {
        int Intake_power;
        if (mode == 1) {
            Intake_power = 1;

        } else if (mode == 2) {
            Intake_power = -1;
        } else {
            Intake_power = 0;
        }

        intakeleft.setPower(Intake_power);
        intakeright.setPower(Intake_power);
        servoL.setPosition(Servo_pos);
        servoR.setPosition(Servo_pos);

        telemetry.addData("Mode", mode);
        telemetry.addData("Power", Intake_power);
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeleft.setPower(0);
        intakeright.setPower(0);
    }
}