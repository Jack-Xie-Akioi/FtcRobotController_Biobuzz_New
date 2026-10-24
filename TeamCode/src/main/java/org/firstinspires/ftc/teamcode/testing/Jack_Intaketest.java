package org.firstinspires.ftc.teamcode.testing;

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

@TeleOp(name = "Jack Intake Test")
@Configurable
public class Jack_Intaketest extends OpMode {
    public static int m = 0;
    public static double Servo_pos = 0.5;

    private DcMotorEx intakeleft;
    private DcMotorEx intakeright;

    private Servo servoL;
    private Servo servoR;

    private TelemetryManager panelsTelemetry;

    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

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
    }

    @Override
    public void start() {

    }

    public void loop() {
        int Intake_power;
        if (m == 1) {
            Intake_power = 1;
        } else if (m == 2) {
            Intake_power = -1;
        } else {
            Intake_power = 0;
        }

        intakeleft.setPower(Intake_power);
        intakeright.setPower(Intake_power);
        servoL.setPosition(Servo_pos-constants.intake.servo_offset);
        servoR.setPosition(Servo_pos);

        panelsTelemetry.debug("Mode", m);
        panelsTelemetry.debug("Power", Intake_power);
        panelsTelemetry.debug("Servo pos", Servo_pos);
        panelsTelemetry.update(telemetry);
    }
}