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

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.constants;

@TeleOp(name = "Everything Test")
@Configurable
public class EverythingTest extends OpMode {
    public static int m = 0;
    public static double Servo_pos = 0.5;

    private DcMotorEx intakeleft;
    private DcMotorEx intakeright;

    private Servo servoL;
    private Servo servoR;

    private TelemetryManager panelsTelemetry;

    private Shooter shooter;
    private Turret turret;

    public static double Intake_power=0;

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

        shooter = new Shooter();
        shooter.init(hardwareMap);

        turret = new Turret();
        turret.init(hardwareMap);
    }

    @Override
    public void start() {
        shooter.flywheelPreset(constants.FLYWHEEL.ON);
    }

    public void loop() {

        shooter.update();
        turret.periodic();

        intakeleft.setPower(Intake_power);
        intakeright.setPower(Intake_power);
        servoL.setPosition(Servo_pos-constants.intake.servo_offset);
        servoR.setPosition(Servo_pos);

        panelsTelemetry.debug("Target RPM", constants.shooter.TARGET_RPM);
        panelsTelemetry.debug("Acutual RPM", shooter.getRPM());
        panelsTelemetry.debug("Caculated Power", shooter.calculate());

        panelsTelemetry.addData("Target", constants.shooter.TARGET_RPM);
        panelsTelemetry.addData("Actual", shooter.getRPM());
        panelsTelemetry.addData("Error", constants.shooter.TARGET_RPM - shooter.getRPM());
        panelsTelemetry.update(telemetry);
    }
}