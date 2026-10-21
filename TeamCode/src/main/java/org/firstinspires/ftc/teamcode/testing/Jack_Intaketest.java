package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Jack Intake Test")
@Config
public class Jack_Intaketest extends OpMode {
    // 0 = stop, 1 = intake, 2 = outtake (edit in FTC Dashboard, then click the save checkmark)
    public static int mode = 1;

    private DcMotorEx intakeleft;
    private DcMotorEx intakeright;

    @Override
    public void init() {
        // Send telemetry to both the Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeleft = hardwareMap.get(DcMotorEx.class, "IntakeLeft");
        intakeright = hardwareMap.get(DcMotorEx.class, "IntakeRight");

        intakeleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeright.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Init done - press START");
        telemetry.update();
    }

    @Override
    public void loop() {
        double power;
        if (mode == 1) {
            power = 1;
        } else if (mode == 2) {
            power = -1;
        } else {
            power = 0;
        }

        intakeleft.setPower(power);
        intakeright.setPower(power);

        telemetry.addData("Status", "loop running");
        telemetry.addData("Mode", mode);
        telemetry.addData("Power", power);
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeleft.setPower(0);
        intakeright.setPower(0);
    }
}