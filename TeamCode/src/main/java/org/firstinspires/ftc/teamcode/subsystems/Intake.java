package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.constants;

public class Intake {

    private DcMotorEx motor;
    public Servo servo;

    private String intakeState;

    public Intake() {

    }

    public void init(HardwareMap map) {
        motor = map.get(DcMotorEx.class, "motor name");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        servo = map.get(Servo.class, "servo name");
    }

    public void update() {

    }

    public void setIntake(constants.INTAKE_PRESETS state) {
        switch (state) {
//            case ON:
//                intakeState = "ON";
//                setPower(1);
//                break;
        }
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public String getIntakeState() {
        return intakeState;
    }
}