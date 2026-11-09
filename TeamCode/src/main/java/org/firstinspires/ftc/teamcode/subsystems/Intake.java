package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.constants;

public class Intake {

    private DcMotorEx motorL;
    private DcMotorEx motorR;

    public Servo servoR;
    public Servo servoL;

    private String intakeState;

    public void init(HardwareMap map) {
        motorR = map.get(DcMotorEx.class, "IntakeRight");
        motorL = map.get(DcMotorEx.class, "IntakeLeft");
        motorL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setDirection(DcMotorSimple.Direction.REVERSE);

        servoR = map.get(Servo.class, "IntakeServoRight");
        servoL = map.get(Servo.class, "IntakeServoLeft");
        servoR.setDirection(Servo.Direction.REVERSE);

    }

    public void update() {

    }

    public void setIntake(constants.INTAKE_PRESETS state) {
        switch (state) {
            case ON:
                intakeState = "ON";
                setPower(1);
                servoL.setPosition(constants.intake.servo_down-constants.intake.servo_offset);
                servoR.setPosition(constants.intake.servo_down);
                break;
            case OFF:
                intakeState= "OFF";
                setPower(0);
                servoL.setPosition(constants.intake.servo_up-constants.intake.servo_offset);
                servoR.setPosition(constants.intake.servo_up);
                break;
            case OUT:
                intakeState= "OUT";
                setPower(-1);
                servoL.setPosition(constants.intake.servo_up-constants.intake.servo_offset);
                servoR.setPosition(constants.intake.servo_up);
                break;
        }
    }


    public void setPower(double power) {//what would this method be used for?
        motorR.setPower(power);
        motorL.setPower(power);
    }

    public String getIntakeState() {
        return intakeState;
    }



}