package org.firstinspires.ftc.teamcode.subsystems;

import static android.webkit.WebSettings.PluginState.OFF;
import static android.webkit.WebSettings.PluginState.ON;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
        servoR = map.get(Servo.class, "IntakeServoRight");
        servoL = map.get(Servo.class, "IntakeServoLeft");

    }

    public void update() {

    }

    public void setIntake(constants.INTAKE_PRESETS state) {
        switch (state) {
            case ON:
                intakeState = "ON";
                motorL.setPower(1);//does this need a specific motor object? template didn't have one
                motorR.setPower(1);
                servoL.setPosition(constants.INTAKE_PRESETS.servoAngleDown);
                servoR.setPosition(constants.INTAKE_PRESETS.servoAngleDown);
                break;
            case OFF:
                intakeState= "OFF";
                motorL.setPower(0);
                motorR.setPower(0);
                servoR.setPosition(constants.INTAKE_PRESETS.servoAngleIdle);
                servoL.setPosition(constants.INTAKE_PRESETS.servoAngleIdle);
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