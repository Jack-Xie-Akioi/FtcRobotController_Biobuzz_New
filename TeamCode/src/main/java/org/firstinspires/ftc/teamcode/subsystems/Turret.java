package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import com.bylazar.configurables.annotations.Configurable;


@Configurable
public class Turret {

    public double SERVO_Pose =0;

    //Hardware
    private Servo left;
    private Servo right;

    //Constants
    public static double SERVO_TO_TURRET_RATIO = 1.333333333;

    public static double SERVO_DELTA = 0.0;

    public static int DIRECTION = 1;
    public static double SERVO_MAX = 0.85;
    public static double SERVO_MIN = 0.15;
    public static double SERVO_ZERO = 0.5;

    private double target;//angle in degrees
    private int lastPos;
    private double pos;//servo pos value

    private boolean aimed;
    private boolean correcting;


    public void init(HardwareMap map) {
        left = map.get(Servo.class, "turretLeft");
        left.setDirection(Servo.Direction.REVERSE);
        right = map.get(Servo.class, "turretRight");
        right.setDirection(Servo.Direction.REVERSE);
    }

    public void reset() {
        setTargetAngle(0);
    }




    public void update(double a) {
        setTargetAngle(a);
    }

    public void periodic() {
        correcting = false;
        pos = angleToServoPosition(target);
        pos = clamp(pos, SERVO_MIN, SERVO_MAX);
        hardwareUpdate(pos);
    }

    public void hardwareUpdate(double pos) {
        left.setPosition(pos + SERVO_DELTA);
        right.setPosition(pos - SERVO_DELTA);
        SERVO_Pose=pos;
    }

    public double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    public void setTargetAngle(double angle) {
        target = angle;
    }

    public void setManualAngle(double angle) {
        left.setPosition(angleToServoPosition(angle));
        right.setPosition(angleToServoPosition(angle));
    }

    public double angleToServoPosition(double angle) {
        return SERVO_ZERO + (angle / SERVO_TO_TURRET_RATIO) / (355);
    }

    private double clamp(double pos, double low, double high) {
        return (Math.max(low, Math.min(high, pos)));
    }
}
