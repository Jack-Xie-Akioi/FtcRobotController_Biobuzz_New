package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.constants;

public class Shooter {
    private DcMotorEx motor;
    private Servo hood;
    private Servo stopper;
    private VoltageSensor battery;
    private ElapsedTime timer;

    public boolean auto = false;
    private double power = 0;
    private double rpm;

    private double voltage = 12.0;
    private double maxVoltageCompensation = 1.25;
    private double minVoltageCompensation = 0.90;

    public String hoodState;
    private boolean flywheelState;
    private boolean stopped = true;

    public Shooter() {

    }

    public void init(HardwareMap map) {
        motor = map.get(DcMotorEx.class, "Shooter");
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // If the flywheel spins the wrong way, uncomment:
        // motor.setDirection(DcMotorEx.Direction.REVERSE);

        hood = map.get(Servo.class, "ShooterHood");
        //stopper = map.get(Servo.class, "Stopper");

        timer = new ElapsedTime();
        timer.reset();

        // Pick the battery sensor with the lowest valid reading
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor vs : map.voltageSensor) {
            double v = vs.getVoltage();
            if (v > 0 && v < minVoltage) {
                minVoltage = v;
                battery = vs;
            }
        }
        voltage = (battery != null) ? battery.getVoltage() : 12.0;

        hoodState = "INIT";
        flywheelState = false;
    }

    public void hoodPreset(constants.HOOD preset) {
        switch (preset) {
            case RESET:
                setHood(constants.shooter.MIN_ANGLE);
                constants.shooter.Hood_pos = constants.shooter.MIN_ANGLE;
                hoodState = "RESET";
                break;
            case MANUAL:
                hoodState = "MANUAL";
                auto = false;
                break;
            case AUTO:
                hoodState = "AUTO";
                auto = true;
                break;
        }
    }

    public void flywheelPreset(constants.FLYWHEEL preset) {
        switch (preset) {
            case OFF:
                power = 0;
                motor.setPower(0);
                flywheelState = false;
                break;
            case ON:
                flywheelState = true;
                break;
        }
    }

    /** Set the flywheel speed and hood position you want. */
    public void setTarget(double targetRPM, double hoodPos) {
        constants.shooter.TARGET_RPM = targetRPM;
        constants.shooter.Target_Hood = hoodPos;
    }

    public void setTargetRPM(double targetRPM) {
        constants.shooter.TARGET_RPM = targetRPM;
    }

    public void setTargetHood(double hoodPos) {
        constants.shooter.Target_Hood = hoodPos;
    }


    /** Call every loop. Runs the flywheel controller and updates the hood. */
    public void update() {
        updateBatteryVoltage();

        if (flywheelState) {
            power = calculate();
            motor.setPower(power);
        } else {
            motor.setPower(0);
        }
        //setHood(constants.shooter.Hood_pos);
    }

    /** Feedforward + proportional control with battery voltage compensation. */
    public double calculate() {
        rpm = motor.getVelocity();

        double setpoint = constants.shooter.TARGET_RPM;
        double error = setpoint - rpm;

        double feedforward = constants.shooter.kS + constants.shooter.kV * setpoint;
        double feedback = constants.shooter.kP * error;

        double out = feedforward + feedback;

        double scale = constants.NOMINAL_VOLTAGE / voltage;
        scale = Math.max(minVoltageCompensation, Math.min(maxVoltageCompensation, scale));
        out *= scale;

        out = Math.max(constants.shooter.CLAMP_NEG_POWER, Math.min(1.0, out));
        return out;
    }


    public void updateBatteryVoltage() {
        if (battery == null) return;

        if (timer.seconds() >= 0.15) {
            double v = battery.getVoltage();
            if (v > 0) {
                voltage = v;
            }
            timer.reset();
        }
    }

    public void setHood(double pos) {
        hood.setPosition(Math.max(0, Math.min(1, pos)));
    }
    public void setPower(double power){
        motor.setPower(power);
    }

    public void setStopper(boolean isStopped) {
        if (!isStopped) {
            stopper.setPosition(constants.shooter.PASSTHROUGH);
            stopped = false;
        } else {
            stopper.setPosition(constants.shooter.STOP);
            stopped = true;
        }
    }

    @NonNull
    @Override
    public String toString() {
        return "Flywheel State: " + flywheelState + "\n" +
                "Hood State: " + hoodState + "\n" +
                "Stopper: " + stopped + "\n" +
                "Hood Position: " + hood.getPosition() + "\n" +
                "Target RPM: " + constants.shooter.TARGET_RPM + "\n" +
                "RPM: " + rpm + "\n" +
                "Power: " + power + "\n" +
                "Voltage: " + voltage;
    }

    public double getPower() {
        return power;
    }

    public double getRPM() {
        return motor.getVelocity();
    }

    public double getHoodAngle() {
        return hood.getPosition();
    }

    public double mToIn(double meters) {
        return meters * 39.3700787;
    }
}