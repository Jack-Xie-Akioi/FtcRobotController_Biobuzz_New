package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.LUT;

public final class Shooter extends Subsystem{

    public enum ShooterState {
        STOPPED,
        RUNNING
    }

    private static final String MOTOR_NAME = "shooter";
    private static final String SERVO_NAME = "hood";

    public static double VELOCITY_TOLERANCE = 10;

    private static final double HOOD_MIN_POSITION = 0.2;
    private static final double HOOD_MAX_POSITION = 0.8;
    private static final double HOOD_INITIAL_POSITION = HOOD_MIN_POSITION;
    private static final double HOOD_SETTLE_MS = 75;

    private final DcMotorEx shooter;
    private final Servo hood;

    private ShooterState shooterState = ShooterState.STOPPED;

    private double targetVel;
    private double measuredVel;
    private double targetHoodPos = HOOD_INITIAL_POSITION;

    private double commandedMotorPower;
    private double lastMotorPower = 0;
    private double lastHoodPos = Double.NaN;

    private final ElapsedTime controlTimer = new ElapsedTime();
    private final ElapsedTime hoodTimer = new ElapsedTime();

    private double integral;
    private double previousVel;
    private boolean controllerInitialized;

    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        hood = hardwareMap.get(Servo.class, SERVO_NAME);

        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setPower(0);
        shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        hood.setDirection(Servo.Direction.FORWARD);
    }

    public void setShooterState(ShooterState state) {
        if (state == null) {
            throw new IllegalArgumentException("SHOOTER STATE NULL");
        }

        if (shooterState != state) {
            resetController();
        }

        shooterState = state;
    }

    public ShooterState getShooterState() {
        return shooterState;
    }

    public void setTargetVel(double velocity) {
        validateVel(velocity);
        targetVel = velocity;
    }

    public double getTargetVel() {
        return targetVel;
    }

    public double getVel() {
        return measuredVel;
    }

    public double getVelError() {
        return targetVel - measuredVel;
    }

    public double getCommandedMotorPower() {
        return commandedMotorPower;
    }

    public void setHoodPos(double position) {
        validateHoodPos(position);
        targetHoodPos = position;
    }

    public double getHoodPos() {
        return targetHoodPos;
    }

    public void setShot(LUT.ShotSettings settings) {
        if (settings == null) {
            throw new IllegalArgumentException("SHOT SETTINGS NULL");
        }

        validateVel(settings.velocity);
        validateHoodPos(settings.hoodPosition);

        targetVel = settings.velocity;
        targetHoodPos = settings.hoodPosition;
    }

    public boolean isAtSpeed() {
        return controllerInitialized && shooterState == ShooterState.RUNNING && targetVel > 0 && Math.abs(getVelError()) <= VELOCITY_TOLERANCE;
    }

    public boolean isHoodReady() {
        return Double.compare(targetHoodPos, lastHoodPos) == 0 && hoodTimer.milliseconds() >= HOOD_SETTLE_MS;
    }

    public boolean isReady() {
        return isAtSpeed() && isHoodReady();
    }

    @Override
    public void read() {
        measuredVel = shooter.getVelocity();
    }

    @Override
    public void update() {
        double dt = controlTimer.seconds();
        controlTimer.reset();

        switch (shooterState) {
            case STOPPED:
                commandedMotorPower = 0;
                resetController();
                break;

            case RUNNING:
                if (targetVel <= 0) {
                    commandedMotorPower = 0;
                    resetController();
                } else {
                    updateVelControl(dt);
                }
                break;
        }
    }

    private void updateVelControl(double dt) {
        double error = targetVel - measuredVel;
        double candidateIntegral = integral;
        double derivative = 0;

        boolean validDt = controllerInitialized && dt > 0 && dt <= 0.1;

        if (Constants.KI == 0) {
            integral = 0;
            candidateIntegral = 0;
        } else if (validDt) {
            candidateIntegral += error * dt;
        }

        if (validDt) {
            derivative = -(measuredVel - previousVel) / dt;
        }

        double baseOutput = Constants.KS + Constants.KV * targetVel + Constants.KP * error + Constants.KD * derivative;

        double candidateOutput = baseOutput + Constants.KI * candidateIntegral;
        double integralChange = Constants.KI * (candidateIntegral - integral);

        if ((candidateOutput >= 0 && candidateOutput <= 1) || (candidateOutput > 1 && integralChange < 0) || (candidateOutput < 0 && integralChange > 0)) {
            integral = candidateIntegral;
        }

        commandedMotorPower = Math.max(0, Math.min(1, baseOutput + Constants.KI * integral));

        previousVel = measuredVel;
        controllerInitialized = true;
    }

    @Override
    public void write() {
        if (Double.compare(commandedMotorPower, lastMotorPower) != 0) {
            shooter.setPower(commandedMotorPower);
            lastMotorPower = commandedMotorPower;
        }

        if (Double.compare(targetHoodPos, lastHoodPos) != 0) {
            hood.setPosition(targetHoodPos);
            lastHoodPos = targetHoodPos;
            hoodTimer.reset();
        }
    }

    @Override
    public void stop() {
        shooterState = ShooterState.STOPPED;
        targetVel = 0;
        commandedMotorPower = 0;

        resetController();

        shooter.setPower(0);
        lastMotorPower = 0;
    }

    private void resetController() {
        integral = 0;
        controllerInitialized = false;
        controlTimer.reset();
    }

    private static void validateVel(double velocity) {
        if (!isFinite(velocity) || velocity < 0) {
            throw new IllegalArgumentException("VELOCITY MUST BE FINITE AND NONNEGATIVE");
        }
    }

    private static void validateHoodPos(double position) {
        if (!isFinite(position)
                || position < HOOD_MIN_POSITION
                || position > HOOD_MAX_POSITION) {
            throw new IllegalArgumentException("HOOD POSITION OUTSIDE LIMITS");
        }
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
