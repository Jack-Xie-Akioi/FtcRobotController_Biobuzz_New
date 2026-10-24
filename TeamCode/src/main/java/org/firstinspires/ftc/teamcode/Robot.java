package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Pantograph;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.LUT;

import java.util.List;

public final class Robot {

    public enum Alliance {
        BLUE,
        RED
    }

    public enum Target {
        BLUE_CLOSE(Alliance.BLUE),
        BLUE_FAR(Alliance.BLUE),
        RED_CLOSE(Alliance.RED),
        RED_FAR(Alliance.RED);

        private final Alliance alliance;

        Target(Alliance alliance) {
            this.alliance = alliance;
        }

        public Alliance getAlliance() {
            return alliance;
        }
    }

    public enum IntakeSequence {
        IDLE_OPEN,
        CLOSING_STOPPER,
        INTAKING,
        WAITING_TO_OPEN,
        MANUAL_OPEN
    }

    public static final int SURFACE_BLUE = Color.rgb(19, 176, 212);

    public static double STOPPER_OPEN_DELAY_MS = 500;
    public static double TURRET_WRAP_WAIT_MS = 1000;

    public final Follower follower;

    public final Intake intake;
    public final Transfer transfer;
    public final Shooter shooter;
    public final Turret turret;
    public final Pantograph pantograph;

    private final Subsystem[] subsystems;
    private final LUT shooterLUT;

    private final List<LynxModule> hubs;
    private final LynxModule.BulkCachingMode[] previousCacheModes;

    private Alliance alliance = Alliance.BLUE;
    private Target selectedTarget = Target.BLUE_CLOSE;

    private IntakeSequence intakeSequence = IntakeSequence.IDLE_OPEN;
    private Intake.IntakeState requestedIntakeState = Intake.IntakeState.STOPPED;

    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final ElapsedTime wrapTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();

    private boolean autoAimEnabled;
    private boolean shootingEnabled;
    private boolean shotAvailable;
    private boolean wrapWaiting;
    private boolean stopperOpenOverride;

    private boolean loopOpen;
    private boolean firstLoop = true;
    private boolean stopped;

    private boolean telemetryEnabled = true;
    private double telemetryIntervalMs = 100;

    private double loopPeriodMs;
    private double distanceToTarget = Double.NaN;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Follower follower) {
        if (hardwareMap == null || follower == null) {
            throw new IllegalArgumentException("HARDWARE MAP AND FOLLOWER REQUIRED");
        }

        this.follower = follower;

        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        pantograph = new Pantograph(hardwareMap);

        subsystems = new Subsystem[] {
                intake,
                transfer,
                shooter,
                turret,
                pantograph
        };

        shooterLUT = new LUT();

        hubs = hardwareMap.getAll(LynxModule.class);
        previousCacheModes = new LynxModule.BulkCachingMode[hubs.size()];

        for (int i = 0; i < hubs.size(); i++) {
            LynxModule hub = hubs.get(i);

            previousCacheModes[i] = hub.getBulkCachingMode();

            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hub.setConstant(SURFACE_BLUE);
        }

        if (telemetry != null) {
            telemetry.setMsTransmissionInterval(100);
        }
    }

    public void setAlliance(Alliance alliance) {
        if (alliance == null) {
            throw new IllegalArgumentException("ALLIANCE NULL");
        }

        if (this.alliance == alliance) {
            return;
        }

        this.alliance = alliance;
        selectedTarget = alliance == Alliance.BLUE ? Target.BLUE_CLOSE : Target.RED_CLOSE;

        shotAvailable = false;
        distanceToTarget = Double.NaN;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void setTarget(Target target) {
        if (target == null) {
            throw new IllegalArgumentException("TARGET NULL");
        }

        if (target.getAlliance() != alliance) {
            throw new IllegalArgumentException("TARGET DOES NOT BELONG TO SELECTED ALLIANCE");
        }

        if (selectedTarget != target) {
            selectedTarget = target;
            shotAvailable = false;
            distanceToTarget = Double.NaN;
        }
    }

    public Target getTarget() {
        return selectedTarget;
    }

    public void switchTarget() {
        switch (selectedTarget) {
            case BLUE_CLOSE:
                setTarget(Target.BLUE_FAR);
                break;

            case BLUE_FAR:
                setTarget(Target.BLUE_CLOSE);
                break;

            case RED_CLOSE:
                setTarget(Target.RED_FAR);
                break;

            case RED_FAR:
                setTarget(Target.RED_CLOSE);
                break;
        }
    }

    public void setAutoAimEnabled(boolean enabled) {
        if (autoAimEnabled == enabled) {
            return;
        }

        if (!enabled) {
            turret.setManualAngleDegrees(turret.getCommandedAngleDegrees());
        }

        autoAimEnabled = enabled;
        shotAvailable = false;
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }

    public void setShootingEnabled(boolean enabled) {
        shootingEnabled = enabled;
    }

    public void setStopperOpenOverride(boolean enabled) {
        stopperOpenOverride = enabled;
    }

    public boolean isStopperOpenOverride() {
        return stopperOpenOverride;
    }

    public boolean isShootingEnabled() {
        return shootingEnabled;
    }

    public boolean isShotAvailable() {
        return autoAimEnabled && shotAvailable;
    }

    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    public void startIntaking() {
        requestedIntakeState = Intake.IntakeState.INTAKING;
    }

    public void reverseIntake() {
        requestedIntakeState = Intake.IntakeState.REVERSED;
    }

    public void stopIntaking() {
        requestedIntakeState = Intake.IntakeState.STOPPED;
    }

    public IntakeSequence getIntakeSequence() {
        return intakeSequence;
    }

    public void beginLoop() {
        requireRunning();

        if (loopOpen) {
            throw new IllegalStateException("CALL UPDATE BEFORE BEGINNING ANOTHER LOOP");
        }

        loopPeriodMs = firstLoop ? 0 : loopTimer.milliseconds();
        loopTimer.reset();
        firstLoop = false;

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        follower.update();

        Pose pose = follower.pose();

        turret.setRobotPose(pose.x(), pose.y(), Math.toDegrees(pose.heading()));

        for (Subsystem subsystem : subsystems) {
            subsystem.read();
        }

        loopOpen = true;
    }

    public void update() {
        requireRunning();

        if (!loopOpen) {
            throw new IllegalStateException("CALL BEGIN LOOP BEFORE UPDATE");
        }

        try {
            updateIntakeSequence();
            updateAiming();

            boolean allowShooter = shootingEnabled && (!autoAimEnabled || shotAvailable);

            shooter.setShooterState(allowShooter ? Shooter.ShooterState.RUNNING : Shooter.ShooterState.STOPPED);

            for (Subsystem subsystem : subsystems) {
                subsystem.update();
            }

            for (Subsystem subsystem : subsystems) {
                subsystem.write();
            }

            if (turret.isWrapRequested()) {
                wrapWaiting = true;
                wrapTimer.reset();
            }

            if (wrapWaiting && wrapTimer.milliseconds() >= TURRET_WRAP_WAIT_MS) {
                wrapWaiting = false;
            }
        } finally {
            loopOpen = false;
        }
    }

    private void updateIntakeSequence() {
        boolean intakeRequested = requestedIntakeState != Intake.IntakeState.STOPPED;

        intake.setIntakeState(Intake.IntakeState.STOPPED);

        if (stopperOpenOverride) {
            transfer.openStopper();
            intake.setIntakeState(requestedIntakeState);
            intakeSequence = IntakeSequence.MANUAL_OPEN;
            return;
        }

        switch (intakeSequence) {
            case IDLE_OPEN:
                if (intakeRequested) {
                    transfer.closeStopper();
                    intakeSequence = IntakeSequence.CLOSING_STOPPER;
                } else {
                    transfer.openStopper();
                }
                break;

            case CLOSING_STOPPER:
                transfer.closeStopper();

                if (!intakeRequested) {
                    intakeTimer.reset();
                    intakeSequence = IntakeSequence.WAITING_TO_OPEN;
                } else if (transfer.isStopperReady()) {
                    intake.setIntakeState(requestedIntakeState);
                    intakeSequence = IntakeSequence.INTAKING;
                }
                break;

            case INTAKING:
                transfer.closeStopper();

                if (intakeRequested) {
                    intake.setIntakeState(requestedIntakeState);
                } else {
                    intakeTimer.reset();
                    intakeSequence = IntakeSequence.WAITING_TO_OPEN;
                }
                break;

            case WAITING_TO_OPEN:
                transfer.closeStopper();

                if (intakeRequested) {
                    intakeSequence = IntakeSequence.CLOSING_STOPPER;
                } else if (intakeTimer.milliseconds() >= STOPPER_OPEN_DELAY_MS) {
                    transfer.openStopper();
                    intakeSequence = IntakeSequence.IDLE_OPEN;
                }
                break;

            case MANUAL_OPEN:
                if (intakeRequested) {
                    transfer.closeStopper();
                    intakeSequence = IntakeSequence.CLOSING_STOPPER;
                } else {
                    transfer.openStopper();
                    intakeSequence = IntakeSequence.IDLE_OPEN;
                }
                break;
        }
    }

    private void updateAiming() {
        shotAvailable = false;

        double targetX;
        double targetY;

        switch (selectedTarget) {
            case BLUE_CLOSE:
                targetX = Constants.BLUE_CLOSE_X;
                targetY = Constants.BLUE_CLOSE_Y;
                break;

            case BLUE_FAR:
                targetX = Constants.BLUE_FAR_X;
                targetY = Constants.BLUE_FAR_Y;
                break;

            case RED_CLOSE:
                targetX = Constants.RED_CLOSE_X;
                targetY = Constants.RED_CLOSE_Y;
                break;

            case RED_FAR:
                targetX = Constants.RED_FAR_X;
                targetY = Constants.RED_FAR_Y;
                break;

            default:
                throw new IllegalStateException("INVALID TARGET");
        }

        if (!isFinite(targetX) || !isFinite(targetY)) {
            throw new IllegalArgumentException("TARGET COORDINATES MUST BE FINITE");
        }

        distanceToTarget = Math.hypot(targetX - turret.getPivotX(), targetY - turret.getPivotY());

        if (!autoAimEnabled) {
            return;
        }

        turret.setTarget(targetX, targetY);

        LUT.ShotSettings settings = shooterLUT.getShot(distanceToTarget);

        if (settings != null) {
            shooter.setShot(settings);
            shotAvailable = true;
        }
    }

    public boolean isWrapWaiting() {
        return wrapWaiting;
    }

    public boolean isReadyToShoot() {
        if (stopped || !shootingEnabled || !shooter.isReady() || wrapWaiting) {
            return false;
        }

        return !autoAimEnabled || (shotAvailable && turret.canAim());
    }

    public double getLoopPeriodMs() {
        return loopPeriodMs;
    }

    public void setTelemetryEnabled(boolean enabled) {
        telemetryEnabled = enabled;
    }

    public void setTelemetryIntervalMs(double intervalMs) {
        if (!isFinite(intervalMs) || intervalMs < 1) {
            throw new IllegalArgumentException("TELEMETRY INTERVAL MUST BE AT LEAST 1 MS");
        }

        telemetryIntervalMs = intervalMs;
        telemetryTimer.reset();
    }

    public boolean telemetryDue() {
        if (stopped || !telemetryEnabled || telemetryTimer.milliseconds() < telemetryIntervalMs) {
            return false;
        }

        telemetryTimer.reset();
        return true;
    }

    public void stop() {
        if (stopped) {
            return;
        }

        stopped = true;
        loopOpen = false;
        shootingEnabled = false;
        requestedIntakeState = Intake.IntakeState.STOPPED;

        RuntimeException failure = null;

        try {
            follower.stop();
            follower.update();
        } catch (RuntimeException e) {
            failure = e;
        }

        for (Subsystem subsystem : subsystems) {
            try {
                subsystem.stop();
            } catch (RuntimeException e) {
                if (failure == null) {
                    failure = e;
                } else if (failure != e) {
                    failure.addSuppressed(e);
                }
            }
        }

        for (int i = 0; i < hubs.size(); i++) {
            try {
                hubs.get(i).setBulkCachingMode(previousCacheModes[i]);
            } catch (RuntimeException e) {
                if (failure == null) {
                    failure = e;
                } else if (failure != e) {
                    failure.addSuppressed(e);
                }
            }
        }

        if (failure != null) {
            throw failure;
        }
    }

    private void requireRunning() {
        if (stopped) {
            throw new IllegalStateException("ROBOT HAS STOPPED");
        }
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}