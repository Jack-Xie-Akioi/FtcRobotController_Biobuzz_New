package org.firstinspires.ftc.teamcode.subsystems;

//Imports
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public final class Transfer extends Subsystem{

    //Case based architecture
    public enum TransferState {
        STOPPED,
        RUNNING,
        REVERSING
    }

    public enum StopperState {
        CLOSED,
        OPEN
    }

    private static final String MOTOR_NAME = "transfer";
    private static final String SERVO_NAME = "stopper";

    //Calibration Necessary !!
    private static final double CLOSED_POSITION = 0.2;
    private static final double OPEN_POSITION = 0.75;

    private static final double CURRENT_THRESHOLD_AMPS = 4.0;
    private static final long CURRENT_DURATION_MS = 100;
    private static final long STOPPER_TRAVEL_MS = 200;

    private final DcMotorEx transfer;
    private final Servo stopper;

    private TransferState transferState = TransferState.STOPPED;
    private StopperState stopperState = StopperState.CLOSED;

    private StopperState appliedStopperState = null;

    private double transferPower = 1.0;
    private double reversePower = 1.0;

    private double measuredCurrentAmps;
    private double commandedMotorPower;
    private double lastMotorPower = 0;

    private boolean currentCutoffLatched;
    private boolean openingPending;

    private final ElapsedTime currentTimer = new ElapsedTime();
    private boolean timingOverCurrent = false;
    private final ElapsedTime stopperTimer = new ElapsedTime();


    //Constructor
    public Transfer(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        stopper = hardwareMap.get(Servo.class, SERVO_NAME);

        transfer.setDirection(DcMotorEx.Direction.FORWARD);
        transfer.setPower(0);
        transfer.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        stopper.setDirection(Servo.Direction.FORWARD);

    }

    //Setter and Getter Methods
    public void setTransferState(TransferState state) {
        if (state == null) {
            throw new IllegalArgumentException("TRANSFER MOTOR STATE NULL");
        }

        transferState = state;
    }

    public TransferState getTransferState() {
        return transferState;
    }

    public void setStopperState(StopperState state) {
        if (state == null) {
            throw new IllegalArgumentException("STOPPER SERVO STATE NULL");
        }

        stopperState = state;
    }

    public StopperState getStopperState() {
        return stopperState;
    }

    //Commands
    public void openStopper() {
        setStopperState(StopperState.OPEN);
    }

    public void closeStopper() {
        setStopperState(StopperState.CLOSED);
    }

    public void setTransferPower(double power) {
        transferPower = validatePower(power);
    }

    public void setReversePower(double power) {
        reversePower = validatePower(power);
    }

    public double getCurrentAmps() {
        return measuredCurrentAmps;
    }

    public boolean isCurrentCutoffLatched() {
        return currentCutoffLatched;
    }

    //Estimated servo completion time for runtime analysis (NOT ABSOLUTE, REQUIRES TUNING)
    public boolean isStopperReady() {
        return appliedStopperState != null && appliedStopperState == stopperState && stopperTimer.milliseconds() >= STOPPER_TRAVEL_MS;
    }

    @Override
    public void read() {
        measuredCurrentAmps = transfer.getCurrent(CurrentUnit.AMPS);
    }

    @Override
    public void update() {
        if (openingPending && stopperState == StopperState.OPEN && isStopperReady()) {
            currentCutoffLatched = false;
            openingPending = false;
            timingOverCurrent = false;
        }

        switch (transferState) {
            case STOPPED:
                commandedMotorPower = 0;
                break;

            case RUNNING:
                commandedMotorPower = transferPower;
                break;

            case REVERSING:
                commandedMotorPower = -reversePower;
                break;
        }

        updateCurrentCutoff();

        if (currentCutoffLatched) {
            commandedMotorPower = 0;
        }
    }

    private void updateCurrentCutoff() {
        boolean monitoring = !currentCutoffLatched
                && lastMotorPower != 0
                && commandedMotorPower != 0;

        if (!monitoring || measuredCurrentAmps < CURRENT_THRESHOLD_AMPS) {
            timingOverCurrent = false;
            return;
        }

        if (!timingOverCurrent) {
            currentTimer.reset();
            timingOverCurrent = true;
        }

        if (currentTimer.milliseconds() >= CURRENT_DURATION_MS) {
            currentCutoffLatched = true;
            timingOverCurrent = false;
        }
    }

    @Override
    public void write() {
        if (Double.compare(commandedMotorPower, lastMotorPower) != 0) {
            transfer.setPower(commandedMotorPower);
            lastMotorPower = commandedMotorPower;
        }

        if (appliedStopperState != stopperState) {
            switch (stopperState) {
                case CLOSED:
                    stopper.setPosition(CLOSED_POSITION);
                    break;

                case OPEN:
                    stopper.setPosition(OPEN_POSITION);
                    break;
            }

            appliedStopperState = stopperState;
            stopperTimer.reset();

            openingPending = stopperState == StopperState.OPEN;
        }
    }

    @Override
    public void stop() {
        transferState = TransferState.STOPPED;
        commandedMotorPower = 0;
        timingOverCurrent = false;
        openingPending = false;

        transfer.setPower(0);
        lastMotorPower = 0;
    }

    //Helper Method(s)
    private static double validatePower(double power) {
        if (Double.isNaN(power) || Double.isInfinite(power) || power < 0 || power > 1) {
            throw new IllegalArgumentException("MOTOR POWER MUST BE BETWEEN 0 AND 1");
        }

        return power;
    }
}
