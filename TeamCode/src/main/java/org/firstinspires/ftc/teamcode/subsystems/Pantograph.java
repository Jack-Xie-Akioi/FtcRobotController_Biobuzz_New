package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class Pantograph extends Subsystem{

    public enum PantographState {
        RAISED,
        LOWERED
    }

    private static final String SERVO_NAME = "pantograph";

    // Calibration necessary !!
    public static double RAISED_POSITION = 0.2;
    public static double LOWERED_POSITION = 0.75;

    private final Servo pantograph;

    private PantographState pantographState = PantographState.RAISED;

    private double commandedServoPos;
    private double lastServoPos = Double.NaN;

    public Pantograph(HardwareMap hardwareMap) {
        pantograph = hardwareMap.get(Servo.class, SERVO_NAME);

        pantograph.setDirection(Servo.Direction.FORWARD);

        update();
    }

    //Getter and Setter Methods
    public void setPantographState(PantographState state) {
        if (state == null) {
            throw new IllegalArgumentException("PANTOGRAPH SERVO STATE NULL");
        }

        pantographState = state;
    }

    public PantographState getPantographState() {
        return pantographState;
    }

    //Commands
    public void raise() {
        setPantographState(PantographState.RAISED);
    }

    public void lower() {
        setPantographState(PantographState.LOWERED);
    }

    @Override
    public void update() {
        switch (pantographState) {
            case RAISED:
                commandedServoPos = RAISED_POSITION;
                break;

            case LOWERED:
                commandedServoPos = LOWERED_POSITION;
                break;
        }
    }

    @Override
    public void write() {
        if (Double.compare(commandedServoPos, lastServoPos) != 0) {
            pantograph.setPosition(commandedServoPos);
            lastServoPos = commandedServoPos;
        }
    }

    @Override
    public void stop() {
        //Empty
    }
}
