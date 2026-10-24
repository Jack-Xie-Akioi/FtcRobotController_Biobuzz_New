package org.firstinspires.ftc.teamcode.subsystems;

//Imports
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class Intake extends Subsystem{

    //Case based architecture
    public enum IntakeState{
        STOPPED,
        INTAKING,
        REVERSED
    }

    public enum LinkageState {
        RAISED,
        LOWERED
    }

    //hardware names within controller application
    private static final String MOTOR_NAME = "intake";
    private static final String LEFT_SERVO_NAME = "intakeLeft";
    private static final String RIGHT_SERVO_NAME = "intakeRight";

    //Calibration Necessary?
    public static double RAISED_POSITION = 0.29;
    public static double LOWERED_POSITION = 0.5;
    public static double SERVO_OFFSET = 0.02;

    private final DcMotorEx intake;
    private final Servo left;
    private final Servo right;

    private IntakeState intakeState = IntakeState.STOPPED;
    private LinkageState linkageState = LinkageState.RAISED;

    private double intakePower = 1.0;
    private double reversePower = 1.0;

    private double commandedMotorPower;
    private double commandedServoPos;

    private double lastMotorPower = Double.NaN;
    private double lastServoPos = Double.NaN;

    //Constructor
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        left = hardwareMap.get(Servo.class, LEFT_SERVO_NAME);
        right = hardwareMap.get(Servo.class, RIGHT_SERVO_NAME);

        //Intake Motor Setup
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setPower(0);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //Linkage Servo Setup
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.REVERSE);

        lastMotorPower = 0;

        update();
    }

    //State setting and getting
    public void setIntakeState(IntakeState state) {
        if (state == null) {
            throw new IllegalArgumentException("INTAKE MOTOR STATE NULL");
        }

        intakeState = state;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public void setLinkageState(LinkageState state) {
        if (state == null) {
            throw new IllegalArgumentException("LINKAGE STATE NULL");
        }

        linkageState = state;
    }

    public LinkageState getLinkageState() {
        return linkageState;
    }

    //Commands
    public void raise() {
        setLinkageState(LinkageState.RAISED);
    }

    public void lower() {
        setLinkageState(LinkageState.LOWERED);
    }

    public void setIntakePower(double power) {
        intakePower = validatePower(power);
    }

    public void setReversePower(double power) {
        reversePower = validatePower(power);
    }

    //Case update upon request
    @Override
    public void update() {
        switch(intakeState) {
            case STOPPED:
                commandedMotorPower = 0;
                break;

            case INTAKING:
                commandedMotorPower = intakePower;
                break;

            case REVERSED:
                commandedMotorPower = reversePower;
                break;
        }

        switch (linkageState) {
            case RAISED:
                commandedServoPos = RAISED_POSITION;
                break;

            case LOWERED:
                commandedServoPos = LOWERED_POSITION;
                break;
        }
    }

    //Write data to hardware
    @Override
    public void write() {
        if (Double.compare(commandedMotorPower, lastMotorPower) != 0) {
            intake.setPower(commandedMotorPower);
            lastMotorPower = commandedMotorPower;
        }
        if (Double.compare(commandedServoPos, lastServoPos) != 0) {
            left.setPosition(commandedServoPos);
            right.setPosition(commandedServoPos + SERVO_OFFSET);

            lastServoPos = commandedServoPos;
        }
    }

    //Force stop
    @Override
    public void stop() {
        intakeState = IntakeState.STOPPED;
        commandedMotorPower = 0;

        intake.setPower(0);
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
