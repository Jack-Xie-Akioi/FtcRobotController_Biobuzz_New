package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Constants;

public final class Turret extends Subsystem {

    public enum TurretState {
        PARKED,
        TRACKING,
        MANUAL
    }

    private static final String LEFT_SERVO_NAME = "tl";
    private static final String RIGHT_SERVO_NAME = "tr";

    public static double SERVO_TRAVEL_DEGREES = 360;
    private static final double GEAR_RATIO = (80.0/20.0)*(60.0/180.0);

    private final Servo left;
    private final Servo right;

    private TurretState turretState = TurretState.PARKED;

    private double robotX;
    private double robotY;
    private double robotHeadingDegrees;

    private double targetX;
    private double targetY;

    private boolean poseAvailable;
    private boolean targetAvailable;

    private double manualAngleDegrees;
    private double requestedAngleDegrees;
    private double commandedAngleDegrees;

    private double minimumAngleDegrees;
    private double maximumAngleDegrees;

    private double commandedLeftPosition;
    private double commandedRightPosition;

    private double lastLeftPosition = Double.NaN;
    private double lastRightPosition = Double.NaN;
    private double lastWrittenAngleDegrees = Double.NaN;

    private double lastMoveDegrees;
    private boolean wrapRequested;
    private boolean aimReachable;

    public Turret(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, LEFT_SERVO_NAME);
        right = hardwareMap.get(Servo.class, RIGHT_SERVO_NAME);

        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.REVERSE);

        update();
    }

    public void setTurretState(TurretState state) {
        if (state == null) {
            throw new IllegalArgumentException("TURRET STATE NULL");
        }

        turretState = state;
    }

    public TurretState getTurretState() {
        return turretState;
    }

    public void setRobotPose(double x, double y, double heading) {
        requireFinite(x);
        requireFinite(y);
        requireFinite(heading);

        robotX = x;
        robotY = y;
        robotHeadingDegrees = heading;

        poseAvailable = true;
    }

    public void setTarget(double x, double y) {
        requireFinite(x);
        requireFinite(y);

        targetX = x;
        targetY = y;

        targetAvailable = true;
        turretState = TurretState.TRACKING;
    }

    public void setManualAngleDegrees(double angle) {
        requireFinite(angle);

        if (angle < -180 || angle > 180) {
            throw new IllegalArgumentException("ANGLE MUST BE BETWEEN -180 AND 180");
        }

        manualAngleDegrees = angle;
        turretState = TurretState.MANUAL;
    }

    public void park() {
        turretState = TurretState.PARKED;
    }

    public double getPivotX() {
        double heading = Math.toRadians(robotHeadingDegrees);

        return robotX - Constants.TURRET_Y_OFFSET * Math.cos(heading);
    }

    public double getPivotY() {
        double heading = Math.toRadians(robotHeadingDegrees);

        return robotY - Constants.TURRET_Y_OFFSET * Math.sin(heading);
    }

    public double getDistanceToTarget() {
        if (!poseAvailable || !targetAvailable) {
            return Double.NaN;
        }

        return Math.hypot(targetX - getPivotX(), targetY - getPivotY());
    }

    public double getRequestedAngleDegrees() {
        return requestedAngleDegrees;
    }

    public double getCommandedAngleDegrees() {
        return commandedAngleDegrees;
    }

    public double getMinimumAngleDegrees() {
        return minimumAngleDegrees;
    }

    public double getMaximumAngleDegrees() {
        return maximumAngleDegrees;
    }

    public double getLastMoveDegrees() {
        return lastMoveDegrees;
    }

    public boolean canAim() {
        return aimReachable;
    }

    public boolean isWrapRequested() {
        return wrapRequested;
    }

    @Override
    public void update() {
        updateAngleLimits();

        boolean validRequest = true;

        switch (turretState) {
            case PARKED:
                requestedAngleDegrees = 0;
                break;

            case MANUAL:
                requestedAngleDegrees = manualAngleDegrees;
                break;

            case TRACKING:
                if (!poseAvailable || !targetAvailable) {
                    validRequest = false;
                    break;
                }

                double dx = targetX - getPivotX();
                double dy = targetY - getPivotY();

                if (Math.hypot(dx, dy) < 0.00001) {
                    validRequest = false;
                    break;
                }

                double fieldAngleDegrees = Math.toRadians(Math.atan2(dy, dx));

                requestedAngleDegrees = normalizeDegrees(fieldAngleDegrees - robotHeadingDegrees - 180);
                break;
        }

        if (validRequest) {
            selectBoundaryEquivalent();

            aimReachable = requestedAngleDegrees >= minimumAngleDegrees && requestedAngleDegrees <= maximumAngleDegrees;

            commandedAngleDegrees = clamp(requestedAngleDegrees, minimumAngleDegrees, maximumAngleDegrees);
        }

        else {
            aimReachable = false;

            commandedAngleDegrees = clamp(commandedAngleDegrees, minimumAngleDegrees, maximumAngleDegrees);
        }

        double turretTravelDegrees = SERVO_TRAVEL_DEGREES * GEAR_RATIO;

        commandedLeftPosition = clamp(Constants.LEFT_CENTER_POSITION + commandedAngleDegrees / turretTravelDegrees, Constants.LEFT_MIN_POSITION, Constants.LEFT_MAX_POSITION);
        commandedRightPosition = clamp(Constants.RIGHT_CENTER_POSITION + commandedAngleDegrees / turretTravelDegrees, Constants.RIGHT_MIN_POSITION, Constants.RIGHT_MAX_POSITION);
    }

    private void updateAngleLimits() {
        requireFinite(SERVO_TRAVEL_DEGREES);
        requireFinite(Constants.TURRET_Y_OFFSET);

        if (SERVO_TRAVEL_DEGREES <= 0) {
            throw new IllegalArgumentException("SERVO RANGE MUST BE POSITIVE");
        }

        validateServoCalibration(Constants.LEFT_CENTER_POSITION, Constants.LEFT_MIN_POSITION, Constants.LEFT_MAX_POSITION);
        validateServoCalibration(Constants.RIGHT_CENTER_POSITION, Constants.RIGHT_MIN_POSITION, Constants.RIGHT_MAX_POSITION);

        double travel = SERVO_TRAVEL_DEGREES * GEAR_RATIO;

        double leftMinimum = (Constants.LEFT_MIN_POSITION - Constants.LEFT_CENTER_POSITION) * travel;

        double leftMaximum = (Constants.LEFT_MAX_POSITION - Constants.LEFT_CENTER_POSITION) * travel;

        double rightMinimum = (Constants.RIGHT_MIN_POSITION - Constants.RIGHT_CENTER_POSITION) * travel;

        double rightMaximum = (Constants.RIGHT_MAX_POSITION - Constants.RIGHT_CENTER_POSITION) * travel;

        minimumAngleDegrees = Math.max(-180, Math.max(leftMinimum, rightMinimum));

        maximumAngleDegrees = Math.min(180, Math.min(leftMaximum, rightMaximum));
    }

    private void selectBoundaryEquivalent() {
        if (Math.abs(Math.abs(requestedAngleDegrees) - 180) > 0.000001) {
            return;
        }

        boolean positiveAllowed = maximumAngleDegrees >= 180;
        boolean negativeAllowed = minimumAngleDegrees <= -180;

        if (positiveAllowed && negativeAllowed) {
            double reference = Double.isNaN(lastWrittenAngleDegrees) ? commandedAngleDegrees : lastWrittenAngleDegrees;
        } else if (positiveAllowed) {
            requestedAngleDegrees = 180;
        } else if (negativeAllowed) {
            requestedAngleDegrees = -180;
        }
    }

    @Override
    public void write() {
        lastMoveDegrees = Double.isNaN(lastWrittenAngleDegrees) ? 0 : commandedAngleDegrees - lastWrittenAngleDegrees;

        wrapRequested = Math.abs(lastMoveDegrees) > 180;

        if (Double.compare(commandedLeftPosition, lastLeftPosition) != 0) {
            left.setPosition(commandedLeftPosition);
            lastLeftPosition = commandedLeftPosition;
        }

        if (Double.compare(commandedRightPosition, lastRightPosition) != 0) {
            right.setPosition(commandedRightPosition);
            lastRightPosition = commandedRightPosition;
        }

        lastWrittenAngleDegrees = commandedAngleDegrees;
    }

    @Override
    public void stop() {
        turretState = TurretState.MANUAL;
        manualAngleDegrees = commandedAngleDegrees;
        wrapRequested = false;
    }

    public static void validateServoCalibration(double center, double minimum, double maximum) {
        requireFinite(center);
        requireFinite(minimum);
        requireFinite(maximum);

        if (minimum < 0 || maximum > 1 || center < minimum || center > maximum) {
            throw new IllegalArgumentException("INVALID TURRET SERVO CALIBRATION");
        }
    }

    public static double normalizeDegrees(double degrees) {
        double result = degrees % 360;

        if (result >= 180) {
            result -= 360;
        }

        if (result <= -180) {
            result += 360;
        }

        return result;
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    private static void requireFinite(double value) {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            throw new IllegalArgumentException("VALUE MUST BE FINITE");
        }
    }
}
