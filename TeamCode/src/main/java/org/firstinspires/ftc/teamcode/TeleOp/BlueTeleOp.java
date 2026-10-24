package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@Config
@TeleOp(name = "Blue TeleOp", group = "Competition")
public final class BlueTeleOp extends LinearOpMode {

    public static double START_X = 0;
    public static double START_Y = 0;
    public static double START_HEADING_DEGREES = 0;

    public static double DRIVE_SCALE = 1.0;
    public static double TRIGGER_THRESHOLD = 0.1;

    private Robot robot;

    private boolean intaking;
    private boolean reversing;
    private boolean feeding;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        Follower follower = Constants.create(hardwareMap);

        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEGREES)));

        robot = new Robot(hardwareMap, telemetry, follower);

        try {
            robot.setAlliance(Robot.Alliance.BLUE);
            robot.setTarget(Robot.Target.BLUE_CLOSE);
            robot.setAutoAimEnabled(false);
            robot.setShootingEnabled(false);

            while (opModeInInit()) {
                robot.follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEGREES)));
                robot.follower.stop();

                robot.beginLoop();

                updateTargetControls();

                robot.update();

                if (robot.telemetryDue()) {
                    sendTelemetry();
                }

                idle();
            }

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            robot.setAutoAimEnabled(true);
            robot.setShootingEnabled(true);

            while (opModeIsActive()) {
                double forward = -gamepad1.left_stick_y * DRIVE_SCALE;
                double lateral = gamepad1.left_stick_x * DRIVE_SCALE;
                double turn = gamepad1.right_stick_x * DRIVE_SCALE;

                robot.follower.manual(forward, lateral, turn);

                robot.beginLoop();

                updateTargetControls();
                update();

                robot.update();

                if (robot.telemetryDue()) {
                    sendTelemetry();
                }
            }
        } finally {
            robot.stop();
        }
    }

    private void updateTargetControls() {
        if (gamepad1.cross) {
            robot.setTarget(Robot.Target.BLUE_CLOSE);
        } else if (gamepad1.circle) {
            robot.setTarget(Robot.Target.BLUE_FAR);
        }
    }

    private void update() {
        intaking = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        reversing = !intaking && gamepad1.left_trigger > TRIGGER_THRESHOLD;
        feeding = !intaking && !reversing && gamepad1.right_bumper;

        robot.setStopperOpenOverride(gamepad1.square);

        if (intaking) {
            robot.intake.lower();
            robot.startIntaking();
        } else if (reversing) {
            robot.intake.raise();
            robot.reverseIntake();
        } else {
            robot.intake.raise();
            robot.stopIntaking();
        }

        if (reversing) {
            robot.pantograph.raise();
        } else if (gamepad1.dpad_up) {
            robot.pantograph.raise();
        } else if (gamepad1.dpad_down) {
            robot.pantograph.lower();
        }

        robot.transfer.setTransferState(feeding ? Transfer.TransferState.RUNNING : Transfer.TransferState.STOPPED);
    }

    private void sendTelemetry() {
        Pose pose = robot.follower.pose();

        telemetry.addData("Alliance", robot.getAlliance());
        telemetry.addData("Target", robot.getTarget());

        telemetry.addData("Robot X", "%.2f", pose.x());
        telemetry.addData("Robot Y", "%.2f", pose.y());
        telemetry.addData("Heading deg", "%.1f", Math.toDegrees(pose.heading()));
        telemetry.addData("Target distance", "%.2f", robot.getDistanceToTarget());

        telemetry.addData("Intaking requested", intaking);
        telemetry.addData("Reversing requested", reversing);
        telemetry.addData("Feeding requested", feeding);
        telemetry.addData("Intake sequence", robot.getIntakeSequence());
        telemetry.addData("Intake linkage", robot.intake.getLinkageState());

        telemetry.addData("Stopper override", robot.isStopperOpenOverride());
        telemetry.addData("Stopper state", robot.transfer.getStopperState());
        telemetry.addData("Transfer amps", "%.2f", robot.transfer.getCurrentAmps());
        telemetry.addData("Transfer cutoff", robot.transfer.isCurrentCutoffLatched());

        telemetry.addData("LUT shot available", robot.isShotAvailable());
        telemetry.addData("Target ticks/sec", "%.1f", robot.shooter.getTargetVel());
        telemetry.addData("Measured ticks/sec", "%.1f", robot.shooter.getVel());
        telemetry.addData("Velocity error", "%.1f", robot.shooter.getVelError());
        telemetry.addData("Hood position", "%.3f", robot.shooter.getHoodPos());
        telemetry.addData("Ready to shoot", robot.isReadyToShoot());

        telemetry.addData("Turret requested deg", "%.1f", robot.turret.getRequestedAngleDegrees());
        telemetry.addData("Turret commanded deg", "%.1f", robot.turret.getCommandedAngleDegrees());
        telemetry.addData("Turret can aim", robot.turret.canAim());
        telemetry.addData("Waiting for wrap", robot.isWrapWaiting());

        telemetry.addData("Pantograph", robot.pantograph.getPantographState());
        telemetry.addData("Loop ms", "%.2f", robot.getLoopPeriodMs());

        telemetry.update();
    }
}