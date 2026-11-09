package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static com.pedropathing.api.Paths.*;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@Autonomous(name = "Red 2.5 Tip Solo")
public class Red_V1_Solo extends OpMode {

    private Follower follower;
    private int pathState = 0;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose scorePose1 = p.of(34.86963090246775, 8.52971223650836, 0);
    private final Pose pickupPose1 = p.of(9.484975902772927, 8.52971223650836, 0);
    private final Pose scorePose2 = p.of(30.722072072072073, 119.62167792792793, 0);
    private final Pose pickupPose2 = p.of(46.927184466019426, 132.3349514563107, 0);
    private final Pose pickupPose3 = p.of(8.907766990291266, 47.450485436893196, 0);
    private final Pose park = p.of(10.166019417475725, 97.04083737864076, 0);
    private final Pose controlPoint1 = p.of(24.348198198198197, 34.46672297297298, 0);
    private final Pose controlPoint2 = p.of(46.5242718446602, 118.12427184466023, 0);
    private final Pose controlPoint3 = p.of(22.172063325461377, 50.36369281903263, 0);

    private Path pollenPickup() {
        return line(scorePose1, pickupPose1).linear(scorePose1, pickupPose1);
    }

    private Path shoot1() {
        return curve(pickupPose1, controlPoint1, scorePose2).linear(pickupPose1
                , scorePose2);
    }

    private Path flower1() {
        return curve(scorePose2, controlPoint2, pickupPose2).linear(scorePose2
                , pickupPose2);
    }

    private Path tip2() {
        return curve(pickupPose2, controlPoint2, scorePose2).linear(pickupPose2
                , scorePose2);
    }

    private Path shoot2() {
        return line(scorePose2, scorePose1).linear(scorePose2, scorePose1);
    }

    private Path flower2() {
        return curve(scorePose1, controlPoint3, pickupPose3).linear(scorePose1
                , pickupPose3);
    }

    private Path tip3() {
        return curve(pickupPose3, controlPoint3, scorePose1).linear(pickupPose3
                , scorePose1);
    }

    private Path park() {
        return line(scorePose1, park).linear(scorePose1, park);
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                follower.follow(pollenPickup());
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.follow(shoot1());
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.follow(flower1());
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.follow(tip2());
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.follow(shoot2());
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.follow(flower2());
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.follow(tip3());
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.follow(park());
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
            case -1:
                requestOpModeStop();
                break;
        }
    }

    private void setPathState(int state) {
        pathState = state;
        telemetry.addData("Path State", pathState);
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(scorePose1);
        follower.update();
    }

    public void loop() {
        follower.update();
        autoPathUpdate();
    }

}
