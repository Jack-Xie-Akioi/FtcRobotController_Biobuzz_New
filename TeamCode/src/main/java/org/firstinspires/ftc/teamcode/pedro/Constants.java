package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(new PinpointLocalizer (h,localizerConfig),new Mecanum(h, drivetrainConfig),new Foresight(foresightConfig));
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FrontLeft");
        c.frontRightName.set("FrontRight");
        c.backLeftName.set("BackLeft");
        c.backRightName.set("BackRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("Pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-2.46063);
        c.yPodOffset.set(-4.2440945);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2255048061923159);
                Controller secondaryTranslationalForward = Controller.proportional(0.0833180002886642);
                Controller primaryTranslationalLateral = Controller.proportional(0.28851467772240835);
                Controller secondaryTranslationalLateral = Controller.proportional(0.10659846416425778);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012028297695017077));
                c.brake.set(Controller.proportionalFeedforward(0.010224053040764515));

                c.headingFeedback.set(Controller.proportional(3.422132995004395));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04106447456261969, 0.007014728466284466));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06727050680525551, 0.05216835459047515));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001651763067933549, 0.0017586054910170715));

                c.maxAchievableForwardVelocity.set(74.5656107165439);
                c.maxAchievableStrafeVelocity.set(59.406435678309435);
                c.naturalForwardDeceleration.set(67.54812251721225);
                c.naturalStrafeDeceleration.set(96.79939214668141);
            }
    );
}