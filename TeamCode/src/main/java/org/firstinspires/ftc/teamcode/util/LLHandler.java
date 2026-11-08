
package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;

public class LLHandler {

    private Limelight3A limelight;
    private String handler;
    private LLResult result;

    private double[] latestResult;

    // BLUE = 0, RED = 1
    private int alliancePipeline;

    // Python pipeline number
    private static final int BALL_PIPELINE = 6;

    // Limelight 3A camera specifications
    private static final double HFOV = 54.5;
    private static final double VFOV = 42.0;

    // Match Python pipeline resolution
    private static final double WIDTH = 640.0;
    private static final double HEIGHT = 480.0;

    // REPLACE WITH YOUR ACTUAL MEASUREMENTS
    // All distances in inches
    private static final double CAMERA_HEIGHT = 12.0;
    private static final double CAMERA_PITCH = 30.0;
    private static final double CAMERA_FORWARD = 0.0;
    private static final double CAMERA_LEFT = 0.0;

    private static final double BALL_RADIUS = 1.4;

    public LLHandler(HardwareMap map, int alliance) {
        limelight = map.get(Limelight3A.class, "ll");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(alliance);

        alliancePipeline = alliance;
        handler = "INIT";
    }

    public void start() {
        limelight.start();
        poll();
        handler = "ON";
    }

    // EXISTING APRILTAG DETECTION
    public double[] poll() {
        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            List<FiducialResult> tags =
                    result.getFiducialResults();

            if (tags != null && !tags.isEmpty()) {

                FiducialResult tag = tags.get(0);

                Pose3D goal =
                        tag.getTargetPoseCameraSpace();

                if (goal != null) {
                    double x = goal.getPosition().x;
                    double y = -goal.getPosition().y;
                    double z = goal.getPosition().z;
                    double tx = -result.getTx();

                    latestResult = new double[]{
                            x, y, z, tx
                    };

                    return latestResult;
                }
            }
        }

        latestResult = new double[]{
                -1001, -1001, -1001, -1001
        };

        return latestResult;
    }

    // SWITCH TO YELLOW BALL DETECTION
    public void useBallPipeline() {
        limelight.pipelineSwitch(BALL_PIPELINE);
    }

    // SWITCH BACK TO APRILTAG DETECTION
    public void useAprilTagPipeline() {
        limelight.pipelineSwitch(alliancePipeline);
    }

    // GET RAW BALL PIXEL COORDINATES
    // Returns [[x1,y1], [x2,y2], ...]
    public List<double[]> getPollenPixels() {

        List<double[]> balls = new ArrayList<>();

        LLResult ballResult = limelight.getLatestResult();

        if (ballResult == null ||
                !ballResult.isValid()) {
            return balls;
        }

        if (ballResult.getPipelineIndex() != BALL_PIPELINE) {
            return balls;
        }

        double[] data = ballResult.getPythonOutput();

        if (data == null || data.length == 0) {
            return balls;
        }

        int count = Math.max(0, (int) data[0]);

        count = Math.min(
                count,
                (data.length - 1) / 2
        );

        for (int i = 0; i < count; i++) {

            double x = data[1 + 2 * i];
            double y = data[2 + 2 * i];

            balls.add(new double[]{x, y});
        }

        return balls;
    }

    // CONVERT ALL BALLS TO FIELD COORDINATES
    // Returns [[fieldX1,fieldY1], ...]
    public List<double[]> getPollenFieldPositions(
            Pose robotPose) {

        List<double[]> fieldBalls = new ArrayList<>();

        List<double[]> pixelBalls =
                getPollenPixels();

        // Camera focal lengths in pixels
        double fx = WIDTH /
                (2 * Math.tan(
                        Math.toRadians(HFOV / 2)
                ));

        double fy = HEIGHT /
                (2 * Math.tan(
                        Math.toRadians(VFOV / 2)
                ));

        double cx = WIDTH / 2.0;
        double cy = HEIGHT / 2.0;

        double pitch = Math.toRadians(CAMERA_PITCH);
        double heading = robotPose.heading();

        double heightDifference =
                CAMERA_HEIGHT - BALL_RADIUS;

        if (heightDifference <= 0) {
            return fieldBalls;
        }

        for (double[] ball : pixelBalls) {

            double pixelX = ball[0];
            double pixelY = ball[1];

            // Pixel coordinates -> angular offsets
            double tx = Math.atan2(pixelX - cx, fx);
            double ty = Math.atan2(cy - pixelY, fy);

            // Camera ray direction
            double u = Math.tan(tx);
            double v = Math.tan(ty);

            // Account for downward camera pitch
            double forwardRay =
                    Math.cos(pitch)
                            + v * Math.sin(pitch);

            double downwardRay =
                    Math.sin(pitch)
                            - v * Math.cos(pitch);

            // Reject rays pointing above the ball plane
            if (downwardRay <= 0 || forwardRay <= 0) {
                continue;
            }

            // Intersect camera ray with ball-center plane
            double scale =
                    heightDifference / downwardRay;

            double cameraForward =
                    scale * forwardRay;

            double cameraRight =
                    scale * u;

            // Camera coordinates -> robot coordinates
            double robotForward =
                    cameraForward + CAMERA_FORWARD;

            double robotLeft =
                    -cameraRight + CAMERA_LEFT;

            // Robot coordinates -> field coordinates
            double fieldX = robotPose.x()
                    + robotForward * Math.cos(heading)
                    - robotLeft * Math.sin(heading);

            double fieldY = robotPose.y()
                    + robotForward * Math.sin(heading)
                    + robotLeft * Math.cos(heading);

            fieldBalls.add(new double[]{
                    fieldX,
                    fieldY
            });
        }

        return fieldBalls;
    }

    public void alliance(int a) {
        alliancePipeline = a;
        limelight.pipelineSwitch(a);
    }

    public void stop() {
        limelight.stop();
        handler = "OFF";
    }

    public double[] getLatestResult() {
        return latestResult;
    }

    @NonNull
    public String toString() {
        return handler;
    }

    public LLResult getResult() {
        return result;
    }
}