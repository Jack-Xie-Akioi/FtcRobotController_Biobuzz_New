package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.api.Paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Optimized Ball Collection")
public class OptimizedBallAuto extends OpMode {

    private Follower follower;
    private Path ballPath;

    private boolean pathStarted = false;

    public static double CURVE_STRENGTH = 0.25;

    public static double START_X = 0;
    public static double START_Y = 0;
    public static double START_HEADING = 0;

    private Pose[] getDetectedBallFieldPoses;


    @Override
    public void init() {

        telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

        follower = Constants.create(hardwareMap);

        follower.setPose(
                new Pose(START_X, START_Y, START_HEADING)
        );

        telemetry.addLine("Initialized");
        telemetry.update();

    }

    public OptimizedBallAuto(Pose[] DetectedBallFieldPoses) {
        getDetectedBallFieldPoses = DetectedBallFieldPoses;
    }

    @Override
    public void start() {
        pathStarted = false;
    }

    @Override
    public void loop() {

        follower.update();

        if (!pathStarted) {

            Pose[] balls = getDetectedBallFieldPoses;

            if (balls.length > 0) {

                ballPath = generateBallPath(balls);

                if (ballPath != null) {

                    follower.follow(ballPath);
                    pathStarted = true;
                }
            }
        }

        telemetry.addData("Robot X", follower.pose().x());
        telemetry.addData("Robot Y", follower.pose().y());
        telemetry.addData("Following Path", follower.isBusy());
        telemetry.update();
    }

    // ==========================================
    // DISTANCE
    // ==========================================

    private double distance(Pose a, Pose b) {

        return Math.hypot(
                a.x() - b.x(),
                a.y() - b.y()
        );
    }

    // ==========================================
    // NEAREST NEIGHBOR + 2-OPT
    // ==========================================

    private Pose[] optimizeRoute(Pose start, Pose[] balls) {

        Pose[] route = new Pose[balls.length + 1];
        boolean[] visited = new boolean[balls.length];

        route[0] = start;

        // Step 1: Nearest Neighbor
        for (int i = 1; i < route.length; i++) {

            int nearest = -1;
            double best = Double.POSITIVE_INFINITY;

            for (int j = 0; j < balls.length; j++) {

                if (!visited[j]) {

                    double d = distance(
                            route[i - 1], balls[j]
                    );

                    if (d < best) {
                        best = d;
                        nearest = j;
                    }
                }
            }

            visited[nearest] = true;
            route[i] = balls[nearest];
        }

        // Step 2: 2-opt optimization
        boolean improved = true;

        while (improved) {

            improved = false;

            for (int i = 1; i < route.length - 1; i++) {

                for (int j = i + 1; j < route.length; j++) {

                    double oldDistance = distance(
                            route[i - 1], route[i]
                    );

                    double newDistance = distance(
                            route[i - 1], route[j]
                    );

                    if (j < route.length - 1) {

                        oldDistance += distance(
                                route[j], route[j + 1]
                        );

                        newDistance += distance(
                                route[i], route[j + 1]
                        );
                    }

                    if (newDistance < oldDistance - 1e-9) {

                        int left = i;
                        int right = j;

                        while (left < right) {

                            Pose temp = route[left];
                            route[left] = route[right];
                            route[right] = temp;

                            left++;
                            right--;
                        }

                        improved = true;
                    }
                }
            }
        }

        return route;
    }

    // ==========================================
    // BEZIER PATH GENERATION
    // ==========================================

    private Path generateBallPath(Pose[] balls) {

        if (balls == null || balls.length == 0) {
            return null;
        }

        Pose[] route = optimizeRoute(
                follower.pose(),
                balls
        );

        List<Path> segments = new ArrayList<>();

        double heading = route[0].heading();

        for (int i = 0; i < route.length - 1; i++) {

            Pose a = route[i];
            Pose b = route[i + 1];

            if (distance(a, b) < 1e-6) {
                continue;
            }

            // Use a line if there is only one target
            if (route.length == 2) {

                segments.add(
                        Paths.line(a, b).constant(heading)
                );

                continue;
            }

            double startDX;
            double startDY;

            if (i == 0) {

                startDX = b.x() - a.x();
                startDY = b.y() - a.y();

            } else {

                Pose previous = route[i - 1];

                startDX = b.x() - previous.x();
                startDY = b.y() - previous.y();
            }

            double endDX;
            double endDY;

            if (i == route.length - 2) {

                endDX = b.x() - a.x();
                endDY = b.y() - a.y();

            } else {

                Pose next = route[i + 2];

                endDX = next.x() - a.x();
                endDY = next.y() - a.y();
            }

            double segmentLength = distance(a, b);

            double handle =
                    segmentLength * CURVE_STRENGTH;

            double startMagnitude = Math.hypot(
                    startDX, startDY
            );

            double endMagnitude = Math.hypot(
                    endDX, endDY
            );

            if (startMagnitude < 1e-6 ||
                    endMagnitude < 1e-6) {

                segments.add(
                        Paths.line(a, b).constant(heading)
                );

                continue;
            }

            // First control point
            Pose control1 = new Pose(
                    a.x() +
                            handle * startDX / startMagnitude,

                    a.y() +
                            handle * startDY / startMagnitude
            );

            // Second control point
            Pose control2 = new Pose(
                    b.x() -
                            handle * endDX / endMagnitude,

                    b.y() -
                            handle * endDY / endMagnitude
            );

            segments.add(
                    Paths.curve(
                            a,
                            control1,
                            control2,
                            b
                    ).constant(heading)
            );
        }

        if (segments.isEmpty()) {
            return null;
        }

        // Combine all segments into ONE Path object
        return Paths.path(
                segments.toArray(new Path[0])
        );
    }

    @Override
    public void stop() {
        follower.stop();
    }
}