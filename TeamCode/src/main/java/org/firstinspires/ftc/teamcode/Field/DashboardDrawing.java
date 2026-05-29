package org.firstinspires.ftc.teamcode.Field;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.PoseHistory;

public class DashboardDrawing {
    public static final double ROBOT_RADIUS = 3;

    public static void drawDebug(Canvas c, Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(c, follower.getCurrentPath());

            double t = follower.getCurrentPath().getClosestPointTValue();
            Pose closest = follower.getPointFromPath(t);

            drawRobot(
                    c,
                    new Pose(
                            closest.getX(),
                            closest.getY(),
                            follower.getCurrentPath().getHeadingGoal(t)
                    )
            );
        }

        drawPoseHistory(c, follower.getPoseHistory());
        drawRobot(c, follower.getPose());
    }

    private static void drawPath(Canvas c, com.pedropathing.paths.Path currentPath) {
        double[][] points = currentPath.getPanelsDrawingPoints();

        for (int i = 0; i < points.length - 1; i++) {
            c.strokeLine(
                    points[i][0], points[i][1],
                    points[i + 1][0], points[i + 1][1]
            );
        }
    }

    public static void drawRobot(Canvas c, Pose pose) {
        if (pose == null) return;

        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        c.strokeCircle(x, y, ROBOT_RADIUS);

        double endX = x + Math.cos(heading) * ROBOT_RADIUS;
        double endY = y + Math.sin(heading) * ROBOT_RADIUS;

        c.strokeLine(x, y, endX, endY);
    }


    public static void drawPoseHistory(Canvas c, PoseHistory history) {
        double[] xs = history.getXPositionsArray();
        double[] ys = history.getYPositionsArray();

        for (int i = 0; i < xs.length - 1; i++) {
            c.strokeLine(xs[i], ys[i], xs[i + 1], ys[i + 1]);
        }
    }
}