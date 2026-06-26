package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PathUtil {

    public static List<Pose2d> generateSemiCircle(double x, double y, int steps, double radius) {

        List<Pose2d> path = new ArrayList<>();

        double xc = x - radius;
        double yc = y;
        double r = radius;

        for (int i = 0; i <= steps; i++) {

            double t = Math.PI * i / steps;

            double px = xc + r * Math.cos(t);
            double py = yc + r * Math.sin(t);

            double dx = -r * Math.sin(t);
            double dy =  r * Math.cos(t);

            Rotation2d heading = new Rotation2d(Math.atan2(dy, dx));

            path.add(new Pose2d(px, py, heading));
        }

        return path;
    }
}

