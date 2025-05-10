package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class AlishbaPedroParking {

        public static PathBuilder builder = new PathBuilder();

        public static PathChain line1 = builder
                .addPath(
                        new BezierLine(
                                new Point(8.000, 80.000, Point.CARTESIAN),
                                new Point(6.231, 7.385, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
