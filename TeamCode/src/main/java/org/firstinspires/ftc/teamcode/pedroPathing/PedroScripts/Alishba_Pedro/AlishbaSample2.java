package org.firstinspires.ftc.teamcode.pedroPathing.PedroScripts.Alishba_Pedro;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class AlishbaSample2 {

        public static PathBuilder builder = new PathBuilder();

        public static PathChain line1 = builder
                .addPath(
                        new BezierLine(
                                new Point(9.000, 105.692, Point.CARTESIAN),
                                new Point(41.077, 118.615, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        public static PathChain line2 = builder
                .addPath(
                        new BezierLine(
                                new Point(41.077, 118.615, Point.CARTESIAN),
                                new Point(14.538, 130.615, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        public static PathChain line3 = builder
                .addPath(
                        new BezierLine(
                                new Point(14.538, 130.615, Point.CARTESIAN),
                                new Point(40.385, 131.769, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        public static PathChain line4 = builder
                .addPath(
                        new BezierLine(
                                new Point(40.385, 131.769, Point.CARTESIAN),
                                new Point(14.769, 130.385, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        public static PathChain line5 = builder
                .addPath(
                        new BezierLine(
                                new Point(14.769, 130.385, Point.CARTESIAN),
                                new Point(41.308, 140.538, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        public static PathChain line6 = builder
                .addPath(
                        new BezierLine(
                                new Point(41.308, 140.538, Point.CARTESIAN),
                                new Point(14.538, 130.385, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        public static PathChain line7 = builder
                .addPath(
                        new BezierLine(
                                new Point(14.538, 130.385, Point.CARTESIAN),
                                new Point(53.769, 97.385, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }
