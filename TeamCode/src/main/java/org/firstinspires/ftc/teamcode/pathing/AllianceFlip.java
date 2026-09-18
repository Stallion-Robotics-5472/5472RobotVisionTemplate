/*
 * Mirrors a plan to the other alliance's side of the field so one authored auto
 * can run as both. Write the path once for your "home" alliance, then flip it
 * at run time based on the alliance the driver selected.
 *
 *   PathChain redPlan  = buildPlan();                 // authored for red
 *   PathChain toRun    = AllianceFlip.forAlliance(redPlan, Alliance.RED, selected);
 *   Pose2d    startNow = AllianceFlip.forAlliance(redStart, Alliance.RED, selected);
 *
 * The transform used is {@link FieldSymmetry#ROTATIONAL} by default, which is
 * the correct one for the 2026-2027 BIOBUZZ field (see FieldSymmetry for the
 * coordinate check). Pass an explicit symmetry if your field differs.
 *
 * Heading handling: TANGENT paths need no heading data at all, because the
 * tangent is recomputed from the transformed control points. CONSTANT and
 * LINEAR headings are transformed explicitly. Curvature (and therefore the
 * follower's centripetal correction) is likewise derived from the transformed
 * points, so it stays consistent under a handedness-reversing mirror.
 *
 * Original implementation for this template.
 */
package org.firstinspires.ftc.teamcode.pathing;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;

public final class AllianceFlip {
    private AllianceFlip() {}

    /** Symmetry of the current season's field. */
    public static final FieldSymmetry DEFAULT_SYMMETRY = FieldSymmetry.ROTATIONAL;

    // ----- conditional helpers (the usual entry points) -----

    /**
     * Returns {@code plan} unchanged when {@code runningAs} matches
     * {@code authoredFor}, otherwise the flipped plan.
     */
    public static PathChain forAlliance(PathChain plan, Alliance authoredFor, Alliance runningAs) {
        return runningAs.needsFlipFrom(authoredFor) ? flip(plan) : plan;
    }

    public static Path forAlliance(Path path, Alliance authoredFor, Alliance runningAs) {
        return runningAs.needsFlipFrom(authoredFor) ? flip(path) : path;
    }

    public static Pose2d forAlliance(Pose2d pose, Alliance authoredFor, Alliance runningAs) {
        return runningAs.needsFlipFrom(authoredFor) ? flip(pose) : pose;
    }

    public static Translation2d forAlliance(Translation2d point, Alliance authoredFor,
                                            Alliance runningAs) {
        return runningAs.needsFlipFrom(authoredFor) ? flip(point) : point;
    }

    // ----- unconditional flips, default symmetry -----

    public static Translation2d flip(Translation2d p) {
        return flip(p, DEFAULT_SYMMETRY);
    }

    public static Pose2d flip(Pose2d p) {
        return flip(p, DEFAULT_SYMMETRY);
    }

    public static BezierCurve flip(BezierCurve c) {
        return flip(c, DEFAULT_SYMMETRY);
    }

    public static Path flip(Path path) {
        return flip(path, DEFAULT_SYMMETRY);
    }

    public static PathChain flip(PathChain chain) {
        return flip(chain, DEFAULT_SYMMETRY);
    }

    // ----- unconditional flips, explicit symmetry -----

    public static Translation2d flip(Translation2d p, FieldSymmetry sym) {
        return new Translation2d(sym.x(p.getX(), p.getY()), sym.y(p.getX(), p.getY()));
    }

    public static double flipHeading(double radians, FieldSymmetry sym) {
        return sym.heading(radians);
    }

    public static Pose2d flip(Pose2d p, FieldSymmetry sym) {
        return new Pose2d(flip(p.getTranslation(), sym),
                new Rotation2d(sym.heading(p.getHeading())));
    }

    public static BezierCurve flip(BezierCurve c, FieldSymmetry sym) {
        Translation2d[] points = new Translation2d[c.getDegree() + 1];
        for (int i = 0; i < points.length; i++) {
            points[i] = flip(c.getControlPoint(i), sym);
        }
        return new BezierCurve(points);
    }

    /** Flips the curve and transforms the heading rule to match. */
    public static Path flip(Path path, FieldSymmetry sym) {
        Path flipped = new Path(flip(path.getCurve(), sym));
        flipped.setSearchSteps(path.getSearchSteps());

        switch (path.getHeadingMode()) {
            case CONSTANT:
                flipped.setConstantHeading(sym.heading(path.getConstantHeading()));
                break;
            case LINEAR:
                flipped.setLinearHeading(sym.heading(path.getStartHeading()),
                        sym.heading(path.getEndHeading()));
                break;
            case TANGENT:
            default:
                // The tangent follows the transformed control points, so only
                // the forward/reverse choice needs carrying over.
                if (path.isTangentReversed()) {
                    flipped.setReverseTangentHeading();
                } else {
                    flipped.setTangentHeading();
                }
                break;
        }
        return flipped;
    }

    public static PathChain flip(PathChain chain, FieldSymmetry sym) {
        Path[] flipped = new Path[chain.size()];
        for (int i = 0; i < flipped.length; i++) {
            flipped[i] = flip(chain.get(i), sym);
        }
        return new PathChain(flipped);
    }
}
