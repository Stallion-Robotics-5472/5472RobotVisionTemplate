package qa;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.BasicMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.lib.util.RobotClock;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.pathing.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.pathing.PathConstants;
import org.firstinspires.ftc.teamcode.shooting.Goal;
import org.firstinspires.ftc.teamcode.shooting.GoalSelector;
import org.firstinspires.ftc.teamcode.shooting.ShootingConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Localization;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;

import java.util.ArrayList;
import java.util.List;

/**
 * A simulated robot on a simulated field, driven by the REAL subsystems.
 *
 * The point is to catch the bugs unit tests structurally cannot: the ones that
 * only appear once localization, the drivetrain, aiming, the shooter and the
 * command scheduler are all running against each other over a whole match, with
 * odometry drifting, vision coming and going, and an alliance in play.
 *
 * What is simulated:
 *   - Chassis physics. Motor powers are read back off the stub motors, run
 *     through mecanum forward kinematics, and integrated. So the drivetrain's
 *     own mixing and direction constants are in the loop, not bypassed.
 *   - Odometry drift. The Pinpoint reports the true pose plus an accumulating
 *     error, the way a real one does.
 *   - A camera with a field of view. Tags are only seen when the goal is in
 *     front of the robot and close enough, and botpose only arrives then.
 *   - Time, in realistic 20 ms slices via RobotClock, so every controller sees
 *     the dt it will see on the field.
 *
 * Deliberately NOT simulated: motor dynamics, wheel slip, vision noise
 * distributions, game piece flight. This is an integration harness for control
 * and coordination logic, not a physics engine.
 */
public class MatchSim {

    // ---- field / robot constants for the simulation ----
    static final double DT = 0.02;                       // 50 Hz, a realistic loop
    static final double MAX_LINEAR_IN_S = 55.0;          // in/sec at full power
    static final double MAX_OMEGA_RAD_S = 6.0;           // rad/sec at full turn power
    static final double CAMERA_FOV_DEG = 27.0;           // half-angle, Limelight-ish
    static final double CAMERA_RANGE_IN = 110.0;
    static final double ODOMETRY_DRIFT_IN_PER_S = 0.35;  // slow, realistic creep
    static final double ROBOT_HALF_WIDTH_IN = 9.0;       // an 18" robot

    // ---- the simulated world ----
    Pose2d truePose;
    Translation2d trueVelocity = new Translation2d();
    double trueOmega = 0.0;
    double driftX = 0.0;
    double driftY = 0.0;
    double simTime = 0.0;
    int wallContacts = 0;

    // How the odometry frame relates to the true field frame. Null until seeded.
    Pose2d seedPose = null;
    Pose2d truthAtSeed = null;

    /** When false the camera reports nothing, simulating a vision dropout. */
    boolean cameraWorking = true;

    // ---- hardware stubs ----
    final HardwareMap map = new HardwareMap();
    final GoBildaPinpointDriver pinpoint = new GoBildaPinpointDriver();
    final Limelight3A camera = new Limelight3A();
    final BasicMotor fl = new BasicMotor();
    final BasicMotor fr = new BasicMotor();
    final BasicMotor bl = new BasicMotor();
    final BasicMotor br = new BasicMotor();
    final DcMotorEx flywheel = new DcMotorEx();
    final BasicMotor feeder = new BasicMotor();
    final Servo hood = new Servo();

    // ---- the real subsystems under test ----
    final Localization localization;
    final DriveSubsystem drive;
    final ShooterSubsystem shooter;
    final GoalSelector goalSelector;

    /** Goals for this simulated field, one per alliance side. */
    final List<Goal> goals;

    MatchSim(Pose2d start, List<Goal> goals) {
        this.goals = goals;
        this.truePose = start;

        map.put(VisionConstants.PINPOINT_NAME, pinpoint);
        map.put(VisionConstants.LIMELIGHT_NAME, camera);
        map.put(PathConstants.FRONT_LEFT_MOTOR, fl);
        map.put(PathConstants.FRONT_RIGHT_MOTOR, fr);
        map.put(PathConstants.BACK_LEFT_MOTOR, bl);
        map.put(PathConstants.BACK_RIGHT_MOTOR, br);
        map.put(ShootingConstants.FLYWHEEL_MOTOR, flywheel);
        map.put(ShootingConstants.FEEDER_MOTOR, feeder);
        map.put(ShootingConstants.HOOD_SERVO, hood);

        RobotClock.setSource(() -> simTime);
        pushOdometry();

        localization = new Localization(map, true);
        drive = new DriveSubsystem(new MecanumDrivetrain(map), localization);
        shooter = new ShooterSubsystem(map);
        goalSelector = new GoalSelector(ShootingConstants.GOAL_STRATEGY,
                goals.toArray(new Goal[0]))
                .withTagMeaning(ShootingConstants.GOAL_TAG_MEANING)
                .withSwitchFrames(ShootingConstants.GOAL_SWITCH_FRAMES);
    }

    /**
     * Tells the Pinpoint stub what it "measures".
     *
     * Modelled the way a real one works: seeding it does not teach it the truth,
     * it just redefines its origin. So if the seed was wrong, every later reading
     * is wrong by that same rigid transform -- which is exactly what makes a
     * backwards seed so quiet, and what this sim needs to reproduce for the
     * mis-seeded scenario to mean anything.
     */
    void pushOdometry() {
        Pose2d reported = (seedPose == null || truthAtSeed == null)
                ? truePose
                // Compose the seed with however far the robot has really moved
                // since it was seeded.
                : seedPose.plus(truePose.minus(truthAtSeed));
        pinpoint.x = reported.getX() + driftX;
        pinpoint.y = reported.getY() + driftY;
        pinpoint.headingRad = reported.getHeading();
    }

    /**
     * Seeds the pose estimate, as an OpMode does at init. Pass something other
     * than the truth to simulate a robot placed backwards or the wrong alliance
     * selected.
     */
    void seedOdometry(Pose2d told) {
        seedPose = told;
        truthAtSeed = truePose;
        localization.setStartingPose(told);
        pushOdometry();
    }

    /**
     * Advances the physical simulation one step: read what the drivetrain
     * actually commanded, work out what the chassis does, and move the robot.
     */
    /**
     * Physical wheel surface speed for one motor.
     *
     * A motor's Direction setting exists precisely to cancel the physical
     * mirroring of the two sides, so on a correctly configured mecanum a positive
     * command to all four drives the robot forward. That means the surface speed
     * is simply the commanded power when the motor matches the standard
     * configuration, and negated when it does not -- which is how DriveTest models
     * it too.
     *
     * Modelling the mirroring separately AND undoing the Direction double-counts
     * it and silently inverts the whole drivetrain.
     */
    static double surfaceSpeed(BasicMotor motor, DcMotorSimple.Direction standard) {
        return motor.power * (motor.direction == standard ? 1.0 : -1.0);
    }

    void stepPhysics() {
        double flS = surfaceSpeed(fl, DcMotorSimple.Direction.REVERSE);
        double blS = surfaceSpeed(bl, DcMotorSimple.Direction.REVERSE);
        double frS = surfaceSpeed(fr, DcMotorSimple.Direction.FORWARD);
        double brS = surfaceSpeed(br, DcMotorSimple.Direction.FORWARD);

        // Standard mecanum forward kinematics.
        double forward = (flS + frS + blS + brS) / 4.0;
        double left = (-flS + frS + blS - brS) / 4.0;
        double omega = (-flS + frS - blS + brS) / 4.0;

        Translation2d robotFrame = new Translation2d(
                forward * MAX_LINEAR_IN_S, left * MAX_LINEAR_IN_S);
        trueVelocity = robotFrame.rotateBy(truePose.getRotation());
        trueOmega = omega * MAX_OMEGA_RAD_S;

        double nextX = truePose.getX() + trueVelocity.getX() * DT;
        double nextY = truePose.getY() + trueVelocity.getY() * DT;

        // The field has walls. Without them a control bug drives the robot to
        // implausible coordinates and every downstream number becomes noise about
        // the escape rather than about the bug.
        double limit = VisionConstants.FIELD_HALF_SIZE_IN - ROBOT_HALF_WIDTH_IN;
        boolean hitWall = Math.abs(nextX) > limit || Math.abs(nextY) > limit;
        nextX = Math.max(-limit, Math.min(limit, nextX));
        nextY = Math.max(-limit, Math.min(limit, nextY));
        if (hitWall) {
            wallContacts++;
        }

        truePose = new Pose2d(nextX, nextY,
                new Rotation2d(truePose.getHeading() + trueOmega * DT));

        // Odometry creeps away from truth while moving. The drift is accumulated in
        // the ROBOT's frame, which is both more physical (slip happens at the
        // wheels) and necessary for the red/blue comparison: a drift fixed in field
        // coordinates would not mirror, and would show up as a false asymmetry.
        double speed = trueVelocity.getNorm();
        if (speed > 1.0) {
            Translation2d step = new Translation2d(
                            ODOMETRY_DRIFT_IN_PER_S * DT,
                            ODOMETRY_DRIFT_IN_PER_S * DT * 0.5)
                    .rotateBy(truePose.getRotation());
            driftX += step.getX();
            driftY += step.getY();
        }
        pushOdometry();
    }

    /**
     * Builds the camera frame for this instant: which tags are in view, and a
     * botpose when at least one is.
     */
    void stepVision(Alliance alliance) {
        LLResult result = new LLResult();
        List<Integer> seen = new ArrayList<>();

        for (Goal goal : cameraWorking ? goals : new ArrayList<Goal>()) {
            Translation2d position = GoalSelector.positionFor(goal, alliance);
            Translation2d toGoal = position.minus(truePose.getTranslation());
            double distance = toGoal.getNorm();
            if (distance > CAMERA_RANGE_IN) {
                continue;
            }
            // Is the goal within the camera's cone? The camera looks along the
            // robot's heading plus the shooter's yaw offset.
            double bearing = Math.atan2(toGoal.getY(), toGoal.getX());
            double off = Math.abs(wrap(bearing - truePose.getHeading()
                    - Math.toRadians(ShootingConstants.SHOOTER_YAW_OFFSET_DEG)));
            if (off <= Math.toRadians(CAMERA_FOV_DEG)) {
                seen.addAll(goal.getTagIds());
            }
        }

        if (seen.isEmpty()) {
            result.valid = false;
            result.tagCount = 0;
            result.botpose = null;
            result.botposeMt2 = null;
        } else {
            result.valid = true;
            result.tagCount = Math.min(2, seen.size());
            // Report the real distance to the nearest visible goal, expressed in
            // whatever unit the SDK is configured to report -- so the sim
            // exercises the unit conversion rather than bypassing it.
            double nearestIn = Double.MAX_VALUE;
            for (Goal goal : goals) {
                if (!goal.isSeenAmong(seen)) {
                    continue;
                }
                nearestIn = Math.min(nearestIn, GoalSelector.positionFor(goal, alliance)
                        .getDistance(truePose.getTranslation()));
            }
            if (nearestIn == Double.MAX_VALUE) {
                nearestIn = 40.0;
            }
            result.avgDist =
                    VisionConstants.BOTPOSE_AVG_DIST_UNIT.fromInches(nearestIn);
            // A good fix: botpose is the true pose. Odometry drift is the error
            // source here, and vision is what should be removing it.
            Pose3D pose = new Pose3D(truePose.getX(), truePose.getY(), truePose.getHeading());
            result.botpose = pose;
            result.botposeMt2 = pose;
            for (int id : seen) {
                result.withTags(id);
            }
        }
        camera.nextResult = result;
    }

    /** One full robot loop: time, physics, vision. Caller then runs the code. */
    void tick(Alliance alliance) {
        simTime += DT;
        stepPhysics();
        stepVision(alliance);
    }

    static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    /** Error between the fused estimate and the truth, inches. */
    double poseError() {
        return localization.getPose().getTranslation().getDistance(truePose.getTranslation());
    }

    /** Error between raw odometry and the truth, inches. */
    double odometryError() {
        return Math.hypot(driftX, driftY);
    }

    double headingErrorDeg() {
        return Math.toDegrees(Math.abs(wrap(
                localization.getPose().getHeading() - truePose.getHeading())));
    }

    void releaseClock() {
        RobotClock.useSystemClock();
    }
}
