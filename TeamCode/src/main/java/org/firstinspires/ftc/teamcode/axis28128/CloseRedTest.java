package org.firstinspires.ftc.teamcode.axis28128;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Close Red Test", group = "Autonomous")
@Configurable
public class CloseRedTest extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    // === HARDWARE ===
    public DcMotor transferMotor, turretMotor, intake;
    public DcMotorEx shooterMotor;
    public Servo spindexerServo, bootKickerServo, shooterServo;
    public DistanceSensor spindexDistance;

    // === SUBSYSTEM CONSTANTS & VARIABLES ===
    public static double shooterMaxTPS = 6200, currentTPS = shooterMaxTPS;
    public static double kV = 0.00009, kS = 0.05, kP = 0.0001;
    public static double SHOOT_ADVANCE_MS = 450;

    public static double TURRET_TPR = 873;
    public static double TURRET_TICKS_PER_RADIAN = TURRET_TPR / (2 * Math.PI);
    public static double TURRET_PWR = 0.3;
    public static double TURRET_ANGLE_SIGN = -1;
    public static double TURRET_ANGLE_OFFSET = (Math.PI / 6) + (Math.PI / 18) + (Math.PI / 36);
    public static int TURRET_TICK_MIN = -390, TURRET_TICK_MAX = 500;

    public static double BALL_DETECT_THRESHOLD = 65;
    public static int BALL_DETECT_CONSECUTIVE = 3;

    public double[] spindexerPos = {0.24, 0.48, 0.72, 0.53, 0.27, 0.01, 1};
    public int spinidx = 0;

    private int closeReadingStreak = 0;
    private boolean ballWasDetected = false;
    private double nextShootAdvanceTime = 0;
    private ElapsedTime currentTimer = new ElapsedTime();
    private ElapsedTime autoActionTimer = new ElapsedTime();

    // === AUTO TOGGLES ===
    public boolean isShootingActive = false;
    public boolean isIntakingActive = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Hardware
        shooterMotor    = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        transferMotor   = hardwareMap.get(DcMotor.class,   "transferMotor");
        turretMotor     = hardwareMap.get(DcMotor.class,   "turretMotor");
        intake          = hardwareMap.get(DcMotor.class,   "intakeMotor");
        spindexerServo  = hardwareMap.get(Servo.class,     "spindexerServo");
        bootKickerServo = hardwareMap.get(Servo.class,     "transferServo");
        shooterServo    = hardwareMap.get(Servo.class,     "shooterServo");
        spindexDistance = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        transfer(false);
        shooterServo.setPosition(0);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize PedroPathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(110.667, 135.414, Math.toRadians(270)));
        paths = new Paths(follower);

        currentTimer.reset();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();      // Update driving
        PoseStorage.currentPose = follower.getPose(); // Hand off pose to teleop
        updateSubsystems();     // Update shooter/intake/turret logic continuously
        autonomousPathUpdate(); // Update auto state machine

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shooting Active", isShootingActive);
        panelsTelemetry.debug("Intaking Active", isIntakingActive);
        panelsTelemetry.update(telemetry);
    }

    // =====================================================================
    // BACKGROUND SUBSYSTEM LOGIC (Runs continuously based on toggles)
    // =====================================================================
    private void updateSubsystems() {
        // --- 1. Shooter PIDF & Power Calculation ---
        double gearRatio = (double) 10 / 16;
        double RPM = (((-shooterMotor.getVelocity()) / 28) * 60) / gearRatio;
        double ff = kS + (kV * currentTPS);
        double error = currentTPS - RPM;
        double pwr = Math.max(0, ff + (kP * error));
        boolean isShooterAtSpeed = (RPM >= currentTPS);

        // --- 2. Integrated Teleop Shooting Sequence ---
        if (isShootingActive) {
            shooterMotor.setPower(pwr);
            transferMotor.setPower(0.2);

            // --- TURRET DISABLED ---
            // double angleToGoal = Math.atan2(140 - follower.getPose().getY(), 140 - follower.getPose().getX());
            // double turretTarget = TURRET_ANGLE_SIGN * (angleToGoal - follower.getPose().getHeading()) + TURRET_ANGLE_OFFSET;
            // setTurretAngle(turretTarget, TURRET_PWR);

            transfer(true);

            // Spindexer Advance Logic
            if (isShooterAtSpeed) {
                if (currentTimer.milliseconds() >= nextShootAdvanceTime && spinidx < 5) {
                    spinidx++;
                    nextShootAdvanceTime = currentTimer.milliseconds() + SHOOT_ADVANCE_MS;
                }
            }
        } else {
            // Reset state when not shooting
            shooterMotor.setPower(0);
            transferMotor.setPower(0);
            transfer(false);

            // Reset for next shooting sequence
            if (spinidx > 3) spinidx = 0;
            nextShootAdvanceTime = 0;
        }

        // --- 3. Intake & Auto-indexing ---
        if (isIntakingActive && spinidx != 4) {
            intake.setPower(0.8);
        } else {
            intake.setPower(0);
        }

        double measuredDistance = spindexDistance.getDistance(DistanceUnit.MM);
        boolean readingIsClose = measuredDistance <= BALL_DETECT_THRESHOLD;

        if (readingIsClose) {
            closeReadingStreak++;
        } else {
            closeReadingStreak = 0;
        }

        boolean ballNearNow = closeReadingStreak >= BALL_DETECT_CONSECUTIVE;

        if (ballNearNow && !ballWasDetected && !isShootingActive && spinidx < 3) {
            spinidx++;
        }
        ballWasDetected = ballNearNow;

        // Clamp spinidx
        if (spinidx > 6) spinidx = 0;
        if (spinidx < 0) spinidx = 0;

        spindexerServo.setPosition(spindexerPos[spinidx]);
    }

    // =====================================================================
    // HARDWARE HELPERS
    // =====================================================================
    public void transfer(boolean active) {
        bootKickerServo.setPosition(active ? 0.3 : 0);
    }

    // =====================================================================
    // AUTONOMOUS STATE MACHINE
    // =====================================================================
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start driving Path 1
                follower.followPath(paths.Path1);
                setPathState(1);
                break;

            case 1:
                // Wait for Path 1 to finish, then start shooting for 3 seconds
                if (!follower.isBusy()) {
                    isShootingActive = true;
                    autoActionTimer.reset();
                    setPathState(2);
                }
                break;

            case 2:
                // Wait 3 seconds while shooting. Then stop shooting, start intake, and drive Path 2
                if (autoActionTimer.seconds() >= 6) {
                    isShootingActive = false;
                    isIntakingActive = true;
                    follower.followPath(paths.Path2);
                    setPathState(3);
                }
                break;

            case 3:
                // Wait for Path 2 to finish, then start Path 3 (intake stays on)
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    setPathState(4);
                }
                break;

            case 4:
                // Wait for Path 3 to finish. Then stop intake, and shoot all remaining balls
                if (!follower.isBusy()) {
                    isIntakingActive = false;
                    isShootingActive = true;
                    setPathState(-1); // Idle state, background subsystem handles the shooting
                }
                break;

            case -1:
                // Robot is completely done with the pathing sequence and is continually shooting
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
    }

    // =====================================================================
    // TURRET MATH HELPERS
    // =====================================================================
    public double normalizeAngle(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle < 0) angle += 2 * Math.PI;
        return angle;
    }

    public double normalizeDelta(double delta) {
        delta = delta % (2 * Math.PI);
        if (delta > Math.PI)       delta -= 2 * Math.PI;
        else if (delta <= -Math.PI) delta += 2 * Math.PI;
        return delta;
    }

    public void setTurretAngle(double targetRadians, double pwr) {
        targetRadians = normalizeAngle(targetRadians);

        double currentRadians = getTurretAngle();
        double delta = normalizeDelta(targetRadians - currentRadians);

        if (Math.abs(delta) < 0.01) {
            turretMotor.setPower(0);
            return;
        }

        double newTarget  = currentRadians + delta;
        int    targetTicks = (int) (newTarget * TURRET_TICKS_PER_RADIAN);
        if (targetTicks < -490) targetTicks += (int) TURRET_TPR;

        if (targetTicks < TURRET_TICK_MIN || targetTicks > TURRET_TICK_MAX) {
            turretMotor.setPower(0);
            return;
        }

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(pwr);
    }

    public double getTurretAngle() {
        return turretMotor.getCurrentPosition() / TURRET_TICKS_PER_RADIAN;
    }

    // =====================================================================
    // PATHS CLASS
    // =====================================================================
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(110.667, 135.414),
                                    new Pose(84.241, 84.742)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(50))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(84.241, 84.742),
                                    new Pose(125, 83.788)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(125, 83.788),
                                    new Pose(84.680, 84.777)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();
        }
    }
}