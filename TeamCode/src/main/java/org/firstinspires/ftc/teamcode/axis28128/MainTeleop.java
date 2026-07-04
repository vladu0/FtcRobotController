package org.firstinspires.ftc.teamcode.axis28128;

import static java.lang.Math.sqrt;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Main Teleop Mode")
@Configurable
public class MainTeleop extends OpMode {
    // Add fields
    public static double BALL_DETECT_THRESHOLD = 65; // between ball (~40) and gap (~100)
    public static int BALL_DETECT_CONSECUTIVE = 3;    // readings needed to confirm
    private int closeReadingStreak = 0;
    public static double SHOOT_ADVANCE_MS = 450; // tune this — time between each ball feed
    private double nextShootAdvanceTime = 0;
    private Follower follower;
    public TelemetryManager telemetryM;

    // Red-alliance aim point; blue mirrors across the field centerline (x = 72).
    public static double GOAL_X = 140, GOAL_Y = 140;
    public static boolean IS_RED_ALLIANCE = true;

    public static double shooterMaxTPS = 6200, shooterMinTPS = 2000, currentTPS = shooterMaxTPS;
    public static double TURRET_TPR = 873;
    public static double TURRET_TICKS_PER_RADIAN = TURRET_TPR / (2 * Math.PI);
    public static double TURRET_PWR = 0.3;

    public static double TURRET_ANGLE_SIGN = -1;
    // Calibrated on-field 2026-07-04 via the BACK-button routine (50.3 deg).
    public static double TURRET_ANGLE_OFFSET = 0.88;

    public static double TURRET_MIN_ANGLE = 0;
    public static double TURRET_MAX_ANGLE = 0;

    public static int TURRET_TICK_MIN = -390;
    public static int TURRET_TICK_MAX = 500;
    // === SHOOTER PIDF TUNING ===
    public static double kV = 0.00009;  // volts (power) per RPM — main feedforward term
    public static double kS = 0.05;     // static friction/minimum power to overcome stiction
    public static double kP = 0.0001;   // proportional correction for RPM error

    public DcMotor transferMotor, turretMotor, intake;
    public DcMotorEx shooterMotor;
    public Servo spindexerServo, bootKickerServo, shooterServo;
    public NormalizedColorSensor colorSensor;
    public DistanceSensor spindexDistance;

    private boolean isShooting = false;
    private boolean turretCalibrationMode = false;
    private static ElapsedTime currentTimer = new ElapsedTime();

    private double currentShooterRPM = 0;
    private double currentDistance = 0;
    private double distRatioDebug = 0;

    public double[] spindexerPos = {0.24, 0.48, 0.72, 0.53, 0.27, 0};

    // BUG FIX #1: lastMeasuredDistance is now only updated inside the timed block,
    // not again at the bottom of loop(). This ensures the ball-detect comparison
    // between consecutive readings is valid.
    public double lastMeasuredDistance = 0, measuredDistance = 0;

    private double timerTarget = 0;
    private final double deltaDistanceSensorReadingsMillis = 120;
    public int spinidx = 0;
    public boolean ballWasDetected = false;

    // Tracks whether PIDF needs to be re-applied (only on change, not every frame).

    @Override
    public void init() {
        shooterMotor   = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        transferMotor  = hardwareMap.get(DcMotor.class,   "transferMotor");
        turretMotor    = hardwareMap.get(DcMotor.class,   "turretMotor");
        intake         = hardwareMap.get(DcMotor.class,   "intakeMotor");

        spindexerServo  = hardwareMap.get(Servo.class, "spindexerServo");
        bootKickerServo = hardwareMap.get(Servo.class, "transferServo");
        shooterServo    = hardwareMap.get(Servo.class, "shooterServo");

        colorSensor    = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        spindexDistance = hardwareMap.get(DistanceSensor.class,       "distanceSensor");

        bootKickerServo.setPosition(0);
        shooterServo.setPosition(0);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.currentPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        currentTimer.reset();
        timerTarget += deltaDistanceSensorReadingsMillis;
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();
        double gearRatio = (double) 10 / 16;
        double RPM = (((-shooterMotor.getVelocity()) / 28) * 60) / gearRatio;

        double ff = kS + (kV * currentTPS);
        double error = currentTPS - RPM;
        double feedback = kP * error;
        double pwr = Math.max(0, ff + feedback);

        // === DISTANCE SENSOR (timed) ===
        // BUG FIX #1: lastMeasuredDistance is captured here and ONLY here,
        // so the ball-detect comparison below sees two genuinely different readings.
        measuredDistance = spindexDistance.getDistance(DistanceUnit.MM);
        isShooting = (RPM >= currentTPS);

        // === SHOOTER / TRANSFER ===
        if (gamepad1.left_bumper) {
            shooterMotor.setPower(pwr);
            transferMotor.setPower(0.2);
            transfer(true);
            if (isShooting) {
                if (currentTimer.milliseconds() >= nextShootAdvanceTime && spinidx < 5) {
                    spinidx++;
                    nextShootAdvanceTime = currentTimer.milliseconds() + SHOOT_ADVANCE_MS;
                }
            }
        } else {
            shooterMotor.setPower(0);
            transferMotor.setPower(0);
            transfer(false);
            // Reset for next shooting sequence
            if (spinidx > 3) spinidx = 0;
            nextShootAdvanceTime = 0;
        }


        // Reset spindexer when bumper released and motor has spun down.
        if (!gamepad1.left_bumper && isShooting) spinidx = 0;

        // === INTAKE ===
        // BUG FIX #7: Intake check now happens AFTER spinidx may have been set to 3
        // above, so the "don't intake while shooting" gate is evaluated correctly.
        if (gamepad1.right_bumper && spinidx != 3) {
            intake.setPower(-0.8);
        } else if (gamepad1.y) {
            intake.setPower(0.8);
        } else {
            intake.setPower(0);
        }

        // === SPINDEXER MANUAL / AUTO ADVANCE ===
        if (gamepad1.dpadRightWasPressed())      spinidx++;
        else if (gamepad1.dpadLeftWasPressed())  spinidx--;

        boolean readingIsClose = measuredDistance <= BALL_DETECT_THRESHOLD;

        if (readingIsClose) {
            closeReadingStreak++;
        } else {
            closeReadingStreak = 0;
        }

        boolean ballNearNow = closeReadingStreak >= BALL_DETECT_CONSECUTIVE;

        if (ballNearNow && !ballWasDetected && !gamepad1.left_bumper && spinidx < 2) {
            spinidx++;
        } else if(ballNearNow && !ballWasDetected && !gamepad1.left_bumper && spinidx == 2) {
            spinidx = 3;
        }
        ballWasDetected = ballNearNow;
        // Wrap at top, floor at 0. Must use array length: spindexerPos has 6
        // entries, so letting spinidx reach 6 crashes the OpMode mid-match.
        if (spinidx >= spindexerPos.length) spinidx = 0;
        if (spinidx < 0) spinidx = 0;

        spindexerServo.setPosition(spindexerPos[spinidx]);

        // === DRIVE ===
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );

        // === TURRET ===
        Pose   currPose    = follower.getPose();
        double rx          = currPose.getX();
        double ry          = currPose.getY();
        double robotHeading = follower.getHeading();

        currentDistance = getDistance(rx, ry);
        PoseStorage.currentPose = currPose;

        // Bearing to goal relative to the robot is (angleToGoal - heading); the whole
        // relative bearing gets flipped by TURRET_ANGLE_SIGN, heading included —
        // otherwise the aim error is 2x the robot heading.
        double angleToGoal  = Math.atan2(goalY() - ry, goalX() - rx);
        double relBearing   = normalizeDelta(angleToGoal - robotHeading);
        double turretTarget = TURRET_ANGLE_SIGN * relBearing + TURRET_ANGLE_OFFSET;

        // The turret tracks the goal only while the shoot button is held; when
        // released it holds its last position. B toggles calibration mode: the
        // turret goes unpowered so it can be aimed at the goal by hand, then
        // BACK recomputes TURRET_ANGLE_OFFSET from the current physical position.
        if (gamepad1.bWasPressed()) turretCalibrationMode = !turretCalibrationMode;

        if (gamepad1.backWasPressed()) {
            TURRET_ANGLE_OFFSET = normalizeDelta(getTurretAngle() - TURRET_ANGLE_SIGN * relBearing);
        }

        if (turretCalibrationMode) {
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turretMotor.setPower(0);
        } else if (gamepad1.left_bumper) {
            setTurretAngle(turretTarget, TURRET_PWR);
        } else {
            // Freeze in place the moment LB is released — without this the motor
            // keeps driving toward its last target after the button is let go.
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(TURRET_PWR);
        }


        // === CLOSE / FAR SPEED TOGGLE ===
        if (gamepad1.xWasPressed()) {
            if (currentTPS == shooterMinTPS) {
                currentTPS = shooterMaxTPS;
            } else {
                currentTPS = shooterMinTPS;
            }
        }

        // === SHOOTER SERVO ANGLE ===
        if (gamepad1.dpad_up) {
            shooterServo.setPosition(shooterServo.getPosition() + 0.05);
        } else if (gamepad1.dpad_down) {
            shooterServo.setPosition(shooterServo.getPosition() - 0.05);
        }

        // === TELEMETRY ===
        double curVelocity = RPM;
        telemetry.addData("Meas Dist",         measuredDistance);
        telemetry.addData("Last Meas Dist",     lastMeasuredDistance);
        telemetry.addData("spinidx",            spinidx);
        telemetry.addData("=== SHOOTER ===",   "");
        telemetry.addData("Distance to Goal",  "%.1f units", currentDistance);
        telemetry.addData("Dist Ratio (0-1)",  "%.2f",       distRatioDebug);
        telemetry.addData("Shooter RPM",       "%.3f",       currentShooterRPM);
        telemetry.addData("Shooting Active",   "%s",         isShooting);
        telemetry.addData("Current Velocity",  curVelocity);
        telemetry.addData("Target Velocity",   currentTPS);
        telemetry.addData("=== TURRET ===",    "");
        telemetry.addData("Turret Mode",       turretCalibrationMode ? "CALIBRATE (hand-aim + BACK)" : "track while shooting (B = calibrate)");
        telemetry.addData("TURRET_ANGLE_SIGN", "%.0f",       TURRET_ANGLE_SIGN);
        telemetry.addData("TURRET_ANGLE_OFFSET","%.2f rad (%.1f deg)", TURRET_ANGLE_OFFSET, Math.toDegrees(TURRET_ANGLE_OFFSET));
        telemetry.addData("Robot Heading",     "%.1f deg",   Math.toDegrees(robotHeading));
        telemetry.addData("Angle To Goal",     "%.1f deg",   Math.toDegrees(angleToGoal));
        telemetry.addData("Rel Bearing",       "%.1f deg",   Math.toDegrees(relBearing));
        telemetry.addData("Turret Target",     "%.1f deg",   Math.toDegrees(turretTarget));
        telemetry.addData("Turret Current",    "%.1f deg",   Math.toDegrees(getTurretAngle()));
        telemetry.addData("Turret Error",      "%.1f deg",   Math.toDegrees(normalizeDelta(turretTarget - getTurretAngle())));
        telemetry.addData("=== POSITION ===",  "");
        telemetry.addData("Robot",             "(%.1f, %.1f)", rx, ry);
        telemetry.addData("Alliance",          IS_RED_ALLIANCE ? "RED" : "BLUE");
        telemetry.addData("Goal Target",       "(%.0f, %.0f)", goalX(), goalY());
        telemetry.addData("Turret Tick Limits","[%d, %d]",   TURRET_TICK_MIN, TURRET_TICK_MAX);
        telemetry.addData("Turret Target Pos", turretMotor.getTargetPosition());
        telemetry.addData("=== PIDF ===",      "");
        telemetry.update();
    }

    // === HELPERS ===

    public void transfer(boolean shouldTransfer) {
        bootKickerServo.setPosition(shouldTransfer ? 0.3 : 0);
    }

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

        if (TURRET_MAX_ANGLE > TURRET_MIN_ANGLE) {
            targetRadians = Math.max(TURRET_MIN_ANGLE, Math.min(TURRET_MAX_ANGLE, targetRadians));
        }

        double currentRadians = getTurretAngle();
        double delta = normalizeDelta(targetRadians - currentRadians);

        if (Math.abs(delta) < 0.01) {
            turretMotor.setPower(0);
            return;
        }

        double newTarget  = currentRadians + delta;
        int    targetTicks = (int) (newTarget * TURRET_TICKS_PER_RADIAN);
        // The [TICK_MIN, TICK_MAX] window spans a full revolution, so any target
        // outside it has an equivalent inside — wrap by one revolution instead of
        // rejecting (the old < -490 threshold left a ~41 deg dead zone).
        if (targetTicks < TURRET_TICK_MIN)      targetTicks += (int) TURRET_TPR;
        else if (targetTicks > TURRET_TICK_MAX) targetTicks -= (int) TURRET_TPR;

        if (targetTicks < TURRET_TICK_MIN || targetTicks > TURRET_TICK_MAX) {
            turretMotor.setPower(0);
            return;
        }

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(pwr);
    }

    public double goalX() {
        return IS_RED_ALLIANCE ? GOAL_X : 144 - GOAL_X;
    }

    public double goalY() {
        return GOAL_Y;
    }

    public double getDistance(double rx, double ry) {
        double dx = goalX() - rx;
        double dy = goalY() - ry;
        return sqrt(dx * dx + dy * dy);
    }

    public double getTurretAngle() {
        return turretMotor.getCurrentPosition() / TURRET_TICKS_PER_RADIAN;
    }
}