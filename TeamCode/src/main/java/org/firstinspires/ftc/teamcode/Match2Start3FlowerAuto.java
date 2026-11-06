package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/*
 * Match2Start3FlowerAuto
 * -----------------------------------------------------------------------------
 * Match 2 Autonomous OpMode for Start 3 position (the hive-down side).
 * From Start 3 our own alliance's hive tags are not visible at the start, so during INIT
 * (Pipeline 0, all tags) the only tags seen belong to the OPPOSING alliance. Our alliance
 * is set to the opposite of the last color seen.
 * After START, switches to our alliance's pipeline and waits up to 12 seconds for our hive
 * to rise (at least 3 of our tags appear).
 *   - Hive rises:       shoot, get pollen at the Start 3 flower (intake on), park.
 *   - Hive never rises: skip shooting, get pollen at the Start 3 flower facing 90 deg
 *                       using only the flipper (intake off), park.
 * The left flipper stays stowed the entire auto; only the right flipper is cycled, and it
 * stays open after the pollen pick (through backing away and parking).
 * Flower coordinates are the same as Match2Start1FlowerAutonomous.
 * At the conclusion of autonomous, saves the final Pinpoint pose into
 * PoseStorage.currentPose so that TeleOp can carry over exact field position.
 * -----------------------------------------------------------------------------
 */

@Autonomous(name="Match2Start3FlowerAuto", group="Linear OpMode", preselectTeleOp="Match2LimeLightTeleop")
public class Match2Start3FlowerAuto extends LinearOpMode {

    // Declare OpMode members for hardware.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intake         = null;
    private DcMotorEx shooter      = null;
    private Servo     rightFlipper = null;
    private Servo     leftFlipper  = null;
    private Servo     gate         = null;

    // goBILDA Pinpoint Odometry Computer
    private GoBildaPinpointDriver pinpoint = null;

    // Limelight 3A, used during INIT to detect alliance color
    private Limelight3A limelight = null;
    private String allianceColor = "NONE"; // "RED", "BLUE", or "NONE"

    private int shooterSpeed = 2475 * 28 / 60; // RPM converted to ticks/sec
    private double filteredShooterRpm = 0;
    private static final double SHOOTER_RPM_FILTER_ALPHA = 0.15; // lower = smoother/slower to settle

    // Pinpoint Pod Offsets (in Inches)
    private double xPodOffsetInches = -3.600; // X pod offset + is left sideways from robot center (-84 mm)
    private double yPodOffsetInches = -(17.0/2.0-2.0); // Y pod offset + is forward from robot center (-168 mm)

    // Starting Match Pose (Start 3 - same as Match2Start3AutonomousOrig)
    private double autoStartXInches = -9.5;
    private double autoStartYInches =  71.0-9.0;
    private double autoStartHeadingDegrees = 90.0; // 0 deg = Facing Forward away from driver

    // Start 3 park location (same as Match2Start3AutonomousOrig)
    private double autoLeaveXInches = -71.0+7.0+9.0;
    private double autoLeaveYInches =  44.0+9.0;
    private double autoLeaveHeadingDegrees = 90.0; // 0 deg = Facing Forward away from driver

    // Start 3 flower location for pollen retrieval (same flower as Match2Start1FlowerAutonomous)
    private double flowerXInches = -30.0;
    private double flowerYInches = 59.0;
    private double flowerHeadingDegrees = 80.0;          // used after shooting (hive rose)
    private double flowerNoShootHeadingDegrees = 90.0;   // used when the hive never rose
    private static final double FLOWER_APPROACH_OFFSET_INCHES = 8.0; // stage this far short (-Y) of the flower

    // Flower settings for this run - chosen once the hive rise wait is over
    private double activeFlowerHeadingDegrees = flowerNoShootHeadingDegrees;
    private boolean runIntakeAtFlower = false;

    // Shooting tuning
    private static final double SHOOT_TOTAL_MS = 3000;        // total shooting time, gate closes at this mark
    private static final double SHOOT_MAX_RPM_WAIT_MS = 1000; // open the gate by this mark even if RPM isn't OK

    // Hive rise detection tuning
    private static final double HIVE_RISE_WAIT_MS = 12000;  // how long to wait for the hive to rise
    private static final int    HIVE_RISE_MIN_TAGS = 3;     // at least this many of our tags must be seen
    private static final double HIVE_RISE_DWELL_MS = 500;   // tags must be seen this long in a row (Limelight data flickers)
    private double hiveTagsSeenSinceMs = -1;
    private int hiveTagCount = 0;

    // Flower pollen retrieval tuning
    private static final int FLOWER_FLIP_CYCLES = 4;         // number of stow/open cycles of the right flipper
    private static final double FLOWER_FLIP_CYCLE_MS = 775;  // time for one stow + open cycle

    // Heading correction tuning
    private static final double MIN_TURN_POWER = 0.17; // measured minimum power to overcome drivetrain static friction
    private static final double MAX_TURN_POWER = 0.4;  // correction power ceiling
    private static final double HEADING_KP = 1.43;     // tuned so ~10 deg error yields ~0.25 power

    // Axial (forward/back) control tuning for moveToWaypoint.
    private static final double AXIAL_KP = 0.027;          // power per inch of forward/back error
    private static final double AXIAL_MIN_POWER = 0.12;    // floor so the last few inches don't crawl
    private static final double AXIAL_MAX_POWER = 0.6;     // power ceiling

    // Strafe (left/right) control tuning for moveToWaypoint.
    private static final double STRAFE_KP = 0.035;         // power per inch of left/right error
    private static final double STRAFE_MIN_POWER = 0.16;   // floor so the last few inches don't crawl
    private static final double STRAFE_MAX_POWER = 0.6;    // power ceiling

    // Move tolerances
    private static final double POSITION_TOLERANCE_INCHES = 1.5;
    private static final double HEADING_TOLERANCE_DEGREES = 2.0;
    private static final double PRECISE_POSITION_TOLERANCE_INCHES = 1.0;
    private static final double PRECISE_HEADING_TOLERANCE_DEGREES = 1.5;

    // Zero-power bands
    private static final double ZERO_POWER_BAND_FRACTION = 0.5;

    // "Fully stopped" thresholds
    private static final double STOP_VELOCITY_THRESHOLD_INCHES_PER_SEC = 2.5;
    private static final double STOP_ANGULAR_VELOCITY_THRESHOLD_DEG_PER_SEC = 8.0;
    private static final double SETTLE_DWELL_MS = 150;

    // RPM_OK tuning
    private static final double RPM_TOLERANCE_PERCENT = 0.015;
    private static final double RPM_TOLERANCE_MIN_FLOOR = 20;
    private static final double RPM_OK_DWELL_MS = 1000;
    private double rpmInBandSinceMs = -1;

    // moveToWaypoint non-blocking state
    private boolean moveActive = false;
    private double activeTargetHeadingDeg;
    private double activeTargetXInches;
    private double activeTargetYInches;
    private double activeTimeoutMs;
    private boolean activeRequireFullStop;
    private double activePositionToleranceInches;
    private double activeHeadingToleranceDegrees;
    private double moveStartTimeMs;
    private double settleStartTimeMs = -1;

    // Autonomous sequencer state.
    private int autoStep = 0;
    private double stepStartTimeMs = -1;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightRear");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftRear");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        rightFlipper = hardwareMap.get(Servo.class, "rightFlipper");
        leftFlipper = hardwareMap.get(Servo.class, "leftFlipper");
        gate = hardwareMap.get(Servo.class, "gate");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        rightFlipper.setDirection(Servo.Direction.FORWARD);
        leftFlipper.setDirection(Servo.Direction.REVERSE);
        gate.setDirection(Servo.Direction.FORWARD);

        gate.setPosition(1.0); // blocked
        leftFlipper.setPosition(1.0); // stowed - stays stowed the entire auto

        shooter.setVelocityPIDFCoefficients(350, 0, 0, 12.53);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize goBILDA Pinpoint with pod offsets and starting pose
        initPinpoint(xPodOffsetInches, yPodOffsetInches, autoStartXInches, autoStartYInches, autoStartHeadingDegrees);

        // Initialize Limelight 3A (Pipeline 0 sees all tags, used to detect alliance color)
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
        } catch (Exception e) {
            telemetry.addData("Limelight Warning", "Limelight hardware not found: " + e.getMessage());
        }

        // Display repeating position telemetry during INIT phase until driver presses START
        while (!isStarted() && !isStopRequested()) {
            pinpoint.update();

            // From Start 3 only the OPPOSING alliance's tags are visible, so our alliance is the
            // opposite of the last color seen. Keep it, so a missed frame doesn't reset it.
            String seenAlliance = detectAllianceColor();
            if (!seenAlliance.equals("NONE")) {
                allianceColor = oppositeAlliance(seenAlliance);
            }

            updateTelemetry("Initialized - Waiting for Start");
            sleep(20);
        }

        waitForStart();
        runtime.reset();

        // Alliance color is now final - switch Limelight to only see our alliance's tags
        selectAlliancePipeline();

        // Run the autonomous sequence until it completes or the match period ends
        while (opModeIsActive()) {
            pinpoint.update();
            updateShooterRpmFilter();

            runAutonomousSequence();

            saveEndPoseToStorage();
            updateTelemetry("Running");
        }

        stopAllDriveMotors(true);
        sleep(500); // Wait 200ms for robot to physically settle after braking
        pinpoint.update();
        saveEndPoseToStorage();
    }

    /**
     * Saves the final robot position and heading from Pinpoint into PoseStorage for TeleOp.
     */
    private void saveEndPoseToStorage() {
        if (pinpoint != null) {
            PoseStorage.currentPose = pinpoint.getPosition();
            PoseStorage.hasValidPose = true;
        }
        PoseStorage.allianceColor = allianceColor;
    }

    /**
     * Looks at all AprilTags the Limelight sees and picks the one closest to the center of the
     * camera view. Red tags are IDs 30-37, blue tags are IDs 38-45.
     * Needs Pipeline 0 (all tags) to be active.
     * @return "RED", "BLUE", or "NONE" if no hive tag is seen
     */
    private String detectAllianceColor() {
        if (limelight == null) {
            return "NONE";
        }

        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid()) {
            return "NONE";
        }

        int closestTagId = -1;
        double closestOffsetDegrees = Double.MAX_VALUE;
        for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
            // Angle between the tag and the center of the camera view
            double offsetDegrees = Math.hypot(fiducial.getTargetXDegrees(), fiducial.getTargetYDegrees());
            if (offsetDegrees < closestOffsetDegrees) {
                closestOffsetDegrees = offsetDegrees;
                closestTagId = fiducial.getFiducialId();
            }
        }

        if (closestTagId >= 30 && closestTagId <= 37) {
            return "RED";
        } else if (closestTagId >= 38 && closestTagId <= 45) {
            return "BLUE";
        }
        return "NONE";
    }

    /**
     * Returns the other alliance color: "RED" -> "BLUE", "BLUE" -> "RED", anything else -> "NONE".
     */
    private String oppositeAlliance(String color) {
        if (color.equals("RED")) {
            return "BLUE";
        } else if (color.equals("BLUE")) {
            return "RED";
        }
        return "NONE";
    }

    /**
     * Counts how many of OUR alliance's hive tags the Limelight sees right now.
     * Red tags are IDs 30-37, blue tags are IDs 38-45. Tags of the other color are ignored,
     * so this stays correct even if the Limelight is still on Pipeline 0.
     * @return number of our tags seen, or 0 if no Limelight / no alliance / no valid result
     */
    private int countAllianceTags() {
        if (limelight == null || allianceColor.equals("NONE")) {
            return 0;
        }

        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid()) {
            return 0;
        }

        int minId = allianceColor.equals("RED") ? 30 : 38;
        int maxId = allianceColor.equals("RED") ? 37 : 45;
        int count = 0;
        for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
            int id = fiducial.getFiducialId();
            if (id >= minId && id <= maxId) {
                count++;
            }
        }
        return count;
    }

    /**
     * Non-blocking hive rise check - call once per loop. Returns true once at least
     * HIVE_RISE_MIN_TAGS of our tags have been seen for HIVE_RISE_DWELL_MS in a row.
     * Limelight data comes and goes, so one good frame isn't enough to count as a rise.
     * If there is no Limelight or our alliance is unknown, this never returns true.
     */
    private boolean hiveRiseDetected() {
        hiveTagCount = countAllianceTags();
        double nowMs = runtime.milliseconds();
        if (hiveTagCount < HIVE_RISE_MIN_TAGS) {
            hiveTagsSeenSinceMs = -1;
            return false;
        }
        if (hiveTagsSeenSinceMs < 0) {
            hiveTagsSeenSinceMs = nowMs;
        }
        return (nowMs - hiveTagsSeenSinceMs) >= HIVE_RISE_DWELL_MS;
    }

    /**
     * Switches the Limelight to the pipeline that only sees our alliance's tags.
     * Pipeline 1 = red tags only, Pipeline 2 = blue tags only.
     * If the alliance is NONE, stays on Pipeline 0 (all tags).
     */
    private void selectAlliancePipeline() {
        if (limelight == null) {
            return;
        }
        if (allianceColor.equals("RED")) {
            limelight.pipelineSwitch(1);
        } else if (allianceColor.equals("BLUE")) {
            limelight.pipelineSwitch(2);
        }
    }

    private void updateTelemetry(String status) {
        Pose2D pose = pinpoint.getPosition();
        telemetry.addData("Status", status);
        telemetry.addData("Alliance", allianceColor);
        telemetry.addData("X (in)", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y (in)", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Auto Step", autoStep);
        telemetry.addData("Hive Tags Seen", hiveTagCount);
        telemetry.addData("RPMcommand=", shooterSpeed * 60 / 28);
        telemetry.addData("RPMActual=", filteredShooterRpm);
        telemetry.update();
    }

    private void stopAllDriveMotors(boolean intakeAlsoStop) {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        if (intakeAlsoStop) {
            intake.setPower(0);
        }
    }

    private double computeHeadingCorrectionPower(double headingErrorRad, double zeroPowerBandDegrees) {
        if (Math.abs(Math.toDegrees(headingErrorRad)) < zeroPowerBandDegrees) {
            return 0.0;
        }
        double rawYaw = headingErrorRad * HEADING_KP;
        rawYaw = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, rawYaw));
        if (Math.abs(rawYaw) < MIN_TURN_POWER) {
            rawYaw = Math.signum(headingErrorRad) * MIN_TURN_POWER;
        }
        return rawYaw;
    }

    private double computeAxisPower(double errorInches, double kp, double minPower,
                                    double zeroPowerBandInches) {
        if (Math.abs(errorInches) < zeroPowerBandInches) {
            return 0.0;
        }
        double power = errorInches * kp;
        if (Math.abs(power) < minPower) {
            power = Math.signum(errorInches) * minPower;
        }
        return power;
    }

    private boolean moveToWaypoint(double targetHeadingDegrees, double targetXInches,
                                    double targetYInches, double timeoutMs, boolean requireFullStop,
                                    double positionToleranceInches, double headingToleranceDegrees) {

        double nowMs = runtime.milliseconds();

        boolean isSameMove = moveActive
                && activeTargetHeadingDeg == targetHeadingDegrees
                && activeTargetXInches == targetXInches
                && activeTargetYInches == targetYInches
                && activeTimeoutMs == timeoutMs
                && activeRequireFullStop == requireFullStop
                && activePositionToleranceInches == positionToleranceInches
                && activeHeadingToleranceDegrees == headingToleranceDegrees;

        if (!isSameMove) {
            moveActive = true;
            activeTargetHeadingDeg = targetHeadingDegrees;
            activeTargetXInches = targetXInches;
            activeTargetYInches = targetYInches;
            activeTimeoutMs = timeoutMs;
            activeRequireFullStop = requireFullStop;
            activePositionToleranceInches = positionToleranceInches;
            activeHeadingToleranceDegrees = headingToleranceDegrees;
            moveStartTimeMs = nowMs;
            settleStartTimeMs = -1;
        }

        Pose2D pose = pinpoint.getPosition();
        double currentXInches = pose.getX(DistanceUnit.INCH);
        double currentYInches = pose.getY(DistanceUnit.INCH);
        double currentHeadingRad = pose.getHeading(AngleUnit.RADIANS);

        double targetHeadingRad = Math.toRadians(targetHeadingDegrees);
        double errX = targetXInches - currentXInches;
        double errY = targetYInches - currentYInches;
        double headingErrorRad = AngleUnit.normalizeRadians(targetHeadingRad - currentHeadingRad);

        double distanceInches = Math.hypot(errX, errY);
        boolean positionOk = distanceInches < positionToleranceInches;
        boolean headingOk = Math.abs(Math.toDegrees(headingErrorRad)) < headingToleranceDegrees;

        boolean timedOut = (nowMs - moveStartTimeMs) >= timeoutMs;

        boolean velocityOk = true;
        if (requireFullStop) {
            double velX = pinpoint.getVelX(DistanceUnit.INCH);
            double velY = pinpoint.getVelY(DistanceUnit.INCH);
            double speedInchesPerSec = Math.hypot(velX, velY);
            double angularVelDegPerSec = Math.abs(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            velocityOk = speedInchesPerSec < STOP_VELOCITY_THRESHOLD_INCHES_PER_SEC
                    && angularVelDegPerSec < STOP_ANGULAR_VELOCITY_THRESHOLD_DEG_PER_SEC;
        }

        boolean toleranceMet = positionOk && headingOk && velocityOk;

        boolean done;
        if (timedOut) {
            done = true;
        } else if (!requireFullStop) {
            done = positionOk && headingOk;
        } else if (toleranceMet) {
            if (settleStartTimeMs < 0) {
                settleStartTimeMs = nowMs;
            }
            done = (nowMs - settleStartTimeMs) >= SETTLE_DWELL_MS;
        } else {
            settleStartTimeMs = -1;
            done = false;
        }

        if (done) {
            stopAllDriveMotors(false);
            moveActive = false;
            return true;
        }

        double robotAxial = errX * Math.cos(currentHeadingRad) + errY * Math.sin(currentHeadingRad);
        double robotStrafeLeft = -errX * Math.sin(currentHeadingRad) + errY * Math.cos(currentHeadingRad);

        double translationZeroBand = positionToleranceInches * ZERO_POWER_BAND_FRACTION;
        double headingZeroBand = headingToleranceDegrees * ZERO_POWER_BAND_FRACTION;

        double axialPower  = computeAxisPower(robotAxial,      AXIAL_KP,  AXIAL_MIN_POWER,  translationZeroBand);
        double strafePower = computeAxisPower(robotStrafeLeft, STRAFE_KP, STRAFE_MIN_POWER, translationZeroBand);

        double scale = Math.max(Math.abs(axialPower) / AXIAL_MAX_POWER, Math.abs(strafePower) / STRAFE_MAX_POWER);
        if (scale > 1.0) {
            axialPower  /= scale;
            strafePower /= scale;
        }
        double yawPower = computeHeadingCorrectionPower(headingErrorRad, headingZeroBand);

        double frontLeftPower = axialPower - strafePower - yawPower;
        double frontRightPower = axialPower + strafePower + yawPower;
        double backLeftPower = axialPower + strafePower - yawPower;
        double backRightPower = axialPower - strafePower + yawPower;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

        return false;
    }

    private void updateShooterRpmFilter() {
        double rawRpm = shooter.getVelocity() * 60.0 / 28.0;
        filteredShooterRpm += SHOOTER_RPM_FILTER_ALPHA * (rawRpm - filteredShooterRpm);
    }

    private boolean RPM_OK() {
        double commandedRpm = shooterSpeed * 60.0 / 28.0;
        double tolerance = Math.max(commandedRpm * RPM_TOLERANCE_PERCENT, RPM_TOLERANCE_MIN_FLOOR);
        boolean inBand = Math.abs(filteredShooterRpm - commandedRpm) < tolerance;

        double nowMs = runtime.milliseconds();
        if (inBand) {
            if (rpmInBandSinceMs < 0) {
                rpmInBandSinceMs = nowMs;
            }
            return (nowMs - rpmInBandSinceMs) >= RPM_OK_DWELL_MS;
        } else {
            rpmInBandSinceMs = -1;
            return false;
        }
    }

    private void runAutonomousSequence() {
        double nowMs = runtime.milliseconds();
        if (stepStartTimeMs < 0) {
            stepStartTimeMs = nowMs;
        }
        double elapsedInStep = nowMs - stepStartTimeMs;

        switch (autoStep) {
            case 0:
                // Spin up the shooter now so it's ready if the hive rises
                shooter.setVelocity(shooterSpeed);
                advanceStep();
                break;

            case 1:
                // Wait up to 12 sec for our hive to rise (at least 3 of our tags appear)
                if (hiveRiseDetected()) {
                    activeFlowerHeadingDegrees = flowerHeadingDegrees;
                    runIntakeAtFlower = true;
                    jumpToStep(10); // hive rose - shoot
                } else if (elapsedInStep >= HIVE_RISE_WAIT_MS) {
                    shooter.setVelocity(0.0);
                    activeFlowerHeadingDegrees = flowerNoShootHeadingDegrees;
                    runIntakeAtFlower = false;
                    jumpToStep(20); // hive never rose - skip shooting
                }
                break;

            // ---- Hive rose: shoot from the start location ----
            case 10:
                // Shooting takes 3 sec total. Open the gate once RPM is OK (or after 1 sec
                // regardless), then close it and stop the shooter at the 3 sec mark.
                if (elapsedInStep >= SHOOT_TOTAL_MS) {
                    gate.setPosition(1.0);  //gate closed
                    shooter.setVelocity(0.0);
                    jumpToStep(20);
                } else if (RPM_OK() || elapsedInStep >= SHOOT_MAX_RPM_WAIT_MS) {
                    gate.setPosition(0.0);  //gate open
                }
                break;

            // ---- Flower pollen retrieval (both paths) ----
            case 20:
                // Move off the wall
                if (moveToWaypoint(autoStartHeadingDegrees, autoStartXInches, autoStartYInches - 8.0, 3000, true, POSITION_TOLERANCE_INCHES, HEADING_TOLERANCE_DEGREES)) {
                    advanceStep();
                }
                if (elapsedInStep >= 5000.0) {
                    stopAllDriveMotors(false);
                    advanceStep();
                }
                break;

            case 21:
                // Open the right flipper (left flipper stays stowed all auto when intake stays off).
                // Intake only runs if we shot (hive rose).
                rightFlipper.setPosition(0.0);
                if (runIntakeAtFlower) {
                    intake.setPower(1.0); //bring in speciman
                    rightFlipper.setPosition(0.0);
                    leftFlipper.setPosition(0.0);
                }
                advanceStep();
                break;

            case 22:
                // Line up just short of the flower
                if (moveToWaypoint(activeFlowerHeadingDegrees, flowerXInches, flowerYInches - FLOWER_APPROACH_OFFSET_INCHES, 5000, true, POSITION_TOLERANCE_INCHES, HEADING_TOLERANCE_DEGREES)) {
                    advanceStep();
                }
                if (elapsedInStep >= 7000.0) {
                    stopAllDriveMotors(false);
                    advanceStep();
                }
                break;

            case 23:
                // Drive forward into the flower
                if (moveToWaypoint(activeFlowerHeadingDegrees, flowerXInches, flowerYInches, 3000, true, PRECISE_POSITION_TOLERANCE_INCHES, PRECISE_HEADING_TOLERANCE_DEGREES)) {
                    advanceStep();
                }
                if (elapsedInStep >= 5000.0) {
                    stopAllDriveMotors(false);
                    advanceStep();
                }
                break;

            case 24:
                // Cycle the right flipper: stowed for the first half of each cycle, open for the second half
                if (elapsedInStep >= FLOWER_FLIP_CYCLES * FLOWER_FLIP_CYCLE_MS) {
                    advanceStep();
                } else if ((elapsedInStep % FLOWER_FLIP_CYCLE_MS) < FLOWER_FLIP_CYCLE_MS / 2.0) {
                    rightFlipper.setPosition(1.0); // stow
                } else {
                    rightFlipper.setPosition(0.0); // open
                }
                break;

            case 25:
                // Leave the right flipper OPEN for the rest of auto (never stow it after the
                // pollen pick) and turn the intake off
                rightFlipper.setPosition(0.0);
                intake.setPower(0.0);
                advanceStep();
                break;

            case 26:
                // Back away from the flower (robot faces +Y, so back up toward -Y)
                if (moveToWaypoint(activeFlowerHeadingDegrees, flowerXInches, flowerYInches - FLOWER_APPROACH_OFFSET_INCHES, 3000, true, POSITION_TOLERANCE_INCHES, HEADING_TOLERANCE_DEGREES)) {
                    rightFlipper.setPosition(0.0);
                    leftFlipper.setPosition(0.0);
                    advanceStep();
                }
                if (elapsedInStep >= 5000.0) {
                    stopAllDriveMotors(false);
                    advanceStep();
                }
                break;

            case 27:
                // Park in the Start 3 park spot
                if (moveToWaypoint(autoLeaveHeadingDegrees, autoLeaveXInches, autoLeaveYInches, 8000, true, PRECISE_POSITION_TOLERANCE_INCHES, PRECISE_HEADING_TOLERANCE_DEGREES)) {
                    advanceStep();
                }
                if (elapsedInStep >= 10000.0) {
                    stopAllDriveMotors(false);
                    advanceStep();
                }
                break;

            default:
                stopAllDriveMotors(true);
                break;
        }
    }

    private void advanceStep() {
        autoStep++;
        stepStartTimeMs = -1;
    }

    // Jump to any step number (used to branch between the hive rose / hive never rose paths)
    private void jumpToStep(int step) {
        autoStep = step;
        stepStartTimeMs = -1;
    }

    private void initPinpoint(double xPodOffsetInches, double yPodOffsetInches, double startXInches, double startYInches, double startHeadingDegrees) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(25.4 * xPodOffsetInches, 25.4 * yPodOffsetInches, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
        sleep(500);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startXInches, startYInches, AngleUnit.DEGREES, startHeadingDegrees));
    }
}
