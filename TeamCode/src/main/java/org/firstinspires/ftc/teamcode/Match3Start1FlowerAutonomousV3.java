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
 * Match3Start1FlowerAutonomousV3
 * -----------------------------------------------------------------------------
 * Match 3 Autonomous OpMode for Start 1 position.
 * After shooting and the intermediate move, drives to a flower and retrieves pollen
 * (flippers open, flower bar GATHERING, intake on, 3 sec pause in the flower), backs away
 * with the intake still on, then parks with the bar RUNNING.
 * Flower bar (servo "flowerBar"): STOWED at start and while shooting, RUNNING when driving.
 * At the conclusion of autonomous, saves the final Pinpoint pose into
 * PoseStorage.currentPose so that TeleOp can carry over exact field position.
 * -----------------------------------------------------------------------------
 */

@Autonomous(name="Match3Start1FlowerAutonomousV3", group="Linear OpMode", preselectTeleOp="Match3LimeLightTeleopV3")
public class Match3Start1FlowerAutonomousV3 extends LinearOpMode {

    // Declare OpMode members for hardware.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intake         = null;
    private DcMotorEx pollenShooter = null;
    private Servo     rightFlipper = null;
    private Servo     leftFlipper  = null;
    private Servo pollenGate = null;
    private Servo flowerBar  = null;

    // goBILDA Pinpoint Odometry Computer
    private GoBildaPinpointDriver pinpoint = null;

    // Limelight 3A, used during INIT to detect alliance color
    private Limelight3A limelight = null;
    private String allianceColor = "NONE"; // "RED", "BLUE", or "NONE"

    // Shooter RPM scales with distance to goal (same as Match3LimeLightTeleopV3):
    //   RPM = RPMGain * RPMSlope * distance(inches) + RPMOffset
    private double pollenShooterRPMSlope = 3.3;
    private double pollenShooterRPMOffset = 2511.3;  // 2640 RPM at 39 in (was 2400, match 2 data was 2366)
    private double pollenShooterRPMGain = 1.0; // fixed in auto (no dpad adjustment)
    private static final double SHOOTER_RPM_MAX_DISTANCE_INCHES = 65.0; // distance used in RPM calculation is limited to this
    private int pollenShooterSpeed = 2366 * 28 / 60; // RPM converted to ticks/sec (recomputed every loop from distance)
    private boolean shooterOn = false; // when true, the shooter velocity is updated every loop from distance
    private double targetDistanceInches = 0.0; // distance from launcher exit to nearest hive opening (inches)

    // Launcher exit offset from robot center (same as Match3LimeLightTeleopV3)
    private double laucherToRobotCenterX=-9.0;
    private double laucherToRobotCenterY=0.0;  //was -4.0 on match2 robot

    // Gate and flipper positions (same as Match3LimeLightTeleopV3)
    private final double pollenGateClosed=1.0;
    private final double pollenGateOpen=0.6;
    private final double flipperRightStowedPosition=0.75;
    private final double flipperRightOpenPosition=0.0;
    private final double flipperLeftStowedPosition=0.8;
    private final double flipperLeftOpenPosition=0.05;

    // Flower bar positions (same as Match3LimeLightTeleopV3) - TUNE with Match3FlowerBarTunerV3 (0.5 = placeholder)
    //   STOWED:    uppermost. Start position and backwall shooting position. Interferes with a running intake.
    //   RUNNING:   middle. Normal position when driving around, required when running the intake.
    //   GATHERING: lowest. Used for retrieving pollen from a flower.
    private final double flowerBarStowedPosition=0.5;
    private final double flowerBarRunningPosition=0.5;
    private final double flowerBarGatheringPosition=0.5;
    private double pollenFilteredRPM = 0;
    private static final double SHOOTER_RPM_FILTER_ALPHA = 0.15; // lower = smoother/slower to settle

    // Pinpoint Pod Offsets (in Inches)
    private double xPodOffsetInches = -3.600; // X pod offset + is left sideways from robot center (-84 mm)
    private double yPodOffsetInches = -(17.0/2.0-2.0); // Y pod offset + is forward from robot center (-168 mm)

    // Starting Match Pose
    private double autoStartXInches = -9.5;
    private double autoStartYInches =  -71.0+9.0;
    private double autoStartHeadingDegrees = -90.0; // 0 deg = Facing Forward away from driver

    private double autoLeaveXInches = -71.0+5.0+9.0;
    private double autoLeaveYInches =  24.0+18.0+9.0;
    private double autoLeaveHeadingDegrees = 90.0; // 0 deg = Facing Forward away from driver

    // Flower location for pollen retrieval
    private double flowerXInches = -30.0;
    private double flowerYInches = 59.0;
    private double flowerHeadingDegrees = 90.0; // flower entry is always the nearest of 0, 90, 180, -90 (was 80)

    // Flower pollen retrieval tuning
    private static final double FLOWER_APPROACH_OFFSET_INCHES = 10.0; // approach point this far short of the flower
    private static final double FLOWER_BAR_MOVE_MS = 1000;     // time allowed for the bar to reach GATHERING before driving into the flower
    private static final double FLOWER_GATHER_DWELL_MS = 3000; // pause in the flower with the bar GATHERING and intake on (replaces flipper cycling)

    // Heading correction tuning
    private static final double MIN_TURN_POWER = 0.17; // measured minimum power to overcome drivetrain static friction
    private static final double MAX_TURN_POWER = 0.6;  // correction power ceiling (was 0.4, large turns were slow)
    private static final double HEADING_KP = 1.43;     // tuned so ~10 deg error yields ~0.25 power

    // Axial (forward/back) control tuning for moveToWaypoint.
    private static final double AXIAL_KP = 0.027;          // power per inch of forward/back error
    private static final double AXIAL_MIN_POWER = 0.15;    // floor so the last few inches don't crawl (was 0.12)
    private static final double AXIAL_MAX_POWER = 0.6;     // power ceiling

    // Strafe (left/right) control tuning for moveToWaypoint.
    private static final double STRAFE_KP = 0.035;         // power per inch of left/right error
    private static final double STRAFE_MIN_POWER = 0.22;   // floor so the last few inches don't crawl (was 0.16, stalled strafes)
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
    private boolean rpmReady = false; // RPM_OK() result, checked every loop so the 1 sec dwell is met while spinning up

    // Shooting tuning (same as the other Match3 V2 autos)
    private static final double SHOOT_GATE_OPEN_MS = 2250;    // gate stays open this long (4 pollen take ~2 sec), then closes
    private static final double SHOOT_MAX_RPM_WAIT_MS = 4000; // open the gate by this mark even if RPM isn't OK (longer here: shooter only starts at START)
    private double gateOpenedAtMs = -1;                       // time the gate opened during the current shot (-1 = not yet)

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
        pollenShooter = hardwareMap.get(DcMotorEx.class, "pollenShooter");
        rightFlipper = hardwareMap.get(Servo.class, "rightFlipper");
        leftFlipper = hardwareMap.get(Servo.class, "leftFlipper");
        pollenGate = hardwareMap.get(Servo.class, "pollenGate");
        flowerBar = hardwareMap.get(Servo.class, "flowerBar");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        pollenShooter.setDirection(DcMotorEx.Direction.REVERSE);
        rightFlipper.setDirection(Servo.Direction.FORWARD);
        leftFlipper.setDirection(Servo.Direction.REVERSE);
        pollenGate.setDirection(Servo.Direction.FORWARD);

        pollenGate.setPosition(pollenGateClosed); // blocked
        flowerBar.setPosition(flowerBarStowedPosition); // stowed is always the starting position

        pollenShooter.setVelocityPIDFCoefficients(450, 0, 0, 13.0);
        pollenShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollenShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            updateTargetDistance(); // keep distance telemetry accurate during INIT

            // Keep the last alliance color seen, so a missed frame doesn't reset it
            String seenAlliance = detectAllianceColor();
            if (!seenAlliance.equals("NONE")) {
                allianceColor = seenAlliance;
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
            updateShooterSpeed(); // shooter RPM follows distance to goal every loop
            rpmReady = RPM_OK();  // check every loop so the RPM dwell is tracked continuously

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
        telemetry.addData("Target Distance (in)", "%.1f", targetDistanceInches);
        telemetry.addData("Auto Step", autoStep);
        telemetry.addData("RPMcommand=", pollenShooterSpeed * 60 / 28);
        telemetry.addData("RPMActual=", pollenFilteredRPM);
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

    private double computeHeadingCorrectionPower(double headingErrorRad, double zeroPowerBandDegrees, boolean robotMoving) {
        if (Math.abs(Math.toDegrees(headingErrorRad)) < zeroPowerBandDegrees) {
            return 0.0;
        }
        double rawYaw = headingErrorRad * HEADING_KP;
        rawYaw = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, rawYaw));
        // The MIN_TURN_POWER floor is only needed to break static friction when the robot is stopped.
        // While the robot is already moving, use plain proportional power so small heading
        // corrections during a strafe don't overshoot and wobble.
        if (!robotMoving && Math.abs(rawYaw) < MIN_TURN_POWER) {
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

        // Robot velocity, used for the full-stop check and for the turn power floor.
        double velX = pinpoint.getVelX(DistanceUnit.INCH);
        double velY = pinpoint.getVelY(DistanceUnit.INCH);
        double speedInchesPerSec = Math.hypot(velX, velY);
        double angularVelDegPerSec = Math.abs(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        boolean robotStopped = speedInchesPerSec < STOP_VELOCITY_THRESHOLD_INCHES_PER_SEC
                && angularVelDegPerSec < STOP_ANGULAR_VELOCITY_THRESHOLD_DEG_PER_SEC;

        boolean velocityOk = true;
        if (requireFullStop) {
            velocityOk = robotStopped;
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
        double yawPower = computeHeadingCorrectionPower(headingErrorRad, headingZeroBand, !robotStopped);

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
        double rawRpm = pollenShooter.getVelocity() * 60.0 / 28.0;
        pollenFilteredRPM += SHOOTER_RPM_FILTER_ALPHA * (rawRpm - pollenFilteredRPM);
    }

    private boolean RPM_OK() {
        double commandedRpm = pollenShooterSpeed * 60.0 / 28.0;
        double tolerance = Math.max(commandedRpm * RPM_TOLERANCE_PERCENT, RPM_TOLERANCE_MIN_FLOOR);
        boolean inBand = Math.abs(pollenFilteredRPM - commandedRpm) < tolerance;

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
                shooterOn = true;
                advanceStep();
                break;

            case 1:
                // Shoot from the start position
                if (runShot(nowMs, elapsedInStep)) {
                    jumpToStep(3);
                }
                break;

            case 3:
                // Done shooting - bar to RUNNING for driving around
                flowerBar.setPosition(flowerBarRunningPosition);
                if (moveToWaypoint(autoStartHeadingDegrees, autoStartXInches, autoLeaveYInches - 21.0, 8000, true, POSITION_TOLERANCE_INCHES, HEADING_TOLERANCE_DEGREES)) {
                    advanceStep();
                }
                if (elapsedInStep >= 10000.0) {
                    stopAllDriveMotors(false);
                    advanceStep();
                }
                break;
            case 4:
                if (moveToWaypoint(90.0, autoStartXInches-3, autoLeaveYInches - 21.0, 8000, true, POSITION_TOLERANCE_INCHES, HEADING_TOLERANCE_DEGREES)) {
                    advanceStep();
                }
                if (elapsedInStep >= 10000.0) {
                    stopAllDriveMotors(false);
                    advanceStep();
                }
                break;

            // ---- Flower pollen retrieval ----
            case 5:
                // Flower approach point: FLOWER_APPROACH_OFFSET_INCHES short (-Y) of the flower,
                // facing the flower. Precise full stop, so the drive into the flower is straight in.
                if (moveToWaypoint(flowerHeadingDegrees, flowerXInches, flowerYInches - FLOWER_APPROACH_OFFSET_INCHES, 5000, true, PRECISE_POSITION_TOLERANCE_INCHES, PRECISE_HEADING_TOLERANCE_DEGREES)) {
                    advanceStep();
                }
                if (elapsedInStep >= 7000.0) {
                    stopAllDriveMotors(false);
                    advanceStep();
                }
                break;

            case 6:
                // Just before the flower entry move: open both flippers, bar to GATHERING, intake on.
                // Wait for the bar to finish moving before driving into the flower.
                rightFlipper.setPosition(flipperRightOpenPosition);
                leftFlipper.setPosition(flipperLeftOpenPosition);
                flowerBar.setPosition(flowerBarGatheringPosition);
                intake.setPower(1.0);
                if (elapsedInStep >= FLOWER_BAR_MOVE_MS) {
                    advanceStep();
                }
                break;

            case 7:
                // Drive into the flower
                if (moveToWaypoint(flowerHeadingDegrees, flowerXInches, flowerYInches, 8000, true, PRECISE_POSITION_TOLERANCE_INCHES, PRECISE_HEADING_TOLERANCE_DEGREES)) {
                    advanceStep();
                }
                if (elapsedInStep >= 10000.0) {
                    stopAllDriveMotors(false);
                    advanceStep();
                }
                break;

            case 8:
                // Pause in the flower (bar GATHERING, intake on)
                if (elapsedInStep >= FLOWER_GATHER_DWELL_MS) {
                    advanceStep();
                }
                break;

            case 9:
                // Stow both flippers. Intake stays on while driving out of the flower.
                rightFlipper.setPosition(flipperRightStowedPosition);
                leftFlipper.setPosition(flipperLeftStowedPosition);
                advanceStep();
                break;

            case 10:
                // Back away from the flower (robot faces +Y, so back up toward -Y).
                // Once out: intake off, bar to STOWED (no more intake this auto).
                if (moveToWaypoint(flowerHeadingDegrees, flowerXInches, flowerYInches - FLOWER_APPROACH_OFFSET_INCHES, 3000, true, POSITION_TOLERANCE_INCHES, HEADING_TOLERANCE_DEGREES)) {
                    intake.setPower(0.0);
                    flowerBar.setPosition(flowerBarStowedPosition);
                    advanceStep();
                }
                if (elapsedInStep >= 5000.0) {
                    stopAllDriveMotors(false);
                    intake.setPower(0.0);
                    flowerBar.setPosition(flowerBarStowedPosition);
                    advanceStep();
                }
                break;

            case 11:
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

    /**
     * Updates targetDistanceInches from the current Pinpoint pose.
     */
    private void updateTargetDistance() {
        Pose2D pose = pinpoint.getPosition();
        calculateHeadingForLauncherAim(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.RADIANS));
    }

    /**
     * Recomputes the pollen shooter speed from distance to goal and sends it to the motor
     * while the shooter is on (0 when off).
     */
    private void updateShooterSpeed() {
        updateTargetDistance();
        // Distance used for RPM is capped at SHOOTER_RPM_MAX_DISTANCE_INCHES
        double rpmDistanceInches = Math.min(targetDistanceInches, SHOOTER_RPM_MAX_DISTANCE_INCHES);
        double pollenShooterRPM = pollenShooterRPMGain * pollenShooterRPMSlope * rpmDistanceInches + pollenShooterRPMOffset;
        pollenShooterSpeed = (int) (pollenShooterRPM * 28 / 60); // RPM converted to ticks/sec
        if (shooterOn) {
            pollenShooter.setVelocity(pollenShooterSpeed);
        } else {
            pollenShooter.setVelocity(0);
        }
    }

    /**
     * Calculates heading in radians so the rearward-facing ball launcher exit point
     * aims directly at the center of the nearest hive opening.
     * Also updates targetDistanceInches. (Same as Match3LimeLightTeleopV3.)
     */
    private double calculateHeadingForLauncherAim(double robotX, double robotY, double robotHeading) {
        // Transform launcher exit offset (match2 robot was -9.0 in X, -4.0 in Y) into field coordinates
        double launcherX = robotX + laucherToRobotCenterX * Math.cos(robotHeading) - laucherToRobotCenterY * Math.sin(robotHeading);
        double launcherY = robotY + laucherToRobotCenterX * Math.sin(robotHeading) + laucherToRobotCenterY * Math.cos(robotHeading);

        // Distance to Hive 1 (-12.75, -16.84)
        double hive1X = -12.75;
        double hive1Y = -16.84;
        double dx1 = launcherX - hive1X;
        double dy1 = launcherY - hive1Y;
        double dist1Sq = dx1 * dx1 + dy1 * dy1;

        // Distance to Hive 2 (-12.75, +16.84)
        double hive2X = -12.75;
        double hive2Y = 16.84;
        double dx2 = launcherX - hive2X;
        double dy2 = launcherY - hive2Y;
        double dist2Sq = dx2 * dx2 + dy2 * dy2;

        // Pick whichever hive is closer to launcher
        double bestHiveX = hive1X;
        double bestHiveY = hive1Y;
        double bestDistSq = dist1Sq;
        if (dist2Sq < dist1Sq) {
            bestHiveX = hive2X;
            bestHiveY = hive2Y;
            bestDistSq = dist2Sq;
        }

        // Update global target distance to hive opening (in inches)
        targetDistanceInches = Math.sqrt(bestDistSq);

        // Heading so rearward launcher (-X_robot) points at target:
        return Math.atan2(launcherY - bestHiveY, launcherX - bestHiveX);
    }

    /**
     * Runs one shot - call once per loop while shooting. Opens the gate as soon as RPM is OK
     * (or after SHOOT_MAX_RPM_WAIT_MS regardless), keeps it open for SHOOT_GATE_OPEN_MS,
     * then closes it and turns the shooter off.
     * @return true once the shot is finished
     */
    private boolean runShot(double nowMs, double elapsedInStep) {
        if (gateOpenedAtMs < 0) {
            if (rpmReady || elapsedInStep >= SHOOT_MAX_RPM_WAIT_MS) {
                pollenGate.setPosition(pollenGateOpen);
                gateOpenedAtMs = nowMs;
            }
            return false;
        }
        if (nowMs - gateOpenedAtMs >= SHOOT_GATE_OPEN_MS) {
            pollenGate.setPosition(pollenGateClosed);
            shooterOn = false;
            gateOpenedAtMs = -1; // ready for the next shot
            return true;
        }
        return false;
    }

    // Jump to any step number
    private void jumpToStep(int step) {
        autoStep = step;
        stepStartTimeMs = -1;
    }

    private void advanceStep() {
        autoStep++;
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
