/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
 * Match1Autonomous
 * -----------------------------------------------------------------------------
 * First autonomous OpMode for this robot. Hardware setup, Pinpoint field-coordinate
 * convention, and heading-correction technique are all carried over from the proven
 * Match1FieldDrive teleop (see that file for the driver-facing control scheme). This
 * file does NOT modify Match1FieldDrive.java - everything needed is duplicated here
 * so the two OpModes stay fully independent.
 *
 * runAutonomousSequence() is a non-blocking step machine built from moveToWaypoint(...),
 * RPM_OK(), and direct shooter/gate commands, with hardcoded waypoints for this start position.
 * -----------------------------------------------------------------------------
 */

@Autonomous(name="Match1Start1AutonomousV2", group="Linear OpMode", preselectTeleOp="Match1Start1TeleopV2")
public class Match1Start1AutonomousV2 extends LinearOpMode {

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

    private int shooterSpeed = 2500 * 28 / 60; // RPM converted to ticks/sec
    private double filteredShooterRpm = 0;
    private static final double SHOOTER_RPM_FILTER_ALPHA = 0.15; // lower = smoother/slower to settle

    // Pinpoint Pod Offsets (in Inches)
    private double xPodOffsetInches = -3.600; // X pod offset + is left sideways from robot center (-84 mm)
    private double yPodOffsetInches = -(17.0/2.0-2.0); // Y pod offset + is forward from robot center (-168 mm)

    // Starting Match Pose
    // TODO: set the real autonomous starting pose once the field starting position for this match is decided.
    private double autoStartXInches = -9.5;
    private double autoStartYInches =  -71.0+9.0;
    private double autoStartHeadingDegrees = -90.0; // 0 deg = Facing Forward away from driver

    private double autoLeaveXInches = -71.0+5.0+9.0;
    private double autoLeaveYInches =  24.0+18.0+9.0;
    private double autoLeaveHeadingDegrees = -90.0; // 0 deg = Facing Forward away from driver

    // Heading correction tuning - copied verbatim from Match1FieldDrive's base (non-aggression)
    // curve. Autonomous has no joystick, so the stick-deflection "aggression" scaling used in
    // teleop does not apply here; moveToWaypoint always uses this base curve.
    private static final double MIN_TURN_POWER = 0.17; // measured minimum power to overcome drivetrain static friction
    private static final double MAX_TURN_POWER = 0.4;  // correction power ceiling
    private static final double HEADING_KP = 1.43;     // tuned so ~10 deg error yields ~0.25 power

    // Axial (forward/back) control tuning for moveToWaypoint.
    private static final double AXIAL_KP = 0.027;          // TUNE ME - power per inch of forward/back error
    private static final double AXIAL_MIN_POWER = 0.12;    // TUNE ME - floor so the last few inches don't crawl (was 0.08)
    private static final double AXIAL_MAX_POWER = 0.6;     // TUNE ME - power ceiling

    // Strafe (left/right) control tuning for moveToWaypoint. Mecanum strafing needs more power
    // than driving forward to move the same distance, so these start higher than the axial ones.
    private static final double STRAFE_KP = 0.035;         // TUNE ME - power per inch of left/right error
    private static final double STRAFE_MIN_POWER = 0.16;   // TUNE ME - floor so the last few inches don't crawl
    private static final double STRAFE_MAX_POWER = 0.6;    // TUNE ME - power ceiling

    // Move tolerances - a move is done once inside BOTH the position and heading tolerance.
    // Normal moves use the looser pair; moves that must be accurate (park, shooting position) use the PRECISE pair.
    private static final double POSITION_TOLERANCE_INCHES = 1.5;       // TUNE ME
    private static final double HEADING_TOLERANCE_DEGREES = 2.0;       // TUNE ME
    private static final double PRECISE_POSITION_TOLERANCE_INCHES = 1.0; // accurate moves: within 1 inch
    private static final double PRECISE_HEADING_TOLERANCE_DEGREES = 1.5; // accurate moves: within 1.5 degrees

    // Zero-power bands. Power is cut to zero (motors BRAKE) only once the error is inside this
    // fraction of the tolerance, so the slide after braking still ends inside the tolerance.
    // Must stay below 0.7 - axial and strafe each stop at this fraction, so if both stop right at
    // the edge the total distance is 1.41x the band, and that must still be inside the tolerance.
    private static final double ZERO_POWER_BAND_FRACTION = 0.5; // TUNE ME

    // "Fully stopped" thresholds used when moveToWaypoint's requireFullStop argument is true.
    private static final double STOP_VELOCITY_THRESHOLD_INCHES_PER_SEC = 2.5;      // TUNE ME
    private static final double STOP_ANGULAR_VELOCITY_THRESHOLD_DEG_PER_SEC = 8.0; // TUNE ME
    private static final double SETTLE_DWELL_MS = 150; // TUNE ME - must stay stopped this long before declaring done

    // RPM_OK tuning - how close to commanded RPM, and for how long, before launch is OK.
    private static final double RPM_TOLERANCE_PERCENT = 0.02; // TUNE ME - +/-2% of commanded RPM
    private static final double RPM_TOLERANCE_MIN_FLOOR = 20; // TUNE ME - absolute RPM floor so band isn't tiny near 0
    private static final double RPM_OK_DWELL_MS = 1000;        // TUNE ME
    private double rpmInBandSinceMs = -1; // -1 = not currently in band

    // moveToWaypoint non-blocking state: identifies the "active" commanded move so repeated
    // calls with the same arguments are recognized as "continuing" the same move, while a call
    // with different arguments (or no active move yet) starts a fresh one.
    private boolean moveActive = false;
    private double activeTargetHeadingDeg;
    private double activeTargetXInches;
    private double activeTargetYInches;
    private double activeTimeoutMs;
    private boolean activeRequireFullStop;
    private double activePositionToleranceInches;
    private double activeHeadingToleranceDegrees;
    private double moveStartTimeMs;        // runtime ms when the active move began
    private double settleStartTimeMs = -1; // runtime ms when tolerance+velocity first satisfied; -1 = not currently settled

    // Autonomous sequencer state.
    private int autoStep = 0;
    private double stepStartTimeMs = -1; // -1 = not yet set for the current step


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

        // Display repeating position telemetry during INIT phase until driver presses START
        while (!isStarted() && !isStopRequested()) {
            pinpoint.update();
            updateTelemetry("Initialized - Waiting for Start");
            sleep(20);
        }

        waitForStart();
        runtime.reset();

        // Run the autonomous sequence until it completes or the match period ends
        while (opModeIsActive()) {
            pinpoint.update();

            runAutonomousSequence();

            updateTelemetry("Running");
        }

        stopAllDriveMotors(true);
    }

    /**
     * Updates telemetry data for Pinpoint pose, shooter speed, and status. Does NOT update
     * filteredShooterRpm - RPM_OK() owns that filter update so it is only applied once per tick.
     * @param status Current status message
     */
    private void updateTelemetry(String status) {
        Pose2D pose = pinpoint.getPosition();
        telemetry.addData("Status", status);
        telemetry.addData("X (in)", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y (in)", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Auto Step", autoStep);
        telemetry.addData("RPMcommand=", shooterSpeed * 60 / 28);
        telemetry.addData("RPMActual=", filteredShooterRpm);
        telemetry.update();
    }

    /**
     * Sets all four drive motor powers to zero, and optionally the intake.
     */
    private void stopAllDriveMotors(boolean intakeAlsoStop) {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        if (intakeAlsoStop) {
            intake.setPower(0);
        }
    }

    /**
     * Base heading-correction curve, copied from Match1FieldDrive's driveFieldOriented(). Unlike
     * teleop there is no joystick-deflection "aggression" scaling here - autonomous always uses
     * this fixed curve, fed by a commanded target heading instead of a stick-derived one.
     * @param headingErrorRad normalized (target - current) heading error, radians
     * @param zeroPowerBandDegrees inside this much error, return 0 power so the motors brake
     * @return signed yaw motor power contribution
     */
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

    /**
     * Proportional power for one drive direction (axial or strafe) with a floor and a zero-power
     * band. Returns 0.0 inside the zero-power band so the motors brake. There is no ceiling here -
     * moveToWaypoint scales axial and strafe down TOGETHER so the robot keeps driving straight.
     * @param errorInches         signed position error in this direction, inches
     * @param kp                  power per inch of error
     * @param minPower            floor so small errors still move the robot
     * @param zeroPowerBandInches inside this much error, return 0 power so the motors brake
     * @return signed motor power for this direction
     */
    private double computeAxisPower(double errorInches, double kp, double minPower,
                                    double zeroPowerBandInches) {
        if (Math.abs(errorInches) < zeroPowerBandInches) {
            return 0.0; // inside zero-power band: stop driving this direction so the motors brake
        }
        double power = errorInches * kp;
        if (Math.abs(power) < minPower) {
            power = Math.signum(errorInches) * minPower;
        }
        return power;
    }

    /**
     * Non-blocking drive-to-waypoint. Call this once per main loop iteration with the SAME
     * arguments every call until it returns true (position+heading tolerance met - and, if
     * requireFullStop is true, the robot has also settled to near-zero velocity - or the
     * timeout has elapsed). Drives the mecanum drivetrain directly each call it is not yet done.
     *
     * Equality between calls is checked with primitive == on the double/boolean arguments. This
     * is intentional, not a bug: callers are expected to pass the same literal/field values on
     * every call for a given step, so exact equality reliably detects "same move in progress" vs.
     * "a new move was requested" without needing a fuzzy-equals epsilon.
     *
     * @param targetHeadingDegrees desired absolute field heading, degrees (0 = facing forward away from driver)
     * @param targetXInches        desired absolute field X position, inches
     * @param targetYInches        desired absolute field Y position, inches
     * @param timeoutMs            maximum time to attempt this move before giving up and declaring done
     * @param requireFullStop      if true, the robot must also settle to near-zero velocity before done is declared
     * @param positionToleranceInches done once within this distance of the target X/Y
     * @param headingToleranceDegrees done once within this many degrees of the target heading
     * @return true once the move is finished (reached, or timed out)
     */
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
            // Starting a brand-new move: snapshot the args as "active" and reset per-move timers.
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
                settleStartTimeMs = nowMs; // just entered the settled state
            }
            done = (nowMs - settleStartTimeMs) >= SETTLE_DWELL_MS;
        } else {
            settleStartTimeMs = -1; // fell out of tolerance/velocity band, reset dwell timer
            done = false;
        }

        if (done) {
            stopAllDriveMotors(false);
            moveActive = false;
            return true;
        }

        // Rotate field-frame position error into robot-frame axial/strafe (same convention as
        // Match1FieldDrive's driveFieldOriented), then drive it through P-control + mecanum combine.
        double robotAxial = errX * Math.cos(currentHeadingRad) + errY * Math.sin(currentHeadingRad);
        double robotStrafeLeft = -errX * Math.sin(currentHeadingRad) + errY * Math.cos(currentHeadingRad);

        // Zero-power bands are a fraction of the tolerances, so braking starts inside the
        // tolerance and the slide after braking still ends inside it.
        double translationZeroBand = positionToleranceInches * ZERO_POWER_BAND_FRACTION;
        double headingZeroBand = headingToleranceDegrees * ZERO_POWER_BAND_FRACTION;

        double axialPower  = computeAxisPower(robotAxial,      AXIAL_KP,  AXIAL_MIN_POWER,  translationZeroBand);
        double strafePower = computeAxisPower(robotStrafeLeft, STRAFE_KP, STRAFE_MIN_POWER, translationZeroBand);

        // If either direction is over its max, shrink BOTH by the same factor. This keeps the
        // axial:strafe ratio the same, so the robot drives in a straight line toward the target
        // instead of curving (capping only one direction would change the ratio and the direction).
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

    /**
     * True once the shooter's filtered actual RPM has been within tolerance of the commanded RPM
     * continuously for RPM_OK_DWELL_MS - i.e. it is safe to launch. This is the only place in this
     * file that updates filteredShooterRpm, so call it every loop iteration while the shooter is
     * spinning up or running (e.g. from the sequencer step that is waiting to launch).
     * @return true if the shooter has stabilized at the commanded RPM
     */
    private boolean RPM_OK() {
        double rawRpm = shooter.getVelocity() * 60.0 / 28.0;
        filteredShooterRpm += SHOOTER_RPM_FILTER_ALPHA * (rawRpm - filteredShooterRpm);

        double commandedRpm = shooterSpeed * 60.0 / 28.0;
        double tolerance = Math.max(commandedRpm * RPM_TOLERANCE_PERCENT, RPM_TOLERANCE_MIN_FLOOR);
        boolean inBand = Math.abs(filteredShooterRpm - commandedRpm) < tolerance;

        double nowMs = runtime.milliseconds();
        if (inBand) {
            if (rpmInBandSinceMs < 0) {
                rpmInBandSinceMs = nowMs; // just entered band
            }
            return (nowMs - rpmInBandSinceMs) >= RPM_OK_DWELL_MS;
        } else {
            rpmInBandSinceMs = -1; // fell out of band, reset dwell
            return false;
        }
    }

    /**
     * Non-blocking autonomous step machine. Call once per main loop iteration; advances autoStep
     * internally once the current step's completion condition is met.
     */
    private void runAutonomousSequence() {
        double nowMs = runtime.milliseconds();
        if (stepStartTimeMs < 0) {
            stepStartTimeMs = nowMs; // first call ever, or just entered a new step
        }
        double elapsedInStep = nowMs - stepStartTimeMs;

        switch (autoStep) {
            case 0:
                //start shooter at preset RPM
                shooter.setVelocity(shooterSpeed);
                advanceStep();
                break;

            case 1:
                // after 4 sec or rpm accuracy good then open gate
                if (elapsedInStep >= 4000.0 || RPM_OK() ) {
                    gate.setPosition(0.0);  //open gate
                    advanceStep();
                }
                break;

            case 2:
                // delay 3 seconds for shooting
                if (elapsedInStep >= 3000.0) {
                    gate.setPosition(1.0);  //close gate
                    shooter.setVelocity(0.0);
                    advanceStep();
                }
                break;

            case 3:
                // drive to close to park
                if(moveToWaypoint(autoStartHeadingDegrees, autoStartXInches, autoLeaveYInches-21.0, 8000, true, POSITION_TOLERANCE_INCHES, HEADING_TOLERANCE_DEGREES)){
                    advanceStep();
                }
                if (elapsedInStep >= 10000.0) {
                    stopAllDriveMotors(false);
                    advanceStep();
                }
                break;

            case 4:
                // drive to park
                if(moveToWaypoint(autoLeaveHeadingDegrees, autoLeaveXInches, autoLeaveYInches, 8000, true, PRECISE_POSITION_TOLERANCE_INCHES, PRECISE_HEADING_TOLERANCE_DEGREES)){
                    advanceStep();
                }
                if (elapsedInStep >= 10000.0) {
                    stopAllDriveMotors(false);
                    advanceStep();
                }
                break;

            default:
                // Sequence complete - hold position.
                stopAllDriveMotors(true);
                break;
        }
    }

    /**
     * Advances to the next sequencer step and resets the step-entry timer sentinel so
     * elapsedInStep starts fresh for the new step on the next call.
     */
    private void advanceStep() {
        autoStep++;
        stepStartTimeMs = -1;
    }

    /**
     * Initializes the goBILDA Pinpoint computer with pod offsets, directions, and initial pose.
     * @param xPodOffsetInches X pod offset sideways from center (in inches)
     * @param yPodOffsetInches Y pod offset forward/backward from center (in inches)
     * @param startXInches Initial X coordinate in inches
     * @param startYInches Initial Y coordinate in inches
     * @param startHeadingDegrees Initial heading in degrees
     */
    private void initPinpoint(double xPodOffsetInches, double yPodOffsetInches, double startXInches, double startYInches, double startHeadingDegrees) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Set odometry pod offsets in INCHES relative to center of robot
        pinpoint.setOffsets( 25.4*xPodOffsetInches, 25.4*yPodOffsetInches, DistanceUnit.MM);

        // Set encoder pod model and directions
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // Reset IMU and position, then wait 300ms for onboard IMU calibration to complete
        pinpoint.resetPosAndIMU();
        sleep(500);

        // Assign initial starting pose in INCHES and DEGREES
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startXInches, startYInches, AngleUnit.DEGREES, startHeadingDegrees));
    }
}
