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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * ROBOT OPERATOR CONTROLS (Gamepad 1):
 * --------------------------------------------------------------------------------
 * FIELD-ORIENTED DRIVETRAIN (goBILDA Pinpoint Native Frame + Mecanum Drive):
 *   - Pinpoint Orientation: +X Arrow points FORWARD (away from driver)
 *   - Starting Pose:        (-72.0 in, -18.0 in, 0.0 rad = 0° facing away from driver)
 *
 *   - Left Stick Y:         Field +X Motion (Push UP = Drive Away from Driver)
 *   - Left Stick X:         Field +Y Motion (Push LEFT = Drive to Driver's Left)
 *   - Right Stick Y/X:      Set Absolute Target Yaw Heading
 *                           (UP = 0° Forward, LEFT = +90° Left, DOWN = 180° Back, RIGHT = -90° Right)
 *
 * INTAKE:
 *   - Right Trigger (>0.2): Press to run Intake FORWARD (analog variable speed)
 *   - Left Trigger: (>0.2): Press to run Intake REVERSE / OUTTAKE (analog variable speed)
 *
 * SHOOTER:
 *   - Y Button:            Press to TOGGLE Shooter ON / OFF
 *   - D-Pad Up:            Press to INCREASE target shooter speed
 *   - D-Pad Down:          Press to DECREASE target shooter speed
 *
 * FLIPPERS & GATE:
 *   - Right Bumper:        Press to TOGGLE Right Flipper (Deployed <-> Stowed)
 *   - Left Bumper:         Press to TOGGLE Left Flipper (Deployed <-> Stowed)
 *   - X Button:            Press to TOGGLE Gate (Unblocked <-> Blocked)
 * --------------------------------------------------------------------------------
 */

@TeleOp(name="Match1Start1TeleopV2", group="Linear OpMode")
//@Disabled
public class Match1Start1TeleopV2 extends LinearOpMode {

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
    private boolean shooteron = false;
    private double filteredShooterRpm = 0;
    private static final double SHOOTER_RPM_FILTER_ALPHA = 0.15; // lower = smoother/slower to settle
    private boolean flipperleftstowed = true;
    private boolean flipperrightstowed = true;
    private boolean gateBlocked = true;

    // Button Indices
    private final int BTN_DPAD_UP = 0;
    private final int BTN_DPAD_DOWN = 1;
    private final int BTN_Y = 2;
    private final int BTN_RIGHT_BUMPER = 3;
    private final int BTN_LEFT_BUMPER = 4;
    private final int BTN_X = 5;

    private final double[] buttonLastPressTime = new double[6];
    private final boolean[] buttonLastState = new boolean[6];

    // Pinpoint Pod Offsets (in Inches)
    private double xPodOffsetInches = -3.600; // X pod offset + is left sideways from robot center (-84 mm)
    private double yPodOffsetInches = -(17.0/2.0-2.0); // Y pod offset + is forward from robot center (-168 mm)

    // Starting Match Pose
    private double autoLeaveXInches = -71.0+5.0+9.0;
    private double autoLeaveYInches =  24.0+18.0+9.0;

    private double autoLeaveHeadingDegrees = -90.0; // 0 deg = Facing Forward away from driver

    private double targetHeading = autoLeaveHeadingDegrees * Math.PI / 180; // Initial target heading: 0 rad (0 deg = Facing Forward away from driver)
    private boolean rightStickWasActive = false; // tracks deadzone state last loop, to detect the release edge

    // Heading correction tuning. Base curve (used for holding, and as the floor when the stick
    // is just past the deadzone) matches continuously into the active-steering curve, so
    // releasing the stick only ever drops turn power, never steps it up.
    private static final double MIN_TURN_POWER = 0.17; // measured minimum power to overcome drivetrain static friction
    private static final double MAX_TURN_POWER = 0.4;  // base correction power ceiling, reached by ~16 deg error
    private static final double HEADING_KP = 1.43;     // tuned so ~10 deg error yields ~0.25 power at base aggression
    private static final double HEADING_TOLERANCE_DEGREES = 2.0; // stop correcting once within this residual error
    private static final double MAX_TURN_AGGRESSION = 1.5; // pushing the stick to full deflection scales gain/ceiling up to this factor (reaches full motor power)
    private static final double B_AIM_STICK_EQUIVALENT = 0.75; // while B is held, turn as hard as the right stick pushed this far (0.2 = deadzone edge, 1.0 = full)




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

        //rightFlipper.setPosition(0.0);  // stowed
        //leftFlipper.setPosition(0.0);   // stowed
        gate.setPosition(1.0);          // blocked

        shooter.setVelocityPIDFCoefficients(350, 0, 0, 12.53);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Initialize goBILDA Pinpoint with pod offsets and starting pose (-72 in, -18 in, 0 deg)
        initPinpoint(xPodOffsetInches, yPodOffsetInches, autoLeaveXInches, autoLeaveYInches, autoLeaveHeadingDegrees);

        // Display repeating position telemetry during INIT phase until driver presses START
        while (!isStarted() && !isStopRequested()) {
            pinpoint.update();
            updateTelemetry("Initialized - Waiting for Start");
            sleep(20);
        }

        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Drive the chassis using field-oriented controls
            driveFieldOriented();

            // Intake control
            if (gamepad1.right_trigger > .2) {
                intake.setPower(gamepad1.right_trigger);
            } else {
                intake.setPower(-gamepad1.left_trigger);
            }

            // Shooter speed adjustments
            if (isPressed(gamepad1.dpad_up, BTN_DPAD_UP)) {
                shooterSpeed = shooterSpeed + 25 * 28 / 60;
                if (shooteron) {
                    shooter.setVelocity(shooterSpeed);
                }
            }

            if (isPressed(gamepad1.dpad_down, BTN_DPAD_DOWN)) {
                shooterSpeed = shooterSpeed - 25 * 28 / 60;
                if (shooteron) {
                    shooter.setVelocity(shooterSpeed);
                }
            }

            // Shooter toggle
            if (isPressed(gamepad1.y, BTN_Y)) {
                if (!shooteron) {
                    shooter.setVelocity(shooterSpeed);
                    shooteron = true;
                } else {
                    shooter.setVelocity(0);
                    shooteron = false;
                }
            }

            // Flippers toggle
            if (isPressed(gamepad1.right_bumper, BTN_RIGHT_BUMPER)) {
                if (flipperrightstowed) {
                    rightFlipper.setPosition(0.0);
                    flipperrightstowed = false;
                } else {
                    rightFlipper.setPosition(1.0);
                    flipperrightstowed = true;
                }
            }

            if (isPressed(gamepad1.left_bumper, BTN_LEFT_BUMPER)) {
                if (flipperleftstowed) {
                    leftFlipper.setPosition(0.0);
                    flipperleftstowed = false;
                } else {
                    leftFlipper.setPosition(1.0);
                    flipperleftstowed = true;
                }
            }

            // Gate toggle
            if (isPressed(gamepad1.x, BTN_X)) {
                if (gateBlocked) {
                    gate.setPosition(0.0);
                    gateBlocked = false;
                } else {
                    gate.setPosition(1.0);
                    gateBlocked = true;
                }
            }

            // Telemetry feedback
            updateTelemetry("Running");
        }
    }

    /**
     * Updates telemetry data for Pinpoint pose, target heading, shooter speed, and status.
     * @param status Current status message
     */
    private void updateTelemetry(String status) {
        Pose2D pose = pinpoint.getPosition();
        telemetry.addData("Status", status);
        telemetry.addData("X (in)", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y (in)", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
        telemetry.addData("Heading (rad)", pose.getHeading(AngleUnit.RADIANS));
        if (shooteron) {
            telemetry.addData("RPMcommand=", shooterSpeed * 60 / 28);
        } else {
            telemetry.addData("RPMcommand=", 0);
        }
        double rawRpm = shooter.getVelocity() * 60 / 28;
        filteredShooterRpm += SHOOTER_RPM_FILTER_ALPHA * (rawRpm - filteredShooterRpm);
        telemetry.addData("RPMActual=", filteredShooterRpm);
        telemetry.update();
    }

    /**
     * Handles field-oriented driving calculations and updates chassis motor powers.
     */
    private void driveFieldOriented() {
        // Update Pinpoint odometer & IMU readings
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        double robotHeading = pose.getHeading(AngleUnit.RADIANS);

        // Left stick field-oriented requests
        double fieldForward = -gamepad1.left_stick_y; // +1 = Move in +X field direction (away from driver)
        double fieldLeft = -gamepad1.left_stick_x;    // +1 = Move in +Y field direction (to driver's left)

        // Convert field movement (+X, +Y) to robot-relative frame (axial, strafe)
        double robotAxial = fieldForward * Math.cos(robotHeading) + fieldLeft * Math.sin(robotHeading);
        double robotStrafeLeft = -fieldForward * Math.sin(robotHeading) + fieldLeft * Math.cos(robotHeading);

        // Right stick sets absolute target heading (UP = 0 rad, LEFT = +PI/2, DOWN = PI, RIGHT = -PI/2)
        double rightStickX = gamepad1.right_stick_x;
        double rightStickY = -gamepad1.right_stick_y; // +1 = UP
        double rightStickMag = Math.hypot(rightStickX, rightStickY);

        boolean rightStickActive = rightStickMag > 0.2; // Deadzone check
        if (gamepad1.b) {
            // B depressed: override target heading so rearward ball launcher (-9in, -5in) aims at nearest hive
            targetHeading = calculateHeadingForLauncherAim(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), robotHeading);
            rightStickActive = true;
        } else if (rightStickActive) {
            // Target angle: UP=0, LEFT=+PI/2, DOWN=PI, RIGHT=-PI/2
            targetHeading = Math.atan2(-rightStickX, rightStickY);  //atan2(y,x) is java, but stick is rotated 90 from field coordinates
        } else if (rightStickWasActive) {
            // Stick was just released - hold wherever we are right now instead of continuing
            // to chase whatever heading the stick last pointed at
            targetHeading = robotHeading;
        }
        rightStickWasActive = rightStickActive;

        // Calculate heading error using up-to-date targetHeading
        double headingError = AngleUnit.normalizeRadians(targetHeading - robotHeading);

        // Pushing the stick further past the deadzone commands more aggressive turning: scale
        // gain and power ceiling together from 1x (just past deadzone) up to MAX_TURN_AGGRESSION
        // (full deflection). At the deadzone edge this equals the base (holding) curve exactly,
        // so releasing the stick only ever drops turn power, never steps it up.
        double aggression = 1.0;
        if (rightStickActive) {
            // While B is held, turn as if the stick were pushed B_AIM_STICK_EQUIVALENT of the way
            double stickDeflection = gamepad1.b ? B_AIM_STICK_EQUIVALENT : rightStickMag;
            double t = (stickDeflection - 0.2) / 0.8; // 0 at deadzone edge, 1 at full deflection
            aggression = 1.0 + t * (MAX_TURN_AGGRESSION - 1.0);
        }
        double effectiveKp = HEADING_KP * aggression;
        double effectiveMaxTurnPower = MAX_TURN_POWER * aggression;

        double yaw;
        if (Math.abs(Math.toDegrees(headingError)) < HEADING_TOLERANCE_DEGREES) {
            // Close enough - stop correcting so we don't dither around the target
            yaw = 0;
        } else {
            double rawYaw = headingError * effectiveKp;
            rawYaw = Math.max(-effectiveMaxTurnPower, Math.min(effectiveMaxTurnPower, rawYaw));
            // Enforce the minimum power needed to overcome drivetrain friction, so small
            // errors don't stall out before reaching the tolerance band
            if (Math.abs(rawYaw) < MIN_TURN_POWER) {
                rawYaw = Math.signum(headingError) * MIN_TURN_POWER;
            }
            yaw = rawYaw;
        }

        // Combine joystick requests for Mecanum wheel powers
        double frontLeftPower = robotAxial - robotStrafeLeft - yaw;
        double frontRightPower = robotAxial + robotStrafeLeft + yaw;
        double backLeftPower = robotAxial + robotStrafeLeft - yaw;
        double backRightPower = robotAxial - robotStrafeLeft + yaw;

        // Normalize powers so no motor exceeds 100%
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send power to drive motors
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
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

    // Debounce function for button toggles
    private boolean isPressed(boolean currentState, int buttonIndex) {
        double now = runtime.milliseconds();
        if (currentState && !buttonLastState[buttonIndex] && (now - buttonLastPressTime[buttonIndex] > 250)) {
            buttonLastState[buttonIndex] = true;
            buttonLastPressTime[buttonIndex] = now;
            return true;
        }
        if (!currentState) {
            buttonLastState[buttonIndex] = false;
        }
        return false;
    }

    /**
     * Calculates heading in radians so the rearward-facing ball launcher exit point (-9in X, -5in Y)
     * aims directly at the center of the nearest hive opening.
     */
    private double calculateHeadingForLauncherAim(double robotX, double robotY, double robotHeading) {
        // Transform launcher exit offset (-9.0 in X, -5.0 in Y) into field coordinates
        double launcherX = robotX - 9.0 * Math.cos(robotHeading) + 5.0 * Math.sin(robotHeading);
        double launcherY = robotY - 9.0 * Math.sin(robotHeading) - 5.0 * Math.cos(robotHeading);

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
        if (dist2Sq < dist1Sq) {
            bestHiveX = hive2X;
            bestHiveY = hive2Y;
        }

        // Heading so rearward launcher (-X_robot) points at target:
        return Math.atan2(launcherY - bestHiveY, launcherX - bestHiveX);
    }
}
