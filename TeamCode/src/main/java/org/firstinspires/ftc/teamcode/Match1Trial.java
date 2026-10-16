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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * ROBOT OPERATOR CONTROLS (Gamepad 1):
 * --------------------------------------------------------------------------------
 * DRIVETRAIN (Mecanum Drive):
 *   - Left Stick Y:        Drive Forward / Backward
 *   - Left Stick X:        Strafe Left / Right
 *   - Right Stick X:       Rotate Clockwise / Counter-Clockwise
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

@TeleOp(name="Match1Trial", group="Linear OpMode")
//@Disabled
public class Match1Trial extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intake         = null;
    private DcMotorEx shooter        =null;
    private Servo     rightFlipper   =null;
    private Servo     leftFlipper    =null;
    private Servo     gate           =null;
    private int shooterSpeed=2500*28/60;//RPM
    private boolean shooteron=false;
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


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
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

        //rightFlipper.setPosition(1.0);  //stowed
        //leftFlipper.setPosition(1.0);   //stowed
        gate.setPosition(1.0);          //blocked

        shooter.setVelocityPIDFCoefficients(350, 0, 0, 12.53);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            if (gamepad1.right_trigger > .2) {
                intake.setPower(gamepad1.right_trigger);
            } else{
                intake.setPower(-gamepad1.left_trigger);
            }
            
            if (isPressed(gamepad1.dpad_up, BTN_DPAD_UP)) {
                shooterSpeed = shooterSpeed + 25*28/60;  //25 rpm steps
                if (shooteron) {
                    shooter.setVelocity(shooterSpeed);
                }
            }
            
            if (isPressed(gamepad1.dpad_down, BTN_DPAD_DOWN)) {
                shooterSpeed = shooterSpeed - 25*28/60;  //25 rpm steps
                if (shooteron) {
                    shooter.setVelocity(shooterSpeed);
                }
            }

            if (isPressed(gamepad1.y, BTN_Y)) {
                if (!shooteron) {
                    shooter.setVelocity(shooterSpeed);
                    shooteron = true;
                } else {
                    shooter.setVelocity(0);
                    shooteron = false;
                }
            }
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
                    leftFlipper.setPosition(0.0);//right
                    flipperleftstowed = false;
                } else {
                    leftFlipper.setPosition(1.0);//left
                    flipperleftstowed = true;
                }
            }

            if (isPressed(gamepad1.x, BTN_X)) {
                if (gateBlocked) {
                    gate.setPosition(0.0);//unblock
                     gateBlocked= false;
                } else {
                    gate.setPosition(1.0);//block
                    gateBlocked = true;
                }
            }

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);


            telemetry.addData("right front power=", frontRightPower);
            telemetry.addData("left front power=", frontLeftPower);
            telemetry.addData("right back power=", backRightPower);
            telemetry.addData("left back power=", backLeftPower);
            if (shooteron) {
                telemetry.addData("RPMcommand=", shooterSpeed * 60 / 28);
            } else {
                telemetry.addData("RPMcommand=", 0);
            }
            telemetry.addData("RPMActual=", shooter.getVelocity() * 60 / 28);
            telemetry.update();
        }
    }

    //the following function debounces switches and requires a delay between presses to prevent issues with chattering switches
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

}
