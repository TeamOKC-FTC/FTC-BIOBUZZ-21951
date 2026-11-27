package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Match3FlowerBarTunerV3
 * --------------------------------------------------------------------------------
 * Tuning OpMode for the V3 robot flower bar (servo "flowerBar"). No driving.
 * Use it to find the STOWED, RUNNING, and GATHERING servo positions, then copy the
 * values shown in telemetry into the flowerBar...Position constants of the Match3 V3 files.
 *
 * gamepad1:
 *   Y            select STOWED and move there
 *   B            select RUNNING and move there
 *   A            select GATHERING and move there
 *   dpad up/down     selected position +0.01 / -0.01 (fine)
 *   dpad right/left  selected position +0.05 / -0.05 (coarse)
 *   right trigger    run the intake (check the bar clears it in RUNNING and GATHERING)
 *
 * All three positions start at 0.5, so the servo moves to center when START is pressed.
 * --------------------------------------------------------------------------------
 */

@TeleOp(name="Match3FlowerBarTunerV3", group="Linear OpMode")
public class Match3FlowerBarTunerV3 extends LinearOpMode {

    private Servo   flowerBar = null;
    private DcMotor intake    = null;
    private ElapsedTime runtime = new ElapsedTime();

    // Positions being tuned (index 0 = STOWED, 1 = RUNNING, 2 = GATHERING)
    private final double[] positions = {0.5, 0.5, 0.5};
    private final String[] names = {"STOWED", "RUNNING", "GATHERING"};
    private int selected = 0;

    private static final double FINE_STEP = 0.01;
    private static final double COARSE_STEP = 0.05;

    // Button Indices
    private final int BTN_Y = 0;
    private final int BTN_B = 1;
    private final int BTN_A = 2;
    private final int BTN_DPAD_UP = 3;
    private final int BTN_DPAD_DOWN = 4;
    private final int BTN_DPAD_RIGHT = 5;
    private final int BTN_DPAD_LEFT = 6;
    private final double[] buttonLastPressTime = new double[7];
    private final boolean[] buttonLastState = new boolean[7];

    @Override
    public void runOpMode() {
        flowerBar = hardwareMap.get(Servo.class, "flowerBar");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flowerBar.setDirection(Servo.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized - bar moves to 0.5 (center) on START");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Select which position to tune
            if (isPressed(gamepad1.y, BTN_Y)) selected = 0;
            if (isPressed(gamepad1.b, BTN_B)) selected = 1;
            if (isPressed(gamepad1.a, BTN_A)) selected = 2;

            // Adjust the selected position
            if (isPressed(gamepad1.dpad_up, BTN_DPAD_UP))       positions[selected] += FINE_STEP;
            if (isPressed(gamepad1.dpad_down, BTN_DPAD_DOWN))   positions[selected] -= FINE_STEP;
            if (isPressed(gamepad1.dpad_right, BTN_DPAD_RIGHT)) positions[selected] += COARSE_STEP;
            if (isPressed(gamepad1.dpad_left, BTN_DPAD_LEFT))   positions[selected] -= COARSE_STEP;
            positions[selected] = Math.max(0.0, Math.min(1.0, positions[selected]));

            // Always hold the bar at the selected position
            flowerBar.setPosition(positions[selected]);

            // Intake test
            intake.setPower(gamepad1.right_trigger > 0.2 ? gamepad1.right_trigger : 0.0);

            telemetry.addData("Selected", names[selected]);
            telemetry.addData("Servo position", "%.2f", positions[selected]);
            telemetry.addLine();
            telemetry.addData("flowerBarStowedPosition",    "%.2f", positions[0]);
            telemetry.addData("flowerBarRunningPosition",   "%.2f", positions[1]);
            telemetry.addData("flowerBarGatheringPosition", "%.2f", positions[2]);
            telemetry.addLine();
            telemetry.addLine("Y=stowed  B=running  A=gathering");
            telemetry.addLine("dpad up/down +-0.01   right/left +-0.05");
            telemetry.addLine("right trigger = intake");
            telemetry.update();
        }
    }

    // Debounce function for button presses - fires once per press
    private boolean isPressed(boolean currentState, int buttonIndex) {
        double now = runtime.milliseconds();
        boolean pressed = false;
        if (currentState && !buttonLastState[buttonIndex]
                && (now - buttonLastPressTime[buttonIndex]) > 100) {
            pressed = true;
            buttonLastPressTime[buttonIndex] = now;
        }
        buttonLastState[buttonIndex] = currentState;
        return pressed;
    }
}
