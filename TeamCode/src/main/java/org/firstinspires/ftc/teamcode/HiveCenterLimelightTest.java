package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/*
 * LIMELIGHT WEB UI PIPELINE CONFIGURATION & COORDINATE SYSTEMS
 * ====================================================================================
 * 1. CAMERA-TO-LAUNCHER OFFSET ("3D / Robot Offset" tab in Limelight Web UI):
 *    - Origin (0,0,0): The physical Ball Launcher Exit Point on your robot.
 *    - +X Axis: Forward distance from launcher exit to camera lens (meters or inches).
 *    - +Y Axis: Left distance from launcher exit to camera lens.
 *    - +Z Axis: Up distance from launcher exit to camera lens.
 *    - Roll / Pitch / Yaw: Rotation of camera lens relative to launcher exit orientation.
 *    -> Result: Limelight treats the Ball Launcher Exit Point as the 3D origin (0,0,0).
 *
 * 2. APRILTAG LAYOUT RELATIVE TO HIVE OPENING CENTER ("AprilTags" tab in Limelight Web UI):
 *    - Origin (0,0,0): The Hive Opening Center itself!
 *    - Each tag's 3D offset (X, Y, Z) is entered directly relative to the Hive Opening Center:
 *      * Tag L1 (IDs 30, 34, 38, 42): X = -6.50", Y = +7.1874", Z = -5.622"
 *      * Tag L2 (IDs 31, 35, 39, 43): X = -2.75", Y = +7.1874", Z = -5.622"
 *      * Tag R1 (IDs 32, 36, 40, 44): X = +2.75", Y = +7.1874", Z = -5.622"
 *      * Tag R2 (IDs 33, 37, 41, 45): X = +6.50", Y = +7.1874", Z = -5.622"
 *    -> Result: getBotpose() returns the 3D target pose directly from Launcher Exit
 *               to Hive Opening Center!
 *
 * 3. HIVE CLUSTER IDs (Tag size = 3.25"):
 *    - RED SCORING Cluster:   IDs 30 - 33
 *    - RED AUDIENCE Cluster:  IDs 34 - 37
 *    - BLUE AUDIENCE Cluster: IDs 38 - 41
 *    - BLUE SCORING Cluster:  IDs 42 - 45
 *
 * 4. YAW STEERING DIRECTION CONVENTION:
 *    - POSITIVE (+yaw): Target is offset to the LEFT.
 *      -> Turn robot CCW (Counter-Clockwise) to align with target.
 *    - NEGATIVE (-yaw): Target is offset to the RIGHT.
 *      -> Turn robot CW (Clockwise) to align with target.
 * ====================================================================================
 */

@TeleOp(name = "HiveTarget Limelight Test", group = "TeleOp")
public class HiveCenterLimelightTest extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Initialize the Limelight hardware mapped as "limelight" in your configuration
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Switch to Pipeline 0 (Ensure Pipeline 0 is set to AprilTag in the Limelight web UI)
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Limelight Initialized. Ready for Match/Test.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // 2D targeting screen errors
                double tx = result.getTx(); // Horizontal error (degrees)
                double ty = result.getTy(); // Vertical error (degrees)

                telemetry.addData("Target Status", "LOCKED");
                telemetry.addData("Bearing Error (tx)", "%.2f deg", tx);
                telemetry.addData("Elevation Error (ty)", "%.2f deg", ty);
                telemetry.addData("Tags Detected", result.getBotposeTagCount());

                // 3D pose between launcher exit and hive center (as configured in Limelight web dashboard)
                Pose3D targetPose = result.getBotpose();

                if (targetPose != null) {
                    double x = targetPose.getPosition().x; // Forward distance (meters)
                    double y = targetPose.getPosition().y; // Lateral distance (meters)
                    double z = targetPose.getPosition().z; // Vertical distance (meters)

                    // Calculate horizontal ground distance from launcher exit to hive center
                    double horizontalDistanceMeters = Math.hypot(x, y);

                    // Yaw angle in degrees of target relative to launcher orientation:
                    //   - POSITIVE (+yaw): Target is offset to the LEFT -> Robot must turn CCW (Counter-Clockwise) to align
                    //   - NEGATIVE (-yaw): Target is offset to the RIGHT -> Robot must turn CW (Clockwise) to align
                    double yawDegrees = targetPose.getOrientation().getYaw(AngleUnit.DEGREES);

                    telemetry.addData("--- Launcher Metrics ---", "");
                    telemetry.addData("Horizontal Distance", "%.2f meters (%.1f inches)", horizontalDistanceMeters, horizontalDistanceMeters * 39.3701);
                    telemetry.addData("Forward (X)", "%.2f m", x);
                    telemetry.addData("Lateral (Y)", "%.2f m", y);
                    telemetry.addData("Vertical (Z)", "%.2f m", z);
                    telemetry.addData("Target Yaw", "%.2f deg", yawDegrees);
                } else {
                    telemetry.addData("3D Pose", "Not Solved Yet");
                }
            } else {
                telemetry.addData("Target Status", "Searching for Hive Tags...");
            }

            telemetry.update();
        }
    }
}
