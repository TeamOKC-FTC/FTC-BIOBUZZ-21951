package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Shared static storage to persist robot pose between Autonomous and TeleOp.
 */
public class PoseStorage {
    public static Pose2D currentPose = new Pose2D(DistanceUnit.INCH, -9.5, -62.0, AngleUnit.DEGREES, -90.0);
    public static boolean hasValidPose = false;
    public static String allianceColor = "NONE"; // "RED", "BLUE", or "NONE" (no good camera data during Autonomous INIT)
}
