package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class Match1TrialTest {

    /**
     * Test helper class simulating the servo toggle logic used in Match1Trial for bumper controls.
     */
    static class MockFlipperController {
        boolean leftFlipperStowed = true;
        boolean rightFlipperStowed = true;
        boolean leftBumperLastPressed = false;
        boolean rightBumperLastPressed = false;

        double leftFlipperPosition = 0.0;
        double rightFlipperPosition = 0.0;

        public void updateLeftBumper(boolean leftBumperPressed) {
            if (!leftBumperPressed) {
                leftBumperLastPressed = false;
            }
            if (leftBumperPressed && !leftBumperLastPressed) {
                leftBumperLastPressed = true;
                if (leftFlipperStowed) {
                    leftFlipperPosition = 1.0; // turn right
                    leftFlipperStowed = false;
                } else {
                    leftFlipperPosition = 0.0; // turn left
                    leftFlipperStowed = true;
                }
            }
        }

        public void updateRightBumper(boolean rightBumperPressed) {
            if (!rightBumperPressed) {
                rightBumperLastPressed = false;
            }
            if (rightBumperPressed && !rightBumperLastPressed) {
                rightBumperLastPressed = true;
                if (rightFlipperStowed) {
                    rightFlipperPosition = 1.0; // turn right
                    rightFlipperStowed = false;
                } else {
                    rightFlipperPosition = 0.0; // turn left
                    rightFlipperStowed = true;
                }
            }
        }
    }

    @Test
    public void testLeftBumperTogglesServo() {
        MockFlipperController controller = new MockFlipperController();

        // Initial state
        assertTrue("Initially left flipper should be stowed", controller.leftFlipperStowed);
        assertEquals("Initial position should be 0.0", 0.0, controller.leftFlipperPosition, 0.001);

        // Press left bumper
        controller.updateLeftBumper(true);
        assertFalse("Left flipper should not be stowed after first toggle", controller.leftFlipperStowed);
        assertEquals("Left flipper should move to position 1.0 (turned right)", 1.0, controller.leftFlipperPosition, 0.001);

        // Continue holding left bumper (should not re-toggle)
        controller.updateLeftBumper(true);
        assertFalse("Left flipper state should remain unchanged while holding bumper", controller.leftFlipperStowed);
        assertEquals("Left flipper position should remain 1.0", 1.0, controller.leftFlipperPosition, 0.001);

        // Release left bumper
        controller.updateLeftBumper(false);

        // Press left bumper second time (toggle back to left / 0.0)
        controller.updateLeftBumper(true);
        assertTrue("Left flipper should be stowed after second toggle", controller.leftFlipperStowed);
        assertEquals("Left flipper should move to position 0.0 (turned left)", 0.0, controller.leftFlipperPosition, 0.001);

        // Release left bumper
        controller.updateLeftBumper(false);
    }

    @Test
    public void testRightBumperTogglesServo() {
        MockFlipperController controller = new MockFlipperController();

        // Initial state
        assertTrue("Initially right flipper should be stowed", controller.rightFlipperStowed);
        assertEquals("Initial position should be 0.0", 0.0, controller.rightFlipperPosition, 0.001);

        // Press right bumper
        controller.updateRightBumper(true);
        assertFalse("Right flipper should not be stowed after first toggle", controller.rightFlipperStowed);
        assertEquals("Right flipper should move to position 1.0 (turned right)", 1.0, controller.rightFlipperPosition, 0.001);

        // Release right bumper
        controller.updateRightBumper(false);

        // Press right bumper second time
        controller.updateRightBumper(true);
        assertTrue("Right flipper should be stowed after second toggle", controller.rightFlipperStowed);
        assertEquals("Right flipper should move to position 0.0 (turned left)", 0.0, controller.rightFlipperPosition, 0.001);
    }

    @Test
    public void testIndependentBumperToggles() {
        MockFlipperController controller = new MockFlipperController();

        // Toggle left bumper
        controller.updateLeftBumper(true);
        controller.updateLeftBumper(false);
        assertEquals(1.0, controller.leftFlipperPosition, 0.001);
        assertEquals(0.0, controller.rightFlipperPosition, 0.001);

        // Toggle right bumper
        controller.updateRightBumper(true);
        controller.updateRightBumper(false);
        assertEquals(1.0, controller.leftFlipperPosition, 0.001);
        assertEquals(1.0, controller.rightFlipperPosition, 0.001);
    }
}
