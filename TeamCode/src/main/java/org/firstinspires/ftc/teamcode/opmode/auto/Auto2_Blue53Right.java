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

package org.firstinspires.ftc.teamcode.opmode.auto;

import static android.os.SystemClock.sleep;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.AllianceColour;
import org.firstinspires.ftc.teamcode.utility.AprilTagLocation;
import org.firstinspires.ftc.teamcode.utility.GamePieceLocation;
import org.firstinspires.ftc.teamcode.utility.VisionProcessorMode;
import org.firstinspires.ftc.teamcode.vision.util.FieldPosition;
import org.firstinspires.ftc.teamcode.vision.util.SpikePosition;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Autonomous operation class for 'BlueFieldLeft' scenario.
 * Extends 'AutoBase' which contains code common to all Auto OpModes'.
 */

@Autonomous(name="Blue53Right", group="OpMode",preselectTeleOp = "GGE Odometry TeleOp")
//@Disabled
public class Auto2_Blue53Right extends AutoBase {

    /**
     * Runs once and initializes the autonomous program.
     * Sets the initial state and the game piece location to UNDEFINED.
     * If we ever come across an instance in our code where gamepieceLocation is UNDEFINED, there
     * is likely a problem.
     */
    @Override
    public void runOpMode() {
        super.runOpMode();
        gamepieceLocation = GamePieceLocation.UNDEFINED; // this is the position that we can't see
        setFieldPosition(FieldPosition.BLUE_FIELD_RIGHT);

        allianceColour = AllianceColour.BLUE;

        // this is setting the initial field coordinates
        // need to set the AprilTagTargets
        targetAprilTags = new ArrayList<>(Arrays.asList(AprilTagLocation.BLUE_LEFT,
                AprilTagLocation.BLUE_CENTRE,
                AprilTagLocation.BLUE_RIGHT));

        lastFieldPos = new Pose2d(0.25,2.2, new Rotation2d(Math.toRadians(0.0)));
        odometry.resetPosition(lastFieldPos,lastFieldPos.getRotation());

        while (opModeInInit()) {
            state = 0;
            SpikePosition spikePos = getSpikePosition();
            switch (spikePos) {
                case RIGHT:
                    gamepieceLocation = GamePieceLocation.RIGHT;
                    break;
                case CENTRE:
                    gamepieceLocation = GamePieceLocation.CENTER;
                    break;
                default:
                    gamepieceLocation = GamePieceLocation.LEFT;
            }
            telemetry.addData("GamePiece Spike line", gamepieceLocation);
            telemetry.addData("LSpikeline",getLeftSpikeSaturation());
            telemetry.addData("Cspikeline",getCenterSpikeSaturation());
            telemetry.addData("RSpikeLine",getRightSpikeSaturation());
            telemetry.update();
        }

        while (opModeIsActive()) {
            // we don't want any streaming to the Driver Station, waste of processing and bandwidth
            visionSystem.stopLiveView();

            updateOdometry();
            //we don't need the front camera anymore,  now need the rear one with april tags
            VisionProcessorMode currentVPMode = visionSystem.setVisionProcessingMode(VisionProcessorMode.REAR_CAMERA_BACKDROP_APRIL_TAG);

            double DirectionNow = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if (gamepieceLocation == GamePieceLocation.LEFT && state == 0) {
                // Start by securing the loaded pixel - always pause after servo claw motions.
                intake.ClawClosed();
                sleep(250);
                // Move forward 25 inches
                moveTo.Forward((int) ((25 * ticksPerInch) * 0.94), 0.25);
                // Rotate -90 degrees
                moveTo.Rotate(-90);
                sleep(700);
                // Move the claw down
                intake.FlipDown();
                // Move forward 4 inches
                moveTo.Forward((int) ((4 * ticksPerInch) * 0.94), 0.4);
                // Open the claw - always pause after servo claw motions.
                intake.ClawOpen();
                sleep (250);
                // End all autos with the wrist up
                intake.FlipUp();
                // Rotate 90 degrees
                moveTo.Rotate(90);
                sleep(700);
                // Move forward 12 inches
                moveTo.Forward((int) ((11 * ticksPerInch) * 0.94), 0.4);
                // Rotate 90 degrees
                moveTo.Rotate(90);
                sleep(700);
                // Move backward 62 inches
                moveTo.Backwards((int) ((56 * ticksPerInch) * 0.94), 0.35);
                // move sideways 24 inches
                moveTo.Right((int) ((32 * ticksPerInch) * 1.04), 0.4); // Calculated ticks by distance * 104% (from last year)
                state = 1;
            } else if (gamepieceLocation == GamePieceLocation.CENTER && state == 0) {
                // Start by securing the loaded pixel - always pause after servo claw motions.
                intake.ClawClosed();
                sleep (250);
                // move forward 12 inches
                moveTo.Forward((int) ((12 * ticksPerInch) * 0.94), 0.25); // Calculated ticks by distance * 94% (from last year)
                // Move the claw down
                intake.FlipDown();
                // Move forward 9 inches
                moveTo.Forward((int) ((9 * ticksPerInch) * 0.94), 0.25);
                // Open the claw - always pause after servo claw motions.
                intake.ClawOpen();
                sleep (250);
                // End all autos with the wrist up
                intake.FlipUp();
                // Rotate 90 degrees
                moveTo.Rotate(90);
                sleep(700);
                // Move backward 25 inches
                moveTo.Backwards((int) ((36 * ticksPerInch) * 0.94), 0.25);
                state = 1;
            } else if (gamepieceLocation == GamePieceLocation.RIGHT && state == 0) {
                // Start by securing the loaded pixel - always pause after servo claw motions.
                intake.ClawClosed();
                sleep (250);
                // move forward 2 inches
                moveTo.Forward((int) ((2 * ticksPerInch) * 0.94), 0.25); // Calculated ticks by distance * 94% (from last year)
                // move sideways 8 inches
                moveTo.Right((int) ((8 * ticksPerInch) * 1.04), 0.4); // Calculated ticks by distance * 104% (from last year)
                // Move the claw down
                intake.FlipDown();
                // move forward 12 inches
                moveTo.Forward((int) ((12 * ticksPerInch) * 0.94), 0.25); // Calculated ticks by distance * 94% (from last year)
                // Open the claw - always pause after servo claw motions.
                intake.ClawOpen();
                sleep (250);
                // End all autos with the wrist up
                intake.FlipUp();
                // move left 9 inches
                moveTo.Left((int) ((11 * ticksPerInch) * 1.04), 0.4); // Calculated ticks by distance * 104% (from last year)
                // move forward 30 inches
                moveTo.Forward((int) ((30 * ticksPerInch) * 0.94), 0.25); // Calculated ticks by distance * 94% (from last year)
                // Rotate 90 degrees
                moveTo.Rotate(90);
                sleep(700);
                // Move backward 62 inches
                moveTo.Backwards((int) ((58 * ticksPerInch) * 0.94), 0.35);
                // move sideways 24 inches
                moveTo.Right((int) ((18 * ticksPerInch) * 1.04), 0.4); // Calculated ticks by distance * 104% (from last year)
                state = 1;
            }
            // Use the GoToAprilTag to get to within 7 inches of the Backdrop
            if (gamepieceLocation == GamePieceLocation.LEFT && state == 1) {
                // Align and drive to April Tag.  1 is BLUE side LEFT.
                if (moveTo.GoToAprilTag(AprilTagLocation.BLUE_LEFT) == true) {
                    state = 2;
                }
            } else if (gamepieceLocation == GamePieceLocation.CENTER && state == 1) {
                // Align and drive to April Tag.  2 is BLUE side CENTER.
                if (moveTo.GoToAprilTag(AprilTagLocation.BLUE_CENTRE) == true) {
                    state = 2;
                }
            } else if (gamepieceLocation == GamePieceLocation.RIGHT && state == 1) {
                // Align and drive to April Tag.  3 is BLUE side RIGHT.
                if (moveTo.GoToAprilTag(AprilTagLocation.BLUE_RIGHT) == true) {
                    state = 2;
                }
            }

            // Complete the auto by dropping the game piece and going to park
            if (gamepieceLocation == GamePieceLocation.LEFT && state == 2) {
                // Move the linear slide to the low scoring position
                linearSlideMove.LinearSlidesLow();
                // Moves the conveyor forward
                conveyor.setPosition(0);
                // Runs the conveyor for 4 seconds
                sleep(4000);
                // Stops the conveyor
                conveyor.setPosition(0.5);
                // Forward 4 inches
                moveTo.Forward((int) ((4 * ticksPerInch) * 0.94), 0.25);
                // Moves the linear slide to the bottom position
                linearSlideMove.LinearSlidesBottom();
                state = 3;
            } else if (gamepieceLocation == GamePieceLocation.CENTER && state == 2) {
                // Move the linear slide to the low scoring position
                linearSlideMove.LinearSlidesLow();
                // Moves the conveyor forward
                conveyor.setPosition(0);
                // Runs the conveyor for 4 seconds
                sleep(4000);
                // Stops the conveyor
                conveyor.setPosition(0.5);
                // Forward 4 inches
                moveTo.Forward((int) ((4 * ticksPerInch) * 0.94), 0.25);
                // Moves the linear slide to the bottom position
                linearSlideMove.LinearSlidesBottom();
                state = 3;
            } else if (gamepieceLocation == GamePieceLocation.RIGHT && state == 2) {
                // Move the linear slide to the low scoring position
                linearSlideMove.LinearSlidesLow();
                // Moves the conveyor forward
                conveyor.setPosition(0);
                // Runs the conveyor for 4 seconds
                sleep(4000);
                // Stops the conveyor
                conveyor.setPosition(0.5);
                // Forward 4 inches
                moveTo.Forward((int) ((4 * ticksPerInch) * 0.94), 0.25);
                // Moves the linear slide to the bottom position
                linearSlideMove.LinearSlidesBottom();
                state = 3;
            }
            // Show the elapsed game time and wheel power.
            displayTelemetry(DirectionNow);

            // Use the GoToPose2D to finalize the auto by parking
            if (state == 3) {
                // Align and drive to April Tag.  1 is BLUE side LEFT.
                // Update IMU and Odometry
                DirectionNow = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                odometrySpeeds = moveTo.GetWheelSpeeds();
                odometry.updateWithTime(odometryTimer.seconds(),
                        new Rotation2d(Math.toRadians(DirectionNow)), odometrySpeeds);

                if (moveTo.GoToPose2d(new Pose2d(1.25, 3.2, new Rotation2d(Math.toRadians(0.0))))) {
                    state = 4;
                }
            }

            if (state == 4){
                // save our last position so teleop can pick it up as a starting point
                updateLastPos();
                state = 5;
            }
        }
    }
}
