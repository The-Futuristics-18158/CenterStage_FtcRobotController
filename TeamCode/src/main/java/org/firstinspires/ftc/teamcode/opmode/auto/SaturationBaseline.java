package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utility.GamePieceLocation;
import org.firstinspires.ftc.teamcode.vision.util.FieldPosition;
import org.firstinspires.ftc.teamcode.vision.util.SpikePosition;
import org.opencv.core.Rect;

/**
 * This is a special OpMode to gather the average saturation of the virtual spike line boxes.
 * We have put boxes around the spike lines to exclude noise that can interfere with getting a
 * proper reading.  We were having problems capturing the end of the field in our image processing
 * and it led to bad results.
 *
 * It is worth checking the camera stream when doing the baseline testing to make sure that the
 * boxes are enclosing the spike lines.
 *
 * Important note:  Currently, we are calibrating ONLY ON THE BLUE_FIELD_LEFT position.  If we
 * find that we need to calibrate for each field position, we will have to update this OpMode
 * This would also allow us to verify the spike line boxes
 */
@Autonomous(name="Saturation Baseline", group="OpMode")
public class SaturationBaseline extends AutoBase {

    private FieldSide selectedFieldSide;
    private SpikePosition selectedSpikeLine;
    private Rect currentRect;
    private int modIncrement;


    enum FieldSide {
        LEFT_FIELD,
        RIGHT_FIELD
    }

    enum ModDirection {
        UP,
        DOWN,
        LEFT,
        RIGHT
    }
    @Override
    public void runOpMode() {
        // finally do the init in the AutoBase that will set up the camera and motors
        super.runOpMode();

        // Change this if you want to calibrate for a different field position
        setFieldPosition(FieldPosition.NOT_ON_FIELD);



        while (opModeInInit()) {

            // which side of the field are we testing?
            if(gamepad1.left_bumper){
                selectedFieldSide = FieldSide.LEFT_FIELD;
            }
            if (gamepad1.right_bumper) {
                selectedFieldSide = FieldSide.RIGHT_FIELD;
            }

            // Select which spike line we want to tune
            if(gamepad1.x){
                selectedSpikeLine = SpikePosition.LEFT;
            }
            if(gamepad1.y){
                selectedSpikeLine = SpikePosition.CENTRE;
            }
            if(gamepad1.b){
                selectedSpikeLine = SpikePosition.RIGHT;
            }

            // want to shift the whole box
            if(gamepad1.right_bumper){
                if(gamepad1.dpad_up) {
                    moveBox(selectedSpikeLine,ModDirection.UP);
                }
            }




            SpikePosition spikePos = getSpikePosition();
            switch (spikePos) {
                case LEFT:
                    gamepieceLocation = GamePieceLocation.LEFT;
                    break;
                case CENTRE:
                    gamepieceLocation = GamePieceLocation.CENTER;
                    break;
                default:
                    gamepieceLocation = GamePieceLocation.RIGHT;
            }

            telemetry.addData("LSpikeSaturation", getLeftSpikeSaturation());
            telemetry.addData("RSpikeSaturation", getRightSpikeSaturation());
            telemetry.addData("CSpikeSaturation", getCenterSpikeSaturation());

            telemetry.update();
        }
        while (opModeIsActive()) {

            // TODO: Here is an idea,  we could map buttons to the different field positions and print the results like in the init_loop
        }
    }

    private void moveBox(SpikePosition selectedSpikeLine, ModDirection fieldSide) {
        // get the current Rect for that spikeline
        switch (fieldSide){
            case LEFT:
                visionSystem.setFieldPosition(FieldPosition.RED_FIELD_LEFT);
                sleep(20); // wait to process some frames
                currentRect = visionSystem.getSpikeBox(selectedSpikeLine);
                currentRect.x = currentRect.x + modIncrement;
            case RIGHT:
                // Todo: Do the same thing for the other directions.
        }
    }

}
