package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.util.concurrent.TimeUnit;

/** AprilTagCamera Subsystem */
public class AprilTagCamera extends SubsystemBase {

    // Used for managing the AprilTag detection process.
    private AprilTagProcessor myAprilTagProcessor;

    // Used to manage the video source.
    private VisionPortal myVisionPortal;

    /** Place code here to initialize subsystem */
    public AprilTagCamera(String CamName) {

        // Build the AprilTag processor
        // set parameters of AprilTagProcessor, then use Builder to build
        myAprilTagProcessor = new AprilTagProcessor.Builder()
                //.setTagLibrary(myAprilTagLibrary)
                //.setNumThreads(tbd)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        // reduce AprilTag image processing by factor of 2
        // should detect 2" tag from 6ft away at 22fps
        SetAprilTagDecimation(2);

        // Build the vision portal
        // set parameters,then use vision builder.
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(RobotContainer.ActiveOpMode.hardwareMap.get(WebcamName.class, CamName))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640,480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();

        myVisionPortal.resumeStreaming();
        // set camera exposure and gain
        // values used from example code
        //setCameraExposure(6, 250);

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }



    // get fresh AprilTag detections (if any) from camera
    // returns list containing info on each tag detected
    public List<AprilTagDetection> GetFreshAprilTagDetections() {
        return myAprilTagProcessor.getFreshDetections();
    }

    // get current AprilTag detections (if any) from camera
    // returns list containing info on each tag detected
    public List<AprilTagDetection> GetCurrentAprilTagDetections() {
        return myAprilTagProcessor.getDetections();
    }

    // sets decimatino of AprilTag processing
    public void SetAprilTagDecimation (int num) {
        myAprilTagProcessor.setDecimation(num);
    }

    // get camera frames per second
    public double GetCameraFPS () {
        return myVisionPortal.getFps();
    }

    // use to turn on/off AprilTag processing
    public void EnableAprilTagProcessing (boolean enable) {
        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, enable);
    }

    /** Set the camera gain and exposure. */
    public void setCameraExposure(int exposureMS, int gain) {

        // set camera gain and exposure
        RobotContainer.ActiveOpMode.sleep(20);

        // set exposure control to manual
        ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            RobotContainer.ActiveOpMode.sleep(50);
        }

        // set exposure and gain
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        RobotContainer.ActiveOpMode.sleep(20);
        GainControl gainControl = myVisionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        RobotContainer.ActiveOpMode.sleep(20);
    }

    /** Sets the camera exposure to automatic */
    public void SetAutoCameraExposure() {
        ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Auto);
    }

}