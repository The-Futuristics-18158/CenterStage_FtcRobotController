package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
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
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        // set apriltag resolution decimation factor
        SetDecimation(1);

        // Build the vision portal
        // set parameters,then use vision builder.
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(RobotContainer.ActiveOpMode.hardwareMap.get(WebcamName.class, CamName))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640,480))
                //.setCameraResolution(new Size(1280,720))
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // set camera exposure and gain
        // values used from example code
        //SetAutoCameraExposure();
        setCameraExposure(2, 250);
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        RobotContainer.ActiveOpMode.telemetry.addData("fps", myVisionPortal.getFps());

        List<AprilTagDetection> tags =  GetCurrentAprilTagDetections();

        /*// revise AprilTag decimation based on ranges
        double minrange = 144.0;
        for (int i=0; i<tags.size(); ++i) {
            if (tags.get(i).ftcPose.range < minrange)
                minrange = tags.get(i).ftcPose.range;
        }
        if (minrange < 12.0)
            SetDecimation(3);
        else if (minrange <48.0)
            SetDecimation(2);
        else
            SetDecimation(1);
*/

        // look for apriltag #4
        for (int i=0; i<tags.size(); ++i)
        {
            RobotContainer.ActiveOpMode.telemetry.addData("AT", tags.get(i).id );

            if (tags.get(i).id == 4)
            {
                String str = JavaUtil.formatNumber(tags.get(i).ftcPose.x, 2) + "," +
                        JavaUtil.formatNumber(tags.get(i).ftcPose.y, 2) + "," +
                        JavaUtil.formatNumber(tags.get(i).ftcPose.z, 2);
                RobotContainer.ActiveOpMode.telemetry.addData("xyz", str);
                str = JavaUtil.formatNumber(tags.get(i).ftcPose.yaw, 2) + "," +
                        JavaUtil.formatNumber(tags.get(i).ftcPose.pitch, 2) + "," +
                        JavaUtil.formatNumber(tags.get(i).ftcPose.roll, 2);
                RobotContainer.ActiveOpMode.telemetry.addData("ypr", str);
                str = JavaUtil.formatNumber(tags.get(i).ftcPose.range, 2) + "," +
                        JavaUtil.formatNumber(tags.get(i).ftcPose.elevation, 2) + "," +
                        JavaUtil.formatNumber(tags.get(i).ftcPose.bearing, 2);
                RobotContainer.ActiveOpMode.telemetry.addData("reb", str);


                //RobotContainer.ActiveOpMode.telemetry.addData("fps", myVisionPortal.getFps());

            }
        }

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

    // sets decimation of AprilTag processing
    public void SetDecimation (int num) {
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

        // wait until camera in streaming mode
        while (myVisionPortal.getCameraState()!= VisionPortal.CameraState.STREAMING)
        {}

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