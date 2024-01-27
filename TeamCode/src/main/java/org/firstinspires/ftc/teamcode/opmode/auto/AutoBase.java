package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Size;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.vision.pipeline.HSVSaturationPipeline;
import org.firstinspires.ftc.teamcode.utility.GamePieceLocation;
import org.firstinspires.ftc.teamcode.utility.IntakeMovement;
import org.firstinspires.ftc.teamcode.utility.LinearSlideMovement;
import org.firstinspires.ftc.teamcode.utility.Movement;
import org.firstinspires.ftc.teamcode.vision.util.FieldPosition;
import org.firstinspires.ftc.teamcode.vision.util.SpikePosition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public abstract class AutoBase extends LinearOpMode {

    // <<<<<<end of configurable parameters >>>>>>>>>>>
    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 960; // modify for your camera
    protected ElapsedTime runtime = new ElapsedTime(); //
    protected DcMotorEx leftFrontDrive = null;
    protected DcMotorEx leftBackDrive = null;
    protected DcMotorEx rightFrontDrive = null;
    protected DcMotorEx rightBackDrive = null;
    protected IMU imu;
    MecanumDriveKinematics kinematics;
    MecanumDriveOdometry odometry;
    ElapsedTime odometryTimer = new ElapsedTime();
    MecanumDriveWheelSpeeds odometrySpeeds = new MecanumDriveWheelSpeeds();
    private RevBlinkinLedDriver blinkinLED;
    protected GamePieceLocation gamepieceLocation;
    Servo leftClaw;
    Servo rightClaw;
    Servo conveyor;
    OpenCvWebcam webcam;

    // Used for managing the AprilTag detection process.
    private AprilTagProcessor myAprilTagProcessor;
    // Used to manage the video source.
    private VisionPortal myVisionPortal;

    HSVSaturationPipeline pipeline;

    Movement moveTo;
    IntakeMovement intake;
    LinearSlideMovement linearSlideMove;
    int state;
    private DcMotorEx leftLinearSlide = null;
    private DcMotorEx rightLinearSlide = null;
    private DcMotorEx wrist = null;


    // Motor is 28 ticks per revolution
    // Gear Ratio is 12:1
    // Wheel diameter is 100mm
    final static double ticksPerInch = (28 * 12) / ((100 * 3.14) / 25.4);

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "gge_cam"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new HSVSaturationPipeline();

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed", "");
                telemetry.update();
            }
        });

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
        myAprilTagProcessor.setDecimation(2);

        // Build the vision portal
        // set parameters,then use vision builder.
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "gge_backup_cam"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                //.setCameraResolution(new Size(1280,720))
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        leftLinearSlide = hardwareMap.get(DcMotorEx.class, "left_linear_slide");
        rightLinearSlide = hardwareMap.get(DcMotorEx.class, "right_linear_slide");
        wrist = hardwareMap.get(DcMotorEx.class, "wrist");

        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");

        conveyor = hardwareMap.get(Servo.class, "conveyor");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        blinkinLED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        double DirectionNow = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Adding in PIDF Config values learned from previous testing
        // These may need to be tuned anytime the motor weights or config changes.
        // Set PIDF values thinking of ...
        // ...P as primary force (set second)
        // ...I as smoothing (set last)
        // ...D as deceleration (set third)
        // ...F as holding / static force (set first)
        // For Mecanum drive, 8, 0, 0.5, 5 works well on Tiny
        // ... and 7, 0.2, 0.1, 8 works on Rosie (heavier bot)
        leftFrontDrive.setVelocityPIDFCoefficients(10, 0.2, 0.1, 8);
        leftBackDrive.setVelocityPIDFCoefficients(10, 0.2, 0.1, 8);
        rightFrontDrive.setVelocityPIDFCoefficients(10, 0.2, 0.1, 8);
        rightBackDrive.setVelocityPIDFCoefficients(10, 0.2, 0.1, 8);
        // For Lift, PIDF values set to reduce jitter on high lift
        leftLinearSlide.setVelocityPIDFCoefficients(8, 0.75, 0, 8);
        rightLinearSlide.setVelocityPIDFCoefficients(8, 0.75, 0, 8);
        // For Wrist, PIDF values set to reduce jitter
        wrist.setVelocityPIDFCoefficients(15, 0.2, 0.05, 16);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Somehow this is reversed from the TeleOp Gge program.
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        wrist.setDirection(DcMotor.Direction.REVERSE);

        intake = new IntakeMovement(rightClaw, leftClaw, wrist, conveyor, telemetry);
        // Pass all needed hardware to the Movement class including the AprilTag detection for refactored GoToAprilTag
        moveTo = new Movement(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, imu, blinkinLED, myAprilTagProcessor, myVisionPortal, odometry, kinematics, odometryTimer, odometrySpeeds, telemetry);
        linearSlideMove = new LinearSlideMovement(leftLinearSlide, rightLinearSlide, intake);

        state = 0;
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();
    }

    /**
     * Sets the camera exposure to automatic
     */
    public void SetAutoCameraExposure() {
        ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Auto);
    }

    protected void displayTelemetry(double DirectionNow) {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Direction Now", JavaUtil.formatNumber(DirectionNow, 2));
        telemetry.addData("Target Position", leftFrontDrive.getTargetPosition());
        telemetry.addData("Left Front Pos", leftFrontDrive.getCurrentPosition());
        telemetry.addData("Right Front Pos", rightFrontDrive.getCurrentPosition());
        telemetry.addData("Left Back Pos", leftBackDrive.getCurrentPosition());
        telemetry.addData("Right Back Pos", rightBackDrive.getCurrentPosition());
        telemetry.addData("state", state);
        telemetry.addData("location", gamepieceLocation);
        telemetry.update();
    }

    protected void setFieldPosition(FieldPosition fPos) {
        pipeline.setFieldPosition(fPos);
    }

    protected SpikePosition getSpikePosition() {
        return pipeline.getSpikePos();
    }

    protected double getLeftSpikeSaturation() {
        return pipeline.getLeftSpikeSaturation();
    }

    protected double getCenterSpikeSaturation() {
        return pipeline.getCenterSpikeSaturation();
    }

    protected double getRightSpikeSaturation() {
        return pipeline.getRightSpikeSaturation();
    }

}

