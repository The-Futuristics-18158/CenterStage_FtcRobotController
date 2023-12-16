package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import org.firstinspires.ftc.teamcode.RobotContainer;

/** DriveTrain Subsystem */
public class DriveTrain extends SubsystemBase {

    // create Mecanum drive and its motors
    private Motor leftFrontDrive = null;
    private Motor leftBackDrive = null;
    private Motor rightFrontDrive = null;
    private Motor rightBackDrive = null;

    final double DEGTORAD = 3.1415926/180.0;

    /** Place code here to initialize subsystem */
    public DriveTrain() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = new Motor(RobotContainer.ActiveOpMode.hardwareMap,"left_front_drive");
        leftBackDrive  = new Motor(RobotContainer.ActiveOpMode.hardwareMap,"left_back_drive");
        rightFrontDrive = new Motor(RobotContainer.ActiveOpMode.hardwareMap,"right_front_drive");
        rightBackDrive = new Motor(RobotContainer.ActiveOpMode.hardwareMap,"right_back_drive");

        // for tiny
        leftFrontDrive.setInverted(false);
        leftBackDrive.setInverted(false);
        rightFrontDrive.setInverted(true);
        rightBackDrive.setInverted(true);

        // set motor braking mode
        leftFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // distance per pulse in m
        // pi*d / pulse-per-revolution = 3.1415 *0.1m / 28
        final double DistancePerPulse = Math.PI * 0.1 / 28.0;
        leftFrontDrive.setDistancePerPulse(DistancePerPulse);
        leftBackDrive.setDistancePerPulse(DistancePerPulse);
        rightFrontDrive.setDistancePerPulse(DistancePerPulse);
        rightBackDrive.setDistancePerPulse(DistancePerPulse);

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
    }

    /** drive robot in field coordinates
     * Inputs: X, y and Rotation speed - all -1 to +1 */
    public void FieldDrive (double Vx, double Vy, double Omega) {

        // get angle of vector rotation angle
        // i.e. neg of gyro angle - in rad
        double rotAngRad = -DEGTORAD*RobotContainer.gyro.getAngle();

        // rotate speed vector by negative of gyro angle
        double x = Vx*Math.cos(rotAngRad) - Vy*Math.sin(rotAngRad);
        double y = Vx*Math.sin(rotAngRad) + Vy*Math.cos(rotAngRad);

        // x,y now in robot coordinates - call robot drive
        RobotDrive(x, y, Omega);
    }

    /** drive robot in robot coordinates
     * Inputs: X, y and Rotation speed */
    public void RobotDrive (double Vx, double Vy, double Omega) {
        // resolve individual mecanum speeds
        double leftFrontPower  = Vx + Vy + Omega;
        double rightFrontPower = Vx - Vy - Omega;
        double leftBackPower   = Vx - Vy + Omega;
        double rightBackPower  = Vx + Vy - Omega;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // set individual motor speeds
        leftFrontDrive.set(leftFrontPower);
        rightFrontDrive.set(rightFrontPower);
        leftBackDrive.set(leftBackPower);
        rightBackDrive.set(rightBackPower);
    }

    /** returns current speeds of mecanum drive wheels */
    public MecanumDriveWheelSpeeds GetWheelSpeeds()
    {
        MecanumDriveWheelSpeeds speeds = new MecanumDriveWheelSpeeds();
        speeds.frontLeftMetersPerSecond = leftFrontDrive.getCorrectedVelocity();
        speeds.rearLeftMetersPerSecond = leftBackDrive.getCorrectedVelocity();
        speeds.frontRightMetersPerSecond = rightFrontDrive.getCorrectedVelocity();
        speeds.rearRightMetersPerSecond = rightBackDrive.getCorrectedVelocity();
        return speeds;
    }

}
