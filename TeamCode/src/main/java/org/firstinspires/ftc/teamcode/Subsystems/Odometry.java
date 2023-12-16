package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.RobotContainer;


/** Odometry Subsystem */
public class Odometry extends SubsystemBase {

    // mecanum kinematics and odemetry objects
    MecanumDriveKinematics kinematics;
    MecanumDriveOdometry odometry;

    // time
    ElapsedTime time;

    /** Place code here to initialize subsystem */
    public Odometry() {

        // set up and initialize timer
        time = new ElapsedTime();
        time.reset();

        // set up mecanum kinematics
        Translation2d FrontLeftMeters = new Translation2d(0.5, -0.5);
        Translation2d FrontRightMeters = new Translation2d(0.5, 0.5);
        Translation2d BackLeftMeters = new Translation2d(-0.5, -0.5);
        Translation2d BackRightMeters = new Translation2d(-0.5, 0.5);

        kinematics = new MecanumDriveKinematics(FrontLeftMeters,
                                                FrontRightMeters,
                                                BackLeftMeters,
                                                BackRightMeters);

        // create mecanum odometry
        odometry = new MecanumDriveOdometry(kinematics,
                    new Rotation2d(0.0));
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // get robot angle from gyro - convet from deg to rad
        double robotanglerad = 0.01745329 * RobotContainer.gyro.getAngle();

        // get wheel speeds from drivetrain
        MecanumDriveWheelSpeeds wheelspeeds = RobotContainer.drivesystem.GetWheelSpeeds();

        //  go ahead and update odometry
        odometry.updateWithTime(time.seconds(),
                new Rotation2d(robotanglerad),
                wheelspeeds);

        // get new odometry and display in telemetry
        Pose2d posn = getPose2d();
        String str = JavaUtil.formatNumber(posn.getX(), 2) + "," +
                    JavaUtil.formatNumber(posn.getY(), 2) + "," +
                JavaUtil.formatNumber(posn.getRotation().getDegrees(), 2);
        RobotContainer.ActiveOpMode.telemetry.addData("Odometry", str);
        RobotContainer.ActiveOpMode.telemetry.update();
    }

    // returns current estimated position of robot
    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
    }

    // sets current position of robot to provided position
    public void setPose2d(Pose2d position) {
        odometry.resetPosition(position,
                new Rotation2d(RobotContainer.gyro.getAngle()));
    }



}