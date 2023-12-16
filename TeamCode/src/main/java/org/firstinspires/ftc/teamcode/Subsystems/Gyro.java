package org.firstinspires.ftc.teamcode.Subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;

/** Gyro Subsystem */
public class Gyro extends SubsystemBase {

    // robot gyroscope
    private IMU gyro;

    // gyro offset
    private double AngleOffset;

    /** Place code here to initialize subsystem */
    public Gyro() {
        // create gyro and initialize it
        AngleOffset = 0.0;
        gyro = RobotContainer.ActiveOpMode.hardwareMap.get(IMU.class, "imu");
        gyro.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        gyro.resetYaw();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        RobotContainer.ActiveOpMode.telemetry.addData("Gyro", JavaUtil.formatNumber(getAngle(), 2));
    }

    /** get gyro angle - returns angle in deg between -180 and 180 */
    public double getAngle() {
        return (-gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))+AngleOffset;
    }

    // resets gyro and offset value
    public void resetGyro() {
        AngleOffset = 0.0;
        AngleOffset -= getAngle();
    }

    // sets gyro to provided angle (in deg)
    public void setAngle(double angle) {

        AngleOffset -= getAngle() - angle;
    }

}
