package org.firstinspires.ftc.teamcode.CommandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.ManualDrive;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class ExampleCommandGroup extends SequentialCommandGroup {

    // constructor
    public ExampleCommandGroup() {

        addCommands (
        // new command1
        // new command2
        // new command3
        );
    }

}

// Example: Lily's 2023 FRC super cube auto - unfortunately took about 0.5to1s too long to use
/*          // enable arm, and lift to stow position
            new InstantCommand(() -> RobotContainer.arm.SetEnableArm(true)),

            // move arm back to drop off cube
            new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.HIGH_DEG)),

            // delay until arm gets back
            new DelayCommand(1.0),

            // place cube
            new InstantCommand(() -> RobotContainer.grabber.setClose()),

            // delay for gripper to close
            new DelayCommand(0.7),

            // move arm to 'forward position' but inside robot bumper)
            // move to 135deg
            new InstantCommand(() -> RobotContainer.arm.SetArmPosition(135.0)),

            // delay for arm to get to stow
            new DelayCommand(1.5),

            // ensure arm is stowed before it is allow to begin moving over charge station
            new SafetyCheckStowPosition(),

            // drive right
            // new DrivetoRelativePose(new Pose2d(0,-2.0, new Rotation2d(0.0)), 1.0, 0.1, 5.0),

            // drive straight
            new DrivetoRelativePose(new Pose2d(5.0, 0, new Rotation2d(0.0)),1.8,0.1, 7.0),

            // pick up cube from floor :)
            new AutoFloorCubePickup(),

            // delay
            new DelayCommand(0.5),

            // drive back
            //new DrivetoRelativePose(new Pose2d(1.0,0, new Rotation2d(0.0)), 1.0, 0.1, 2.0),

            // drive left to center
            new DrivetoRelativePose(new Pose2d(-1.0,2.0, new Rotation2d(0.0)), 1.8, 0.1, 5.0),

            // drive straight onto charge station
            new DrivetoRelativePose(new Pose2d(-1.5, 0, new Rotation2d(0.0)),1.0,0.1, 30.0),

            // balance
            new AutoBalance()

 */