// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.List;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Subsystem.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;




public class bounceSwerveControllerCommand extends CommandBase {
  public static final SwerveDriveSubsystem drive = new SwerveDriveSubsystem();
  private Timer timer = new Timer();
  private Trajectory bounce;
  private Pose2d finalPose;

  private final PIDController xController = new PIDController(1.0, 0.4, 0);
  private final PIDController yController = new PIDController(1.0, 0.4, 0);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(1.0, 0.4, 0,
            kThetaControllerConstraints);
  //private final Consumer<SwerveModuleState[]> outputModuleStates;





  /** Creates a new swerveControllerCommand. */
  public bounceSwerveControllerCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    /*TrajectoryConfig config =
    new TrajectoryConfig(kRealMaxMPS, kMaxAcceleration)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(drive.kinematics);
    
    config.setReversed(true);

    Trajectory bounce = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(              
        new Translation2d(0.4545, 3.7545),
        new Translation2d(0.7333, 6.0573),
        new Translation2d(-0.5818, -4.8058),
        new Translation2d(0.7576, 6.2576),
        new Translation2d(-1.1273, -9.3113),
        new Translation2d(0.8242, 6.8082),
        new Translation2d(-0.8242, -6.8082),
        new Translation2d(0.2182, 1.8022),
        new Translation2d(1.4485, 11.9645),
        new Translation2d(0.0667, 0.5507),
        new Translation2d(-0.1515, -1.2515),
        new Translation2d(-0.1545, -1.2765),
        new Translation2d(-1.0576, -8.7356),
        new Translation2d(0.3455, 2.8535),
        new Translation2d(1.3212, 10.9132),
        new Translation2d(-0.1333, -1.1013),
        new Translation2d(-1.5333, -12.6653),
        new Translation2d(-0.6061, -5.0061)),
        new Pose2d(0.0001, 0.0001, new Rotation2d(0)),            
          config);

          
      SwerveControllerCommand bounceSwerveControllerCommand = new SwerveControllerCommand(
        bounce,
        drive::getPose, 
        drive.kinematics,
        //Position controllers
        xController,
        yController,
        thetaController,
        drive::setModuleStates,
        drive
      );*/

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //this.autonav3 = autonav3;
    //timer.reset();
    //timer.start();

    //finalPose = bounce.sample(bounce.getTotalTimeSeconds()).poseMeters;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*double curTime = timer.get();

    var desiredState = bounce.sample(curTime);
    var desiredPose = desiredState.poseMeters;

    var poseError = desiredPose.relativeTo(drive.getPose());

    double targetXVel = xController.calculate(
        drive.getPose().getTranslation().getX(),
        desiredPose.getTranslation().getX());

    double targetYVel = yController.calculate(
        drive.getPose().getTranslation().getY(),
        desiredPose.getTranslation().getY());

    // The robot will go to the desired rotation of the final pose in the trajectory,
    // not following the poses at individual states.
    double targetAngularVel = thetaController.calculate(
        drive.getPose().getRotation().getRadians(),
        finalPose.getRotation().getRadians());

    double vRef = desiredState.velocityMetersPerSecond;

    targetXVel += vRef * poseError.getRotation().getCos();
    targetYVel += vRef * poseError.getRotation().getSin();

    var targetChassisSpeeds = new ChassisSpeeds(targetXVel, targetYVel, targetAngularVel);

    var targetModuleStates = drive.kinematics.toSwerveModuleStates(targetChassisSpeeds);

    drive.setModuleStates(targetModuleStates);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        timer.stop();*/

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return timer.hasElapsed(bounce.getTotalTimeSeconds());  }
}
}
