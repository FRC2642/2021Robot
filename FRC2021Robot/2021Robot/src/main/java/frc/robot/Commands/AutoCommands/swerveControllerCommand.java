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




public class swerveControllerCommand extends CommandBase {
  public static final SwerveDriveSubsystem drive = new SwerveDriveSubsystem();
  private final Timer timer = new Timer();
  private Trajectory autonav3;
  private Pose2d finalPose;

  private final PIDController xController = new PIDController(1.0, 0.4, 0);
  private final PIDController yController = new PIDController(1.0, 0.4, 0);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(1.0, 0.4, 0,
            kThetaControllerConstraints);
  //private final Consumer<SwerveModuleState[]> outputModuleStates;





  /** Creates a new swerveControllerCommand. */
  public swerveControllerCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    TrajectoryConfig config =
    new TrajectoryConfig(kRealMaxMPS, kMaxAcceleration)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(drive.kinematics);
    
    config.setReversed(true);

    Trajectory autonav3 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(              
        new Translation2d(5,0),
        new Translation2d(0, 5)),
        new Pose2d(5, 5, new Rotation2d(0)),             
          config);

          
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        autonav3,
        drive::getPose, 
        drive.kinematics,
        //Position controllers
        xController,
        yController,
        thetaController,
        drive::setModuleStates,
        drive
      );

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //finalPose = autonav3.sample(autonav3.getTotalTimeSeconds()).poseMeters;

    timer.reset();
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*double curTime = timer.get();

    var desiredState = autonav3.sample(curTime);
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

    drive.setModuleStates(targetModuleStates);*/

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        timer.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return timer.hasElapsed(autonav3.getTotalTimeSeconds());  }
}
}
