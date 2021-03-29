/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



/**
 * Represents a swerve drive style drivetrain.
 */
public class Drivetrain {
  public CANSparkMax frontLeftDriveMotor, frontLeftAngleMotor;
  public CANSparkMax frontRightDriveMotor, frontRightAngleMotor;
  public CANSparkMax backLeftDriveMotor, backLeftAngleMotor;
  public CANSparkMax backRightDriveMotor, backRightAngleMotor;

  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  public AHRS navx;

  public double robotLength = 0.889;
  public double robotWidth = 0.6223;
  SwerveDriveOdometry m_odometry;
  SwerveDriveKinematics m_kinematics;

  /*public SwerveModule m_frontLeft;
  public SwerveModule m_frontRight;
  public SwerveModule m_backLeft;
  public SwerveModule m_backRight;*/

private SwerveModule backRight;
private SwerveModule backLeft;
private SwerveModule frontRight;
private SwerveModule frontLeft;


  Translation2d m_frontLeftLocation;
  Translation2d m_frontRightLocation;
  Translation2d m_backLeftLocation;
  Translation2d m_backRightLocation;





  //private final AnalogGyro m_gyro = new AnalogGyro(0);



  public Drivetrain() {
    try{
      navx = new AHRS();
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);}
    }
    
    public void drive(double x1, double y1, double x2){
      double r = Math.sqrt ((robotLength * robotLength) + (robotWidth * robotWidth));
       y1 *= -1;
  
      double a = x1 - x2 * (robotLength / r);
      double b = x1 + x2 * (robotLength / r);
      double c = y1 - x2 * (robotWidth / r);
      double d = y1 + x2 * (robotWidth / r);
  
      double backRightSpeed = Math.sqrt ((a * a) + (d * d));
      double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
      double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
      double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));
      
      double backRightAngle = Math.atan2 (a, d) / Math.PI;
      double backLeftAngle = Math.atan2 (a, c) / Math.PI;
      double frontRightAngle = Math.atan2 (b, d) / Math.PI;
      double frontLeftAngle = Math.atan2 (b, c) / Math.PI;

      backRight.drive (backRightSpeed, backRightAngle);
      backLeft.drive (backLeftSpeed, backLeftAngle);
      frontRight.drive (frontRightSpeed, frontRightAngle);
      frontLeft.drive (frontLeftSpeed, frontLeftAngle);
  
  
    }

    public void SwerveDrive (SwerveModule backRight, SwerveModule backLeft, SwerveModule frontRight, SwerveModule frontLeft) {
      this.backRight = backRight;
      this.backLeft = backLeft;
      this.frontRight = frontRight;
      this.frontLeft = frontLeft;
  }
  
    //m_gyro.reset();
    /*m_frontLeftLocation = new Translation2d(0.381, 0.381);
    m_frontRightLocation = new Translation2d(0.381, -0.381);
    m_backLeftLocation = new Translation2d(-0.381, 0.381);
    m_backRightLocation = new Translation2d(-0.381, -0.381);

    frontLeftDriveMotor = new CANSparkMax(1, MotorType.kBrushless);
    frontLeftAngleMotor = new CANSparkMax(2, MotorType.kBrushless);
    frontRightDriveMotor = new CANSparkMax(3, MotorType.kBrushless);
    frontRightAngleMotor = new CANSparkMax(4, MotorType.kBrushless);
    backLeftDriveMotor = new CANSparkMax(5, MotorType.kBrushless);
    backLeftAngleMotor = new CANSparkMax(6, MotorType.kBrushless);
    backRightDriveMotor = new CANSparkMax(7, MotorType.kBrushless);
    backRightAngleMotor = new CANSparkMax(8, MotorType.kBrushless);

     m_frontLeft = new SwerveModule(frontLeftDriveMotor, frontLeftAngleMotor);
     m_frontRight = new SwerveModule(frontRightDriveMotor, frontRightAngleMotor);
     m_backLeft = new SwerveModule(backLeftDriveMotor, backLeftAngleMotor);
     m_backRight = new SwerveModule(backRightDriveMotor, backRightAngleMotor);
  
    m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle());

    m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);*/

  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  /*public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-navx.getAngle());
  }*/

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  
  //@SuppressWarnings("ParameterName")
  
  /*public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }*/

  /**
   * Updates the field relative position of the robot.
   */
  /*public void updateOdometry() {
    m_odometry.update(
        getAngle(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
    );
  }
}*/
