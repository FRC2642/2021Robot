package frc.robot;

import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class SwerveModule {

  /*private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;*/
  
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final CANEncoder m_driveEncoder;
  private final CANAnalog m_turningEncoder;

  private final double MAX_VOLTS = 4.95;
  
  /*private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration
      = 2 * Math.PI; // radians per second squared*/
  



  private final PIDController m_drivePIDController;
  /*private final ProfiledPIDController m_turningPIDController
      = new ProfiledPIDController(1, 0, 0,
      new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));*/

  // Gains are for example purposes only - must be determined for your own robot!
  //private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);
  //private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);

  /**
   * Constructs a SwerveModule.
   *
   * @param m_driveMotor   ID for the drive motor.
   * @param m_turningMotor ID for the turning motor.
   */
  public SwerveModule(int m_driveMotor, int m_turningMotor) {

    this.m_driveMotor =  new CANSparkMax m_driveMotor;
    this.m_turningMotor = new CANSparkMax m_turningMotor;
    
    m_drivePIDController = new PIDController(1, 0, 0);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);
  
    

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_turningEncoder.setPositionConversionFactor(359.0 / 3.3); //voltage into degrees
    //m_driveEncoder.setVelocityConversionFactor((1.0 / 60) * (4 * Math.PI) * .0254); //rpm into MPS  

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void drive (double speed, double angle) {
    m_driveMotor.set (speed);

    double setpoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5); // Optimization offset can be calculated here.
    if (setpoint < 0) {
        setpoint = MAX_VOLTS + setpoint;
    }
    if (setpoint > MAX_VOLTS) {
        setpoint = setpoint - MAX_VOLTS;
    }

    m_drivePIDController.setSetpoint (setpoint);


  }
  
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  /*public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(
        m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(
        m_turningEncoder.getPosition(), state.angle.getRadians()
    );

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }*/
}