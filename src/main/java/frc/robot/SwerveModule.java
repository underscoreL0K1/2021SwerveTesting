// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final double kCircumference = kWheelRadius * 2 * Math.PI;
  private static final double kDriveRatio = 8.16;
  private static final double kTurningRatio = 12.8;
  private static final int kEncoderResolution = 2048;
  double targetVelocity = 1 * 2048 / 600; // X RPM 

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration
      = 2 * Math.PI; // radians per second squared
  
  
  private WPI_TalonFX m_driveMotor;
   private double drivekP = 0.00015;
   private double drivekI = 0;
   private double drivekD = 0;
   private double drivekF = .025;
  private WPI_TalonFX m_turningMotor;
   private double turningkP = 0.00015;
   private double turningkI = 0;
   private double turningkD = 0;
   private double turningkF = .025;
  private final int driveMotorChannel;
  private final int turningMotorChannel;
  static double offsetter;




void setDrivePIDF( double p, 
 double i, 
 double d, 
 double f){
  drivekP = p;
  drivekI = i;
  drivekD = d;
  drivekF = f;
  m_driveMotor.config_kP(0, drivekP);
  m_driveMotor.config_kI(0, drivekI);
  m_driveMotor.config_kD(0, drivekD);
  m_driveMotor.config_kF(0, drivekF);
}


void setTurningPIDF( double p, 
 double i, 
 double d, 
 double f){
  turningkP = p;
  turningkI = i;
  turningkD = d;
  turningkF = f;
  m_turningMotor.config_kP(0, turningkP);
  m_turningMotor.config_kI(0, turningkI);
  m_turningMotor.config_kD(0, turningkD);
  m_turningMotor.config_kF(0, turningkF);
}


  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, Double correction) {
    
    
    offsetter = correction;

    this.driveMotorChannel = driveMotorChannel;
    this.turningMotorChannel = turningMotorChannel;
      m_driveMotor = new WPI_TalonFX(driveMotorChannel);
      m_turningMotor = new WPI_TalonFX(turningMotorChannel);
      m_driveMotor.configFactoryDefault();
      m_turningMotor.configFactoryDefault();
      m_turningMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
      m_turningMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
      // Doesn't work, we need to use cancoders in order to get wheel position rather than motor shaft position
      // Otherwise, will rotate past one rotation, giving us a value we can't use.
      // sensorCollection.setIntegratedSensorPosition(absoluteValue - offset, 20);
      //m_turningMotor.setSensorPhase(PhaseSensor);
      //m_turningMotor.setInverted(true);
      //if (correction > 0) {m_turningMotor.setSelectedSensorPosition(correction);}

      //sensorCollection.setIntegratedSensorPosition(absoluteValue - offset, 20);



      m_turningMotor.setNeutralMode(NeutralMode.Brake);
      m_driveMotor.setNeutralMode(NeutralMode.Brake);
    setDrivePIDF(0.0008,0,0,0.048);
    setTurningPIDF(1,0.0,0,0.048);

    // 50% power to turning - gets 10610 units/100ms
    // 50% power to driving - 10700 units/100ms



    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
   // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    //m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity()
    , getAngle());
  }

  public Rotation2d getAngle() {
    return new Rotation2d((2*Math.PI/(2048*kTurningRatio))*(m_turningMotor.getSelectedSensorPosition()%(2048*kTurningRatio)));
  }

  public double inputAngle;

  public double setpoint;

  public double inputVelocity;

    
  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getAngle());
    long nearestDegree = Math.round(state.angle.getDegrees());
    
    double setTurnValue = (2048/360.0)*(nearestDegree);
    
    //System.out.println("corrector" + SwerveModule.offsetter);


    // Calculate the drive output from the drive PID controller.
    //2048 encoder ticks per rotation, input is m/s, we want ticks per 100ms
    inputVelocity = 2048/(10*kCircumference)*state.speedMetersPerSecond*kDriveRatio;
    m_driveMotor.set(TalonFXControlMode.Velocity, inputVelocity);

    // Calculate the turning motor output from the turning PID controller.
    //2048 encoder ticks per rotation, 2pi radians per rotation, so the conversion factor is 2048/2pi radians
    inputAngle = nearestDegree;
    setpoint = setTurnValue*kTurningRatio;

    
    m_turningMotor.set(TalonFXControlMode.Position, setpoint);

    

    SmartDashboard.putNumber("Setpoint", setpoint);


/*
    SmartDashboard.putNumber("BL", BL.getSelectedSensorPosition());
    SmartDashboard.putNumber("BR", BR.getSelectedSensorPosition());
    SmartDashboard.putNumber("FL", FL.getSelectedSensorPosition());
    SmartDashboard.putNumber("FR", FR.getSelectedSensorPosition());
*/

    // System.out.print(inputAngle + "\t");
  }
}