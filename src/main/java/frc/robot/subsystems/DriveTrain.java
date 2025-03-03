// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants;

public class DriveTrain extends SubsystemBase {
  //DriveTrain Motors
  private TalonFX m_leftLeader;
  private TalonFX m_leftFollower;
  private TalonFX m_rightLeader;
  private TalonFX m_rightFollower;
  //private Pigeon2 m_gyro = new Pigeon2(13);

  //DutyCycle
  private final DutyCycleOut leftOut;
  private final DutyCycleOut rightOut;

  //DriveTrain Configurations
  private TalonFXConfiguration m_rightConfiguration = new TalonFXConfiguration();
  private TalonFXConfiguration m_leftConfiguration = new TalonFXConfiguration();
  private CurrentLimitsConfigs m_currentConfig = new CurrentLimitsConfigs();

  //Encoders
  private Encoder m_rightEncoder = new Encoder(1, 2);
  private Encoder m_leftEncoder = new Encoder(3, 4);

  //Odometry and ClosedLoopControl
  private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0.525);
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ChassisConstants.k_chasssisKS,ChassisConstants.k_chasssisKV,ChassisConstants.k_chasssisKA);
  private PIDController LeftPIDController = new PIDController(ChassisConstants.k_chasssisKP, 0, 0);
  private PIDController righController = new PIDController(ChassisConstants.k_chasssisKP, 0, 0);
 // private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0);
  private RobotConfig config;

  public DriveTrain() {
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    leftOut = new DutyCycleOut(0);
    rightOut = new DutyCycleOut(0);

    //Initialize Hardware
    m_leftLeader = new TalonFX(ChassisConstants.k_leftLeader);
    m_leftFollower = new TalonFX(ChassisConstants.k_leftFollower);
    m_rightLeader = new TalonFX(ChassisConstants.k_rightLeader);
    m_rightFollower = new TalonFX(ChassisConstants.k_rightFollower);
   // m_gyro = new Pigeon2(ChassisConstants.k_pygeon);

    //Motor configuration
    m_currentConfig.StatorCurrentLimitEnable = true;
    m_currentConfig.StatorCurrentLimit = ChassisConstants.k_statorLimit;
    m_currentConfig.SupplyCurrentLimitEnable = true;
    m_currentConfig.SupplyCurrentLimit = ChassisConstants.k_supplyLimit;

    m_leftConfiguration.CurrentLimits = m_currentConfig;  
    m_leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_rightConfiguration.CurrentLimits = m_currentConfig;
    m_rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
  
    m_leftLeader.getConfigurator().apply(m_leftConfiguration);
    m_leftFollower.getConfigurator().apply(m_leftConfiguration);
    m_rightFollower.getConfigurator().apply(m_rightConfiguration);
    m_rightFollower.getConfigurator().apply(m_rightConfiguration);
    
    m_leftLeader.setNeutralMode(NeutralModeValue.Brake);
    m_leftFollower.setNeutralMode(NeutralModeValue.Brake);
    m_rightLeader.setNeutralMode(NeutralModeValue.Brake);
    m_rightFollower.setNeutralMode(NeutralModeValue.Brake);
    
    m_leftFollower.setControl(new Follower(m_leftLeader.getDeviceID(), false));
    m_rightFollower.setControl(new Follower(m_rightLeader.getDeviceID(), false));

    m_leftLeader.setSafetyEnabled(true);
    m_rightLeader.setSafetyEnabled(true);
    
    //Encoder setup
    m_rightEncoder.setDistancePerPulse(ChassisConstants.k_encoderDistancePerPulse);
    m_leftEncoder.setDistancePerPulse(ChassisConstants.k_encoderDistancePerPulse);

    m_rightEncoder.setReverseDirection(false);  
    m_leftEncoder.setReverseDirection(true);  
      
    resetPosition();
/* 
    resetPosition();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

     try{
    AutoBuilder.configure(
      this::getPose, 
      this::resetPose, 
      this::getChassisSpeeds, 
      this::driveChassisSpeeds,
      new PPLTVController(0.02,4.8),
      config,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, 
      this);
      }catch(Exception e){
        DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
      }*/
  }

  public void controlledDrive(double fwd, double rot){
    double x = Math.abs(fwd) > 0.09 ? -fwd*0.6 : 0;
    double y = Math.abs(rot) > 0.09 ? rot*0.6 : 0;
    leftOut.Output = x + y;
    rightOut.Output = x - y;
    m_leftLeader.setControl(leftOut);
    m_rightLeader.setControl(rightOut);
  }

  //PathPlanner Shenanigans
  /* 
   public Pose2d getPose(){
   return m_odometry.getPoseMeters();
    
  }

  public void resetPose(Pose2d newPose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), newPose);
  }
*/
  public double getDistance(){
    // Return the process variable measurement here...
    double leftDistance = m_leftEncoder.getDistance();
    double rightDistance = m_rightEncoder.getDistance();
    return (leftDistance + rightDistance) / 2;
  }

  public ChassisSpeeds getChassisSpeeds(){
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    return m_kinematics.toChassisSpeeds(wheelSpeeds);   
  }

  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    
    
    DifferentialDriveWheelSpeeds speeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
    
    
    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
      LeftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
      righController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
      
    final VoltageOut leftvoltage = new VoltageOut((leftOutput + leftFeedforward));
    final VoltageOut righVoltage = new VoltageOut((rightOutput + rightFeedforward));

    m_leftLeader.setControl(leftvoltage);
    m_rightLeader.setControl(righVoltage);

    
  }

  //Odometry Functions

  public void resetPosition(){
    m_leftLeader.setPosition(0);
    m_rightLeader.setPosition(0);
   // m_gyro.setYaw(0);
   // m_gyro.reset();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   /*  SmartDashboard.putNumber("Angle", m_gyro.getYaw().getValueAsDouble());
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    SmartDashboard.putNumber("X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y", m_odometry.getPoseMeters().getY()); */
  }
}