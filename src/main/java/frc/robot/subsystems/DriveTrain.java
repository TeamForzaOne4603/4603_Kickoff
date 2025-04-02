// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants;

public class DriveTrain extends SubsystemBase {
  private static DriveTrain mInstance;

  public static DriveTrain getInstance() {
    if (mInstance == null) {
      mInstance = new DriveTrain();
    }
    return mInstance;
  }


  //DriveTrain Motors
  private TalonFX m_leftLeader;
  private TalonFX m_leftFollower;
  private TalonFX m_rightLeader;
  private TalonFX m_rightFollower;
  private Pigeon2 m_gyro = new Pigeon2(ChassisConstants.k_pygeon);
  private Orchestra musics = new Orchestra();

  //DutyCycle
  private final DutyCycleOut leftOut;
  private final DutyCycleOut rightOut;

  //DriveTrain Configurations
  private TalonFXConfiguration m_rightConfiguration = new TalonFXConfiguration();
  private TalonFXConfiguration m_leftConfiguration = new TalonFXConfiguration();
  private CurrentLimitsConfigs m_currentConfig = new CurrentLimitsConfigs();
  private SlewRateLimiter accelerationRamp = new SlewRateLimiter(20, -1.5, 0);

  //Encoders
  private Encoder m_rightEncoder = new Encoder(6, 7);
  private Encoder m_leftEncoder = new Encoder(8, 9);

  //Odometry and ClosedLoopControl
  private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0.525);
  private SimpleMotorFeedforward rightfeedforward = new SimpleMotorFeedforward(ChassisConstants.k_chasssisKS,ChassisConstants.k_chasssisKV,ChassisConstants.k_chasssisKA);
  private SimpleMotorFeedforward leftfeedforward = new SimpleMotorFeedforward(ChassisConstants.k_chasssisKS,ChassisConstants.k_chasssisKV,ChassisConstants.k_chasssisKA);

  private PIDController LeftPIDController = new PIDController(2.4, 0, 0);
  private PIDController righController = new PIDController(2.4, 0, 0);
  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0);
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
    m_gyro = new Pigeon2(ChassisConstants.k_pygeon);

    musics.addInstrument(m_leftFollower,0);
    musics.addInstrument(m_rightFollower,1);
    musics.addInstrument(m_leftLeader,3);
    musics.addInstrument(m_rightLeader,2);
    var status = musics.loadMusic("cucaracha.chrp");
  
    //Motor configuration
    m_currentConfig.StatorCurrentLimitEnable = true;
    m_currentConfig.StatorCurrentLimit = ChassisConstants.k_statorLimit;
    m_currentConfig.SupplyCurrentLimitEnable = true;
    m_currentConfig.SupplyCurrentLimit = ChassisConstants.k_supplyLimit;

    m_leftConfiguration.CurrentLimits = m_currentConfig;  
    m_leftConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_leftConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.6;
    m_rightConfiguration.CurrentLimits = m_currentConfig;
    m_rightConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
    m_rightConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.6;
  
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

    m_leftLeader.setSafetyEnabled(false);
    m_rightLeader.setSafetyEnabled(false);
    
    //Encoder setup
    m_rightEncoder.setDistancePerPulse(ChassisConstants.k_encoderDistancePerPulse);
    m_leftEncoder.setDistancePerPulse(ChassisConstants.k_encoderDistancePerPulse);

    m_rightEncoder.setReverseDirection(false);  
    m_leftEncoder.setReverseDirection(true);  

    resetPosition();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

     try{
    AutoBuilder.configure(
      this::getPose, 
      this::resetPose, 
      this::getChassisSpeeds, 
      this::driveChassisSpeeds,
      new PPLTVController(0.02,1.5),
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
      }
  }
  //Manual Drive
  public void controlledDrive(double fwd, double rot, Boolean turbo){
    double turbos = 1;
    if(turbo){
      turbos = 1.5;
    } else {
      turbos = 1;
    }
    double x = Math.abs(fwd) > 0.11 ? accelerationRamp.calculate(-fwd*0.50 * turbos) : accelerationRamp.calculate(0); //Get forward axis, 0.11 deadband
    double y = Math.abs(rot) > 0.11 ? rot*0.3 : 0; //Get rotational axis, 0.11 deadband
    double limit = NewElevator.getInstance().getPosition() > 28 ? 0.5 : 1; // If elevator is up, the chassis gets slower
   
    leftOut.Output = (x + y) * limit;
    rightOut.Output = (x - y) * limit;
    m_leftLeader.setControl(leftOut);
    m_rightLeader.setControl(rightOut); 
  }

  //Odometry functions
   public Pose2d getPose(){
   return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d newPose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), newPose);
  }

  public ChassisSpeeds getChassisSpeeds(){
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    return m_kinematics.toChassisSpeeds(wheelSpeeds);   
  }

  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    DifferentialDriveWheelSpeeds speeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
    
    speeds.desaturate(3);
    
    final double leftFeedforward = leftfeedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = rightfeedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
      LeftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
      righController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
      
    final VoltageOut leftvoltage = new VoltageOut(-(leftOutput + leftFeedforward));
    final VoltageOut righVoltage = new VoltageOut(-(rightOutput + rightFeedforward));

    m_leftLeader.setControl(leftvoltage);
    m_rightLeader.setControl(righVoltage);
  }
  //Odometry Functions

  public void resetPosition(){
    m_leftLeader.setPosition(0);
    m_rightLeader.setPosition(0);
    m_gyro.setYaw(0);
    m_gyro.reset();
  }

  public Command stop(){
    return runEnd(() -> {musics.play();},()->{musics.stop();});}
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    SmartDashboard.putNumber("Angle", m_gyro.getYaw().getValueAsDouble());
    SmartDashboard.putNumber("X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y", m_odometry.getPoseMeters().getY()); 
    SmartDashboard.putNumber("temp sup izq", m_leftLeader.getDeviceTemp().getValueAsDouble()); 
    SmartDashboard.putNumber("temp inf izq", m_leftFollower.getDeviceTemp().getValueAsDouble()); 
    SmartDashboard.putNumber("temp sup der", m_rightLeader.getDeviceTemp().getValueAsDouble()); 
    SmartDashboard.putNumber("temp inf der", m_rightFollower.getDeviceTemp().getValueAsDouble()); 
    SmartDashboard.putNumber("izq", m_leftEncoder.getDistance()); 
    SmartDashboard.putNumber("der", m_rightEncoder.getDistance());
  }
}