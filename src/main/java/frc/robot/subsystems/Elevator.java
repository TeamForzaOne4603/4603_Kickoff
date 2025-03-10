// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  //No intentes entender esto aza, solo cambia las constantes si quieres cambiar la altura
  /** Creates a new Elevator. */
  private PeriodicIO mPeriodicIO;
  private static Elevator mInstance;

  public static Elevator getInstance() {
    if (mInstance == null) {
      mInstance = new Elevator();
    }
    return mInstance;
  }

  private SparkMax mLeftMotor;
  //private RelativeEncoder mLeftEncoder;
  private SparkClosedLoopController mLeftPIDController;

  private SparkMax mRightMotor;

  private TrapezoidProfile mProfile;
  private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
  private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
  private double prevUpdateTime = Timer.getFPGATimestamp();
  private Encoder mLeftEncoder = new Encoder(0, 1);
  private PIDController controller = new PIDController(1, 0, 0);
  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0.01, 8, 0.1);
  private double position = 0;

  public Elevator() {
    super("Elevator");
    mLeftEncoder.setDistancePerPulse(1/285.666);

    mLeftEncoder.setReverseDirection(false);  
    mLeftEncoder.reset();
    
    mPeriodicIO = new PeriodicIO();
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    elevatorConfig.closedLoop
        .pid(ElevatorConstants.k_elevatorKP, ElevatorConstants.k_elevatorKD, ElevatorConstants.k_elevatorKI)
        .iZone(ElevatorConstants.k_elevatorKIZone);

    elevatorConfig.smartCurrentLimit(ElevatorConstants.k_supplyLimit);

    elevatorConfig.idleMode(IdleMode.kBrake);
  //  elevatorConfig.limitSwitch.reverseLimitSwitchEnabled(true);

    mLeftEncoder.setDistancePerPulse(1/285.666);

    mLeftEncoder.setReverseDirection(false);  

  // LEFT ELEVATOR MOTOR
    mLeftMotor = new SparkMax(ElevatorConstants.k_leftMotor, MotorType.kBrushless);
    mLeftPIDController = mLeftMotor.getClosedLoopController();
    mLeftMotor.configure(elevatorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // RIGHT ELEVATOR MOTOR
    mRightMotor = new SparkMax(ElevatorConstants.k_rightMotor, MotorType.kBrushless);
    mRightMotor.configure(elevatorConfig.follow(mLeftMotor, true), com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);


    mProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            ElevatorConstants.k_maxVelocity,
            ElevatorConstants.k_maxAcceleration));
  }
  public enum ElevatorState {
    NONE,
    STOW,
    L2,
    L3,
    L4,
    A1,
    A2
  }

  private static class PeriodicIO {
    double elevator_target = 8;
    double elevator_power = 0.0;

    boolean is_elevator_pos_control = false;

    ElevatorState state = ElevatorState.STOW;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run/* 
       double curTime = Timer.getFPGATimestamp();
      double dt = curTime - prevUpdateTime;
      prevUpdateTime = curTime;
      if (mPeriodicIO.is_elevator_pos_control) {
        // Update goal
        mGoalState.position = mPeriodicIO.elevator_target;
  
        // Calculate new state
        prevUpdateTime = curTime;
        mCurState = mProfile.calculate(dt, mCurState, mGoalState);
  
        // Set PID controller to new state
        mLeftPIDController.setReference(
            mCurState.position,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ElevatorConstants.kG,
            ArbFFUnits.kVoltage);
       }else {
        mCurState.position = mLeftEncoder.getDistance();
        mCurState.velocity = 0;
        mLeftMotor.set(mPeriodicIO.elevator_power);}
       
      SmartDashboard.putNumber("Elevador", mLeftEncoder.getDistance());
      SmartDashboard.putNumber("Setpoint", mGoalState.position);

      mLeftMotor.set(controller.calculate(mLeftEncoder.getDistance(), position));
  }

  public double goToPosition(){
    var feed = feedforward.calculate(10);
    var pid = controller.calculate(20);
    return feed + pid;
  }

  public Command goL2(){
    return runOnce(()-> {position = 20;});
  }

     
  public void stop() {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = 0.0;

    mLeftMotor.set(0.0);
  }

  public ElevatorState getState() {
    return mPeriodicIO.state;
  }

  public void setElevatorPower(double power) {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = power;
  }

  public void goToElevatorStow() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = ElevatorConstants.kStowHeight;
    mPeriodicIO.state = ElevatorState.STOW;
  }

  public void goToElevatorL2() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = ElevatorConstants.kL2Height;
    mPeriodicIO.state = ElevatorState.L2;
  }

  public void goToElevatorL3() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = ElevatorConstants.kL3Height;
    mPeriodicIO.state = ElevatorState.L3;
  }

  public void goToElevatorL4() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = ElevatorConstants.kL4Height;
    mPeriodicIO.state = ElevatorState.L4;
  }

  public void goToAlgaeLow() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = ElevatorConstants.kLowAlgaeHeight;
    mPeriodicIO.state = ElevatorState.A1;
  }

  public void goToAlgaeHigh() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = ElevatorConstants.kHighAlgaeHeight;
    mPeriodicIO.state = ElevatorState.A2;
  }

  public Command goUp(){
    return runEnd(()-> {mLeftMotor.set(0.4);}, ()-> {mLeftMotor.set(0);});

  }
  public Command goDown(){
    return runEnd(()-> {mLeftMotor.set(-0.3);}, ()-> {mLeftMotor.set(0);});

  }
  public Command resetPosition(){
    return runOnce(()-> {mLeftEncoder.reset();});

  }
}

/*
 * // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.NewElevatorConstants;

public class NewElevator extends SubsystemBase {
  /** Creates a new NewElevator. 
  private static NewElevator mInstance;
  public static NewElevator getInstance() {
    if (mInstance == null) {
      mInstance = new NewElevator();
    }
    return mInstance;
  }

  private SparkMax m_leftMotor = new SparkMax(NewElevatorConstants.k_leftMotor, MotorType.kBrushless);
  private SparkMax m_rightMotor = new SparkMax(NewElevatorConstants.k_rightMotor, MotorType.kBrushless);
  private Encoder m_encoder = new Encoder(0, 1);

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(NewElevatorConstants.k_maxVelocity, NewElevatorConstants.k_maxAcceleration);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(NewElevatorConstants.k_P, NewElevatorConstants.k_I, NewElevatorConstants.k_D, m_constraints, 0.02);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(NewElevatorConstants.k_S, NewElevatorConstants.k_G, NewElevatorConstants.k_V);

  private boolean isInPositionControl = false;
  private double setPoint = 0;

  public NewElevator() {
    m_encoder.setDistancePerPulse(1/285.666);
    m_encoder.setReverseDirection(false);  

    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    elevatorConfig.smartCurrentLimit(ElevatorConstants.k_supplyLimit);
    elevatorConfig.idleMode(IdleMode.kBrake);
    elevatorConfig.limitSwitch.reverseLimitSwitchEnabled(false);
    m_leftMotor.configure(elevatorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // RIGHT ELEVATOR MOTOR
    m_rightMotor.configure(elevatorConfig.follow(m_leftMotor, true), com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isInPositionControl) {
      m_leftMotor.setVoltage(m_controller.calculate(m_encoder.getDistance(), setPoint) + m_feedforward.calculate(m_controller.getSetpoint().velocity));
    }
    SmartDashboard.putNumber("Elevator", m_encoder.getDistance());
  }
  

  //*Manual Commands
  public Command manualMove(double speed){
  runOnce(()->{  isInPositionControl = false;});
  return runEnd(()->{

  m_leftMotor.set(0.3);
 }, ()-> {
  m_leftMotor.set(0);
 });}
 

 //Position Commands
 public Command goToPosition (double position){
  return runOnce(()->{
    setPosition(position);
  });}

  public void setPosition(double position){
    isInPositionControl = true;
    setPoint = position;
  }
}
 */
