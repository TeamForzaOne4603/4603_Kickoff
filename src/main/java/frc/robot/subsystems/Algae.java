// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * Dead Code
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 */

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgueConstants;

public class Algae extends SubsystemBase {
  private static Algae mInstance;

  public static Algae getInstance() {
    if (mInstance == null) {
      mInstance = new Algae();
    }
    return mInstance;
  }

  /** Creates a new Algae. */
  private SparkMax mWristMotor;
  private RelativeEncoder encoder;
  private final ProfiledPIDController mWristPIDController;
  private final ArmFeedforward mWristFeedForward;
  private double setPoint = 0;
  private int state = 0;

  private SparkMax mIntakeMotor;

  public Algae() {
    mWristMotor = new SparkMax(AlgueConstants.k_WristhMotor, MotorType.kBrushless);
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    wristConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AlgueConstants.k_currentBrazo)
        .inverted(true);

    mWristMotor.configure(
        wristConfig,
        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    mWristPIDController = new ProfiledPIDController(
      AlgueConstants.kWristP,
        AlgueConstants.kWristI,
        AlgueConstants.kWristD,
        new TrapezoidProfile.Constraints(
          AlgueConstants.kWristMaxVelocity,
            AlgueConstants.kWristMaxAcceleration));

    // Wrist Feedforward
    mWristFeedForward = new ArmFeedforward(
        AlgueConstants.kWristKS,
        AlgueConstants.kWristKG,
        AlgueConstants.kWristKV,
        AlgueConstants.kWristKA);

    mIntakeMotor = new SparkMax(AlgueConstants.k_AlgueMotor, MotorType.kBrushless);
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
    
    intakeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(AlgueConstants.k_currentBrazo)
            .inverted(true);
    
        mIntakeMotor.configure(
            intakeConfig,
            com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);  
            
    encoder = mWristMotor.getEncoder();
    state = 0;
  }

  

  @Override
  public void periodic() {
    double pidCalc = mWristPIDController.calculate(getWristAngle(), setPoint);
    double ffCalc = mWristFeedForward.calculate(Math.toRadians(getWristReferenceToHorizontal()),
        Math.toRadians(mWristPIDController.getSetpoint().velocity));

    //mWristMotor.setVoltage(pidCalc + ffCalc); 


    SmartDashboard.putNumber("Brazo", encoder.getPosition());

    /*switch (state) {
      case 0:
        
        break;
      case 1:
        
        break;
      case 2:
      mIntakeMotor.set(-0.2);
        break;
      case 3:
      mIntakeMotor.set(0.2);
        break;

      case 4:
        
          break;
    
      default:
        break;*/
    }
  

  public Command take (double speed, double setpoint){
    return runEnd(()->{
      setPoint = setpoint;
      if (isInPosition()) {
        mIntakeMotor.set(speed);
      }
          
    }, 
    ()-> {
      if (speed > 0) {
        state = 3;
      } else {
        state = 2;
      }
      mIntakeMotor.set(0);
    });
  }


  public Command shoot(){
    return runEnd(()-> {
      /*if (state == 2 || state == 1) {
        state = 1;
        
      } else if (state == 3 || state == 4) {
        state = 4;
        mIntakeMotor.set(-0.3);*/
      //}
      mWristMotor.set(0.8);
    }, 
    ()-> {state = 0;
      mIntakeMotor.set(0);
    });
  }

  

  public Command goUp(){
    return runEnd(()-> {mWristMotor.set(0.4);}, ()-> {mWristMotor.set(0);});
  }
  public Command goDown(){
    return runEnd(()-> {mWristMotor.set(-0.4);}, ()-> {mWristMotor.set(0);});
  }

  public Command reset(){
    return runOnce(()-> {encoder.setPosition(0);});
  }

  public Command goToPosition(double Position){
    return runOnce(()->{setPoint = Position;});
  }

  public Command resetPosition(double Position){
    return runOnce(()->{encoder.setPosition(0);});
  }


  public double getWristAngle() {
    return encoder.getPosition() * 1.9;
  }

  public double getWristReferenceToHorizontal() {
    return getWristAngle() + AlgueConstants.kWristOffset;
  }

    public boolean isInPosition(){
      if(encoder.getPosition() >= setPoint + 0.3 || encoder.getPosition() <= setPoint - 0.3){
          return true;
      } else {
          return false;
      }
  }
}
/*
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * Dead Code
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 */