// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static class ChassisConstants{
        public static final int k_leftLeader = 3; 
        public static final int k_leftFollower = 2;
        public static final int k_rightLeader = 5;  
        public static final int k_rightFollower = 4;
        public static final int k_pygeon = 13;

        public static final int k_statorLimit = 140;
        public static final int k_supplyLimit = 65;

        public static final double k_encoderCPR = 2048;
        public static final double k_wheelDiameterMeters = Units.inchesToMeters(6);
        public static final double k_encoderDistancePerPulse = -(Math.PI * k_wheelDiameterMeters) / k_encoderCPR;

        public static final double k_chasssisKP = 1.0474;
        public static final double k_chasssisKV = 0.1392;
        public static final double k_chasssisKS = 1.0281;
        public static final double k_chasssisKA = 0.12516;
    }

    public static class ElevatorConstants{
        public static final  int k_rightMotor = 24;
        public static final  int k_leftMotor = 25;

        public static final double k_elevatorKP = 0.35;
        public static final double k_elevatorKI = 0;
        public static final double k_elevatorKD = 0;
        public static final double k_elevatorKIZone = 17;
        public static final double kG = 8.5;
        public static final double k_maxVelocity = 90;
        public static final double k_maxAcceleration = 180;

        public static final int k_supplyLimit = 40;

        public static final double kStowHeight = 15;
        public static final double kL2Height = 50;
        public static final double kL3Height = 80;
        public static final double kL4Height = 90;
        public static final double kMaxHeight = 26;
        public static final double kGroundAlgaeHeight = 0;
        public static final double kScoreAlgaeHeight = 0.0;
        public static final double kLowAlgaeHeight = 15;
        public static final double kHighAlgaeHeight = 12;
    }

    public static class AlgueConstants{
        public static final int k_AlgueMotor =23;
        public static final int k_WristhMotor = 20;

        public static final double k_position = 24.0;
        public static final int k_currentBrazo = 40;

        public static final double kWristP = 0.42;
    public static final double kWristI = 0.0;
    public static final double kWristD = 0.0;

    public static final double kWristKS = 0.03;
    public static final double kWristKG = 0.4;
    public static final double kWristKV = 0.150;
    public static final double kWristKA = 0.0;

    public static final double kWristOffset = 90;

    public static final double kWristMaxVelocity = 690.0;
    public static final double kWristMaxAcceleration = 1380.0;
    }

    public static class CoralConstants{
        
        public static final  int k_rightMotor = 26;
        public static final  int k_leftMotor = 22;
        public static final int k_laserCAN = 21;
    }

    public static class NewElevatorConstants{
        public static final  int k_rightMotor = 24;
        public static final  int k_leftMotor = 25;

        public static final double k_P = 1.6;
        public static final double k_I = 0;
        public static final double k_D = 0.001;
        public static final double k_IZone = 17;
        public static final double k_G = 0.11;
        public static final double k_maxVelocity = 55;
        public static final double k_maxAcceleration = 80;
        public static final double k_V = 0.34;
        public static final double k_S = 0.02;

        public static final double kStowHeight = 1;
        public static final double kL2Height = 10;
        public static final double kL3Height = 28;
        public static final double kL4Height = 60;
        public static final double kMaxHeight = 70;

        public static final double k_encoderCPR = 2048;
        public static final double k_wheelDiameterMeters = Units.inchesToMeters(1+(7/8));
        public static final double k_encoderDistancePerPulse = (Math.PI * k_wheelDiameterMeters) / k_encoderCPR*200;
    }

    public static class ClimberConstants {
        public static final int k_armId = 32;
    }
}
