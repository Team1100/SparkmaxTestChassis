// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

public class AutoDriveTest extends CommandBase {
  private final Drive m_drive; 
  
  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");
  String data = "";
  int m_counter = 0;
  double m_startTime = 0;
  double m_priorAutoSpeed = 0;
  double autospeed = 0;
  double[] m_numberArray = new double[10];
  ArrayList<Double> m_entries = new ArrayList<>();


  /** Creates a new TankDrive. */
  public AutoDriveTest(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
      m_drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetRightEncoder();
    m_drive.resetLeftEncoder();
    m_startTime = Timer.getFPGATimestamp();
    m_counter = 0;
    m_priorAutoSpeed = 0;
    autospeed = 0;
    data = "";
    telemetryEntry.setString(data);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    // The battery voltage is scaled to 12V
    
    double battery = RobotController.getBatteryVoltage();

    double motorVolts = battery * Math.abs(m_priorAutoSpeed);

    double leftMotorVolts = motorVolts;
    double rightMotorVolts = motorVolts;
 
    double autospeed = m_priorAutoSpeed -.001;
    //autospeed = -0.6;
    double runTime = now - m_startTime;
    SmartDashboard.putNumber("Run Time", runTime);
    //if (runTime >= 3){
    //  autospeed = 0.0;
    //}

    SmartDashboard.putNumber("autospeed", autospeed);
    m_priorAutoSpeed = autospeed;

    m_drive.setRightMotorSpeed(autospeed);
    m_drive.setLeftMotorSpeed(autospeed);

  
   SmartDashboard.putNumber("Auto Speed", autospeed);

   double leftCurrent = m_drive.getleftMotoCurrnt();
   SmartDashboard.putNumber("Left Current", leftCurrent);

   double rightCurrent = m_drive.getrightMotoCurrnt();
   SmartDashboard.putNumber("Right Current", rightCurrent);

   double leftTemp = m_drive.getleftMotorTemp();
   SmartDashboard.putNumber("Left Temp", leftTemp);
   
   double rightTemp = m_drive.getrightMotorTemp();
   SmartDashboard.putNumber("Right Temp", rightTemp);

   double rightRate = m_drive.getrightMotorRate();
   SmartDashboard.putNumber("Right Rate", rightRate);

   double leftRate = m_drive.getleftMotorRate();
   SmartDashboard.putNumber("Left Rate", leftRate);

   double rightPosition = m_drive.getrightMotorPosition();
   SmartDashboard.putNumber("Right Position", rightPosition);

   double leftPosition = m_drive.getleftMotorPosition();
   SmartDashboard.putNumber("Left Position", leftPosition);
 
   double gyroAngle = m_drive.getgyro();
   SmartDashboard.putNumber("Gryo", gyroAngle);

   m_numberArray[0] = now;
   m_numberArray[1] = battery;
   m_numberArray[2] = autospeed;
   m_numberArray[3] = leftCurrent;
   m_numberArray[4] = rightCurrent;
   m_numberArray[5] = leftPosition;
   m_numberArray[6] = rightPosition;
   m_numberArray[7] = leftRate;
   m_numberArray[8] = rightRate;
   m_numberArray[9] = gyroAngle;

   // Add data to a string that is uploaded to NT
   for (double num : m_numberArray) {
     m_entries.add(num);
   }
   m_counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //double elapsedTime = Timer.getFPGATimestamp() - startTime;
 
    //drive.tankDrive(0, 0);
    // data processing step
    data = m_entries.toString();
    data = data.substring(1, data.length() - 1) + ", ";
    telemetryEntry.setString(data);
    m_entries.clear();
    data = "";
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
