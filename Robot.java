/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private final WPI_TalonSRX m_LeftBack = new WPI_TalonSRX(4);
  private final WPI_VictorSPX m_LeftFront = new WPI_VictorSPX(3);
  private final WPI_VictorSPX m_RightBack = new WPI_VictorSPX(2);
  private final WPI_TalonSRX m_RightFront = new WPI_TalonSRX(1);
  private final SpeedControllerGroup m_LeftMotors = new SpeedControllerGroup(m_LeftBack, m_LeftFront);
  private final SpeedControllerGroup m_RightMotors = new SpeedControllerGroup(m_RightBack, m_RightFront);
  private final DifferentialDrive m_Drive = new DifferentialDrive(m_LeftMotors, m_RightMotors);

  private final XboxController m_Controller = new XboxController(1);

  // LIMELIGHT STUFF
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
  private double tv, tx, ty, ta;
  private double error, integralError, derivativeError, previousError;

  // CONSTANTS TO TUNE/SET
  private static final double kSteer = 0.12; // how hard to turn toward the target
  private static final double kProportional = 0.80; // proportional constant
  private static final double kIntegral = 0.00; // integral constant
  private static final double kDerivative = 0.00; // derivative constant
  private static final double kFeedForward = 0.05; // feedforward constant
  private static final double kMaxDrive = 0.60; // Simple speed limit so we don't drive too fast
  private static final double kMaxTurn = 0.70; // ^ turn limit
  private static final double kTargetHeight = 92; // height of target above floor (in)
  private static final double kCameraHeight = 47.5; // height of camera above floor (in)
  private static final double kMountingAngle = 30; // angle that camera is mounted at (deg)

  // CLOSEST TARGET STUFF
  private static final double[] kTargetAreas = { 6.0, 2.5, 0.85 }; // INPUT YOUR WANTED TARGETAREA VALUES HERE
  private static final double[] kTargetDistances = { 20, 40, 60 }; // INPUT YOUR WANTED DISTANCES HERE
  private static double[] distancesToTargets = new double[kTargetAreas.length]; // change this to the length of whichever array you are using
  private static double closestTargetArea = 0;
  private static double closestTargetDistance = 0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_LeftBack.setInverted(false);
    m_LeftFront.setInverted(false);
    m_RightBack.setInverted(false);
    m_RightFront.setInverted(false);

    m_LeftMotors.setInverted(false);
    m_RightMotors.setInverted(false);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {

    updateLimelightTracking();

    double steer = m_Controller.getX(Hand.kRight) * kMaxTurn;
    double drive = -m_Controller.getY(Hand.kLeft) * kMaxDrive;

    if (m_Controller.getAButton()) { // if A button pressed
      moveWithLimelight(getClosestTargetArea(kTargetAreas, ta)); // use closest target area function
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); // set to pipeline 0
    } else if (m_Controller.getBButton()) { // if B button pressed
      // moveWithLimelight(getClosestTargetDistance(kTargetDistances, distanceToTarget(ty))); // use closest target distance function
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); // set to pipeline 0
      m_Drive.arcadeDrive(0, 0);
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1); // set to pipeline 1
      SmartDashboard.putNumber("Closest Target (Area)", 0); // set this to zero when driver pipeline is active so ppl don't get confused
      SmartDashboard.putNumber("Closest Target (Distance)", 0); // ^
      m_Drive.arcadeDrive(drive, steer); // otherwise drive normally
    }
  }

  @Override
  public void testPeriodic() {
  }

  /**
   * This function implements a simple method of generating driving and steering
   * commands based on the tracking data from a limelight camera.
   */
  public void updateLimelightTracking() {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    SmartDashboard.putNumber("Target Detected", tv);
    SmartDashboard.putNumber("Horizontal Error", tx);
    SmartDashboard.putNumber("Vertical Error", ty);
    SmartDashboard.putNumber("Target Area", ta);
    SmartDashboard.putNumber("Closest Target (Area)", closestTargetArea);
    SmartDashboard.putNumber("Closest Target (Distance)", closestTargetDistance);

    // using target area
    closestTargetArea = getClosestTargetArea(kTargetAreas, ta);

    // using target distance
    closestTargetDistance = getClosestTargetDistance(kTargetDistances, distanceToTarget(ty));

    if (tv < 1.0) { // IF TARGET IS DETECTED
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      return;
    }

    m_LimelightHasValidTarget = true;
  }

  // function that returns the distance to the target
  public double distanceToTarget(double targetAngle) {
    /*
     * Math: http://docs.limelightvision.io/en/latest/cs_estimating_distance.html
     * 
     * tangent = opposite / adjacent 
     * tan(a1+a2) = (h2-h1) / d
     * 
     * where: a1 = angle the camera is mounted at (probably somewhere between 0-90deg) 
     *        a2 = angle of the camera to the target (its the "ty" value output by limelight) 
     *        h2 = height of the target h1 = height of the camera
     * 
     * distance of camera to target = height from camera to target / angle of camera to target 
     * d = (h2-h1) / tan(a1+a2)
     */

     // the only thing that changes in this equation is targetAngle, which is ty in this project
    return (kTargetHeight - kCameraHeight) / Math.tan(kMountingAngle + targetAngle);
  }

  public void moveWithLimelight(double targetArea) {
    // use a proportional loop for steering
    double steerError = tx * kSteer;
    m_LimelightSteerCommand = steerError;

    // try to drive forward until the target area reaches our desired area
    error = targetArea - ta; // error = target - actual
    integralError += error * 0.02; // integral increased by error * time (0.02 seconds per loop)
    derivativeError = (error - previousError) / .02; // derivative = change in error / time (0.02 seconds per loop)

    // PID Loop for drive
    double driveError = (error*kProportional) + (integralError*kIntegral) + (derivativeError*kDerivative) + kFeedForward;

    // don't let the robot drive too fast into the goal
    if (driveError > kMaxDrive) {
      driveError = kMaxDrive;
    } else if (driveError < -kMaxDrive) {
      driveError = -kMaxDrive;
    }
    m_LimelightDriveCommand = driveError;

    previousError = error; // update previousError to current error

    if (m_LimelightHasValidTarget) { // if limelight sees target
      m_Drive.arcadeDrive(m_LimelightDriveCommand, m_LimelightSteerCommand); // drive using command-tuned values
    } else {
      m_Drive.arcadeDrive(0.0, 0.0); // otherwise do nothing
    }
  }

  // function that returns the value in kTargetAreas closest to your robot 
  public double getClosestTargetArea(double[] targetAreas, double currentTargetArea) {
    // setup the array containing distances to targets
    for (int i = 0; i < kTargetAreas.length; i++) {
      // if currentTA above target (array value), will give back negative value
      // if currentTA below target (array value), will give back positive value
      double thisTargetArea = kTargetAreas[i] - currentTargetArea;
      distancesToTargets[i] = thisTargetArea; // set index i of dist targets array
    }

    // find lowest distance to targets
    double lowestDistance = 100.0; // arbitrary initial value
    for (int i = 0; i < kTargetAreas.length; i++) {
      // take abs value of distance to target (read above comments)
      if (Math.abs(distancesToTargets[i]) < lowestDistance) {
        lowestDistance = distancesToTargets[i];
      }
    }

    // now with lowest distance get index of target area
    int targetIndex = 0;
    for (int i = 0; i < kTargetAreas.length; i++) {
      if (lowestDistance == distancesToTargets[i]) {
        targetIndex = i;
      }
    }

    return kTargetAreas[targetIndex];
  }

  // same function as above but uses target distance instead of the targetArea calculated by limelight
  public double getClosestTargetDistance(double[] targetAreas, double currentDistance) {
    // setup the array containing distances to targets
    for (int i = 0; i < kTargetDistances.length; i++) {
      // if currentDist above target (array value), will give back negative value
      // if currentDist below target (array value), will give back positive value
      double thisTargetDistance = kTargetDistances[i] - currentDistance;
      distancesToTargets[i] = thisTargetDistance; // set index i of dist to targets array
    }

    // find lowest distance to targets
    double lowestDistance = 100.0; // arbitrary initial value
    for (int i = 0; i < kTargetDistances.length; i++) {
      // take abs value of distance to target (read above comments)
      if (Math.abs(distancesToTargets[i]) < lowestDistance) {
        lowestDistance = distancesToTargets[i];
      }
    }

    // now with lowest distance get index of target distance
    int targetIndex = 0;
    for (int i = 0; i < kTargetDistances.length; i++) {
      if (lowestDistance == distancesToTargets[i]) {
        targetIndex = i;
      }
    }

    return kTargetDistances[targetIndex];
  }
}
