// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Threads;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.obstacles.ObstacleAvoidance;
import frc.robot.subsystems.tracking.Tracking;
import frc.robot.subsystems.vision.Vision;
import frc.robot.superstructure.SS;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;


public class Robot extends LoggedRobot {

    private Drive drive;
    private Elevator elevator;
    private Vision vision;
    private Tracking tracking;
    private ObstacleAvoidance obstacles;
    private SS superstructure;
    private ControlScheme controls;


    @Override
    public void robotInit() {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        switch (Constants.currentMode) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter("U/logs/" + BuildConstants.GIT_BRANCH));
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                setUseTiming(false); 
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;

        }
        Logger.start();

        instantiateSubsystems();

        vision.enable();

        var modules = new SwerveModuleConstants[] {
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight
        };
        for (var constants : modules) {
            if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
                    || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
                throw new RuntimeException(
                        "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
            }
        }
        
        if (Constants.currentMode == Constants.Mode.SIM) {
            drive.resetGyroAndPose(0.0, new Pose2d(2.0, 2.0, new Rotation2d(0.0)));
        }
        
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);

        controls.update();
        superstructure.periodic();
        elevator.periodic();
        obstacles.periodic();
        tracking.periodic();
        drive.periodic();
        vision.periodic();

        PerfTracker.periodic();
        Threads.setCurrentThreadPriority(false, 10);

    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        controls.init();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }

    
    public void instantiateSubsystems() {
        elevator = Elevator.getInstance();
        obstacles = ObstacleAvoidance.getInstance();
        drive = Drive.getInstance();
        vision = Vision.getInstance();
        tracking = Tracking.getInstance();
        superstructure = SS.getInstance();
        controls = new ControlScheme(superstructure, drive);
    }

}
