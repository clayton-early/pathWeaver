public class Main{
    //Need m_robotDrive, as setting up the drive. I don't know what we are going to use for this at the moment
    //Uses meters
    //This example uses tank drive, but we can change that
    //Drive train creation example:
    //https://docs.wpilib.org/en/latest/docs/software/examples-tutorials/trajectory-tutorial/creating-drive-subsystem.html
    //This example uses a 2 wheel drive system, so you may have to add a few things for a 4 wheel system

    // TUNE THESE VARIABLES
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    // https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-characterization/introduction.html
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    /*P value in PID*/public static final double kPDriveVel = 8.5;
    //tune these variables end

    /*horisontal distance between wheels in meters*/public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    /*The maximum speed that our robot can drive*/public static final double kMaxSpeedMetersPerSecond = 3;
    /*The maximum acceleration for our robot*/public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    //tuning these shouldnt be needed if we are using meters
    //if they are, go here: https://docs.wpilib.org/en/latest/docs/software/advanced-control/trajectories/ramsete.html#constructing-the-ramsete-controller-object
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), kDriveKinematics, /*our battery. Purposefully set a little lower to account for overshoot*/10);

     // Create config for trajectory
     TrajectoryConfig config =
     new TrajectoryConfig(kMaxSpeedMetersPerSecond,
                          kMaxAccelerationMetersPerSecondSquared)
         // Add kinematics to ensure max speed is actually obeyed
         .setKinematics(kDriveKinematics)
         // Apply the voltage constraint
         .addConstraint(autoVoltageConstraint);

    //Change this value to the path of our new trajectory
    //current path is sample that I made
    Trajectory trajectory = TrajectoryUtil.fromPathweaverJSON(Paths.get("output/Unnamed_0.wpilib.json"));


    public Command getAutonomousCommand() {
    
        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            /*needs a system that gives the current pose(location)*/m_robotDrive::getPose,
            new RamseteController(kRamseteB, kRamseteZeta),
            new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), kDriveKinematics,
            /*We should return the wheel speeds in our drive*/m_robotDrive::getWheelSpeeds,
            new PIDController(kPDriveVel, 0, 0),
            new PIDController(kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            /*Will output volts*/m_robotDrive::tankDriveVolts,
            m_robotDrive
        );
    
        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
      }

}