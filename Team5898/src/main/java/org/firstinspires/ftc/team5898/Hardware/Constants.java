package org.firstinspires.ftc.team5898.Hardware;

import androidx.annotation.NonNull;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

public class Constants {
  private Constants() {}
  
  /** Hardware device names */
  public static class Hardware {
    @NonNull public static String MOTOR_FRONT_LEFT = "fl";
    @NonNull public static String MOTOR_FRONT_RIGHT = "fr";
    @NonNull public static String MOTOR_BACK_LEFT = "bl";
    @NonNull public static String MOTOR_BACK_RIGHT = "br";
      @NonNull
      public static String MOTOR_LAUNCHER_RIGHT = "lr";
      @NonNull
      public static String MOTOR_LAUNCHER_LEFT = "ll";
      @NonNull
      public static String MOTOR_INTAKE = "intake";
    @NonNull public static String IMU = "imu";
      @NonNull
      public static String SERVO_GATE = "gate";
  }

  /** Robot physical dimensions and drivetrain parameters; Basic Autonomous only!!! */
  public static class Drive {
    public static double COUNTS_PER_ROTATION   = 537.7;
    public static double WHEEL_DIAMETER_INCHES = 3.779;
    public static double DRIVE_GEAR_REDUCTION  = 1.0;
    public static double COUNTS_PER_INCH =
        (COUNTS_PER_ROTATION * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_INCHES * Math.PI);
  }

  /** PIDF values with default values for Launcher1 and Launcher2. Use with DcMotorEx class */
  @Configurable
  public static class PIDFLauncherLeft {
    @Sorter(sort = 0)
    public static double kP = 0.0;
    @Sorter(sort = 1)
    public static double kI = 0.0;
    @Sorter(sort = 2)
    public static double kD = 0.0;
    @Sorter(sort = 3)
    public static double kF = 0.0;
  }

  @Configurable
  public static class PIDFLauncherRight {
    @Sorter(sort = 4)
    public static double kP = 0.0;
    @Sorter(sort = 5)
    public static double kI = 0.0;
    @Sorter(sort = 6)
    public static double kD = 0.0;
    @Sorter(sort = 7)
    public static double kF = 0.0;
  }

    @Configurable
    public static class Gate {
        @Sorter(sort = 8)
        public static double GATE_OPEN = 0.5;
        @Sorter(sort = 9)
        public static double GATE_CLOSED = 1.0;
    }

    @Configurable
    public static class TeleOp {
        @Sorter(sort = 13)
        public static double INTAKE_POWER = 1.0;
        @Sorter(sort = 14)
        public static double LAUNCHER_VELOCITY = 2000.0; // ticks/sec, tune on robot
        @Sorter(sort = 15)
        public static double LAUNCHER_TOLERANCE = 50.0; // ticks/sec
    }

    @Configurable
    public static class Auto {
        @Sorter(sort = 16)
        public static double DRIVE_POWER = 0.5;
        @Sorter(sort = 17)
        public static double STRAFE_MULTIPLIER = 1.1; // mecanum slips sideways, tune on robot
        @Sorter(sort = 18)
        public static double TURN_KP = 0.02;
        @Sorter(sort = 19)
        public static double TURN_MAX_POWER = 0.5;
        @Sorter(sort = 20)
        public static double TURN_TOLERANCE = 1.0;
        @Sorter(sort = 21)
        public static double STEP_TIMEOUT_SEC = 5.0;
    }

  @Configurable
  public static class Limelight {
      @Sorter(sort = 10)
    public static double TARGET_AREA_THRESHOLD = 2.0;
      @Sorter(sort = 11)
    public static double BLUE_ALLIANCE_TX = 3.30;
      @Sorter(sort = 12)
    public static double RED_ALLIANCE_TX = 0.31;
  }
}
