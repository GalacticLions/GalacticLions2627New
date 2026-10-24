package org.firstinspires.ftc.team5898.Hardware;

import androidx.annotation.NonNull;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

public class Constants {
  private Constants() {}
  
  /** Hardware device names */
  public static class Hardware {
      @NonNull
      public static String MOTOR_FRONT_LEFT = "FL";
      @NonNull
      public static String MOTOR_FRONT_RIGHT = "FR";
      @NonNull
      public static String MOTOR_BACK_LEFT = "BL";
      @NonNull
      public static String MOTOR_BACK_RIGHT = "BR";
      @NonNull
      public static String MOTOR_LAUNCHER_RIGHT = "LR";
      @NonNull
      public static String MOTOR_LAUNCHER_LEFT = "LL";
      @NonNull
      public static String MOTOR_INTAKE = "intake";
      @NonNull
      public static String IMU = "imu";
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
    public static double kP = 250.0;
    @Sorter(sort = 1)
    public static double kI = 0.0;
    @Sorter(sort = 2)
    public static double kD = 0.1;
    @Sorter(sort = 3)
    public static double kF = 25.0;
  }

  @Configurable
  public static class PIDFLauncherRight {
    @Sorter(sort = 4)
    public static double kP = 250.0;
    @Sorter(sort = 5)
    public static double kI = 0.0;
    @Sorter(sort = 6)
    public static double kD = 0.1;
    @Sorter(sort = 7)
    public static double kF = 25.0;
  }

    @Configurable
    public static class Gate {
        @Sorter(sort = 8)
        public static double GATE_OPEN = 1.0;
        @Sorter(sort = 9)
        public static double GATE_CLOSED = 0.0;
        @Sorter(sort = 26)
        public static double GATE_STEP = 0.1; // how far one dpad press moves the gate in GateTuner
    }

    @Configurable
    public static class TeleOp {
        @Sorter(sort = 13)
        public static double INTAKE_POWER = 1.0;
        @Sorter(sort = 14)
        public static double LAUNCHER_VELOCITY = 1200; // ticks/sec, tune on robot
        @Sorter(sort = 15)
        public static double LAUNCHER_TOLERANCE = 200.0; // ticks/sec
    }

    @Configurable
    public static class Auto {
        @Sorter(sort = 16)
        public static double DRIVE_POWER = 0.7;
        @Sorter(sort = 17)
        public static double STRAFE_MULTIPLIER = 1.0; // mecanum slips sideways, tune on robot
        @Sorter(sort = 18)
        public static double TURN_KP = 0.02;
        @Sorter(sort = 19)
        public static double TURN_MAX_POWER = 0.5;
        @Sorter(sort = 20)
        public static double TURN_TOLERANCE = 1.0;
        @Sorter(sort = 21)
        public static double STEP_TIMEOUT_SEC = 5.0;
        @Sorter(sort = 22)
        public static double SHOOT_TIME_SEC = 5.0; // no ball sensor, so shoot for a fixed time
        @Sorter(sort = 23)
        public static double REFILL_CUTOFF_SEC = 0.0; // skip the garden refill if the first SHOOT ends later than this; 0 = off
        @Sorter(sort = 24)
        public static double LAUNCHER_TOLERANCE = 150.0; // ticks/sec, looser than TeleOp so a shot's dip doesn't close the gate
        @Sorter(sort = 27)
        public static double DRIVE_ACCEL_SEC = 0.5; // seconds to ramp 0 -> full power, must be > 0; lower = faster, too low slips
        @Sorter(sort = 28)
        public static double DRIVE_DECEL_INCHES = 16; // inches to brake from full power, must be > 0; lower = faster, too low overshoots
        @Sorter(sort = 29)
        public static double DRIVE_MIN_POWER = 0.15; // enough to keep the robot moving at the end
        @Sorter(sort = 30)
        public static double DRIVE_TOLERANCE_INCHES = 0.5;
        @Sorter(sort = 31)
        public static double TURN_MIN_POWER = 0.1; // just enough to overcome friction, so turns don't stall short of the target
    }

  @Configurable
  public static class Limelight {
      @Sorter(sort = 10)
    public static double TARGET_AREA_THRESHOLD = 2.0;
      @Sorter(sort = 11)
    public static double BLUE_ALLIANCE_TX = 3.30;
      @Sorter(sort = 12)
    public static double RED_ALLIANCE_TX = 0.31;
      @Sorter(sort = 25)
      public static double MAX_STALENESS_MS = 100.0; // ignore frames older than this, so a frozen camera stops the robot
  }
}
