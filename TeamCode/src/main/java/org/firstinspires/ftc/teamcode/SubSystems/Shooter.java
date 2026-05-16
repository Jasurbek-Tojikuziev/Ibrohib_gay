package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Shooter {
    private DcMotorEx shooterMotor1, shooterMotor2;
    private Servo hood;
    private Servo shooterStop;
    private Servo intakeStop;

    public enum HoodPosition {
        CLOSE(0.0),   // ≤30 cm - flat angle
        MIDDLE(0.7),  // ~70 cm - medium angle
        FAR(1.0);     // 150+ cm - high angle

        public final double position;

        HoodPosition(double position) {
            this.position = position;
        }
    }

    public enum ShooterState {
        IDLE,
        OPEN_STOP,
        FEED,
        RESET
    }

    private static final double STOP_OPEN = 0.2;
    private static final double STOP_CLOSE = 0.4;
    private static final double INTAKE_STOP_ON = 0.9;   // Позиция во время стрельбы
    private static final double INTAKE_STOP_OFF = 1.0;  // Обычная позиция (не стреляем)
    private static final double OPEN_STOP_TIME = 0.06;

    // Single continuous feed time (FTC Dashboard tunable)
    // Start at 0.8s, decrease until all 3 balls reliably exit
    public static double FEED_TIME = 1.0;

    // SDK PIDF coefficients (RUN_USING_ENCODER, firmware-level ~1kHz)
    public static double PIDF_P        = 100.0;
    public static double PIDF_I        = 0.0;
    public static double PIDF_D        = 0.0;
    public static double PIDF_F        = 14;
    public static double TARGET_VELOCITY = 1300.0; // ticks/sec, fallback when no distance

    // Active braking: when current velocity exceeds target by DECEL_THRESHOLD,
    // command (target - DECEL_BOOST) so firmware PIDF brakes harder.
    // Tune DECEL_BOOST up if still too slow, down if flywheel overshoots below target.
    public static double DECEL_THRESHOLD = 50.0;  // ticks/sec above target to trigger braking
    public static double DECEL_BOOST     = 300.0; // ticks/sec below target to command during braking

    // Flywheel velocity formula coefficients (4th order polynomial)
    // y = -0.00000695632x^4 + 0.00131953x^3 - 0.0277327x^2 + 1.91371x + 917.18732
    public static double VELOCITY_A = -0.00000695632;
    public static double VELOCITY_B =  0.00131953;
    public static double VELOCITY_C = -0.0277327;
    public static double VELOCITY_D =  1.91371;
    public static double VELOCITY_E =  917.18732;

    public static double VELOCITY_READY_THRESHOLD = 0.93; // 93% of target = "at speed"

    // Velocity limits (clamp)
    private static final double MIN_VELOCITY = 0.0;        // Минимальная velocity
    private static final double MAX_VELOCITY = 1700.0;
    public static double FLYWHEEL_OFFSET = 0.0;           // Offset для калибровки (tunable)

    // Hood angle formula (sinusoidal)
    // y = 22640283.64 * sin(0.000001166157613x + 1.570374221) - 22640281.79
    public static double HOOD_AMP    = 22640283.64;
    public static double HOOD_FREQ   = 0.000001166157613;
    public static double HOOD_PHASE  = 1.570374221;
    public static double HOOD_SHIFT  = -22640281.79;

    // Hood angle limits
    private static final double MIN_HOOD_ANGLE = 0.0;
    private static final double MAX_HOOD_ANGLE = 0.6;
    public static double HOOD_OFFSET = 0.0;

    // Last-applied PIDF — used to detect FTC Dashboard changes (I and D locked to 0)
    private double lastP = PIDF_P, lastF = PIDF_F;
    private boolean prevDecelerating = false;

    private double targetVelocity = 0; // Целевая скорость в ticks/sec

    private HoodPosition currentHoodPosition = HoodPosition.MIDDLE;
    private ShooterState currentState = ShooterState.IDLE;
    private ElapsedTime stateTimer = new ElapsedTime();
    private boolean openStopExecuted = false;
    private boolean resetExecuted = false;
    private boolean manualStopOverride = false; // Ручное открытие shooterStop (приоритет над FSM)
    private double lastHoodDistance = -1; // Последнее расстояние для hood (-1 = не инициализировано)
    private double lastVelocityDistance = -1; // Последнее расстояние для velocity (-1 = не инициализировано)

    // --- Motion compensation (while-moving shooting) ---
    private Follower follower = null;
    private double motionGoalX = 0;
    private double motionGoalY = 0;

    // Time-of-flight lookup table (seconds vs distance in inches).
    // Calibrate by measuring actual flight time at each distance.
    // Starting values borrowed from a similar robot — tune on your robot.
    public static double[] TOF_DISTANCES = { 40,   50,   60,   70,   80,   90,   100,  110,  120,  130,  140,  150 };
    public static double[] TOF_VALUES    = { 0.42, 0.44, 0.55, 0.53, 0.57, 0.57, 0.60, 0.62, 0.65, 0.70, 0.70, 0.70 };

    private boolean feedStarted = false; // Flag to call intake.on() only once per FEED entry

    // Hood сглаживание для уменьшения jittering
    private static final double HOOD_SMOOTHING = 0.6;  // EMA factor для hood servo
    private double smoothedHoodPosition = 0.0;  // Текущая сглаженная позиция hood

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        hood = hardwareMap.get(Servo.class, "shooterHood");
        shooterStop = hardwareMap.get(Servo.class, "shooterStop");
        intakeStop = hardwareMap.get(Servo.class, "intakeStop");

        // Motor1 — master, firmware-level velocity PIDF (~1kHz)
        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(PIDF_P, 0, 0, PIDF_F));

        // Motor2 — slave, no encoder, synced via feedforward each loop
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Инициализируем smoothed hood position перед первым setHoodPosition
        smoothedHoodPosition = HoodPosition.CLOSE.position;
        setHoodPosition(HoodPosition.CLOSE);

        // Обычные позиции когда не стреляем
        shooterStop.setPosition(STOP_CLOSE);
        intakeStop.setPosition(INTAKE_STOP_OFF);
    }

    /**
     * Helper: Clamp value between min and max
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Вычисляет target velocity на основе расстояния до цели
     * Использует линейную функцию: V = M*d + B
     * Формула из Desmos (в дюймах, R²=0.9967):
     * y = 5.54742x + 945.62201
     *
     * @param distanceInches Расстояние до цели в дюймах
     * @return Target velocity в ticks/sec
     */
    private double calculateTargetVelocity(double distanceInches) {
        double d2 = distanceInches * distanceInches;
        double d3 = d2 * distanceInches;
        double d4 = d3 * distanceInches;
        double velocity = VELOCITY_A * d4 + VELOCITY_B * d3 + VELOCITY_C * d2
                        + VELOCITY_D * distanceInches + VELOCITY_E;

        return clamp(velocity, MIN_VELOCITY, MAX_VELOCITY) + FLYWHEEL_OFFSET;
    }

    /**
     * Вычисляет hood angle на основе расстояния до цели
     * Использует квадратичную формулу: angle = A*d² + B*d + C
     * Формула из Desmos - калибровка для робота (в дюймах)
     *
     * @param distanceInches Расстояние до цели в дюймах (от Pedro Pathing odometry)
     * @return Hood servo position (0.0 - 1.0)
     */
    private double calculateHoodAngle(double distanceInches) {
        // Sinusoidal: y = AMP * sin(FREQ * x + PHASE) + SHIFT
        double angle = HOOD_AMP * Math.sin(HOOD_FREQ * distanceInches + HOOD_PHASE) + HOOD_SHIFT;

        return clamp(angle, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE) + HOOD_OFFSET;
    }

    /**
     * Обновляет позицию Hood на основе расстояния до цели
     * Расстояние в дюймах (от Pedro Pathing odometry)
     * Использует deadzone 10.0 units (~10 inches) для предотвращения лишних движений
     * Использует динамическую формулу вместо ступенчатой
     */
    public void updateHood(double distance) {
        if (distance > 0) {
            double targetPosition = calculateHoodAngle(distance);

            // EMA сглаживание для уменьшения jittering
            if (lastHoodDistance < 0) {
                smoothedHoodPosition = targetPosition;
            } else {
                smoothedHoodPosition = smoothedHoodPosition + HOOD_SMOOTHING * (targetPosition - smoothedHoodPosition);
            }

            hood.setPosition(smoothedHoodPosition);

            if (smoothedHoodPosition < 0.2) {
                currentHoodPosition = HoodPosition.CLOSE;
            } else if (smoothedHoodPosition < 0.45) {
                currentHoodPosition = HoodPosition.MIDDLE;
            } else {
                currentHoodPosition = HoodPosition.FAR;
            }

            lastHoodDistance = distance;
        }
    }

    /**
     * Обновляет target velocity на основе расстояния до цели
     * Расстояние в см
     * Использует deadzone 5 см для предотвращения лишних изменений
     */
    public void updateVelocity(double distance) {
        if (distance > 0) {
            targetVelocity = calculateTargetVelocity(distance);
            lastVelocityDistance = distance;
        }
    }

    // -----------------------------------------------------------------------
    // Motion compensation API
    // -----------------------------------------------------------------------

    /** Set the Pedro Pathing follower so the shooter can read robot velocity. */
    public void setFollower(Follower follower) {
        this.follower = follower;
    }

    /** Set goal coordinates (field inches). Call once in init() per alliance. */
    public void setGoal(double x, double y) {
        motionGoalX = x;
        motionGoalY = y;
    }

    /** Interpolates time-of-flight from the lookup table for a given distance. */
    private double calculateTimeOfFlight(double dist) {
        if (dist <= TOF_DISTANCES[0]) return TOF_VALUES[0];
        if (dist >= TOF_DISTANCES[TOF_DISTANCES.length - 1]) return TOF_VALUES[TOF_VALUES.length - 1];
        for (int i = 0; i < TOF_DISTANCES.length - 1; i++) {
            if (dist < TOF_DISTANCES[i + 1]) {
                double t = (dist - TOF_DISTANCES[i]) / (TOF_DISTANCES[i + 1] - TOF_DISTANCES[i]);
                return TOF_VALUES[i] + t * (TOF_VALUES[i + 1] - TOF_VALUES[i]);
            }
        }
        return TOF_VALUES[TOF_VALUES.length - 1];
    }

    /**
     * Updates hood and flywheel velocity with motion compensation.
     * Computes a virtual goal shifted by robot velocity * time-of-flight,
     * then uses the distance to that virtual goal for hood/velocity lookup.
     * Falls back to real distance if follower is not set.
     * Call every loop() instead of manual setHoodPosition/setTargetVelocity.
     */
    public void updateWithMotion() {
        if (follower == null) return;

        Pose pose = follower.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();

        double realDist = Math.hypot(motionGoalX - robotX, motionGoalY - robotY);
        if (realDist <= 0) return;

        Vector vel = follower.getVelocity();
        double velX = vel.getXComponent();
        double velY = vel.getYComponent();

        double tof = calculateTimeOfFlight(realDist);

        // Virtual goal = where the goal "appears" to be from a moving robot frame
        double virtualGoalX = motionGoalX - velX * tof;
        double virtualGoalY = motionGoalY - velY * tof;
        double virtualDist = Math.hypot(virtualGoalX - robotX, virtualGoalY - robotY);

        updateVelocity(virtualDist);
        updateHood(virtualDist);
    }

    // -----------------------------------------------------------------------

    public void startShoot() {
        if (currentState == ShooterState.IDLE) {
            // Skip SPIN_UP — flywheel is always pre-spun (TeleOp: on() at start, Auto: on() before path)
            currentState = ShooterState.OPEN_STOP;
            stateTimer.reset();
        }
    }

    public boolean isIdle() {
        return currentState == ShooterState.IDLE;
    }

    public void updateFSM(Intake intake) {
        switch (currentState) {
            case IDLE:
                break;

            case OPEN_STOP:
                // Открываем оба servo для стрельбы (ТОЛЬКО ОДИН РАЗ)
                if (!openStopExecuted) {
                    shooterStop.setPosition(STOP_OPEN);
                    intakeStop.setPosition(INTAKE_STOP_ON);
                    openStopExecuted = true;
                }
                if (stateTimer.seconds() >= OPEN_STOP_TIME) {
                    currentState = ShooterState.FEED;
                    stateTimer.reset();
                    openStopExecuted = false;
                }
                break;

            case FEED:
                // Continuous feed — all 3 balls exit in one uninterrupted intake run
                if (!feedStarted) {
                    intake.on();
                    feedStarted = true;
                }
                if (stateTimer.seconds() >= FEED_TIME) {
                    // intake.off() убран — управление intake остаётся за Auto/TeleOp
                    feedStarted = false;
                    currentState = ShooterState.RESET;
                    stateTimer.reset();
                }
                break;

            case RESET:
                // Возвращаем оба servo в обычные позиции (ТОЛЬКО ОДИН РАЗ)
                if (!resetExecuted) {
                    // Закрываем shooterStop ТОЛЬКО если нет ручного override
                    if (!manualStopOverride) {
                        shooterStop.setPosition(STOP_CLOSE);
                    }
                    intakeStop.setPosition(INTAKE_STOP_OFF);
                    // НЕ выключаем shooter моторы - пусть крутятся постоянно в TeleOp
                    // off();
                    // intake.off() убран — Auto/TeleOp сами управляют intake через startCollect/fireShot
                    resetExecuted = true;
                }
                currentState = ShooterState.IDLE;
                resetExecuted = false; // Сброс для следующего раза
                break;
        }
    }

    /**
     * Применяет velocity к master мотору (SDK PIDF на firmware уровне ~1kHz)
     * и синхронизирует slave через feedforward.
     * Также переприменяет PIDF если изменили в FTC Dashboard.
     * Вызывать в каждом loop().
     */
    public void updatePID() {
        if (targetVelocity == 0) {
            shooterMotor1.setVelocity(0);
            shooterMotor2.setPower(0);
            return;
        }

        double currentVel = getCurrentVelocity();
        boolean decelerating = currentVel > targetVelocity + DECEL_THRESHOLD;

        // During deceleration: F=0 so firmware F-term doesn't cancel P-term braking.
        // Restore full PIDF when back at speed or when dashboard values changed.
        // I and D locked to 0 (Pratt's method: only P and F needed for flywheels)
        if (decelerating != prevDecelerating
                || PIDF_P != lastP || PIDF_F != lastF) {
            double f = decelerating ? 0 : PIDF_F;
            shooterMotor1.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(PIDF_P, 0, 0, f));
            lastP = PIDF_P; lastF = PIDF_F;
        }
        prevDecelerating = decelerating;

        // Master: SDK держит PIDF сам на firmware уровне
        shooterMotor1.setVelocity(targetVelocity);

        // Slave: feedforward + P correction — motor2 mirrors master's effort
        if (!decelerating) {
            double error = targetVelocity - currentVel;
            shooterMotor2.setPower((PIDF_F * targetVelocity + PIDF_P * error) / 32767.0);
        } else {
            shooterMotor2.setPower(0);
        }
    }

    public void on() {
        targetVelocity = TARGET_VELOCITY;
    }

    public void off() {
        targetVelocity = 0;
        shooterMotor1.setVelocity(0);
        shooterMotor2.setPower(0);
    }

    public void setTargetVelocity(double velocity) {
        // Сбрасываем deadzone только если velocity реально изменилась —
        // иначе fallback будет блокировать Vision обновления каждый loop
        if (Math.abs(targetVelocity - velocity) > 1.0) {
            lastVelocityDistance = -1;
        }
        targetVelocity = velocity;
    }

    public boolean atSpeed() {
        if (targetVelocity <= 0) return false;
        return getCurrentVelocity() >= targetVelocity * VELOCITY_READY_THRESHOLD;
    }

    public double getCurrentVelocity() {
        return shooterMotor1.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setHoodPosition(HoodPosition position) {
        if (position != null) {
            // Применяем сглаживание и для manual override
            smoothedHoodPosition += HOOD_SMOOTHING * (position.position - smoothedHoodPosition);
            hood.setPosition(smoothedHoodPosition);
            currentHoodPosition = position;
        }
    }

    public HoodPosition getCurrentHoodPosition() {
        return currentHoodPosition;
    }

    public double getHoodServoPosition() {
        return hood.getPosition();
    }


    public ShooterState getCurrentState() {
        return currentState;
    }

    public void resetDeadzones() {
        lastHoodDistance = -1;
        lastVelocityDistance = -1;
    }

    public boolean isShooting() {
        return currentState != ShooterState.IDLE;
    }

    public void reset() {
        // Полный сброс shooter в начальное состояние
        off(); // Выключаем моторы
        manualStopOverride = false; // Сбрасываем manual override
        shooterStop.setPosition(STOP_CLOSE); // Закрываем stop
        intakeStop.setPosition(INTAKE_STOP_OFF); // Обычная позиция
        setHoodPosition(HoodPosition.CLOSE);
        currentState = ShooterState.IDLE; // Сбрасываем FSM
        stateTimer.reset();
        lastHoodDistance = -1; // Сбрасываем deadzone tracking
        lastVelocityDistance = -1; // Сбрасываем velocity deadzone tracking
    }

    /**
     * Ручное управление shooterStop (приоритет над FSM)
     * Когда активен - shooterStop остается открытым, FSM не может закрыть
     *
     * @param enabled true - держать открытым, false - FSM работает как обычно
     */
    public void setManualStopOverride(boolean enabled) {
        manualStopOverride = enabled;

        if (manualStopOverride) {
            // Немедленно открываем shooterStop
            shooterStop.setPosition(STOP_OPEN);
        }
        // Если disabled - FSM закроет shooterStop когда нужно (в RESET state)
    }

    /**
     * Принудительно закрывает shooterStop (для dpad_down)
     */
    public void forceCloseStop() {
        shooterStop.setPosition(STOP_CLOSE);
    }

    /**
     * Устанавливает позицию hood напрямую (для fallback когда нет Vision tag)
     * @param position позиция servo (0.0-1.0)
     */
    public void setHoodPosition(double position) {
        hood.setPosition(clamp(position, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE) + HOOD_OFFSET);
        // CRITICAL FIX: Сбрасываем deadzone tracking чтобы следующий updateHood точно сработал
        // Без этого если fallback установит hood=0, а потом Vision увидит тег на близком расстоянии,
        // deadzone может блокировать обновление и hood останется на 0
        lastHoodDistance = -1;
    }

}