# AutoHelper — шпаргалка

AutoHelper — это 5 методов, которые убирают повторяющийся код из switch/case в авто файлах.
Switch/case остаётся как был, просто каждый case становится короче.

## Создание

```java
private AutoHelper h;

// в init(), после создания follower, shooter, intake, turret, pathTimer:
h = new AutoHelper(follower, shooter, intake, turret, pathTimer);
```


---

## Методы

### 1. `h.pathDone(minTime)`

**Что делает:** Проверяет, что путь закончен И прошло минимум `minTime` секунд.

**Когда использовать:** Почти в каждом case — это основное условие перехода.

| Раньше | Сейчас |
|--------|--------|
| `if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0)` | `if (h.pathDone(2.0))` |

### 2. `h.pathDoneAndIdle(minTime)`

**Что делает:** То же что `pathDone` + проверяет что шутер вернулся в IDLE (закончил стрелять).

**Когда использовать:** После выстрела, перед сбором следующего мяча. Гарантирует что шутер
завершил весь FSM цикл (SPIN_UP -> OPEN_STOP -> FEED -> RESET -> IDLE).

| Раньше | Сейчас |
|--------|--------|
| `if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5 && shooter.isIdle())` | `if (h.pathDoneAndIdle(1.5))` |

### 3. `h.goToShoot(shootPose)`

**Что делает:** Строит динамический путь (BezierLine) от текущей позиции робота до shootPose
и начинает движение. Heading держится постоянным (heading из shootPose).

**Когда использовать:** Когда робот собрал мяч и возвращается на позицию для стрельбы.
Путь строится динамически потому что робот может быть не ровно в конце предыдущего пути.

| Раньше | Сейчас |
|--------|--------|
| `follower.followPath(`<br>&nbsp;&nbsp;`follower.pathBuilder()`<br>&nbsp;&nbsp;&nbsp;&nbsp;`.addPath(new BezierLine(follower.getPose(), shootPose))`<br>&nbsp;&nbsp;&nbsp;&nbsp;`.setConstantHeadingInterpolation(Math.toRadians(0))`<br>&nbsp;&nbsp;&nbsp;&nbsp;`.build(),`<br>&nbsp;&nbsp;`true);` | `h.goToShoot(shootPose);` |

### 4. `h.startCollect(path, speed, turretAngle)`

**Что делает:** Включает intake, поворачивает turret на нужный угол, запускает движение по пути.

**Когда использовать:** Когда едем собирать мяч. Три действия в одном вызове.

| Раньше | Сейчас |
|--------|--------|
| `intake.on();`<br>`turret.setTargetAngle(-55);`<br>`follower.followPath(path4, 0.8, true);` | `h.startCollect(path4, 0.8, -55);` |

### 5. `h.fireShot()`

**Что делает:** Выключает intake и запускает стрельбу.

**Когда использовать:** Когда робот приехал на позицию стрельбы и готов стрелять.

| Раньше | Сейчас |
|--------|--------|
| `intake.off();`<br>`shooter.startShoot();` | `h.fireShot();` |

---

## Полный пример: сбор + стрельба одного мяча

Типичный цикл "собрать мяч -> вернуться -> выстрелить" занимает 3 case:

```java
// Едем собирать мяч
case 4:
    if (h.pathDoneAndIdle(1.5)) { h.startCollect(path4, 0.8, -55); setPathState(5); }
    break;

// Возвращаемся на позицию стрельбы
case 5:
    if (h.pathDone(2.0)) { h.goToShoot(shootPose); setPathState(6); }
    break;

// Стреляем
case 6:
    if (h.pathDone(2.0)) { h.fireShot(); setPathState(7); }
    break;
```

**Раньше тот же цикл:**

```java
case 4:
    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5 && shooter.isIdle()) {
        intake.on();
        turret.setTargetAngle(-55);
        follower.followPath(path4, 0.8, true);
        setPathState(5);
    }
    break;

case 5:
    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0) {
        follower.followPath(
            follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), shootPose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build(),
            true);
        setPathState(6);
    }
    break;

case 6:
    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0) {
        intake.off();
        shooter.startShoot();
        setPathState(7);
    }
    break;
```

---

## Когда НЕ использовать хелперы

Если case делает что-то нестандартное — пиши напрямую. Хелперы не покрывают все случаи:

```java
// Первый case — просто запуск пути, без условия
case 0:
    follower.followPath(path1, true);
    setPathState(50);
    break;

// Сбор без turret угла (ball 12 в RedAutoClose)
case 7:
    if (h.pathDoneAndIdle(1.5)) {
        intake.on();
        follower.followPath(path6, true);
        setPathState(8);
    }
    break;

// Парковка с turret reset
case 10:
    if (h.pathDone(1.9)) {
        follower.followPath(path8, true);
        turret.setTargetAngle(0);
        setPathState(13);
    }
    break;
```

Условия (`h.pathDone`, `h.pathDoneAndIdle`) можно использовать даже если действия нестандартные.
