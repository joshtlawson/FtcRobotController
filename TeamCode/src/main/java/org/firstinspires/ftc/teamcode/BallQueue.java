package org.firstinspires.ftc.teamcode;

import java.util.ArrayDeque;
import java.util.Deque;

final class BallQueue<T> {
    private static final int CAPACITY = 2;

    private final T defaultBall;
    private final double advanceIntervalSeconds;
    private final Deque<T> balls = new ArrayDeque<>();
    private boolean advancing;
    private boolean ballVisible;
    private T lastDetectedBall;
    private double nextAdvanceSeconds;

    BallQueue(T defaultBall, double advanceIntervalSeconds) {
        if (defaultBall == null || advanceIntervalSeconds <= 0 || !Double.isFinite(advanceIntervalSeconds)) {
            throw new IllegalArgumentException("A default ball and a positive finite interval are required");
        }
        this.defaultBall = defaultBall;
        this.advanceIntervalSeconds = advanceIntervalSeconds;
        reset();
    }

    void reset() {
        balls.clear();
        for (int i = 0; i < CAPACITY; i++) {
            balls.addLast(defaultBall);
        }
        advancing = false;
        ballVisible = false;
        lastDetectedBall = defaultBall;
    }

    void update(boolean intakeRunning, boolean gateOpen, double nowSeconds, T visibleBall) {
        if (intakeRunning && gateOpen) {
            if (!advancing) {
                advancing = true;
                nextAdvanceSeconds = nowSeconds + advanceIntervalSeconds;
            } else if (nowSeconds >= nextAdvanceSeconds) {
                double advances = Math.floor((nowSeconds - nextAdvanceSeconds) / advanceIntervalSeconds) + 1;
                for (int i = 0; i < Math.min(advances, CAPACITY); i++) {
                    balls.removeFirst();
                    balls.addLast(defaultBall);
                }
                nextAdvanceSeconds += advances * advanceIntervalSeconds;
            }
        } else {
            advancing = false;
        }

        if (intakeRunning && visibleBall != null && !ballVisible) {
            if (!visibleBall.equals(lastDetectedBall)) {
                balls.removeLast();
                balls.addLast(visibleBall);
            }
            lastDetectedBall = visibleBall;
        }
        ballVisible = intakeRunning && visibleBall != null;
    }

    T oldest() {
        return balls.getFirst();
    }

    @Override
    public String toString() {
        return balls.toString();
    }
}
