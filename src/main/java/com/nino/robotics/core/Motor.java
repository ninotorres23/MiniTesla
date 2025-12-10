package com.nino.robotics.core;

import com.nino.robotics.util.MathUtil;

public class Motor {
    private float speed; // Range from -1.0 (full reverse) to 1.0 (full forward)

    public Motor() {
        this.speed = 0.0f;
    }

    public float getSpeed() {
        return speed;
    }

    public void setSpeed(float speed) {
        this.speed = MathUtil.clamp(speed, -1.0f, 1.0f);
    }
}
