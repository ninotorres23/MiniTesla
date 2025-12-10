package com.nino.robotics.ai;

import com.nino.robotics.core.RobotCar;
import com.nino.robotics.core.RobotController;
import com.nino.robotics.util.Config;

public class temp1 implements RobotController {

    private enum State {
        LINE_FOLLOWING,
        OBSTACLE_AVOIDANCE
    }

    private State currentState = State.LINE_FOLLOWING;

    // Line-following sub-states
    private enum LineState {
        FOLLOWING,          // Middle sensor on line -> go straight with small nudges
        TURNING_LEFT,       // In-place turn left at a corner until mid sees line
        TURNING_RIGHT,      // In-place turn right at a corner until mid sees line
        STOPPED,            // Stopped after losing the line
        SEARCHING,          // Pivoting/arching to find the line again
        HALTED              // Gave up (no line found)
    }

    private LineState lineState = LineState.SEARCHING;

    // Line following configuration
    private static final float BASE_SPEED = 0.18f;     // slower for more precise control
    private static final float LINE_THRESHOLD = 0.30f; // standard threshold
    private static final float NUDGE_SCALE = 0.88f;    // slightly stronger nudge
    private static final float PIVOT_SPEED = 0.22f;    // faster pivot to reacquire
    private static final int SEARCH_TIMEOUT_TICKS = 300; // ~6.0s at 20ms per tick
    private static final float MIN_FWD = 0.12f;       // ensure arcs/forward have enough pull
    private static final float TURN_ARC_SCALE = 0.45f; // inside wheel scale when arcing (legacy)
    private static final int TURN_TICKS_MAX = 90;     // max ticks to pivot during a corner (~1.8s)
    private static final int SNAP_ON_TURN_TICKS = 0;   // snap heading immediately on turn
    private static final int SNAP_FORWARD_TICKS = 8;   // short straight after snap
    private static final int POST_TURN_HOLD_TICKS = 30; // longer hold after turn
    private static final int MIN_CENTER_STABLE_TICKS = 10; // require mid to be stable before turns
    private static final int NO_MID_GRACE_TICKS = 3;   // tolerate brief mid drop

    // Memory of the last seen direction: -1 = left, 0 = center, 1 = right
    private int lastSeenDir = 0;
    // Memory of the last non-center side seen (left/right only). 0 if unknown.
    private int lastNonCenterDir = 0;
    private int searchCounter = 0;
    // SEARCHING sub-phases: 0 pivot-left, 1 arc-left, 2 pivot-right, 3 arc-right
    private int searchPhase = 0;
    private int phaseTicks = 0;
    // Turn detection: require side-only detection to persist before treating as a turn
    private static final int TURN_DETECT_TICKS = 4;
    private int sideHoldCounter = 0;
    // Side cue alternating phases (0=pivot, 1=arc)
    private int sideCuePhase = 0;
    private int sideCueTicks = 0;
    // Stabilization: keep straight for a few ticks once center is reacquired
    private static final int CENTER_HOLD_TICKS = 12;
    private int centerHold = 0;
    private int turnTicks = 0;
    // expected turn direction for current turn: -1 = left, 1 = right, 0 = none
    private int expectedTurnDir = 0;
    private boolean turnSnapped = false;
    private int snapForwardTicks = 0;
    
    private int postTurnHold = 0;
    private int centerStableTicks = 0;
    
    private int noMidCount = 0;

    @Override
    public void updateControl(RobotCar car, float delta) {
        float ultrasonicDist = car.getUltrasonicSensor().readValue();

        if (ultrasonicDist < Config.ULTRASONIC_STOP_DISTANCE) {
            currentState = State.OBSTACLE_AVOIDANCE;
        } else {
            currentState = State.LINE_FOLLOWING;
        }

        switch (currentState) {
            case LINE_FOLLOWING:
                followLine(car);
                break;
            case OBSTACLE_AVOIDANCE:
                avoidObstacle(car);
                break;
        }
    }

    private void followLine(RobotCar car) {
        // Read sensor values (0-1, where 1 means on the line)
        float left = car.getLeftLineSensor().readValue();
        float mid = car.getMidLineSensor().readValue();
        float right = car.getRightLineSensor().readValue();
        
        // Update last seen direction whenever any sensor sees the line
        if (left > LINE_THRESHOLD || mid > LINE_THRESHOLD || right > LINE_THRESHOLD) {
            if (left >= mid && left >= right) {
                lastSeenDir = -1;
                lastNonCenterDir = -1; // prefer side memory when available
            } else if (right >= mid && right >= left) {
                lastSeenDir = 1;
                lastNonCenterDir = 1;  // prefer side memory when available
            } else {
                lastSeenDir = 0;
                // do not overwrite lastNonCenterDir when center dominates
            }
        }

        // Debug output - show state and sensors
        System.out.println(String.format("State:%s Last:%d | L:%.2f M:%.2f R:%.2f",
                lineState, lastSeenDir, left, mid, right));

        switch (lineState) {
            case FOLLOWING:
                // If just completed a turn, hold straight to avoid immediate re-turns
                if (postTurnHold > 0) {
                    // Cancel hold immediately if mid is lost to avoid driving off the line
                    if (mid <= LINE_THRESHOLD) {
                        postTurnHold = 0;
                    } else {
                        postTurnHold--;
                        // Allow very gentle nudge during hold to avoid drifting off-center
                        if (left > right + 0.05f) {
                            car.setMotorSpeeds(BASE_SPEED, BASE_SPEED * 0.95f);
                            System.out.println("Following: post-hold gentle nudge left (" + postTurnHold + ")");
                        } else if (right > left + 0.05f) {
                            car.setMotorSpeeds(BASE_SPEED * 0.95f, BASE_SPEED);
                            System.out.println("Following: post-hold gentle nudge right (" + postTurnHold + ")");
                        } else {
                            car.setMotorSpeeds(BASE_SPEED, BASE_SPEED);
                            System.out.println("Following: post-turn hold straight (" + postTurnHold + ")");
                        }
                        break;
                    }
                }
                if (mid > LINE_THRESHOLD) {
                    noMidCount = 0;
                    // Track how long center has been stably on the line
                    if (centerStableTicks < MIN_CENTER_STABLE_TICKS) centerStableTicks++;
                    // On the line -> go straight with slight nudge if drifting
                    if (centerHold > 0) {
                        centerHold--;
                        car.setMotorSpeeds(BASE_SPEED, BASE_SPEED);
                        System.out.println("Following: stabilize straight (hold=" + (centerHold) + ")");
                    } else {
                        if (left > right + 0.05f) {
                            // Line biased to left -> steer LEFT (slow RIGHT wheel)
                            car.setMotorSpeeds(BASE_SPEED, BASE_SPEED * NUDGE_SCALE);
                            System.out.println("Following: nudge left");
                        } else if (right > left + 0.05f) {
                            // Line biased to right -> steer RIGHT (slow LEFT wheel)
                            car.setMotorSpeeds(BASE_SPEED * NUDGE_SCALE, BASE_SPEED);
                            System.out.println("Following: nudge right");
                        } else {
                            car.setMotorSpeeds(BASE_SPEED, BASE_SPEED);
                            System.out.println("Following: straight");
                        }
                    }
                    // reset any side-hold when centered
                    sideHoldCounter = 0;
                } else if (left > LINE_THRESHOLD && mid <= LINE_THRESHOLD && right <= LINE_THRESHOLD
                        && centerStableTicks >= MIN_CENTER_STABLE_TICKS && postTurnHold == 0) {
                    // Possible left turn/end; require persistence before acting
                    centerStableTicks = 0; // mid is off; require stability again before next turn
                    sideHoldCounter++;
                    if (sideHoldCounter >= TURN_DETECT_TICKS) {
                        // Invert mapping: left-only at corner -> commit to RIGHT turn
                        lineState = LineState.TURNING_RIGHT;
                        turnTicks = 0;
                        expectedTurnDir = 1;
                        turnSnapped = false;
                        snapForwardTicks = 0;
                        System.out.println("Following: left-only -> TURN RIGHT (pivot)");
                        sideHoldCounter = 0;
                    } else {
                        // Probe forward but arc RIGHT to bring mid back over the line
                        car.setMotorSpeeds(BASE_SPEED, BASE_SPEED * 0.8f);
                        System.out.println("Following: probing left -> arc right (" + sideHoldCounter + ")");
                    }
                } else if (right > LINE_THRESHOLD && mid <= LINE_THRESHOLD && left <= LINE_THRESHOLD
                        && centerStableTicks >= MIN_CENTER_STABLE_TICKS && postTurnHold == 0) {
                    // Possible right turn/end; require persistence before acting
                    centerStableTicks = 0; // mid is off; require stability again before next turn
                    sideHoldCounter++;
                    if (sideHoldCounter >= TURN_DETECT_TICKS) {
                        // Invert mapping: right-only at corner -> commit to LEFT turn
                        lineState = LineState.TURNING_LEFT;
                        turnTicks = 0;
                        expectedTurnDir = -1;
                        turnSnapped = false;
                        snapForwardTicks = 0;
                        System.out.println("Following: right-only -> TURN LEFT (pivot)");
                        sideHoldCounter = 0;
                    } else {
                        // Probe forward but arc LEFT to bring mid back over the line
                        car.setMotorSpeeds(BASE_SPEED * 0.8f, BASE_SPEED);
                        System.out.println("Following: probing right -> arc left (" + sideHoldCounter + ")");
                    }
                } else {
                    // Grace period: keep moving forward briefly with slight bias before deciding lost
                    noMidCount++;
                    if (noMidCount <= NO_MID_GRACE_TICKS) {
                        if (left > LINE_THRESHOLD && right <= LINE_THRESHOLD) {
                            // bias right wheel faster (turn right slightly)
                            car.setMotorSpeeds(BASE_SPEED * 0.9f, BASE_SPEED);
                            System.out.println("Following: grace forward (bias right) (" + noMidCount + ")");
                            break;
                        } else if (right > LINE_THRESHOLD && left <= LINE_THRESHOLD) {
                            // bias left wheel faster (turn left slightly)
                            car.setMotorSpeeds(BASE_SPEED, BASE_SPEED * 0.9f);
                            System.out.println("Following: grace forward (bias left) (" + noMidCount + ")");
                            break;
                        }
                    }
                    // Fallback: if a single side still sees the line, re-enter TURNING instead of stopping
                    if (left > LINE_THRESHOLD && right <= LINE_THRESHOLD) {
                        lineState = LineState.TURNING_RIGHT; // left-only -> turn right
                        turnTicks = 0;
                        expectedTurnDir = 1;
                        turnSnapped = false;
                        snapForwardTicks = 0;
                        System.out.println("Following: mid lost, LEFT-only -> TURN RIGHT (fallback)");
                    } else if (right > LINE_THRESHOLD && left <= LINE_THRESHOLD) {
                        lineState = LineState.TURNING_LEFT;  // right-only -> turn left
                        turnTicks = 0;
                        expectedTurnDir = -1;
                        turnSnapped = false;
                        snapForwardTicks = 0;
                        System.out.println("Following: mid lost, RIGHT-only -> TURN LEFT (fallback)");
                    } else {
                        // Truly lost -> stop and search
                        car.setMotorSpeeds(0, 0);
                        lineState = LineState.STOPPED;
                        searchCounter = 0;
                        sideHoldCounter = 0;
                        centerHold = 0;
                        centerStableTicks = 0;
                        System.out.println("Following: lost line -> stop");
                    }
                }
                break;

            case TURNING_LEFT:
                if (mid > LINE_THRESHOLD) {
                    // Mid sees line again -> resume following with stabilization
                    car.setMotorSpeeds(BASE_SPEED, BASE_SPEED);
                    lineState = LineState.FOLLOWING;
                    centerHold = CENTER_HOLD_TICKS;
                    expectedTurnDir = 0;
                    turnSnapped = false;
                    snapForwardTicks = 0;
                    postTurnHold = POST_TURN_HOLD_TICKS;
                    System.out.println("TurningLeft: mid reacquired -> following");
                } else if (turnTicks >= TURN_TICKS_MAX) {
                    car.setMotorSpeeds(0, 0);
                    lineState = LineState.STOPPED;
                    searchCounter = 0;
                    System.out.println("TurningLeft: timeout -> search");
                } else {
                    if (!turnSnapped) {
                        float newAngle = car.getAngle() + 90f; // left turn snap
                        car.setAngle(newAngle);
                        turnSnapped = true;
                        System.out.println("TurningLeft: snapped heading +90°");
                    }
                    if (snapForwardTicks < SNAP_FORWARD_TICKS) {
                        car.setMotorSpeeds(MIN_FWD, MIN_FWD);
                        snapForwardTicks++;
                        System.out.println("TurningLeft: snap-forward straight (" + snapForwardTicks + ")");
                    } else {
                        // Revert to simple pivot-in-place until mid finds the line
                        car.setMotorSpeeds(-PIVOT_SPEED, PIVOT_SPEED);
                        System.out.println("TurningLeft: pivot-in-place");
                    }
                    turnTicks++;
                }
                break;

            case TURNING_RIGHT:
                if (mid > LINE_THRESHOLD) {
                    // Mid sees line again -> resume following with stabilization
                    car.setMotorSpeeds(BASE_SPEED, BASE_SPEED);
                    lineState = LineState.FOLLOWING;
                    centerHold = CENTER_HOLD_TICKS;
                    expectedTurnDir = 0;
                    turnSnapped = false;
                    snapForwardTicks = 0;
                    postTurnHold = POST_TURN_HOLD_TICKS;
                    System.out.println("TurningRight: mid reacquired -> following");
                } else if (turnTicks >= TURN_TICKS_MAX) {
                    car.setMotorSpeeds(0, 0);
                    lineState = LineState.STOPPED;
                    searchCounter = 0;
                    System.out.println("TurningRight: timeout -> search");
                } else {
                    if (!turnSnapped) {
                        float newAngle = car.getAngle() - 90f; // right turn snap
                        car.setAngle(newAngle);
                        turnSnapped = true;
                        System.out.println("TurningRight: snapped heading -90°");
                    }
                    if (snapForwardTicks < SNAP_FORWARD_TICKS) {
                        car.setMotorSpeeds(MIN_FWD, MIN_FWD);
                        snapForwardTicks++;
                        System.out.println("TurningRight: snap-forward straight (" + snapForwardTicks + ")");
                    } else {
                        // Revert to simple pivot-in-place until mid finds the line
                        car.setMotorSpeeds(PIVOT_SPEED, -PIVOT_SPEED);
                        System.out.println("TurningRight: pivot-in-place");
                    }
                    turnTicks++;
                }
                break;

            case STOPPED:
                // Immediately start searching based on last seen direction
                // Initialize search sequence based on last non-center side if known
                searchPhase = (lastNonCenterDir == -1) ? 0 : ((lastNonCenterDir == 1) ? 2 : 0);
                phaseTicks = 0;
                searchCounter = 0;
                sideCuePhase = 0;
                sideCueTicks = 0;
                // Prefer current sensor cue to set expected turn direction
                if (left > LINE_THRESHOLD && right <= LINE_THRESHOLD) {
                    expectedTurnDir = 1; // RIGHT
                    System.out.println("Stopped: expectedTurnDir set RIGHT from sensor cue");
                } else if (right > LINE_THRESHOLD && left <= LINE_THRESHOLD) {
                    expectedTurnDir = -1; // LEFT
                    System.out.println("Stopped: expectedTurnDir set LEFT from sensor cue");
                } else {
                    // No current cue: do not infer from stale memory to avoid wrong-direction turns
                    expectedTurnDir = 0;
                }
                // Snap heading to expected turn direction if known, to quickly face next segment
                if (expectedTurnDir != 0) {
                    float newAngle = car.getAngle() + (expectedTurnDir == -1 ? 90f : -90f);
                    car.setAngle(newAngle);
                    System.out.println("Stopped: snapped heading " + (expectedTurnDir == -1 ? "+90°" : "-90°"));
                }
                lineState = LineState.SEARCHING;
                car.setMotorSpeeds(0, 0);
                System.out.println("Stopped: begin search (phase=" + searchPhase + ")");
                break;

            case SEARCHING:
                if (mid > LINE_THRESHOLD) {
                    // Reacquired line with center sensor -> resume following
                    car.setMotorSpeeds(BASE_SPEED, BASE_SPEED);
                    lineState = LineState.FOLLOWING;
                    searchCounter = 0;
                    sideCuePhase = 0;
                    sideCueTicks = 0;
                    centerHold = CENTER_HOLD_TICKS;
                    postTurnHold = POST_TURN_HOLD_TICKS; // hold straight briefly after reacquire
                    noMidCount = 0;
                    centerStableTicks = 0;
                    expectedTurnDir = 0; // clear any prior turn intent
                    System.out.println("Searching: reacquired -> following");
                } else if (searchCounter > SEARCH_TIMEOUT_TICKS) {
                    // Timed out -> halt indefinitely
                    car.setMotorSpeeds(0, 0);
                    lineState = LineState.HALTED;
                    System.out.println("Searching: timeout -> halted");
                } else {
                    // If we have a committed turn direction, bias the search strictly that way
                    if (expectedTurnDir == 1) { // RIGHT
                        // Alternate brief forward rolls with right pivot to avoid spinning in place
                        if ((searchCounter % 12) < 6) {
                            car.setMotorSpeeds(MIN_FWD, MIN_FWD);
                            System.out.println("Searching: biased RIGHT forward roll");
                        } else {
                            car.setMotorSpeeds(PIVOT_SPEED, -PIVOT_SPEED);
                            System.out.println("Searching: biased RIGHT pivot");
                        }
                    } else if (expectedTurnDir == -1) { // LEFT
                        if ((searchCounter % 12) < 6) {
                            car.setMotorSpeeds(MIN_FWD, MIN_FWD);
                            System.out.println("Searching: biased LEFT forward roll");
                        } else {
                            car.setMotorSpeeds(-PIVOT_SPEED, PIVOT_SPEED);
                            System.out.println("Searching: biased LEFT pivot");
                        }
                    } else {
                        // No expectation: use side cues, else deterministic sweep
                        if (left > LINE_THRESHOLD && mid <= LINE_THRESHOLD && right <= LINE_THRESHOLD) {
                            if (sideCuePhase == 0) {
                                car.setMotorSpeeds(-PIVOT_SPEED, PIVOT_SPEED); // pivot left
                                if (++sideCueTicks >= 8) { sideCuePhase = 1; sideCueTicks = 0; }
                                System.out.println("Searching: cue left -> pivot");
                            } else {
                                float l = Math.max(MIN_FWD, BASE_SPEED);
                                float r = Math.max(MIN_FWD, BASE_SPEED * 0.4f);
                                car.setMotorSpeeds(l, r); // arc right with min forward
                                if (++sideCueTicks >= 12) { sideCuePhase = 0; sideCueTicks = 0; }
                                System.out.println("Searching: cue left -> arc right");
                            }
                        } else if (right > LINE_THRESHOLD && mid <= LINE_THRESHOLD && left <= LINE_THRESHOLD) {
                            if (sideCuePhase == 0) {
                                car.setMotorSpeeds(PIVOT_SPEED, -PIVOT_SPEED); // pivot right
                                if (++sideCueTicks >= 8) { sideCuePhase = 1; sideCueTicks = 0; }
                                System.out.println("Searching: cue right -> pivot");
                            } else {
                                float l = Math.max(MIN_FWD, BASE_SPEED * 0.4f);
                                float r = Math.max(MIN_FWD, BASE_SPEED);
                                car.setMotorSpeeds(l, r); // arc left with min forward
                                if (++sideCueTicks >= 12) { sideCuePhase = 0; sideCueTicks = 0; }
                                System.out.println("Searching: cue right -> arc left");
                            }
                        } else {
                            // No cues -> deterministic pivot+arc sweep
                            switch (searchPhase) {
                                case 0:
                                    car.setMotorSpeeds(-PIVOT_SPEED, PIVOT_SPEED);
                                    if (phaseTicks >= 20) { searchPhase = 1; phaseTicks = 0; }
                                    System.out.println("Searching: phase 0 pivot left");
                                    break;
                                case 1:
                                    car.setMotorSpeeds(BASE_SPEED * 0.5f, BASE_SPEED);
                                    if (phaseTicks >= 25) { searchPhase = 2; phaseTicks = 0; }
                                    System.out.println("Searching: phase 1 arc left");
                                    break;
                                case 2:
                                    car.setMotorSpeeds(PIVOT_SPEED, -PIVOT_SPEED);
                                    if (phaseTicks >= 40) { searchPhase = 3; phaseTicks = 0; }
                                    System.out.println("Searching: phase 2 pivot right");
                                    break;
                                case 3:
                                    car.setMotorSpeeds(BASE_SPEED, BASE_SPEED * 0.5f);
                                    if (phaseTicks >= 25) { searchPhase = 0; phaseTicks = 0; }
                                    System.out.println("Searching: phase 3 arc right");
                                    break;
                            }
                            phaseTicks++;
                            sideCuePhase = 0;
                            sideCueTicks = 0;
                        }
                    }
                    searchCounter++;
                }
                break;

            case HALTED:
                // Remain stopped
                car.setMotorSpeeds(0, 0);
                System.out.println("Halted: no line");
                break;
        }
        
        // Small delay to prevent the method from being called too frequently
        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    

    private void avoidObstacle(RobotCar car) {
        // Simple obstacle avoidance - stop
        car.setMotorSpeeds(0, 0);
        // More advanced: could turn left or right
    }
}
