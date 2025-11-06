package org.firstinspires.ftc.teamcode;

public class VisionStates {

    private VisionState currentState;
    private VisionState lastState;

    public void setState(VisionState state) {
        currentState = state;
    }

    public VisionState getState() {
        return currentState;
    }

    public VisionState getLastState() {
        return lastState;
    }

    public void updateLastState() {
        lastState = currentState;
    }

    public void updateLastState(VisionState newState) {
        lastState = newState;
    }

    public boolean hasStateChanged() {
        return currentState != lastState;
    }

    public enum VisionState {
        MOTIF,
        SHOOT
    }
}
