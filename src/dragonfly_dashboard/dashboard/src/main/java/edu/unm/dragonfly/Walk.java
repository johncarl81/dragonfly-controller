package edu.unm.dragonfly;

public enum Walk {
    WALK(1),
    RANGE(2);

    public final int id;

    Walk(int id) {
        this.id = id;
    }
}