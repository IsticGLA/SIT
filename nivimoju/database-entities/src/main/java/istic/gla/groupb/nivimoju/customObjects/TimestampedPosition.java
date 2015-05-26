package istic.gla.groupb.nivimoju.customObjects;

import istic.gla.groupb.nivimoju.entity.Position;

import java.io.Serializable;

/**
 * Created by vivien on 22/05/15.
 */
public class TimestampedPosition implements Serializable {
    private Position position;
    private long timestamp;

    public TimestampedPosition(Position position, long timestamp) {
        this.position = position;
        this.timestamp = timestamp;
    }

    public Position getPosition() {
        return position;
    }

    public void setPosition(Position position) {
        this.position = position;
    }

    public long getTimestamp() {
        return timestamp;
    }

    public void setTimestamp(long timestamp) {
        this.timestamp = timestamp;
    }
}
