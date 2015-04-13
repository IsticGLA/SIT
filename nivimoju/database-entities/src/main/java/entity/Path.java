package entity;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by jeremy on 13/04/15.
 */
public class Path {

    List<Position> positions;
    boolean closed;

    public Path(){
        this.positions = new ArrayList<>();
        this.closed = false;
    }

    public List<Position> getPositions() {
        return positions;
    }

    public void setPositions(List<Position> positions) {
        this.positions = positions;
    }

    public boolean isClosed() {
        return closed;
    }

    public void setClosed(boolean closed) {
        this.closed = closed;
    }
}
