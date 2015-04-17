package entity;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by jeremy on 13/04/15.
 */
public class Path implements Serializable {

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

    public void addPosition(Position position){
        positions.add(position);
    }

    public boolean isClosed() {
        return closed;
    }

    public void setClosed(boolean closed) {
        this.closed = closed;
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        builder.append("Path{")
                .append("positions=[");
        for(Position pos : positions){
            builder.append(pos.toString());
        }
        builder.append("], closed=" + closed +
                '}');
        return builder.toString();
    }
}
