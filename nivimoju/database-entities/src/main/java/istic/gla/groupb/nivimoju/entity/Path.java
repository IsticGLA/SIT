package istic.gla.groupb.nivimoju.entity;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by jeremy on 13/04/15.
 */
public class Path implements Serializable {

    private long idPath;
    private List<Position> positions;
    private boolean closed;

    public Path(){
        this.idPath = -1;
        this.positions = new ArrayList<>();
        this.closed = false;
    }

    public Path(long idPath){
        this.idPath = idPath;
        this.positions = new ArrayList<>();
        this.closed = false;
    }

    public long getIdPath() {
        return idPath;
    }

    public void setIdPath(long idPath) {
        this.idPath = idPath;
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
