package istic.gla.groupb.nivimoju.drone.latlong;

import org.apache.commons.lang3.StringUtils;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by sacapuces on 13/04/15.
 */
public class LocalPath {
    List<LocalCoordinate> positions;
    boolean closed;

    public LocalPath(){
        this.positions = new ArrayList<>();
        this.closed = false;
    }

    public List<LocalCoordinate> getPositions() {
        return positions;
    }

    public void setPositions(List<LocalCoordinate> positions) {
        this.positions = positions;
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
        builder.append("LocalPath{")
                .append("positions=[");
        builder.append(StringUtils.join(positions, ", "));
        builder.append("], closed=" + closed +
                '}');
        return builder.toString();
    }
}
