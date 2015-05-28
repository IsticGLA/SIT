package istic.gla.groupb.nivimoju.entity;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by jeremy on 13/04/15.
 */
public class Area implements Serializable {

    private long idArea;
    private List<Position> positions;

    public Area(){
        this.idArea = -1;
        this.positions = new ArrayList<>();
    }

    public Area(long idArea){
        this.idArea = idArea;
        this.positions = new ArrayList<>();
    }

    public long getIdArea() {
        return idArea;
    }

    public void setIdArea(long idArea) {
        this.idArea = idArea;
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


    @Override
    public String toString() {
        return "Area{" +
                "idArea=" + idArea +
                ", positions=" + positions +
                '}';
    }
}
