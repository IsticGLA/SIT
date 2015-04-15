package istic.gla.groupb.nivimoju.drone.client;

import istic.gla.groupb.nivimoju.drone.latlong.LocalCoordinate;

/**
 * Created by sacapuces on 15/04/15.
 */
public class DroneInfo {
    private String label;
    private LocalCoordinate position;

    public String getLabel() {

        return label;
    }

    public void setLabel(String label) {
        this.label = label;
    }

    public LocalCoordinate getPosition() {
        return position;
    }

    public void setPosition(LocalCoordinate position) {
        this.position = position;
    }

    @Override
    public String toString() {
        return "DroneInfo{" +
                "label='" + label + '\'' +
                ", position=" + position +
                '}';
    }
}
