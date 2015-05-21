package istic.gla.groupb.nivimoju.drone;

import istic.gla.groupb.nivimoju.drone.latlong.LocalCoordinate;

import java.util.Arrays;

/**
 * Container of infos about a single image
 */
public class FlaskImage {

    LocalCoordinate position;
    String image;
    String droneLabel;
    int width;

    public LocalCoordinate getPosition() {
        return position;
    }

    public void setPosition(LocalCoordinate position) {
        this.position = position;
    }

    public String getImage() {
        return image;
    }

    public void setImage(String image) {
        this.image = image;
    }

    public String getDroneLabel() {
        return droneLabel;
    }

    public void setDroneLabel(String droneLabel) {
        this.droneLabel = droneLabel;
    }

    public int getWidth() {
        return width;
    }

    public void setWidth(int width) {
        this.width = width;
    }

    @Override
    public String toString() {
        return "FlaskImage{" +
                "position=" + position +
                ", image=" + image +
                ", droneLabel='" + droneLabel + '\'' +
                ", width=" + width +
                '}';
    }
}
