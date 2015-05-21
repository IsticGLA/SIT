package istic.gla.groupb.nivimoju.drone;

import istic.gla.groupb.nivimoju.drone.latlong.LocalCoordinate;

/**
 * Container of infos about a single base64Image
 */
public class FlaskImage {

    LocalCoordinate position;
    String base64Image;
    String droneLabel;
    int width;

    public LocalCoordinate getPosition() {
        return position;
    }

    public void setPosition(LocalCoordinate position) {
        this.position = position;
    }

    public String getBase64Image() {
        return base64Image;
    }

    public void setBase64Image(String base64Image) {
        this.base64Image = base64Image;
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
                ", base64Image=" + base64Image +
                ", droneLabel='" + droneLabel + '\'' +
                ", width=" + width +
                '}';
    }
}
