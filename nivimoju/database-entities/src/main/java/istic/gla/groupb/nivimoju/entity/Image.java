package istic.gla.groupb.nivimoju.entity;

import istic.gla.groupb.nivimoju.util.Constant;
import org.apache.log4j.Logger;

import java.util.Arrays;

/**
 * Created by jeremy on 19/05/15.
 */
public class Image extends AbstractEntity {
    Logger logger = Logger.getLogger(Image.class);

    private long timestamp;
    private double[] position;
    private String base64Image;
    private long idIntervention;

    public Image(){
        super();
        this.type = Constant.TYPE_IMAGE;
    }

    public Image(long timestamp, double[] position, String base64Image, long idIntervention){
        super();
        this.type = Constant.TYPE_IMAGE;
        this.timestamp = timestamp;
        this.position = position;
        this.base64Image = base64Image;
        this.idIntervention = idIntervention;
    }

    public long getTimestamp() {
        return timestamp;
    }

    public void setTimestamp(long timestamp) {
        this.timestamp = timestamp;
    }

    public double[] getPosition() {
        return position;
    }

    public void setPosition(double[] position) {
        this.position = position;
    }

    public String getBase64Image() {
        return base64Image;
    }

    public void setBase64Image(String base64Image) {
        this.base64Image = base64Image;
    }

    public long getIdIntervention() {
        return idIntervention;
    }

    public void setIdIntervention(long idIntervention) {
        this.idIntervention = idIntervention;
    }

    public Position[] boundAroundPoint(){
        double R = 6371;  // earth radius in km
        double radius = 0.002; // km
        double lat = this.getPosition()[0];
        double lon = this.getPosition()[1];
        double x1 = lat - Math.toDegrees(radius/R);
        double x2 = lat + Math.toDegrees(radius/R);
        double y1 = lon - Math.toDegrees(radius/R/Math.cos(Math.toRadians(lat)));
        double y2 = lon + Math.toDegrees(radius/R/Math.cos(Math.toRadians(lat)));
        logger.debug("Position : " + lat + "   " + lon);
        logger.debug("FirstPoint : " + x1 + "   " + y1);
        logger.debug("SecondPoint : " + x2 + "   " + y2);
        Position[] tab = {new Position(x1, y1), new Position(x2, y2)};
        return tab;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        Image image1 = (Image) o;

        if (idIntervention != image1.idIntervention) return false;
        if (timestamp != image1.timestamp) return false;
        if (!base64Image.equals(image1.base64Image)) return false;
        if (!Arrays.equals(position, image1.position)) return false;

        return true;
    }

    @Override
    public int hashCode() {
        int result = (int) (timestamp ^ (timestamp >>> 32));
        result = 31 * result + Arrays.hashCode(position);
        result = 31 * result + base64Image.hashCode();
        result = 31 * result + (int) (idIntervention ^ (idIntervention >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return "Image{" +
                ", timestamp=" + timestamp +
                ", position=" + Arrays.toString(position) +
                ", base64Image=" + base64Image +
                ", idIntervention=" + idIntervention +
                '}';
    }
}
