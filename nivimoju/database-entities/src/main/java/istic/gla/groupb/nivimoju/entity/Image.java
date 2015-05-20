package istic.gla.groupb.nivimoju.entity;

import istic.gla.groupb.nivimoju.util.Constant;

import java.util.Arrays;

/**
 * Created by jeremy on 19/05/15.
 */
public class Image extends AbstractEntity {

    private double width;
    private double height;
    private long timestamp;
    private double[] position;
    private int[] image;
    private long idIntervention;

    public Image(){
        super();
        this.type = Constant.TYPE_IMAGE;
    }

    public Image(double width, long timestamp, double[] position, int[] bytes, long idIntervention){
        super();
        this.type = Constant.TYPE_IMAGE;
        this.width = width;
        this.height = bytes.length / (3 * width);
        this.timestamp = timestamp;
        this.position = position;
        this.image = bytes;
        this.idIntervention = idIntervention;
    }

    public double getWidth() {
        return width;
    }

    public void setWidth(double width) {
        this.width = width;
    }

    public double getHeight() {
        return height;
    }

    public void setHeight(double height) {
        this.height = height;
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

    public int[] getImage() {
        return image;
    }

    public void setImage(int[] image) {
        this.image = image;
    }

    public long getIdIntervention() {
        return idIntervention;
    }

    public void setIdIntervention(long idIntervention) {
        this.idIntervention = idIntervention;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        Image image1 = (Image) o;

        if (Double.compare(image1.height, height) != 0) return false;
        if (idIntervention != image1.idIntervention) return false;
        if (timestamp != image1.timestamp) return false;
        if (Double.compare(image1.width, width) != 0) return false;
        if (!Arrays.equals(image, image1.image)) return false;
        if (!Arrays.equals(position, image1.position)) return false;

        return true;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        temp = Double.doubleToLongBits(width);
        result = (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(height);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        result = 31 * result + (int) (timestamp ^ (timestamp >>> 32));
        result = 31 * result + Arrays.hashCode(position);
        result = 31 * result + Arrays.hashCode(image);
        result = 31 * result + (int) (idIntervention ^ (idIntervention >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return "Image{" +
                "width=" + width +
                ", height=" + height +
                ", timestamp=" + timestamp +
                ", position=" + Arrays.toString(position) +
                ", image=" + Arrays.toString(image) +
                ", idIntervention=" + idIntervention +
                '}';
    }
}
