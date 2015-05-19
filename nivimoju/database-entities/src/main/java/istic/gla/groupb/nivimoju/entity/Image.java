package istic.gla.groupb.nivimoju.entity;

import istic.gla.groupb.nivimoju.util.Constant;

import java.sql.Timestamp;

/**
 * Created by jeremy on 19/05/15.
 */
public class Image extends AbstractEntity {

    private double width;
    private double heigth;
    private Timestamp timestamp;
    private double[] position;
    private byte[] image;

    public Image(){
        super();
        this.type = Constant.TYPE_IMAGE;
    }

    public Image(double w, double h, Timestamp t, double[] p, byte[] b){
        super();
        this.type = Constant.TYPE_IMAGE;
        this.width = w;
        this.heigth = h;
        this.timestamp = t;
        this.position = p;
        this.image = b;
    }

    public double getWidth() {
        return width;
    }

    public void setWidth(double width) {
        this.width = width;
    }

    public double getHeigth() {
        return heigth;
    }

    public void setHeigth(double heigth) {
        this.heigth = heigth;
    }

    public Timestamp getTimestamp() {
        return timestamp;
    }

    public void setTimestamp(Timestamp timestamp) {
        this.timestamp = timestamp;
    }

    public double[] getPosition() {
        return position;
    }

    public void setPosition(double[] position) {
        this.position = position;
    }

    public byte[] getImage() {
        return image;
    }

    public void setImage(byte[] image) {
        this.image = image;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        Image image1 = (Image) o;

        if (Double.compare(image1.width, width) != 0) return false;
        if (Double.compare(image1.heigth, heigth) != 0) return false;
        if (!timestamp.equals(image1.timestamp)) return false;
        if (!position.equals(image1.position)) return false;
        return image.equals(image1.image);

    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        temp = Double.doubleToLongBits(width);
        result = (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(heigth);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        result = 31 * result + timestamp.hashCode();
        result = 31 * result + position.hashCode();
        result = 31 * result + image.hashCode();
        return result;
    }

    @Override
    public String toString() {
        return "Image{" +
                "width=" + width +
                ", heigth=" + heigth +
                ", timestamp=" + timestamp +
                ", position=" + position +
                ", image=" + image +
                '}';
    }
}
