package istic.gla.groupeb.flerjeco;

import android.os.Parcel;
import android.os.Parcelable;
import android.util.Pair;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by jules on 12/02/15.
 */
public class Coordinates implements Parcelable {

    private List<Pair<Double, Double>> coordinates;

    public Coordinates(List<Pair<Double, Double>> coordinates) {
        this.coordinates = coordinates;
    }

    private Coordinates(Parcel in) {
        coordinates = new ArrayList<Pair<Double, Double>>();
        double _coordinates [][] = (double[][])in.readArray(ClassLoader.getSystemClassLoader());
        for(int i = 0; i < _coordinates.length; i++) {
            coordinates.add(new Pair<Double, Double>(_coordinates[i][0], _coordinates[i][1]));
        }
    }

    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel dest, int flags) {
        double _coordinates [][]= new double[coordinates.size()][2];
        for(int i = 0; i < coordinates.size(); i++) {
            _coordinates[i][0] = coordinates.get(i).first;
            _coordinates[i][1] = coordinates.get(i).second;
        }
        dest.writeArray(_coordinates);
    }

    public static final Parcelable.Creator<Coordinates> CREATOR
            = new Parcelable.Creator<Coordinates>() {
        public Coordinates createFromParcel(Parcel in) {
            return new Coordinates(in);
        }

        public Coordinates[] newArray(int size) {
            return new Coordinates[size];
        }
    };

    public List<Pair<Double, Double>> getCoordinates() {
        return coordinates;
    }
}
