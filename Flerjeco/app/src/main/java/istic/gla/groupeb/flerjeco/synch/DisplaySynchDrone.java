package istic.gla.groupeb.flerjeco.synch;


import android.os.Parcel;
import android.os.Parcelable;

/**
 * Created by amhachi on 13/04/15.
 */
public abstract class DisplaySynchDrone implements Parcelable{
    @Override
    public int describeContents() {
        return 0;
    }

    public static  DisplaySynchDrone displaySynchDrone ;

    protected DisplaySynchDrone() {
        DisplaySynchDrone.displaySynchDrone = this;
    }

    public static final Parcelable.Creator<DisplaySynchDrone> CREATOR
            = new Parcelable.Creator<DisplaySynchDrone>() {
        public DisplaySynchDrone createFromParcel(Parcel in) {
            return displaySynchDrone;
        }

        public DisplaySynchDrone[] newArray(int size) {
            return new DisplaySynchDrone[size];
        }
    };

    @Override
    public void writeToParcel(Parcel dest, int flags) {}

    public abstract void ctrlDisplay();

}
