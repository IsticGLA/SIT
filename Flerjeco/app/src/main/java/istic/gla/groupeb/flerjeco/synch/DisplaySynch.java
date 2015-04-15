package istic.gla.groupeb.flerjeco.synch;


import android.os.Parcel;
import android.os.Parcelable;

/**
 * Created by amhachi on 13/04/15.
 */
public abstract class DisplaySynch implements Parcelable{
    @Override
    public int describeContents() {
        return 0;
    }

    public static  DisplaySynch displaySynch ;

    protected DisplaySynch() {
        DisplaySynch.displaySynch = this;
    }

    public static final Parcelable.Creator<DisplaySynch> CREATOR
            = new Parcelable.Creator<DisplaySynch>() {
        public DisplaySynch createFromParcel(Parcel in) {
            return displaySynch;
        }

        public DisplaySynch[] newArray(int size) {
            return new DisplaySynch[size];
        }
    };

    @Override
    public void writeToParcel(Parcel dest, int flags) {}

    public abstract void ctrlDisplay();

}
