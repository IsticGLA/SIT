package istic.gla.groupeb.flerjeco.utils;

import android.util.Log;

import com.google.android.gms.maps.model.LatLng;

import istic.gla.groupeb.flerjeco.agent.droneVisualisation.VisualisationMapFragment;

public final class LatLngUtils {
    private static final String TAG = LatLngUtils.class.getSimpleName();

    private LatLngUtils(){}

    // Get middle point between two coordinates
    public static LatLng midPoint(double lat1,double lon1,double lat2,double lon2){

        double dLon = Math.toRadians(lon2 - lon1);

        //convert to radians
        lat1 = Math.toRadians(lat1);
        lat2 = Math.toRadians(lat2);
        lon1 = Math.toRadians(lon1);

        double Bx = Math.cos(lat2) * Math.cos(dLon);
        double By = Math.cos(lat2) * Math.sin(dLon);
        double lat3 = Math.atan2(Math.sin(lat1) + Math.sin(lat2), Math.sqrt((Math.cos(lat1) + Bx) * (Math.cos(lat1) + Bx) + By * By));
        double lon3 = lon1 + Math.atan2(By, Math.cos(lat1) + Bx);

        //print out in degrees
        Log.i(TAG, lat1 + "  " + lon1 + "       " + lat2 + "   " + lon2);
        Log.i(TAG, Math.toDegrees(lat3) + " " + Math.toDegrees(lon3));

        return new LatLng(Math.toDegrees(lat3), Math.toDegrees(lon3));
    }

    // get angle between two coordinates
    public static double angleFromCoordinate(double lat1, double long1, double lat2, double long2) {
        double dLon = (long2 - long1);

        double y = Math.sin(dLon) * Math.cos(lat2);
        double x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1)
                * Math.cos(lat2) * Math.cos(dLon);

        double brng = Math.atan2(y, x);

        brng = Math.toDegrees(brng);
        brng = (brng + 360) % 360;
        brng = 360 - brng;

        return brng;
    }
}
