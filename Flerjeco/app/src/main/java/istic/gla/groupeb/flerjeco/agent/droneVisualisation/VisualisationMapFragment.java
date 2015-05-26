package istic.gla.groupeb.flerjeco.agent.droneVisualisation;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Base64;
import android.util.Log;
import android.util.Pair;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.MapView;
import com.google.android.gms.maps.MapsInitializer;
import com.google.android.gms.maps.model.BitmapDescriptor;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.CameraPosition;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.LatLngBounds;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;

import org.apache.commons.collections4.CollectionUtils;

import java.sql.Timestamp;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import istic.gla.groupb.nivimoju.customObjects.TimestampedPosition;
import istic.gla.groupb.nivimoju.entity.Drone;
import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Path;
import istic.gla.groupb.nivimoju.entity.Position;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.agent.DronesMapFragment;
import istic.gla.groupeb.flerjeco.springRest.GetLastImageTask;
import istic.gla.groupeb.flerjeco.springRest.GetPositionDroneTask;
import istic.gla.groupeb.flerjeco.springRest.IImageActivity;
import istic.gla.groupeb.flerjeco.synch.IntentWraper;

/**
 * Fragment de carte pour les drones
 */
public class VisualisationMapFragment extends Fragment implements DronesMapFragment, IImageActivity {

    private static final String TAG = VisualisationMapFragment.class.getSimpleName();
    final static String ARG_POSITION = "position";

    // all variables for the Google Map
    MapView mMapView;
    private GoogleMap googleMap;
    private List<Pair<Polyline, Marker>> polylines;
    private List<Marker> markers;
    private Map<String, Marker> dronesMarkers;

    // list of all the path of the intervention
    private List<Path> pathList;
    // current intervention
    private Intervention inter;
    // list all drone of the intervention

    private boolean refreshDrones = false;

    //List of last images to draw
    private List<Image> images = new ArrayList<>();
    private List<TimestampedPosition> timestampedPositions = new ArrayList<>();

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // inflate and return the layout
        View v = inflater.inflate(R.layout.map_view, container, false);
        mMapView = (MapView) v.findViewById(R.id.mapView);
        mMapView.onCreate(savedInstanceState);

        // needed to get the map to refresh immediately
        mMapView.onResume();

        // Initialize Google Map
        try {
            MapsInitializer.initialize(getActivity().getApplicationContext());
        } catch (Exception e) {
            e.printStackTrace();
        }

        // get the googleMap
        googleMap = mMapView.getMap();

        VisualisationActivity activity = (VisualisationActivity) getActivity();
        inter = activity.getIntervention();

        // Center the camera on the intervention position
        CameraPosition cameraPosition = new CameraPosition.Builder()
                .target(new LatLng(activity.getIntervention().getLatitude(), activity.getIntervention().getLongitude())).zoom(16).build();
        googleMap.animateCamera(CameraUpdateFactory
                .newCameraPosition(cameraPosition));

        initMap(inter.getWatchPath());
        return v;
    }

    @Override
    public void onStart() {
        super.onStart();

        updateMapView();
    }

    /**
     * Update the Google Map with the path you just clicked
     */
    public void updateMapView() {
        // clear the Google Map
        clearGoogleMap();

        new GetLastImageTask(this).execute(inter.getId(), timestampedPositions);

        if(getActivity() == null) {
            return;
        }

        for(Image image : images) {
            byte[] decodedString = Base64.decode(image.getBase64Image(), Base64.DEFAULT);
            Bitmap decodedByte = BitmapFactory.decodeByteArray(decodedString, 0, decodedString.length);
            double lat = ((VisualisationActivity) getActivity()).getIntervention().getLatitude();
            double lon = ((VisualisationActivity) getActivity()).getIntervention().getLongitude();
            drawImageMarker(lat, lon, decodedByte, "test");
        }

        // get the intervention from the main activity
        inter = ((VisualisationActivity) getActivity()).getIntervention();
        pathList = inter.getWatchPath();

        // Create LatLngBound to zoom on the set of positions in the path
        LatLngBounds.Builder bounds = new LatLngBounds.Builder();

        for (Path p : pathList){

            // get all the positions of the path to draw it
            List<Position> positions = p.getPositions();

            for (int i = 0; i < positions.size(); i++){
                double latitude = positions.get(i).getLatitude();
                double longitude = positions.get(i).getLongitude();
                LatLng latLng = new LatLng(latitude, longitude);
                bounds.include(latLng);

                // create marker
                MarkerOptions marker = new MarkerOptions().position(latLng);
                // Changing marker icon
                marker.icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_AZURE));
                // adding marker
                Marker m = googleMap.addMarker(marker);
                // add the marker on the markers list
                markers.add(m);

                // draw line between two points if is not the first
                if (i > 0){
                    LatLng previousLatLng = new LatLng(positions.get(i-1).getLatitude(), positions.get(i-1).getLongitude());
                    drawLine(latLng, previousLatLng);
                // else if it the path is closed, draw line between first and last point
                } if (i == positions.size()-1 && p.isClosed() && positions.size() > 2){
                    LatLng firstLatLng = new LatLng(positions.get(0).getLatitude(), positions.get(0).getLongitude());
                    drawLine(firstLatLng, latLng);
                }
            }
        }
        //googleMap.animateCamera(CameraUpdateFactory.newLatLngBounds(bounds.build(), 50));
    }

    public void drawImageMarker(double latitude, double longitude, Bitmap image, String text) {
        Bitmap.Config conf = Bitmap.Config.ARGB_8888;
        Bitmap bmp = Bitmap.createBitmap(72, 83, conf);
        Canvas canvas = new Canvas(bmp);

        Paint color = new Paint();
        color.setTextSize(20);
        color.setColor(Color.WHITE);


        canvas.drawBitmap(Bitmap.createScaledBitmap(image, 72, 72, true), 0, 0, color);
        canvas.drawBitmap(Bitmap.createScaledBitmap(BitmapFactory.decodeResource(getResources(), R.drawable.marker), 72, 83, true), 0, 0, color);

        googleMap.addMarker(new MarkerOptions()
                .position(new LatLng(latitude, longitude))
                .icon(BitmapDescriptorFactory.fromBitmap(bmp))
                .anchor(0.5f, 1)
                .title(text)).showInfoWindow();
        CameraPosition cameraPosition = new CameraPosition.Builder().target(new LatLng(latitude, longitude)).zoom(3.0f).build();
        googleMap.animateCamera(CameraUpdateFactory.newCameraPosition(cameraPosition));
    }

    /**
     * Init the Google Map
     * @param pathList the list of paths
     */
    public void initMap(List<Path> pathList){
        this.pathList = pathList;
        this.polylines = new ArrayList<>();
        this.markers = new ArrayList<>();
        this.dronesMarkers = new HashMap<>();
    }

    /**
     * Clear the Google Map
     */
    public void clearGoogleMap(){
        googleMap.clear();
        this.polylines.clear();
        this.markers.clear();
        this.dronesMarkers.clear();
    }

    /**
     * Drw polyline on the Google Map from first and last LatLng
     * @param first start point of the polyline to draw
     * @param last last point of the polyline to draw
     */
    public void drawLine(LatLng first, LatLng last){
        // First you need rotate the bitmap of the arrowhead somewhere in your code
        float rotationDegrees = (float) angleFromCoordinate(last.latitude, last.longitude, first.latitude, first.longitude);

        // Create the rotated arrowhead bitmap
        Bitmap arrow = BitmapFactory.decodeResource(getResources(), R.drawable.arrow);
        BitmapDescriptor bitmapDescriptorFactory = BitmapDescriptorFactory.fromBitmap(arrow);
        // Get the middle position
        LatLng middlePos = midPoint(first.latitude, first.longitude, last.latitude, last.longitude);
        // Now we are gonna to add a marker
        Marker mArrowhead = googleMap.addMarker(new MarkerOptions()
                .position(middlePos)
                .flat(true)
                .anchor(0.5f, 0.5f)
                .rotation(rotationDegrees)
                .icon(bitmapDescriptorFactory));

        Polyline line = googleMap.addPolyline((new PolylineOptions())
                .add(first, last).width(3).color(Color.parseColor("#9b24a6"))
                .geodesic(true));

        polylines.add(new Pair<Polyline, Marker>(line, mArrowhead));
    }

    // Get middle point between two coordinates
    private LatLng midPoint(double lat1,double lon1,double lat2,double lon2){

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
        Log.i("TEST", lat1 + "  " + lon1 + "       " + lat2 + "   " + lon2);
        Log.i("TEST", Math.toDegrees(lat3) + " " + Math.toDegrees(lon3));

        return new LatLng(Math.toDegrees(lat3), Math.toDegrees(lon3));
    }

    // get angle between two coordinates
    private double angleFromCoordinate(double lat1, double long1, double lat2, double long2) {

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

    // private variable for reducing object creations
    private Set<String> labels = new HashSet<>();
    /**
     * Show the marker for the drone of the intervention
     */
    public void showDrones(Drone[] tab, long duration){
        if(refreshDrones) {
            labels.clear();
            for(Drone drone : tab){
                labels.add(drone.getLabel());
                if(dronesMarkers.containsKey(drone.getLabel())){
                    //animate existing marker
                    /**animateMarker(dronesMarkers.get(drone.getLabel()),
                            new LatLng(drone.getLatitude(), drone.getLongitude()),
                            false, duration
                    );*/
                    dronesMarkers.get(drone.getLabel()).setPosition(new LatLng(drone.getLatitude(), drone.getLongitude()));
                } else{
                    //create a new marker
                    Marker m = googleMap.addMarker(new MarkerOptions()
                            .position(new LatLng(drone.getLatitude(), drone.getLongitude()))
                            .title(drone.getLabel())
                            .anchor(0.5f, 0.5f)
                            .icon(BitmapDescriptorFactory.fromResource(R.drawable.ic_drone)));
                    dronesMarkers.put(drone.getLabel(), m);
                }
            }
            //delete marker for unassigned drones
            if(!labels.containsAll(dronesMarkers.keySet())){
                for(String labelToRemove : CollectionUtils.removeAll(dronesMarkers.keySet(), labels)){
                    dronesMarkers.get(labelToRemove).remove(); //remove the marker
                    dronesMarkers.remove(labelToRemove);
                }
            }
            new GetPositionDroneTask(this, ((VisualisationActivity) getActivity()).getIntervention().getId()).execute();
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        mMapView.onResume();

        long id = ((VisualisationActivity)getActivity()).getIntervention().getId();
        refreshDrones = true;
        new GetPositionDroneTask(this, id).execute();
    }

    @Override
    public void onPause() {
        super.onPause();
        refreshDrones = false;
        mMapView.onPause();
        IntentWraper.stopService();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        mMapView.onDestroy();
    }

    @Override
    public void onLowMemory() {
        super.onLowMemory();
        mMapView.onLowMemory();
    }

    public void refreshDrone() {
        if(inter != null) {
            new GetPositionDroneTask(this, inter.getId()).execute();
        }
    }

    @Override
    public void updateImages(Image[] images) {
        for(Image image : images) {
            for(TimestampedPosition timestampedPosition : timestampedPositions) {
                if(timestampedPosition.getPosition().equals(image.getPosition())) {
                    this.timestampedPositions.remove(timestampedPosition);
                    this.timestampedPositions.add(new TimestampedPosition(new Position(image.getPosition()[0], image.getPosition()[1]), image.getTimestamp()));
                    break;
                }
            }
            for(Image _image : this.images) {
                if(_image.getPosition().equals(image.getPosition())) {
                    this.images.remove(_image);
                    this.images.add(image);
                    break;
                }
            }
            this.images.add(image);
        }
    }
}

