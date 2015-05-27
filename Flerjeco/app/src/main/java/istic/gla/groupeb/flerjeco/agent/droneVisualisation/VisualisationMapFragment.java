package istic.gla.groupeb.flerjeco.agent.droneVisualisation;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.Bundle;
import android.os.Handler;
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
import com.google.android.gms.maps.model.Polygon;
import com.google.android.gms.maps.model.PolygonOptions;
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
import istic.gla.groupb.nivimoju.entity.Area;
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
import istic.gla.groupeb.flerjeco.utils.LatLngUtils;

/**
 * Fragment de carte pour les drones
 */
public class VisualisationMapFragment extends Fragment implements DronesMapFragment, IImageActivity, GoogleMap.OnMarkerClickListener {

    private static final String TAG = VisualisationMapFragment.class.getSimpleName();
    final static String ARG_POSITION = "position";

    // all variables for the Google Map
    MapView mMapView;
    private GoogleMap googleMap;
    private List<Pair<Polyline, Marker>> polylines;
    private List<Marker> markers;
    private List<Polygon> polygons;
    private PolygonOptions polygonOptions;

    private VisualisationActivity activity;

    private LatLngBounds mBounds;
    // list of all the path of the intervention
    private List<Path> pathList;
    // list of all the area of the intervention
    private List<Area> areaList;
    // current intervention
    private Intervention inter;

    // list all drone of the intervention
    private Map<String, Marker> dronesMarkers;
    private boolean refreshDrones = false;

    GetPositionDroneTask getPositionDroneTask;
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

        activity = (VisualisationActivity) getActivity();
        inter = activity.getIntervention();

        this.pathList = activity.getIntervention().getWatchPath();
        this.areaList = activity.getIntervention().getWatchArea();
        this.polylines = new ArrayList<>();
        this.polygons = new ArrayList<>();
        this.markers = new ArrayList<>();
        this.dronesMarkers = new HashMap<>();
        this.polygonOptions = new PolygonOptions()
                .strokeColor(Color.parseColor("#9b24a6"))
                .fillColor(Color.argb(40, 238, 238, 0))
                .strokeWidth((float) 2);

        new Handler().postDelayed(new Runnable()
        {
            @Override
            public void run()
            {
                if(googleMap != null && mBounds != null)
                    googleMap.animateCamera(CameraUpdateFactory.newLatLngBounds(mBounds, 50));
            }
        }, 500);

        return v;
    }

    /**
     * Update the Google Map with the path you just clicked
     */
    public void updateMapView() {
        // clear the Google Map
        clearGoogleMap();
        googleMap.setOnMarkerClickListener(this);

        if (getActivity() == null) {
            return;
        }

        // get the intervention from the main activity
        inter = activity.getIntervention();
        pathList = inter.getWatchPath();
        areaList = inter.getWatchArea();

        // Create LatLngBound to zoom on the set of positions in the path
        LatLngBounds.Builder bounds = new LatLngBounds.Builder();

        // Draw paths of the intervention
        for (Path p : pathList) {
            // get all the positions of the path to draw it
            List<Position> positions = p.getPositions();

            for (int i = 0; i < positions.size(); i++) {
                double latitude = positions.get(i).getLatitude();
                double longitude = positions.get(i).getLongitude();
                LatLng latLng = new LatLng(latitude, longitude);
                bounds.include(latLng);

                //add position to put an image
                timestampedPositions.add(new TimestampedPosition(new Position(latitude, longitude), new Timestamp(0).getTime()));

                Marker _marker = null;
                for (Image image : images) {
                    if(image.getPosition()[0] == latitude && image.getPosition()[1] == longitude) {
                        byte[] decodedString = Base64.decode(image.getBase64Image(), Base64.DEFAULT);
                        Bitmap decodedByte = BitmapFactory.decodeByteArray(decodedString, 0, decodedString.length);
                        _marker = drawImageMarker(image.getPosition()[0], image.getPosition()[1], decodedByte);
                        markers.add(_marker);
                        break;
                    }
                }
                if(_marker == null) {
                    // create marker
                    MarkerOptions marker = new MarkerOptions().position(latLng);
                    // Changing marker icon
                    marker.icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_VIOLET));
                    Marker m = googleMap.addMarker(marker);
                    // add the marker on the markers list
                    markers.add(m);
                }

                // draw line between two points if is not the first
                if (i > 0) {
                    LatLng previousLatLng = new LatLng(positions.get(i - 1).getLatitude(), positions.get(i - 1).getLongitude());
                    drawLine(latLng, previousLatLng);
                }
                if (i == positions.size() - 1 && p.isClosed() && positions.size() > 2) {
                    LatLng firstLatLng = new LatLng(positions.get(0).getLatitude(), positions.get(0).getLongitude());
                    drawLine(firstLatLng, latLng);
                }
            }
        }

        // Draw area of the intervention
        for (Area area : areaList){
            for (Position pos : area.getPositions()) {
                LatLng latLng = new LatLng(pos.getLatitude(), pos.getLongitude());
                bounds.include(latLng);
                polygonOptions.add(latLng);
            }
            polygons.add(googleMap.addPolygon(polygonOptions));
        }

        if (markers.size() <= 0 && polygons.size() <= 0) {
            bounds.include(new LatLng(inter.getLatitude(), inter.getLongitude()));
        }
        this.mBounds = bounds.build();

        //Update images
        new GetLastImageTask(this).execute(inter.getId(), timestampedPositions);
    }

    public Marker drawImageMarker(double latitude, double longitude, Bitmap image) {
        Bitmap.Config conf = Bitmap.Config.ARGB_8888;
        Bitmap bmp = Bitmap.createBitmap(72, 83, conf);
        Canvas canvas = new Canvas(bmp);

        Paint color = new Paint();
        color.setTextSize(20);
        color.setColor(Color.WHITE);


        canvas.drawBitmap(Bitmap.createScaledBitmap(image, 72, 72, true), 0, 0, color);
        canvas.drawBitmap(Bitmap.createScaledBitmap(BitmapFactory.decodeResource(getResources(), R.drawable.marker), 72, 83, true), 0, 0, color);

        return googleMap.addMarker(new MarkerOptions()
                .position(new LatLng(latitude, longitude))
                .icon(BitmapDescriptorFactory.fromBitmap(bmp))
                .anchor(0.5f, 1));
    }

    /**
     * Clear the Google Map
     */
    public void clearGoogleMap(){
        googleMap.clear();
        this.polylines.clear();
        this.markers.clear();
        this.dronesMarkers.clear();
        this.polygons.clear();
        this.polygonOptions = new PolygonOptions()
                .strokeColor(Color.parseColor("#9b24a6"))
                .fillColor(Color.argb(40, 238, 238, 0))
                .strokeWidth((float) 2);
    }

    /**
     * Drw polyline on the Google Map from first and last LatLng
     * @param first start point of the polyline to draw
     * @param last last point of the polyline to draw
     */
    public void drawLine(LatLng first, LatLng last){
        // First you need rotate the bitmap of the arrowhead somewhere in your code
        float rotationDegrees = (float) LatLngUtils.angleFromCoordinate(last.latitude, last.longitude, first.latitude, first.longitude);

        // Create the rotated arrowhead bitmap
        Bitmap arrow = BitmapFactory.decodeResource(getResources(), R.drawable.arrow);
        BitmapDescriptor bitmapDescriptorFactory = BitmapDescriptorFactory.fromBitmap(arrow);
        // Get the middle position
        LatLng middlePos = LatLngUtils.midPoint(first.latitude, first.longitude, last.latitude, last.longitude);
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

    // private variable for reducing object creations
    private Set<String> labels = new HashSet<>();

    /**
     * Show the marker for the drone of the intervention
     */
    public void showDrones(Drone[] tab){
        if(refreshDrones) {
            labels.clear();
            for(Drone drone : tab){
                labels.add(drone.getLabel());
                if(dronesMarkers.containsKey(drone.getLabel())){
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
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        mMapView.onResume();
        updateMapView();

        refreshDrones = true;
        getPositionDroneTask = new GetPositionDroneTask(this, inter.getId());
        getPositionDroneTask.execute();
    }

    @Override
    public void onPause() {
        super.onPause();
        getPositionDroneTask.cancel((true));
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

    public void refreshDrones() {
        if(inter != null && refreshDrones) {
            getPositionDroneTask = new GetPositionDroneTask(this, inter.getId());
            getPositionDroneTask.execute();
        } else{
            Log.e(TAG, "inter : " + inter + "refreshDrones : " + refreshDrones);
        }
    }


    @Override
    public boolean onMarkerClick(Marker marker) {
        if(dronesMarkers.containsKey(marker.getTitle())){
            activity.loadDroneStream(marker.getTitle());
        } else{
            activity.loadImageSlider(marker.getPosition());
        }
        return false;
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

