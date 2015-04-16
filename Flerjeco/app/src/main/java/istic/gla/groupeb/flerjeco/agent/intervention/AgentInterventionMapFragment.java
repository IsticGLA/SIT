package istic.gla.groupeb.flerjeco.agent.intervention;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.MapView;
import com.google.android.gms.maps.MapsInitializer;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.CameraPosition;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.LatLngBounds;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import entity.Intervention;
import entity.Resource;
import entity.StaticData;
import istic.gla.groupeb.flerjeco.FlerjecoApplication;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.icons.Danger;
import istic.gla.groupeb.flerjeco.icons.Sensitive;
import istic.gla.groupeb.flerjeco.icons.Vehicle;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;
import util.ResourceCategory;
import util.ResourceRole;
import util.State;

/**
 * A fragment that launches other parts of the demo application.
 */
public class AgentInterventionMapFragment extends Fragment implements ISynchTool {

    final static String ARG_POSITION = "position";
    private static final String TAG = AgentInterventionMapFragment.class.getSimpleName();

    MapView mMapView;
    Button buttonValidateResources;
    Button buttonCancelResources;
    private GoogleMap googleMap;

    int position = -1;

    private Intervention intervention;
    private LatLngBounds.Builder bounds;
    private StaticData[] staticDataTab;
    private Set<Resource> resources = new HashSet<>();
    private Map<String, com.google.android.gms.maps.model.Marker> markers = new HashMap<>();

    @Override
    public void refresh() {
        if (null != getActivity()){
            // clear lists
            clearMapData();
            // fill lists
            initMap();
        }
    }

    private void clearMapData(){
        for (Resource resource : resources){
            if (markers.get(resource.getLabel()) != null) {
                markers.get(resource.getLabel()).remove();
            }
        }
        resources.clear();
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {

        // inflat and return the layout
        View v = inflater.inflate(R.layout.map_view, container,
                false);
        mMapView = (MapView) v.findViewById(R.id.mapView);
        mMapView.onCreate(savedInstanceState);

        FlerjecoApplication flerjecoApplication = FlerjecoApplication.getInstance();
        if (flerjecoApplication != null) {
            staticDataTab = flerjecoApplication.getStaticData();
        }

        mMapView.onResume();// needed to get the map to refresh immediately

        buttonCancelResources = (Button) v.findViewById(R.id.buttonCancelResources);

        try {
            MapsInitializer.initialize(getActivity().getApplicationContext());
        } catch (Exception e) {
            e.printStackTrace();
        }

        googleMap = mMapView.getMap();

        AgentInterventionActivity interventionActivity = (AgentInterventionActivity) getActivity();
        intervention = interventionActivity.intervention;
        initMap();

        /*googleMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
            @Override
            public void onMapClick(LatLng latLng) {
                double latitude = latLng.latitude;
                double longitude = latLng.longitude;
                Log.i(getActivity().getLocalClassName(), "Click on the Map at " + latitude + ", " + longitude + " for item " + position);

                Resource resourceToPut = resourcesToPutOnMap.get(position);

                // create marker
                MarkerOptions marker = new MarkerOptions().position(
                        new LatLng(latitude, longitude)).title(resourceToPut.getLabel());
                // Changing marker icon
                drawMarker(marker, resourceToPut);

                if (markers.get(resourceToPut.getLabel()) != null) {
                    markers.get(resourceToPut.getLabel()).remove();
                }
                // adding marker
                Marker markerAdded = googleMap.addMarker(marker);
                markers.put(resourceToPut.getLabel(), markerAdded);

                Log.d(getClass().getSimpleName(), "resource : " + resourceToPut.getLabel());

                resourcesPutOnMap.add(resourceToPut);

                buttonValidateResources.setVisibility(View.VISIBLE);
                buttonCancelResources.setVisibility(View.VISIBLE);
            }
        });*/
        return v;
    }

    public void cancelResources(){
        for (Resource resource : resources){
            if (markers.get(resource.getLabel()) != null) {
                markers.get(resource.getLabel()).remove();
            }
        }
        resources.clear();
        buttonValidateResources.setVisibility(View.GONE);
        buttonCancelResources.setVisibility(View.GONE);
    }

    public void initMap(){
        // get intervention
        intervention = ((AgentInterventionActivity) getActivity()).intervention;

        if (null != intervention){
            // Create LatLngBound to zoom on the set of positions in the path
            bounds = new LatLngBounds.Builder();
            boolean isPositionResource = false;

            if (staticDataTab != null && staticDataTab.length > 0){
                for (StaticData data : staticDataTab){
                    MarkerOptions marker = new MarkerOptions().position(
                            new LatLng(data.getLatitude(), data.getLongitude()));
                    drawStaticMarker(marker, data);
                    googleMap.addMarker(marker);
                }
            }

            if (intervention.getResources().size()>0){

                for (final Resource resource : intervention.getResources()){
                    double latitude = resource.getLatitude();
                    double longitude = resource .getLongitude();
                    if(latitude != 0 && longitude != 0) {
                        Log.i(TAG, "Latitude : "+latitude+" Longitude : "+longitude);
                        isPositionResource = true;
                        LatLng latLng = new LatLng(latitude, longitude);
                        bounds.include(latLng);
                    }

                    State resourceState = resource.getState();
                    if (State.active.equals(resourceState) || State.planned.equals(resourceState)){
                        // create marker
                        MarkerOptions marker = new MarkerOptions().position(
                                new LatLng(resource.getLatitude(), resource.getLongitude())).title(resource.getLabel());

                        marker.draggable(true);
                        // Changing marker icon
                        drawMarker(marker, resource);
                        // adding marker
                        Marker markerAdded = googleMap.addMarker(marker);
                        markers.put(resource.getLabel(), markerAdded);

                        resources.add(resource);
                    }
                    googleMap.setOnMarkerClickListener(new GoogleMap.OnMarkerClickListener() {
                        @Override
                        public boolean onMarkerClick(Marker marker) {
                            if (!"incident".equals(resource.getLabel())) {
                                ((AgentInterventionActivity) getActivity()).showManageResourceDialog(resource);
                            }
                            return false;
                        }
                    });
                }
            }

            if(!isPositionResource) {
                CameraPosition cameraPosition = new CameraPosition.Builder()
                        .target(new LatLng(intervention.getLatitude(), intervention.getLongitude())).zoom(12).build();

                googleMap.animateCamera(CameraUpdateFactory
                        .newCameraPosition(cameraPosition));
            } else {
                googleMap.setOnCameraChangeListener(new GoogleMap.OnCameraChangeListener() {

                    @Override
                    public void onCameraChange(CameraPosition arg0) {
                        // Move camera.
                        googleMap.moveCamera(CameraUpdateFactory.newLatLngBounds(bounds.build(), 50));
                        // Remove listener to prevent position reset on camera move.
                        googleMap.setOnCameraChangeListener(null);
                    }
                });
            }
        }


    }

    public int getPosition() {
        return position;
    }

    public void setPosition(int position) {
        this.position = position;
    }

    @Override
    public void onResume() {
        super.onResume();
        mMapView.onResume();
    }

    @Override
    public void onPause() {
        super.onPause();
        mMapView.onPause();
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

    public void drawStaticMarker(MarkerOptions markerOptions, StaticData data){
        Bitmap bmp = null;
        switch (data.getMarkerType()){
            case waterSource:
                bmp = BitmapFactory.decodeResource(getResources(), R.drawable.watersource);
                break;
            case danger:
                Danger danger = new Danger();
                bmp = Bitmap.createBitmap(60, 60, Bitmap.Config.ARGB_8888);
                Canvas mCanvas = new Canvas(bmp);
                danger.drawIcon(mCanvas);
                break;
            case incident:
                bmp = BitmapFactory.decodeResource(getResources(), R.drawable.incident);
                break;
        }
        if (bmp != null) {
            markerOptions.icon(BitmapDescriptorFactory.fromBitmap(bmp));
        }
    }

    public void drawMarker(MarkerOptions markerOptions, Resource resource){
        ResourceCategory category = resource.getResourceCategory();
        Bitmap mBitmap = null;
        if (category!=null){
            switch (category){
                case vehicule:
                    ResourceRole role = resource.getResourceRole() != null ? resource.getResourceRole() : ResourceRole.otherwise;
                    String name = resource.getLabel()+" "+resource.getIdRes();
                    Vehicle mVehicle = new Vehicle(name, role, resource.getState());
                    int width = mVehicle.getRect().width();
                    int height = mVehicle.getRect().height()+mVehicle.getRect2().height()+10;
                    mBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
                    Canvas mCanvas = new Canvas(mBitmap);
                    mVehicle.drawIcon(mCanvas);
                    break;
                case dragabledata:
                    String label = resource.getLabel();
                    ResourceRole resourceRole = resource.getResourceRole() != null ? resource.getResourceRole() : ResourceRole.otherwise;
                    if ("incident".equals(label)){
                        mBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.incident);
                    } else if ("danger".equals(label)) {
                        Danger danger = new Danger(resourceRole);
                        mBitmap = Bitmap.createBitmap(40, 40, Bitmap.Config.ARGB_8888);
                        Canvas dCanvas = new Canvas(mBitmap);
                        danger.drawIcon(dCanvas);
                    } else if ("sensitive".equals(label)) {
                        Sensitive sensitive = new Sensitive(resourceRole);
                        mBitmap = Bitmap.createBitmap(40, 40, Bitmap.Config.ARGB_8888);
                        Canvas sCanvas = new Canvas(mBitmap);
                        sensitive.drawIcon(sCanvas);
                    }
                    break;
            }
            if (mBitmap != null){
                markerOptions.icon(BitmapDescriptorFactory.fromBitmap(mBitmap));
            }

        }
    }

    public Map<String, Marker> getMarkers() {
        return markers;
    }
}