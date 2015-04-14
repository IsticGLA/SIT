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
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import entity.Intervention;
import entity.Resource;
import entity.StaticData;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.icons.Danger;
import istic.gla.groupeb.flerjeco.icons.Vehicle;
import util.ResourceRole;
import util.State;

/**
 * A fragment that launches other parts of the demo application.
 */
public class AgentInterventionMapFragment extends Fragment {

    final static String ARG_POSITION = "position";
    final static String STATIC_DATA = "staticdatas";

    MapView mMapView;
    Button buttonValidateResources;
    Button buttonCancelResources;
    private GoogleMap googleMap;

    int position = -1;

    private Intervention intervention;
    private StaticData[] staticDataTab;
    private List<Resource> resources = new ArrayList<>();
    private List<Resource> resourcesToPutOnMap = new ArrayList<>();
    private Set<Resource> resourcesPutOnMap = new HashSet<>();
    private Map<String, com.google.android.gms.maps.model.Marker> markers = new HashMap<>();


    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {

        Bundle args = getArguments();
        if (args != null) {
            // Set article based on argument passed in
            Object[] objects1 = (Object[]) args.getSerializable(STATIC_DATA);
            staticDataTab = new StaticData[objects1.length];
            for (int i = 0; i < objects1.length; i++) {
                staticDataTab[i] = (StaticData) objects1[i];
            }
        }
        // inflat and return the layout
        View v = inflater.inflate(R.layout.map_view, container,
                false);
        mMapView = (MapView) v.findViewById(R.id.mapView);
        mMapView.onCreate(savedInstanceState);
        mMapView.onResume();// needed to get the map to display immediately

        buttonValidateResources = (Button) v.findViewById(R.id.buttonValidateResources);
        buttonCancelResources = (Button) v.findViewById(R.id.buttonCancelResources);

        try {
            MapsInitializer.initialize(getActivity().getApplicationContext());
        } catch (Exception e) {
            e.printStackTrace();
        }

        googleMap = mMapView.getMap();

        AgentInterventionActivity interventionActivity = (AgentInterventionActivity) getActivity();
        initMap(interventionActivity.intervention);

        googleMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
            @Override
            public void onMapClick(LatLng latLng) {
                double latitude = latLng.latitude;
                double longitude = latLng.longitude;
                Log.i(getActivity().getLocalClassName(), "Click on the Map at " + latitude + ", " + longitude + " for item " + position);
                // create marker
                MarkerOptions marker = new MarkerOptions().position(
                        new LatLng(latitude, longitude)).title(resourcesToPutOnMap.get(position).getLabel());
                // Changing marker icon
                drawMarker(marker, resourcesToPutOnMap.get(position));

                if (markers.get(resourcesToPutOnMap.get(position).getLabel()) != null) {
                    markers.get(resourcesToPutOnMap.get(position).getLabel()).remove();
                }
                // adding marker
                Marker markerAdded = googleMap.addMarker(marker);
                markers.put(resourcesToPutOnMap.get(position).getLabel(), markerAdded);

                resourcesPutOnMap.add(resourcesToPutOnMap.get(position));

                buttonValidateResources.setVisibility(View.VISIBLE);
                buttonCancelResources.setVisibility(View.VISIBLE);
            }
        });
        return v;
    }

    public void cancelResources(){
        for (Resource resource : resourcesPutOnMap){
            if (markers.get(resource.getLabel()) != null) {
                markers.get(resource.getLabel()).remove();
            }
        }
        resourcesPutOnMap.clear();
        buttonValidateResources.setVisibility(View.GONE);
        buttonCancelResources.setVisibility(View.GONE);
    }

    public void updateMapView(int position) {
        Resource resource = resources.get(position);
        if (resource.getState() == State.planned || resource.getState() == State.active) {
            CameraPosition cameraPosition = new CameraPosition.Builder()
                    .target(new LatLng(resource.getLatitude(), resource.getLongitude())).zoom(16).build();
            googleMap.animateCamera(CameraUpdateFactory
                    .newCameraPosition(cameraPosition));
            this.position = position;
        }
    }

    public void initMap(Intervention intervention){
        this.intervention = intervention;

        if (staticDataTab != null && staticDataTab.length > 0){
            for (StaticData data : staticDataTab){
                MarkerOptions marker = new MarkerOptions().position(
                        new LatLng(data.getLatitude(), data.getLongitude()));
                drawStaticMarker(marker, data);
                googleMap.addMarker(marker);
            }
        }

        if (intervention.getResources().size()>0){

            for (Resource resource : intervention.getResources()){
                State resourceState = resource.getState();
                if (State.active.equals(resourceState) || State.planned.equals(resourceState)){
                    // create marker
                    MarkerOptions marker = new MarkerOptions().position(
                            new LatLng(resource.getLatitude(), resource.getLongitude())).title(resource.getLabel());
                    // Changing marker icon

                    drawMarker(marker, resource);
                    // adding marker
                    googleMap.addMarker(marker);

                    resources.add(resource);
                }else if (State.validated.equals(resourceState)){
                    resourcesToPutOnMap.add(resource);
                }

            }
        }

        CameraPosition cameraPosition = new CameraPosition.Builder()
                .target(new LatLng(intervention.getLatitude(), intervention.getLongitude())).zoom(12).build();

        googleMap.animateCamera(CameraUpdateFactory
                .newCameraPosition(cameraPosition));
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
                bmp = Bitmap.createBitmap(50, 50, Bitmap.Config.ARGB_8888);
                Canvas mCanvas = new Canvas(bmp);
                danger.drawDanger(mCanvas);
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
        switch (resource.getResourceCategory()){
            case vehicule:
                ResourceRole role = ResourceRole.otherwise;
                if (resource.getResourceRole()!=null) {
                    role = resource.getResourceRole();
                }
                Vehicle mVehicle = new Vehicle(resource.getLabel(), role, resource.getState());
                int width = mVehicle.getRect().width();
                int height = mVehicle.getRect().height()+mVehicle.getRect2().height()+10;
                Bitmap mBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
                Canvas mCanvas = new Canvas(mBitmap);
                mVehicle.drawVehicle(mCanvas);
                markerOptions.icon(BitmapDescriptorFactory.fromBitmap(mBitmap));
                break;
            case drone:
                break;
        }
    }

    public Set<Resource> getResourcesPutOnMap() {
        return resourcesPutOnMap;
    }
}