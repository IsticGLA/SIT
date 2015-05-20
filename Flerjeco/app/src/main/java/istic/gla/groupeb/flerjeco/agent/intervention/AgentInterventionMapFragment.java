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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Resource;
import istic.gla.groupb.nivimoju.entity.StaticData;
import istic.gla.groupeb.flerjeco.FlerjecoApplication;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.icons.Danger;
import istic.gla.groupeb.flerjeco.icons.Sensitive;
import istic.gla.groupeb.flerjeco.icons.Vehicle;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;
import istic.gla.groupb.nivimoju.util.ResourceCategory;
import istic.gla.groupb.nivimoju.util.ResourceRole;
import istic.gla.groupb.nivimoju.util.State;

/**
 * A fragment that launches other parts of the demo application.
 */
public class AgentInterventionMapFragment extends Fragment implements ISynchTool {

    final static String ARG_POSITION = "position";
    private static final String TAG = AgentInterventionMapFragment.class.getSimpleName();

    MapView mMapView;
    private GoogleMap googleMap;

    int position = -1;

    private boolean initMap = true;
    private Intervention intervention;
    private List<Resource> resources = new ArrayList<>();
    private Map<String, com.google.android.gms.maps.model.Marker> labelsMarkersHashMap = new HashMap<>();
    private Map<String, Resource> labelsResourcesHashMap = new HashMap<>();

    @Override
    public void refresh() {
        if (null != getActivity()){

            Log.i(TAG, "refresh, clearMapData");
            // clear lists
            clearMapData();
            // fill lists
            initMap();
        }
    }

    private void clearMapData(){
        for (Resource resource : resources){
            if (labelsMarkersHashMap.get(resource.getLabel()) != null) {
                labelsMarkersHashMap.get(resource.getLabel()).remove();
            }
        }
        resources.clear();
        labelsResourcesHashMap.clear();
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {

        View v = inflater.inflate(R.layout.map_view, container,
                false);

        mMapView = (MapView) v.findViewById(R.id.mapView);
        mMapView.onCreate(savedInstanceState);
        mMapView.onResume();// needed to get the map to refresh immediately

        try {
            MapsInitializer.initialize(getActivity().getApplicationContext());
        } catch (Exception e) {
            e.printStackTrace();
        }

        googleMap = mMapView.getMap();

        initMap();

        return v;
    }

    public void initMap(){
        // get intervention
        intervention = ((AgentInterventionActivity) getActivity()).intervention;

        addStaticData();

        if (null != intervention){
            // Create LatLngBound to zoom on the set of positions in the path
            final LatLngBounds.Builder bounds = new LatLngBounds.Builder();
            boolean isPositionResource = false;

            if (intervention.getResources().size()>0){

                for (final Resource resource : intervention.getResources()){
                    double latitude = resource.getLatitude();
                    double longitude = resource .getLongitude();
                    if(latitude != 0 && longitude != 0) {

                        LatLng latLng = new LatLng(latitude, longitude);

                        State resourceState = resource.getState();

                        if (State.active.equals(resourceState) || State.planned.equals(resourceState)){

                            String resourceLabel = resource.getLabel();
                            // create marker
                            MarkerOptions marker = new MarkerOptions().position(latLng).title(resourceLabel);
                            drawMarker(marker, resource);
                            // adding marker
                            Marker markerAdded = googleMap.addMarker(marker);

                            labelsMarkersHashMap.put(resourceLabel, markerAdded);
                            resources.add(resource);
                            labelsResourcesHashMap.put(resourceLabel, resource);

                            Log.i(TAG, "Label "+resourceLabel+", Latitude : "+latitude+", Longitude : "+longitude);

                            if (intervention.getResources().size()>1) {
                                isPositionResource = true;
                                bounds.include(latLng);
                            }
                        }

                    }


                }
            }

            if(!isPositionResource && initMap) {
                CameraPosition cameraPosition = new CameraPosition.Builder()
                        .target(new LatLng(intervention.getLatitude(), intervention.getLongitude())).zoom(16).build();

                googleMap.animateCamera(CameraUpdateFactory
                        .newCameraPosition(cameraPosition));
                initMap = false;
            } else {
                googleMap.setOnCameraChangeListener(new GoogleMap.OnCameraChangeListener() {

                    @Override
                    public void onCameraChange(CameraPosition arg0) {
                        // Move camera.
                        try {
                            googleMap.moveCamera(CameraUpdateFactory.newLatLngBounds(bounds.build(), 50));
                        } catch (IllegalStateException e) {
                            Log.e(TAG, "Error: no included points for camera update");
                        }
                        // Remove listener to prevent position reset on camera move.
                        googleMap.setOnCameraChangeListener(null);
                    }
                });
            }

            googleMap.setOnMarkerClickListener(new GoogleMap.OnMarkerClickListener() {
                @Override
                public boolean onMarkerClick(Marker marker) {
                    Resource resource = labelsResourcesHashMap.get(marker.getTitle());
                    if (resource != null && !resource.getLabel().contains("incident")) {
                        ((AgentInterventionActivity) getActivity()).showManageResourceDialog(resource);
                    }
                    return false;
                }
            });

            googleMap.setOnMarkerDragListener(new GoogleMap.OnMarkerDragListener() {
                @Override
                public void onMarkerDragStart(Marker marker) {

                }

                @Override
                public void onMarkerDrag(Marker marker) {
                    
                }

                @Override
                public void onMarkerDragEnd(Marker marker) {
                    /*Resource resource = labelsResourcesHashMap.get(marker.getTitle());
                    if (resource != null) {
                        resource.setState(State.planned);
                        if (null != getActivity()) {
                            ((AgentInterventionActivity) getActivity()).resourceUpdated(resource);
                        }
                    }*/
                    /*String title = marker.getTitle();
                    LatLng latLng = marker.getPosition();
                    Resource resource = labelsResourcesHashMap.get(title);
                    resource.setLatitude(latLng.longitude);
                    resource.setLongitude(latLng.longitude);
                    if (null != getActivity()){
                        ((AgentInterventionActivity) getActivity()).resourceUpdated(resource);
                    }*/
                }
            });
        }


    }

    public int getPosition() {
        return position;
    }

    public void setPosition(int position) {
        this.position = position;
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
        markerOptions.draggable(true);
        Bitmap mBitmap = null;
        if (category!=null){
            switch (category){
                case vehicule:
                    String name = resource.getLabel();
                    ResourceRole role = resource.getResourceRole() != null ? resource.getResourceRole() : ResourceRole.otherwise;
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
                    if (label.contains("incident")){
                        mBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.incident);
                    } else if (label.contains("danger")) {
                        Danger danger = new Danger(resourceRole);
                        mBitmap = Bitmap.createBitmap(40, 40, Bitmap.Config.ARGB_8888);
                        Canvas dCanvas = new Canvas(mBitmap);
                        danger.drawIcon(dCanvas);
                    } else if (label.contains("sensitive")) {
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

    private void addStaticData(){
        StaticData[] staticDataTab;

        FlerjecoApplication flerjecoApplication = FlerjecoApplication.getInstance();
        if (flerjecoApplication != null) {
            staticDataTab = flerjecoApplication.getStaticData();

            if (staticDataTab != null && staticDataTab.length > 0){
                for (StaticData data : staticDataTab){
                    MarkerOptions marker = new MarkerOptions().position(
                            new LatLng(data.getLatitude(), data.getLongitude()));
                    drawStaticMarker(marker, data);
                    googleMap.addMarker(marker);
                }
            }
        }
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

    public Map<String, Marker> getLabelsMarkersHashMap() {
        return labelsMarkersHashMap;
    }
    public Map<String, Resource> getLabelsResourcesHashMap() {
        return labelsResourcesHashMap;
    }
}