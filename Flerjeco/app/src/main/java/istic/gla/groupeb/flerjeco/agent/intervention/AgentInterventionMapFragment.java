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
import android.widget.FrameLayout;
import android.widget.ImageView;

import com.google.android.gms.maps.CameraUpdate;
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
import istic.gla.groupb.nivimoju.util.MarkerType;
import istic.gla.groupb.nivimoju.util.ResourceCategory;
import istic.gla.groupb.nivimoju.util.ResourceRole;
import istic.gla.groupb.nivimoju.util.State;
import istic.gla.groupeb.flerjeco.FlerjecoApplication;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.icons.Danger;
import istic.gla.groupeb.flerjeco.icons.Sensitive;
import istic.gla.groupeb.flerjeco.icons.Vehicle;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;

/**
 * A fragment that launches other parts of the demo application.
 */
public class AgentInterventionMapFragment extends Fragment implements ISynchTool {

    final static String ARG_POSITION = "position";
    private static final String TAG = AgentInterventionMapFragment.class.getSimpleName();

    MapView mMapView;
    private View mProgressView;
    private GoogleMap googleMap;
    private ImageView imageViewToDrag;
    private boolean markerOnLeftFragment = false;

    int position = -1;

    private boolean initMap = true;
    private Intervention intervention;
    private List<Resource> resources = new ArrayList<>();
    private Map<String, com.google.android.gms.maps.model.Marker> labelsMarkersHashMap = new HashMap<>();
    private Map<String, Bitmap> labelsBitmapHashMap = new HashMap<>();
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

        mProgressView = v.findViewById(R.id.map_progress);
        mProgressView.setVisibility(View.VISIBLE);

        mMapView = (MapView) v.findViewById(R.id.mapView);
        mMapView.onCreate(savedInstanceState);
        mMapView.onResume();// needed to get the map to refresh immediately

        try {
            MapsInitializer.initialize(getActivity().getApplicationContext());
        } catch (Exception e) {
            e.printStackTrace();
        }

        googleMap = mMapView.getMap();
        googleMap.getUiSettings().setScrollGesturesEnabled(false);

        initMap();

        return v;
    }

    public void initMap(){
        // get intervention
        intervention = ((AgentInterventionActivity) getActivity()).getIntervention();

        addStaticData();

        if (null != intervention){
            // Create LatLngBound to zoom on the set of positions in the path
            final LatLngBounds.Builder bounds = new LatLngBounds.Builder();

            if (intervention.getResources().size()>0){

                for (final Resource resource : intervention.getResources()){
                    double latitude = resource.getLatitude();
                    double longitude = resource .getLongitude();

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

                        Log.i(TAG, "Label "+resourceLabel+", Role : "+resource.getResourceRole());

                        bounds.include(latLng);
                    }
                }
            }

            if(initMap) {

                CameraPosition cameraPosition = new CameraPosition.Builder()
                        .target(new LatLng(intervention.getLatitude(), intervention.getLongitude())).zoom(9).build();

                googleMap.moveCamera(CameraUpdateFactory
                        .newCameraPosition(cameraPosition));
                initMap = false;

                googleMap.setOnCameraChangeListener(new GoogleMap.OnCameraChangeListener() {
                    @Override
                    public void onCameraChange(CameraPosition arg0) {
                        mProgressView.setVisibility(View.INVISIBLE);

                        CameraUpdate cuf;

                        if (resources.size()>1) {
                            cuf = CameraUpdateFactory.newLatLngBounds(bounds.build(), 100);
                        }
                        else if (resources.size()==1){
                            cuf = CameraUpdateFactory.newLatLngZoom(new LatLng(resources.get(0).getLatitude(), resources.get(0).getLongitude()),16);
                        }
                        else {
                            cuf = CameraUpdateFactory.newLatLngZoom(new LatLng(intervention.getLatitude(), intervention.getLongitude()),16);
                        }

                        googleMap.animateCamera(cuf, new GoogleMap.CancelableCallback() {
                            @Override
                            public void onFinish() {
                                googleMap.getUiSettings().setScrollGesturesEnabled(true);
                            }
                            @Override
                            public void onCancel() {
                            }
                        });

                        googleMap.setOnCameraChangeListener(null);
                    }
                });
            }

            googleMap.setOnMarkerClickListener(new GoogleMap.OnMarkerClickListener() {
                @Override
                public boolean onMarkerClick(Marker marker) {
                    Resource resource = labelsResourcesHashMap.get(marker.getTitle());
                    if (resource != null) {
                        ((AgentInterventionActivity) getActivity()).showManageResourceDialog(resource);
                    }
                    return false;
                }
            });

            googleMap.setOnMarkerDragListener(new GoogleMap.OnMarkerDragListener() {

                FrameLayout frameLayout = (FrameLayout) getActivity().findViewById(R.id.fragment_container);

                @Override
                public void onMarkerDragStart(Marker marker) {
                    Bitmap bitmap = labelsBitmapHashMap.get(marker.getTitle());
                    imageViewToDrag = new ImageView(((AgentInterventionActivity) getActivity()).getContext());
                    imageViewToDrag.setImageBitmap(bitmap);
                    ((AgentInterventionActivity) getActivity()).stopSynchro();
                }

                @Override
                public void onMarkerDrag(Marker marker) {
                    Resource resource = labelsResourcesHashMap.get(marker.getTitle());

                    if (googleMap.getProjection().toScreenLocation(marker.getPosition()).x<0
                            && !markerOnLeftFragment
                            && !ResourceCategory.dragabledata.equals(resource.getResourceCategory())){
                        frameLayout.addView(imageViewToDrag);
                        markerOnLeftFragment = true;
                        marker.setVisible(false);
                    }
                    else if(imageViewToDrag!=null
                            && googleMap.getProjection().toScreenLocation(marker.getPosition()).x>=0
                            && markerOnLeftFragment
                            && !ResourceCategory.dragabledata.equals(resource.getResourceCategory())){
                        frameLayout.removeView(imageViewToDrag);
                        markerOnLeftFragment = false;
                        marker.setVisible(true);
                        Log.i(TAG,"Marker on map");
                    }

                }

                @Override
                public void onMarkerDragEnd(Marker marker) {
                    Resource resource = labelsResourcesHashMap.get(marker.getTitle());

                    if (resource != null && null != getActivity()) {
                        if (!ResourceCategory.dragabledata.equals(resource.getResourceCategory()) && markerOnLeftFragment){
                            frameLayout.removeView(imageViewToDrag);
                            LatLng latLng = new LatLng(0,0);
                            ((AgentInterventionActivity) getActivity()).updateResourceOnDrop(resource,latLng,State.arrived);
                        }
                        else{
                            LatLng latLng = marker.getPosition();
                            ((AgentInterventionActivity) getActivity()).updateResourceOnDrop(resource,latLng,State.planned);
                        }
                    }
                    ((AgentInterventionActivity) getActivity()).startSynchro();

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
        if (data.getMarkerType().equals(MarkerType.waterSource)){
            bmp = BitmapFactory.decodeResource(getResources(), R.drawable.watersource);
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
                    Log.i(TAG,"Etat de la resource : "+resource.getState());
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
                labelsBitmapHashMap.put(resource.getLabel(), mBitmap);
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
    public Map<String, Bitmap> getLabelsBitmapHashMap() {
        return labelsBitmapHashMap;
    }
}