/*
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package istic.gla.groupeb.flerjeco.agent.intervention;

import android.app.ActionBar;
import android.content.Intent;
import android.graphics.Point;
import android.os.AsyncTask;
import android.os.Bundle;
import android.support.v4.app.DialogFragment;
import android.support.v4.app.FragmentActivity;
import android.support.v4.app.FragmentTransaction;
import android.util.Log;
import android.view.DragEvent;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.LinearLayout;

import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.MapView;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MarkerOptions;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Set;

import entity.Intervention;
import entity.Resource;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.agent.planZone.PlanZoneActivity;
import istic.gla.groupeb.flerjeco.login.LoginActivity;
import istic.gla.groupeb.flerjeco.springRest.SpringService;
import istic.gla.groupeb.flerjeco.view.IconView;
import util.ResourceCategory;
import util.ResourceRole;
import util.State;

public class AgentInterventionActivity extends FragmentActivity
        implements AgentInterventionResourcesFragment.OnResourceSelectedListener, ActionBar.TabListener {

    private static final String TAG = AgentInterventionActivity.class.getSimpleName();

    protected Intervention intervention;

    private AgentInterventionResourcesFragment firstFragment;
    private AgentInterventionMapFragment mapFragment;

    int mCurrentPosition = -1;

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        intervention = new Intervention();

        /*Bundle extras = getIntent().getExtras();

        if (extras != null){
            intervention = (Intervention) extras.getSerializable("intervention");
        }*/


        intervention.setLatitude(48.117749);
        intervention.setLongitude(-1.677297);
        List<Resource> resourceList = new ArrayList<>();
        resourceList.add(new Resource("Resource0", State.validated, ResourceRole.fire, ResourceCategory.vehicule, 0, 0));
        resourceList.add(new Resource("Resource1", State.validated, ResourceRole.people, ResourceCategory.vehicule, 48.117749, -1.677297));
        resourceList.add(new Resource("Resource2", State.validated, ResourceRole.fire, ResourceCategory.vehicule, 48.127749, -1.657297));
        resourceList.add(new Resource("Resource3", State.validated, ResourceRole.commands, ResourceCategory.vehicule, 48.107749, -1.687297));
        resourceList.add(new Resource("Resource4", State.validated, ResourceRole.people, ResourceCategory.vehicule, 0, 0));
        resourceList.add(new Resource("Resource5", State.validated, ResourceRole.fire, ResourceCategory.vehicule, 0, 0));
        resourceList.add(new Resource("VSAP", State.validated, ResourceRole.people, ResourceCategory.vehicule, 0, 0));
        resourceList.add(new Resource("Resource7", State.validated, ResourceRole.people, ResourceCategory.vehicule, 0, 0));
        resourceList.add(new Resource("Resource8", State.validated, ResourceRole.commands, ResourceCategory.vehicule, 0, 0));
        resourceList.add(new Resource("Resource9", State.validated, ResourceRole.fire, ResourceCategory.vehicule, 0, 0));

        intervention.setResources(resourceList);

        setContentView(R.layout.activity_second);

        // Check whether the activity is using the layout version with
        // the fragment_container FrameLayout. If so, we must add the first fragment
        if (findViewById(R.id.fragment_container) != null) {

            // However, if we're being restored from a previous state,
            // then we don't need to do anything and should return or else
            // we could end up with overlapping fragments.
            if (savedInstanceState != null) {
                return;
            }

            // Create an instance of ExampleFragment
            firstFragment = new AgentInterventionResourcesFragment();

            // In case this activity was started with special instructions from an Intent,
            // pass the Intent's extras to the fragment as arguments
            firstFragment.setArguments(getIntent().getExtras());

            // Add the fragment to the 'fragment_container' FrameLayout
            getSupportFragmentManager().beginTransaction()
                    .add(R.id.fragment_container, firstFragment).commit();

            findViewById(R.id.fragment_container).setOnDragListener(new MyDragListener());
            findViewById(R.id.map_fragment).setOnDragListener(new MyDragListener());
        }

        final ActionBar actionBar = getActionBar();

        // Specify that tabs should be displayed in the action bar.
        actionBar.setNavigationMode(ActionBar.NAVIGATION_MODE_TABS);

        // Add 2 tabs, specifying the tab's text and TabListener
        ActionBar.Tab tab = actionBar.newTab();
        tab.setText("Intervention");
        tab.setTabListener(this);
        actionBar.addTab(tab);

        tab = actionBar.newTab();
        tab.setText("Drone");
        tab.setTabListener(this);
        actionBar.addTab(tab);
    }

    public void onResourceSelected(int position) {

        mapFragment = (AgentInterventionMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);



        if (mapFragment != null) {
            //save the current position
            mCurrentPosition = position;
            mapFragment.setPosition(position);

        } else {
            // If the frag is not available, we're in the one-pane layout and must swap frags...

            // Create fragment and give it an argument for the selected article
            AgentInterventionMapFragment newFragment = new AgentInterventionMapFragment();
            Bundle args = new Bundle();
            args.putInt(AgentInterventionMapFragment.ARG_POSITION, position);
            newFragment.setArguments(args);
            FragmentTransaction transaction = getSupportFragmentManager().beginTransaction();

            // Replace whatever is in the fragment_container view with this fragment,
            // and add the transaction to the back stack so the user can navigate back
            transaction.replace(R.id.fragment_container, newFragment);
            transaction.addToBackStack(null);

            // Commit the transaction
            transaction.commit();
        }
    }

    public void showDialogRequest(View view) {
        // Create the fragment and show it as a dialog.
        DialogFragment vehicleDialog = new VehicleRequestDialog();
        Bundle bundle = new Bundle();
        bundle.putLong(VehicleRequestDialog.INTERVENTION,10);
        vehicleDialog.setArguments(bundle);
        vehicleDialog.show(getSupportFragmentManager(), "vehicle_dialog");
    }

    public void validateResources(View v){
        Set<Resource> resourceSet = mapFragment.getResourcesPutOnMap();
        for (Resource resource : resourceSet){
            Log.i(getClass().getSimpleName(),resource.getLabel());
        }
    }

    public void cancelResources(View v){
        mapFragment.cancelResources();
    }

    // Action Menu for Logout
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu items for use in the action bar
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.menu_logout, menu);

        return super.onCreateOptionsMenu(menu);
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.menu_logout:
                Intent intent = new Intent(AgentInterventionActivity.this, LoginActivity.class);
                intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP | Intent.FLAG_ACTIVITY_NEW_TASK);
                startActivity(intent);
                finish();
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }
    }

    /**
     * Update lists of resources and map
     * @param intervention
     */
    public void updateIntervention(Intervention intervention) {
        //TODO update lists of resources and map

    }

    class MyDragListener implements View.OnDragListener {

        @Override
        public boolean onDrag(View v, DragEvent event) {

            View view = (View) event.getLocalState();

            switch (event.getAction()) {
                case DragEvent.ACTION_DRAG_STARTED:
                    // do nothing
                    break;
                case DragEvent.ACTION_DRAG_ENTERED:
                    break;
                case DragEvent.ACTION_DRAG_EXITED:
                    break;
                case DragEvent.ACTION_DROP:
                    if(!(v instanceof FrameLayout)){
                        view.setVisibility(View.VISIBLE);
                        return false;
                    }
                    if(!(((FrameLayout) v).getChildAt(0) instanceof MapView)){
                        view.setVisibility(View.VISIBLE);
                        return false;
                    }

                    MapView mapView = (MapView) ((FrameLayout) v).getChildAt(0);

                    GoogleMap googleMap = mapView.getMap();

                    int x = (int) event.getX();
                    int y = (int) event.getY();

                    Point point = new Point(x,y);

                    LatLng latLng = mapView.getMap().getProjection().fromScreenLocation(point);

                    IconView iconView = (IconView) ((LinearLayout)view).getChildAt(0);

                    Resource resource = iconView.getResource();

                    resource.setLatitude(latLng.latitude);
                    resource.setLongitude(latLng.longitude);

                    UpdateIntervention mUpdateIntervention = new UpdateIntervention();
                    mUpdateIntervention.execute(intervention.getId(), resource);

                    MarkerOptions marker = new MarkerOptions().position(latLng).title(resource.getLabel());
                    // Changing marker icon
                    mapFragment.drawMarker(marker, resource);
                    // adding marker
                    googleMap.addMarker(marker);
                    break;
                case DragEvent.ACTION_DRAG_ENDED:
                    if (!event.getResult()){
                        view.setVisibility(View.VISIBLE);
                    }
                    break;
                default:
                    if (!event.getResult()){
                        v.setVisibility(View.VISIBLE);
                    }
                    break;
            }
            return true;
        }
    }

    @Override
    public void onTabSelected(ActionBar.Tab tab, android.app.FragmentTransaction ft) {
        if(tab.getText().toString().equals("Drone")) {
            Intent intent = new Intent(AgentInterventionActivity.this, PlanZoneActivity.class);
            Bundle bundle = new Bundle();
            bundle.putSerializable("intervention", intervention);
            intent.putExtras(bundle);
            startActivity(intent);
            finish();
        }
    }

    @Override
    public void onTabUnselected(ActionBar.Tab tab, android.app.FragmentTransaction ft) {
        if(tab.getText().toString().equals("Intervention")) {
            finish();
        }
    }

    @Override
    public void onTabReselected(ActionBar.Tab tab, android.app.FragmentTransaction ft) {
    }

    private class UpdateIntervention extends AsyncTask<Object, Void, Intervention> {

        @Override
        protected Intervention doInBackground(Object... params) {
            return new SpringService().updateResourceIntervention(params);
        }

        @Override
        protected void onPostExecute(Intervention intervention){
            updateIntervention(intervention);
        }
    }
}