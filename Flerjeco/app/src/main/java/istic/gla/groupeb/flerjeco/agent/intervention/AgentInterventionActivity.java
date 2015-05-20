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

import android.content.Context;
import android.content.Intent;
import android.graphics.Point;
import android.os.AsyncTask;
import android.os.Bundle;
import android.support.v4.app.DialogFragment;
import android.support.v4.app.FragmentTransaction;
import android.util.Log;
import android.view.DragEvent;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.LinearLayout;

import com.google.android.gms.maps.MapView;
import com.google.android.gms.maps.model.LatLng;

import java.util.List;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Resource;
import istic.gla.groupb.nivimoju.util.State;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.TabbedActivity;
import istic.gla.groupeb.flerjeco.login.LoginActivity;
import istic.gla.groupeb.flerjeco.springRest.GetInterventionTask;
import istic.gla.groupeb.flerjeco.springRest.IInterventionActivity;
import istic.gla.groupeb.flerjeco.springRest.SpringService;
import istic.gla.groupeb.flerjeco.synch.DisplaySynch;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;
import istic.gla.groupeb.flerjeco.synch.IntentWraper;

public class AgentInterventionActivity extends TabbedActivity
        implements AgentInterventionResourcesFragment.OnResourceSelectedListener, ISynchTool, IInterventionActivity {

    private static final String TAG = AgentInterventionActivity.class.getSimpleName();

    private AgentInterventionResourcesFragment firstFragment;
    private AgentInterventionMapFragment mapFragment;


    int mCurrentPosition = -1;

    @Override
    public void refresh(){
        if (null != intervention) {
            new GetInterventionTask(this, intervention.getId()).execute();
        }
    }
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        /*List<Resource> resourceList = new ArrayList<>();
        resourceList.add(new Resource("Resource0", State.validated, ResourceRole.fire, ResourceCategory.vehicule, 0, 0));
        resourceList.add(new Resource("Resource1", State.validated, ResourceRole.people, ResourceCategory.vehicule, 48.117749, -1.677297));
        resourceList.add(new Resource("Resource2", State.validated, ResourceRole.fire, ResourceCategory.vehicule, 48.127749, -1.657297));
        resourceList.add(new Resource("Resource3", State.validated, ResourceRole.commands, ResourceCategory.vehicule, 48.107749, -1.687297));
        resourceList.add(new Resource("Resource4", State.validated, ResourceRole.people, ResourceCategory.vehicule, 0, 0));
        resourceList.add(new Resource("Resource5", State.validated, ResourceRole.fire, ResourceCategory.vehicule, 0, 0));
        resourceList.add(new Resource("VSAP", State.validated, ResourceRole.people, ResourceCategory.vehicule, 0, 0));
        resourceList.add(new Resource("Resource7", State.validated, ResourceRole.people, ResourceCategory.vehicule, 0, 0));
        resourceList.add(new Resource("Resource8", State.validated, ResourceRole.commands, ResourceCategory.vehicule, 0, 0));
        reprivate TableJavaFragment frag3 = new TableJavaFragment();sourceList.add(new Resource("Resource9", State.validated, ResourceRole.fire, ResourceCategory.vehicule, 0, 0));
        intervention.setResources(resourceList);*/

        setContentView(R.layout.activity_intervention_agent);

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
        bundle.putLong(VehicleRequestDialog.INTERVENTION,intervention.getId());
        vehicleDialog.setArguments(bundle);
        vehicleDialog.show(getSupportFragmentManager(), "vehicle_dialog");
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

    public void resourceUpdated(Resource resource){
        List<Resource> resList = intervention.getResources();
        Resource temp = null;
        for (Resource res : resList){
            if (res.getIdRes() == resource.getIdRes()){
                temp = res;
            }
        }
        if (temp != null){
            resList.remove(temp);
            resList.add(resource);
        }
        UpdateIntervention updateIntervention = new UpdateIntervention();
        updateIntervention.execute(intervention);
        //refresh();
    }
    /**
     * Update lists of resources and map
     * @param intervention
     */
    public void updateIntervention(Intervention intervention) {
        Log.i(TAG, "updateIntervention");
        this.intervention = intervention;
        //update lists of resources and map
        if (firstFragment != null){
            firstFragment.refresh();
        }
        if (mapFragment != null){
            mapFragment.refresh();
        }
    }

    @Override
    public Context getContext() {
        return getApplicationContext();
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

                    int x = (int) event.getX();
                    int y = (int) event.getY();

                    Point point = new Point(x,y);

                    LatLng latLng = mapView.getMap().getProjection().fromScreenLocation(point);

                    Resource resource;

                    List<Resource> resourceList = firstFragment.getResourceList();
                    List<Resource> additionalResourceList = firstFragment.getAdditionalResourceList();

                    if (((LinearLayout)view).getChildAt(0) instanceof ImageView){
                        resource = resourceList.get(mCurrentPosition);
                        resource.setLatitude(latLng.latitude);
                        resource.setLongitude(latLng.longitude);
                        resource.setState(State.planned);
                        resourceList.remove(mCurrentPosition);
                        firstFragment.getIconBitmapResourceList().remove(mCurrentPosition);
                        firstFragment.getResourceImageAdapter().notifyDataSetChanged();
                    }else{
                        resource = additionalResourceList.get(mCurrentPosition);
                        resource.setLatitude(latLng.latitude);
                        resource.setLongitude(latLng.longitude);
                        resource.setState(State.active);
                        List<Resource> resources = intervention.getResources();
                        resources.add(resource);
                    }

                    UpdateIntervention mUpdateIntervention = new UpdateIntervention();
                    mUpdateIntervention.execute(intervention);

                    /*MarkerOptions marker = new MarkerOptions().position(latLng).title(label);
                    marker.draggable(true);

                    // Changing marker icons
                    mapFragment.drawMarker(marker, resource);
                    // adding marker
                    Marker markerAdded = googleMap.addMarker(marker);
                    mapFragment.getLabelsMarkersHashMap().put(label, markerAdded);
                    mapFragment.getLabelsResourcesHashMap().put(label, resource);*/
                    break;
                case DragEvent.ACTION_DRAG_ENDED:
                    view.setVisibility(View.VISIBLE);
                    break;
                default:
                    v.setVisibility(View.VISIBLE);
                    break;
            }
            return true;
        }
    }

    public void showManageResourceDialog(Resource resource){
        ChangeStateDialogFragment fragment = new ChangeStateDialogFragment();
        Bundle args = new Bundle();
        args.putSerializable("resource", resource);
        fragment.setArguments(args);
        fragment.show(getSupportFragmentManager(), "changeState_dialog");
    }

    public class UpdateIntervention extends AsyncTask<Intervention, Void, Intervention> {
        private SpringService service = new SpringService();

        @Override
        protected Intervention doInBackground(Intervention... intervention) {
            Log.i(TAG, "Start doInbackground updateIntervention");
            return service.updateIntervention(intervention[0]);
        }

        @Override
        protected void onPostExecute(Intervention intervention){
            Log.i(TAG, "Start onPostExecute updateIntervention");
            updateIntervention(intervention);
            Log.i(TAG, "End update intervention");
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        if(intervention != null) {
            DisplaySynch displaySynch = new DisplaySynch() {
                @Override
                public void ctrlDisplay() {
                    refresh();
                }
            };
            String url = "notify/intervention/" + intervention.getId();
            IntentWraper.startService(url, displaySynch);
        }
    }

    @Override
    protected void onStop() {
        super.onStop();
    }

    @Override
    protected void onPause() {
        super.onPause();
        IntentWraper.stopService();
    }
}