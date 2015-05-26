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

import android.content.ClipData;
import android.content.Context;
import android.content.Intent;
import android.graphics.Point;
import android.os.AsyncTask;
import android.os.Bundle;
import android.support.v4.app.DialogFragment;
import android.util.Log;
import android.view.DragEvent;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.FrameLayout;

import com.google.android.gms.maps.MapView;
import com.google.android.gms.maps.model.LatLng;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Resource;
import istic.gla.groupb.nivimoju.util.ResourceCategory;
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
        implements ISynchTool, IInterventionActivity {

    private static final String TAG = AgentInterventionActivity.class.getSimpleName();

    private AgentInterventionResourcesFragment firstFragment;
    private AgentInterventionMapFragment mapFragment;

    @Override
    public void refresh(){
        if (null != intervention) {
            new GetInterventionTask(this, intervention.getId()).execute();
        }
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

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        //opening transition animations
        overridePendingTransition(0, android.R.anim.fade_out);

        intervention = new Intervention();

        Bundle extras = getIntent().getExtras();

        if (extras != null){
            Log.i(TAG, "getExtras not null");
            intervention = (Intervention) extras.getSerializable("intervention");
        }

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


            mapFragment = (AgentInterventionMapFragment)
                    getSupportFragmentManager().findFragmentById(R.id.map_fragment);

            findViewById(R.id.fragment_container).setOnDragListener(new MyDragListener());
            findViewById(R.id.map_fragment).setOnDragListener(new MyDragListener());
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

                    ClipData clipData = event.getClipData();
                    ClipData.Item item = clipData.getItemAt(0);
                    Bundle extras = item.getIntent().getExtras();
                    if (extras!=null){
                        resource = (Resource) extras.getSerializable("resource");
                        int position = extras.getInt("position");
                        if (!ResourceCategory.dragabledata.equals(resource.getResourceCategory())){
                            firstFragment.getResourceList().remove(resource);
                            firstFragment.getIconBitmapResourceList().remove(position);
                            firstFragment.getResourceImageAdapter().notifyDataSetChanged();
                        }
                        else{
                            resource.setIdRes(-1);
                        }
                        updateResourceOnDrop(resource, latLng, State.planned);
                    }

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

    public void updateResource(Resource resource){
        UpdateResourceIntervention mUpdateResourceIntervention = new UpdateResourceIntervention(intervention.getId(),resource);
        mUpdateResourceIntervention.execute();
    }

    public void updateResourceOnDrop(Resource resource, LatLng latLng, State state){

        resource.setLatitude(latLng.latitude);
        resource.setLongitude(latLng.longitude);
        resource.setState(state);


        Log.i(TAG, "RESOURCE : ");
        Log.i(TAG, "id : "+resource.getIdRes());
        Log.i(TAG, "label : "+resource.getLabel());
        Log.i(TAG, "category : "+resource.getResourceCategory());

        updateResource(resource);
    }

    public void showManageResourceDialog(Resource resource){
        ChangeStateDialogFragment fragment = new ChangeStateDialogFragment();
        Bundle args = new Bundle();
        args.putSerializable("resource", resource);
        fragment.setArguments(args);
        fragment.show(getSupportFragmentManager(), "changeState_dialog");
    }

    public class UpdateResourceIntervention extends AsyncTask<Object[], Void, Intervention> {
        private SpringService service = new SpringService();
        private long interventionId;
        private Resource resource;

        public UpdateResourceIntervention(long interventionId, Resource resource){
            this.interventionId = interventionId;
            this.resource = resource;
        }

        @Override
        protected Intervention doInBackground(Object[]... params) {
            Log.i(TAG, "Start doInbackground UpdateResourceIntervention");
            return service.updateResourceIntervention(interventionId,resource);
        }

        @Override
        protected void onPostExecute(Intervention intervention){
            Log.i(TAG, "Start onPostExecute UpdateResourceIntervention");
            updateIntervention(intervention);
            Log.i(TAG, "End update intervention");
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        startSynchro();
    }

    @Override
    protected void onStop() {
        super.onStop();
        stopSynchro();
    }

    @Override
    protected void onPause() {
        super.onPause();
        //closing transition animations
        overridePendingTransition(R.anim.activity_open_scale,R.anim.activity_close_translate);
    }


    public void startSynchro(){
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

    public void stopSynchro(){
        IntentWraper.stopService();
    }

}