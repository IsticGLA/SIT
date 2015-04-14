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

import android.os.Bundle;
import android.support.v4.app.DialogFragment;
import android.support.v4.app.FragmentActivity;
import android.support.v4.app.FragmentTransaction;
import android.view.View;

import java.util.ArrayList;
import java.util.List;

import entity.Intervention;
import entity.Resource;
import entity.StaticData;
import istic.gla.groupeb.flerjeco.R;
import util.State;

public class AgentInterventionActivity extends FragmentActivity
        implements AgentInterventionResourcesFragment.OnResourceSelectedListener {

    private static final String TAG = AgentInterventionActivity.class.getSimpleName();

    protected Intervention intervention;
    private StaticData[] staticDataTab;

    private AgentInterventionResourcesFragment firstFragment;

    int mCurrentPosition = -1;

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        /*Bundle extras = getIntent().getExtras();
        if (extras != null) {
            Object[] objects1 = (Object[]) extras.getSerializable("staticdatas");
            staticDataTab = new StaticData[objects1.length];
            for (int i=0; i<objects1.length; i++){
                staticDataTab[i] = (StaticData) objects1[i];
            }
        }*/

        intervention = new Intervention();
        intervention.setLatitude(48.117749);
        intervention.setLongitude(-1.677297);
        List<Resource> resourceList = new ArrayList<>();
        resourceList.add(new Resource("Resource0", State.validated, 0, 0));
        resourceList.add(new Resource("Resource1", State.active, 48.117749, -1.677297));
        resourceList.add(new Resource("Resource2", State.active, 48.127749, -1.657297));
        resourceList.add(new Resource("Resource3", State.planned, 48.107749, -1.687297));
        resourceList.add(new Resource("Resource4", State.validated, 0, 0));
        resourceList.add(new Resource("Resource5", State.waiting, 0, 0));
        resourceList.add(new Resource("VSAP", State.refused, 0, 0));
        resourceList.add(new Resource("Resource7", State.refused, 0, 0));
        resourceList.add(new Resource("Resource8", State.waiting, 0, 0));
        resourceList.add(new Resource("Resource9", State.waiting, 0, 0));

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
        }
    }

    public void onResourceSelected(int position) {

        AgentInterventionMapFragment mapFragment = (AgentInterventionMapFragment)
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
            args.putSerializable("staticdatas", staticDataTab);
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

}