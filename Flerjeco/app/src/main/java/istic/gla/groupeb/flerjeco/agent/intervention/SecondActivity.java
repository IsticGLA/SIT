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
import istic.gla.groupeb.flerjeco.R;
import util.State;

public class SecondActivity extends FragmentActivity
        implements ResourcesFragment.OnResourceSelectedListener {

    protected Intervention intervention;

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        intervention = new Intervention();
        intervention.setLatitude(48.117749);
        intervention.setLongitude(-1.677297);
        List<Resource> resourceList = new ArrayList<>();
        resourceList.add(new Resource("Resource1", State.active, 48.117749, -1.677297));
        resourceList.add(new Resource("Resource2", State.active, 48.127749, -1.657297));
        resourceList.add(new Resource("Resource3", State.planned, 48.107749, -1.687297));
        resourceList.add(new Resource("Resource4", State.validated, 48.017749, -1.477297));
        resourceList.add(new Resource("Resource5", State.waiting, 48.147749, -1.677297));

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
            /*ResourcesFragment firstFragment = new ResourcesFragment();

            // In case this activity was started with special instructions from an Intent,
            // pass the Intent's extras to the fragment as arguments
            firstFragment.setArguments(getIntent().getExtras());

            // Add the fragment to the 'fragment_container' FrameLayout
            getSupportFragmentManager().beginTransaction()
                    .add(R.id.fragment_container, firstFragment).commit();*/
        }
    }

    public void onResourceSelected(int position) {

        MapFragment mapFragment = (MapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);

        if (mapFragment != null) {
            // If article frag is available, we're in two-pane layout...

            // Call a method in the ArticleFragment to update its content
            mapFragment.updateMapView(position);

        } else {
            // If the frag is not available, we're in the one-pane layout and must swap frags...

            // Create fragment and give it an argument for the selected article
            MapFragment newFragment = new MapFragment();
            Bundle args = new Bundle();
            args.putInt(MapFragment.ARG_POSITION, position);
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