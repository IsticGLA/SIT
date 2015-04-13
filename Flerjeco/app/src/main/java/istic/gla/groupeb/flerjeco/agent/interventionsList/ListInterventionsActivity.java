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
package istic.gla.groupeb.flerjeco.agent.interventionsList;

import android.content.Intent;
import android.os.Bundle;
import android.support.v4.app.FragmentActivity;
import android.support.v4.app.FragmentTransaction;
import android.view.View;

import entity.Intervention;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.agent.intervention.SecondActivity;
import istic.gla.groupeb.flerjeco.springRest.SpringService;

public class ListInterventionsActivity extends FragmentActivity
        implements InterventionsNamesFragment.OnResourceSelectedListener {

    private static final String TAG = SpringService.class.getSimpleName();
    protected Intervention[] interventionTab;
    private int position=0;

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Bundle extras = getIntent().getExtras();
        if (extras != null) {
            interventionTab = (Intervention[]) extras.getSerializable("interventions");
        }

        setContentView(R.layout.activity_list_interventions);

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
            InterventionsNamesFragment firstFragment = new InterventionsNamesFragment();

            // In case this activity was started with special instructions from an Intent,
            // pass the Intent's extras to the fragment as arguments
            firstFragment.setArguments(getIntent().getExtras());

            // Add the fragment to the 'fragment_container' FrameLayout
            getSupportFragmentManager().beginTransaction()
                    .add(R.id.fragment_container, firstFragment).commit();
        }
    }

    public void onResourceSelected(int position) {

        MapListInterventionsFragment mapFragment = (MapListInterventionsFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);

        if (mapFragment != null) {
            // If article frag is available, we're in two-pane layout...

            //save the current position
            this.position = position;

            // Call a method in the ArticleFragment to update its content
            mapFragment.updateMapView(position);

        } else {
            // If the frag is not available, we're in the one-pane layout and must swap frags...

            // Create fragment and give it an argument for the selected article
            mapFragment = new MapListInterventionsFragment();
            Bundle args = new Bundle();
            args.putInt(MapListInterventionsFragment.ARG_POSITION, position);
            mapFragment.setArguments(args);
            FragmentTransaction transaction = getSupportFragmentManager().beginTransaction();

            // Replace whatever is in the fragment_container view with this fragment,
            // and add the transaction to the back stack so the user can navigate back
            transaction.replace(R.id.fragment_container, mapFragment);
            transaction.addToBackStack(null);

            // Commit the transaction
            transaction.commit();
        }
    }

    public Intervention[] getInterventions() {
        return interventionTab;
    }

    public void selectIntervention(View view) {
        Intent intent = new Intent(ListInterventionsActivity.this, SecondActivity.class);
        Bundle bundle = new Bundle();

        bundle.putSerializable("intervention", getInterventions()[position]);

        intent.putExtras(bundle);
        startActivity(intent);
    }
}