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
package istic.gla.groupeb.flerjeco.codis.intervention;


import android.content.Intent;
import android.os.Bundle;
import android.support.v4.app.DialogFragment;
import android.support.v4.app.FragmentActivity;
import android.support.v4.app.FragmentTransaction;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import java.util.Arrays;
import java.util.List;

import entity.Intervention;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.login.LoginActivity;
import istic.gla.groupeb.flerjeco.springRest.SpringService;

public class InterventionActivity extends FragmentActivity
        implements InterventionFragment.OnResourceSelectedListener {

    private static final String TAG = SpringService.class.getSimpleName();
    protected Intervention[] interventionTab;
    private int position=0;
    private InterventionFragment firstFragment;

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Bundle extras = getIntent().getExtras();
        if (extras != null) {
            Object[] objects = (Object[]) extras.getSerializable("interventions");
            interventionTab = new Intervention[objects.length];
            for(int i=0;i<objects.length;i++) {
                interventionTab[i] = (Intervention) objects[i];
                Log.d("IntervAct", interventionTab[i].getName() + " - " + interventionTab[i].getId());
            }
        }

        setContentView(R.layout.activity_list_interventions_codis);

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
            firstFragment = new InterventionFragment();

            // In case this activity was started with special instructions from an Intent,
            // pass the Intent's extras to the fragment as arguments
            firstFragment.setArguments(getIntent().getExtras());

            // Add the fragment to the 'fragment_container' FrameLayout
            getSupportFragmentManager().beginTransaction()
                    .add(R.id.fragment_container, firstFragment).commit();
        }
    }

    public void onResourceSelected(int position) {

        ResourcesFragment resourcesFragment = (ResourcesFragment)
                getSupportFragmentManager().findFragmentById(R.id.resources_fragment);

        if (resourcesFragment != null) {
            // If article frag is available, we're in two-pane layout...

            //save the current position
            this.position = position;

            // Call a method in the ArticleFragment to update its content
            resourcesFragment.updateResources(interventionTab[position]);

        } else {
            // If the frag is not available, we're in the one-pane layout and must swap frags...

            // Create fragment and give it an argument for the selected article
            resourcesFragment = new ResourcesFragment();
            Bundle args = new Bundle();
            args.putInt("position", position);
            resourcesFragment.setArguments(args);
            FragmentTransaction transaction = getSupportFragmentManager().beginTransaction();

            // Replace whatever is in the fragment_container view with this fragment,
            // and add the transaction to the back stack so the user can navigate back
            transaction.replace(R.id.fragment_container, resourcesFragment);
            transaction.addToBackStack(null);

            // Commit the transaction
            transaction.commit();
        }
    }

    public Intervention[] getInterventions() {
        return interventionTab;
    }

    public void addIntervention(Intervention intervention) {
        int oldLength = interventionTab.length;
        Intervention[] tmpIntervention = interventionTab;
        interventionTab = new Intervention[oldLength+1];
        for(int i=0;i<oldLength;i++) {
            interventionTab[i] = tmpIntervention[i];
        }
        interventionTab[oldLength] = intervention;
    }

    public void updateInterventions() {
        ((InterventionFragment) getSupportFragmentManager().getFragments().get(0)).updateList();
    }

    public void showDialogIntervention(View view) {
        // Create the fragment and show it as a dialog.
        DialogFragment newFragment = new InterventionDialogFragment();
        newFragment.show(getSupportFragmentManager(), "intervention_dialog");
    }

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
                Intent intent = new Intent(InterventionActivity.this, LoginActivity.class);
                startActivity(intent);
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }
    }
}