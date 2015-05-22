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


import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.support.v4.app.DialogFragment;
import android.support.v4.app.FragmentTransaction;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.TabbedActivity;
import istic.gla.groupeb.flerjeco.login.LoginActivity;
import istic.gla.groupeb.flerjeco.springRest.GetAllInterventionsTask;
import istic.gla.groupeb.flerjeco.springRest.IInterventionsActivity;
import istic.gla.groupeb.flerjeco.synch.DisplaySynch;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;
import istic.gla.groupeb.flerjeco.synch.IntentWraper;

public class InterventionActivity extends TabbedActivity
        implements InterventionFragment.OnResourceSelectedListener, ISynchTool, IInterventionsActivity {

    private static final String TAG = InterventionActivity.class.getSimpleName();
    protected Intervention[] interventionTab;
    protected Intervention intervention;
    private int position=0;
    private InterventionFragment firstFragment;

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        //opening transition animations
        overridePendingTransition(0, android.R.anim.fade_out);

        DisplaySynch displaySynch = new DisplaySynch() {
            @Override
            public void ctrlDisplay() {
                refresh();
            }
        };
        String url = "notify/intervention";
        IntentWraper.startService(url, displaySynch);

        Bundle extras = getIntent().getExtras();
        if (extras != null) {
            Object[] objects = (Object[]) extras.getSerializable("interventions");
            if(objects != null) {
                interventionTab = new Intervention[objects.length];
                for (int i = 0; i < objects.length; i++) {
                    interventionTab[i] = (Intervention) objects[i];
                }
            }
            intervention = (Intervention) extras.getSerializable("intervention");
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

            //save tsuper.onStop();he current position
            this.position = position;
            intervention = interventionTab[position];

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

    public void updateIntervention(Intervention intervention) {
        if(intervention != null) {
            for (int i = 0; i < interventionTab.length; i++) {
                if (interventionTab[i].getId() == intervention.getId()) {
                    interventionTab[i].setResources(intervention.getResources());
                }
            }
        }
    }

    public void updateInterventions() {
        InterventionFragment fragment = (InterventionFragment)getSupportFragmentManager().getFragments().get(0);
        if(fragment != null) {
            fragment.updateList();
            fragment.listViewInterventions.setItemChecked(position, true);
            this.intervention = interventionTab[position];
        }
    }

    public void updateCurrentIntervention() {
        ResourcesFragment fragment = (ResourcesFragment) getSupportFragmentManager().getFragments().get(1);
        if(null != fragment)
            fragment.updateResources(interventionTab[position]);
    }

    public void showDialogIntervention(View view) {
        // Create the fragment and show it as a dialog.
        DialogFragment newFragment = new InterventionDialogFragment();
        newFragment.show(getSupportFragmentManager(), "intervention_dialog");
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
                Intent intent = new Intent(InterventionActivity.this, LoginActivity.class);
                intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP | Intent.FLAG_ACTIVITY_NEW_TASK);
                startActivity(intent);
                finish();
            default:
                return super.onOptionsItemSelected(item);
        }
    }

    @Override
    public void refresh() {
        new GetAllInterventionsTask(InterventionActivity.this).execute();
    }

    @Override
    public void updateInterventions(Intervention[] interventions) {
        if(interventions != null) {
            interventionTab = interventions;
            updateInterventions();
            updateCurrentIntervention();
        }
    }

    @Override
    public Context getContext() {
        return getApplicationContext();
    }

    @Override
    protected void onResume() {
        super.onResume();
        DisplaySynch displaySynch = new DisplaySynch() {
            @Override
            public void ctrlDisplay() {
                refresh();
            }
        };
        String url = "notify/intervention";
        IntentWraper.startService(url, displaySynch);
    }

    @Override
    protected void onPause() {
        super.onPause();
        IntentWraper.stopService();
    }

    @Override
    protected void onStop() {
        super.onStop();
        //closing transition animations
        overridePendingTransition(R.anim.activity_open_scale,R.anim.activity_close_translate);
        IntentWraper.stopService();
    }


}