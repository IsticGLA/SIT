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

import android.os.Build;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.TextView;

import org.joda.time.format.DateTimeFormat;
import org.joda.time.format.DateTimeFormatter;

import java.util.ArrayList;
import java.util.List;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Resource;
import istic.gla.groupb.nivimoju.util.ResourceCategory;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;
import istic.gla.groupb.nivimoju.util.State;

public class ResourcesFragment extends Fragment implements ISynchTool {

    private ListView listViewResources;
    private ListView listViewRequests;
    private TextView titleTextView;

    // The container Activity must implement this interface so the frag can deliver messages
    public interface OnResourceSelectedListener {
        /** Called by HeadlinesFragment when a list item is selected */
        public void onResourceSelected(int position);
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState){
        super.onCreate(savedInstanceState);

        View v = inflater.inflate(R.layout.fragment_list_resources, container,
                false);

        listViewResources = (ListView) v.findViewById(R.id.listViewAgentResources);
        listViewRequests = (ListView) v.findViewById(R.id.listViewAgentRequests);
        titleTextView = (TextView) v.findViewById(R.id.codis_intervention_title);

        InterventionActivity interventionActivity = (InterventionActivity) getActivity();
        if(interventionActivity!=null) {
            if (interventionActivity.getInterventions() != null) {
                if (interventionActivity.getInterventions().length > 0) {
                    updateResources(interventionActivity.getInterventions()[0]);
                }
            }
        }

        return v;
    }

    @Override
    public void onStart() {
        super.onStart();

        // When in two-pane layout, set the listview to highlight the selected list item
        // (We do this during onStart because at the point the listview is available.)
        if (getFragmentManager().findFragmentById(R.id.map_fragment) != null) {
            listViewResources.setChoiceMode(ListView.CHOICE_MODE_SINGLE);
        }
    }

    public void updateResources(Intervention intervention) {
        List<String> labelsResources = new ArrayList<>();
        List<Resource> requests = new ArrayList<>();

        // We need to use a different list item layout for devices older than Honeycomb
        int layout = Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB ?
                android.R.layout.simple_list_item_activated_1 : android.R.layout.simple_list_item_1;

        ((InterventionActivity)getActivity()).updateIntervention(intervention);
        for (Resource resource : intervention.getResources()){
            State resourceState = resource.getState();
            if(resource.getResourceCategory().equals(ResourceCategory.vehicule)) {
                if (State.active.equals(resourceState) || State.planned.equals(resourceState) || State.validated.equals(resourceState) || State.arrived.equals(resourceState)) {
                    labelsResources.add(resource.getLabel());
                } else if (State.waiting.equals(resourceState)) {
                    requests.add(resource);
                }
            }
        }

        listViewResources.setAdapter(new ArrayAdapter(getActivity(), layout, labelsResources));
        listViewRequests.setAdapter(new ResourceAdapter(getActivity(), layout, requests, intervention.getId(), this));
        titleTextView.setVisibility(View.VISIBLE);
        DateTimeFormatter dtf = DateTimeFormat.forPattern("MM/dd/yyyy HH:mm:ss");
        String creationDate = dtf.print(intervention.getCreationDate());
        if(intervention.getAddress() != null){
            titleTextView.setText(
                    String.format(getString(R.string.codis_intervention_title),
                            intervention.getName(),
                            intervention.getAddress(),
                            creationDate));
        } else{
            titleTextView.setText(
                    String.format(getString(R.string.codis_intervention_title_without_address),
                            intervention.getName(),
                            creationDate));
        }

    }

    @Override
    public void refresh() {

    }
}