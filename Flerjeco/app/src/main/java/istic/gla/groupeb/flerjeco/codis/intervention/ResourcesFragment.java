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

import java.util.ArrayList;
import java.util.List;

import entity.Intervention;
import entity.Resource;
import istic.gla.groupeb.flerjeco.R;
import util.State;

public class ResourcesFragment extends Fragment {

    private ListView listViewResources;
    private ListView listViewRequests;

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

        InterventionActivity interventionActivity = (InterventionActivity) getActivity();
        updateResources(interventionActivity.getInterventions()[0]);

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

        for (Resource resource : intervention.getResources()){
            State resourceState = resource.getState();
            if (State.active.equals(resourceState) || State.planned.equals(resourceState) || State.validated.equals(resourceState)){
                labelsResources.add(resource.getLabel());
            }else{
                requests.add(resource);
            }
        }

        listViewResources.setAdapter(new ArrayAdapter(getActivity(), layout, labelsResources));
        listViewRequests.setAdapter(new ResourceAdapter(getActivity(), layout, requests, intervention.getId(), this));
    }
}