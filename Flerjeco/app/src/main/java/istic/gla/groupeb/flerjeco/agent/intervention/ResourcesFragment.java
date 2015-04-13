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

import android.app.Activity;
import android.os.Build;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ListView;

import java.util.ArrayList;
import java.util.List;

import entity.Resource;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.adapter.IconViewAdapter;
import istic.gla.groupeb.flerjeco.agent.intervention.SecondActivity;
import istic.gla.groupeb.flerjeco.icons.Vehicle;
import istic.gla.groupeb.flerjeco.view.IconView;
import util.State;

public class ResourcesFragment extends Fragment {
    OnResourceSelectedListener mCallback;

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

        View v = inflater.inflate(R.layout.resource_view, container,
                false);

        listViewResources = (ListView) v.findViewById(R.id.listViewResources);
        listViewRequests = (ListView) v.findViewById(R.id.listViewRequests);

        // We need to use a different list item layout for devices older than Honeycomb
        int layout = Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB ?
                android.R.layout.simple_list_item_activated_1 : android.R.layout.simple_list_item_1;


        List<String> labelsResources = new ArrayList<>();
        List<String> labelsRequests = new ArrayList<>();

        List<Resource> resourceList = new ArrayList<>();
        List<Resource> requestList = new ArrayList<>();

        SecondActivity secondActivity = (SecondActivity) getActivity();
        for (Resource resource : secondActivity.intervention.getResources()){
            State resourceState = resource.getState();


            if (State.active.equals(resourceState) || State.planned.equals(resourceState)){
                labelsResources.add(resource.getLabel());
                resourceList.add(resource);
            }else{
                labelsRequests.add(resource.getLabel());
                requestList.add(resource);
            }
        }

        //listViewResources.setAdapter(new ArrayAdapter<String>(getActivity(), layout, labelsResources));
        //listViewRequests.setAdapter(new ArrayAdapter<String>(getActivity(), layout, labelsRequests));

        listViewResources.setAdapter(new ResourceAdapter(getActivity(), R.layout.item_request, resourceList));
        listViewRequests.setAdapter(new ResourceAdapter(getActivity(), R.layout.item_request, requestList));

        /*
        List<IconView> iconViewList = new ArrayList<>();
        List<Vehicle> mVehicleList = new ArrayList<>();
        mVehicleList.add(new Vehicle("VSAP"));
        mVehicleList.add(new Vehicle("VSAP"));
        mVehicleList.add(new Vehicle("VSAP"));
        for(Vehicle vehicle : mVehicleList){
            vehicle.changeFunction(Vehicle.Function.Commands);
            iconViewList.add(new IconView(this.getActivity(), vehicle));
        }

        //listViewResources.setAdapter(new ArrayAdapter<String>(getActivity(), layout, labelsResources));
        listViewResources.setAdapter(new IconViewAdapter(this.getActivity(), R.layout.list_row, iconViewList));
        listViewRequests.setAdapter(new ArrayAdapter<String>(getActivity(), layout, labelsRequests));

        */

        listViewResources.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> adapterView, View view, int position, long id) {
                mCallback.onResourceSelected(position);
                listViewResources.setItemChecked(position,true);
            }
        });

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

    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);

        // This makes sure that the container activity has implemented
        // the callback interface. If not, it throws an exception.
        try {
            mCallback = (OnResourceSelectedListener) activity;
        } catch (ClassCastException e) {
            throw new ClassCastException(activity.toString()
                    + " must implement OnHeadlineSelectedListener");
        }
    }
}