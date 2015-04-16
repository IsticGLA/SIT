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
import android.content.ClipData;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ListView;

import java.util.ArrayList;
import java.util.List;

import entity.Resource;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.adapter.RequestAdapter;
import istic.gla.groupeb.flerjeco.adapter.ResourceIconAdapter;
import istic.gla.groupeb.flerjeco.adapter.ResourceImageAdapter;
import istic.gla.groupeb.flerjeco.icons.Vehicle;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;
import util.ResourceCategory;
import util.State;

public class AgentInterventionResourcesFragment extends Fragment implements ISynchTool {
    OnResourceSelectedListener mCallback;

    private ListView listViewResources;
    private ListView listViewRequests;
    private ListView listViewAdditionalResources;
    private List<Resource> resourceList = new ArrayList<>();
    private List<Bitmap> iconBitmapResourceList = new ArrayList<>();
    private List<Resource> requestList = new ArrayList<>();
    private List<Resource> additionalResourceList = new ArrayList<>();

    @Override
    public void refresh() {
        // TODO
        // clear lists
        clearData();
        // fill lists
        fillResourcesAndRequests();
    }

    // The container Activity must implement this interface so the frag can deliver messages
    public interface OnResourceSelectedListener {
        /** Called by HeadlinesFragment when a list item is selected */
        public void onResourceSelected(int position);
    }

    public void clearData(){
        resourceList.clear();
        iconResourceList.clear();
        requestList.clear();
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState){
        super.onCreate(savedInstanceState);

        View v = inflater.inflate(R.layout.resource_view, container,
                false);

        fillAdditionalResources();

        listViewAdditionalResources = (ListView) v.findViewById(R.id.listViewAditionalResources);
        listViewResources = (ListView) v.findViewById(R.id.listViewAgentResources);
        listViewRequests = (ListView) v.findViewById(R.id.listViewAgentRequests);

        fillResourcesAndRequests();

        listViewAdditionalResources.setAdapter(new ResourceIconAdapter(getActivity(), R.layout.item_resource_agent_only_icon, additionalResourceList));
        listViewResources.setAdapter(new ResourceImageAdapter(getActivity(), R.layout.item_resource_agent_only_icon, resourceList, iconBitmapResourceList));
        listViewRequests.setAdapter(new RequestAdapter(getActivity(), R.layout.item_request_agent, requestList));

        listViewResources.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> adapterView, View view, int position, long id) {
                mCallback.onResourceSelected(position);
                listViewResources.setItemChecked(position,true);
            }
        });

        listViewResources.setOnItemLongClickListener(new AdapterView.OnItemLongClickListener() {
            @Override
            public boolean onItemLongClick(AdapterView<?> adapterView, View view, int position, long l) {
                mCallback.onResourceSelected(position);
                ClipData data = ClipData.newPlainText("", "");
                View.DragShadowBuilder shadowBuilder = new View.DragShadowBuilder(view);
                view.startDrag(data, shadowBuilder, view, 0);
                view.setVisibility(View.INVISIBLE);
                return true;
            }
        });

        listViewAdditionalResources.setOnItemLongClickListener(new AdapterView.OnItemLongClickListener() {
            @Override
            public boolean onItemLongClick(AdapterView<?> parent, View view, int position, long id) {
                mCallback.onResourceSelected(position);
                ClipData data = ClipData.newPlainText("", "");
                View.DragShadowBuilder shadowBuilder = new View.DragShadowBuilder(view);
                view.startDrag(data, shadowBuilder, view, 0);
                view.setVisibility(View.VISIBLE);
                return true;
            }
        });

        return v;
    }

    private void fillAdditionalResources(){

        Resource incident = new Resource("incident", State.validated, 0, 0);
        incident.setResourceCategory(ResourceCategory.dragabledata);
        Resource danger = new Resource("danger", State.validated, 0, 0);
        danger.setResourceCategory(ResourceCategory.dragabledata);
        Resource sensitive = new Resource("sensitive", State.validated, 0, 0);
        sensitive.setResourceCategory(ResourceCategory.dragabledata);

        additionalResourceList.add(incident);
        additionalResourceList.add(danger);
        additionalResourceList.add(sensitive);
    }

    private void fillResourcesAndRequests(){

        AgentInterventionActivity interventionActivity = (AgentInterventionActivity) getActivity();
        for (Resource resource : interventionActivity.intervention.getResources()){
            State resourceState = resource.getState();
            if (State.validated.equals(resourceState)){
                resourceList.add(resource);
                Vehicle mVehicle = new Vehicle(resource.getLabel(), resource.getResourceRole(), resource.getState());
                int width = mVehicle.getRect().width();
                int height = mVehicle.getRect().height()+mVehicle.getRect2().height()+10;
                Bitmap mBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
                Canvas mCanvas = new Canvas(mBitmap);
                mVehicle.drawIcon(mCanvas);
                iconBitmapResourceList.add(mBitmap);
                Log.i("RESOURCELIST",resource.getLabel());
            }else if (State.waiting.equals(resourceState) || State.refused.equals(resourceState) ){
                requestList.add(resource);
            }
        }

    }

    @Override
    public void onStart() {
        super.onStart();

        // When in two-pane layout, set the listview to highlight the selected list item
        // (We do this during onStart because at the point the listview is available.)
        if (getFragmentManager().findFragmentById(R.id.map_fragment) != null) {
            listViewResources.setChoiceMode(ListView.CHOICE_MODE_SINGLE);

            if(resourceList.size()>0){
                mCallback.onResourceSelected(0);
                listViewResources.setItemChecked(0,true);
            }
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

    public List<Resource> getResourceList() {
        return resourceList;
    }

    public List<Resource> getAdditionalResourceList() {
        return additionalResourceList;
    }
}