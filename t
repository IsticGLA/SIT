[1mdiff --git a/Flerjeco/app/build.gradle b/Flerjeco/app/build.gradle[m
[1mindex fa884a0..710a28c 100644[m
[1m--- a/Flerjeco/app/build.gradle[m
[1m+++ b/Flerjeco/app/build.gradle[m
[36m@@ -33,5 +33,9 @@[m [mdependencies {[m
     compile 'org.springframework.android:spring-android-rest-template:1.0.1.RELEASE'[m
     compile 'com.fasterxml.jackson.core:jackson-databind:2.3.2'[m
     compile 'org.apache.commons:commons-collections4:4.0'[m
[32m+[m[32m    compile "com.android.support:support-v4:+"[m
[32m+[m[32m    compile 'com.squareup.picasso:picasso:2.3.2'[m
[32m+[m[32m    compile 'com.nineoldandroids:library:2.4.0'[m
[32m+[m[32m    compile 'com.daimajia.slider:library:1.1.5@aar'[m
     compile files('libs/nivimoju.jar')[m
 }[m
[1mdiff --git a/Flerjeco/app/src/main/java/istic/gla/groupeb/flerjeco/agent/droneVisualisation/VisualisationActivity.java b/Flerjeco/app/src/main/java/istic/gla/groupeb/flerjeco/agent/droneVisualisation/VisualisationActivity.java[m
[1mindex 722696e..f0acdba 100644[m
[1m--- a/Flerjeco/app/src/main/java/istic/gla/groupeb/flerjeco/agent/droneVisualisation/VisualisationActivity.java[m
[1m+++ b/Flerjeco/app/src/main/java/istic/gla/groupeb/flerjeco/agent/droneVisualisation/VisualisationActivity.java[m
[36m@@ -38,7 +38,7 @@[m [mpublic class VisualisationActivity extends TabbedActivity implements ISynchTool,[m
             intervention = (Intervention) extras.getSerializable("intervention");[m
         }[m
 [m
[31m-        // Set the content view with the activity_plan_zone layout[m
[32m+[m[32m        // Set the content view with the activity_visualisation layout[m
         setContentView(R.layout.activity_visualisation);[m
 [m
         // Check whether the activity is using the layout version with[m
[1mdiff --git a/Flerjeco/app/src/main/java/istic/gla/groupeb/flerjeco/springRest/SpringService.java b/Flerjeco/app/src/main/java/istic/gla/groupeb/flerjeco/springRest/SpringService.java[m
[1mindex b12cb6c..4ba8359 100644[m
[1m--- a/Flerjeco/app/src/main/java/istic/gla/groupeb/flerjeco/springRest/SpringService.java[m
[1m+++ b/Flerjeco/app/src/main/java/istic/gla/groupeb/flerjeco/springRest/SpringService.java[m
[36m@@ -2,6 +2,8 @@[m [mpackage istic.gla.groupeb.flerjeco.springRest;[m
 [m
 import android.util.Log;[m
 [m
[32m+[m[32mimport com.google.android.gms.maps.model.LatLng;[m
[32m+[m
 import org.springframework.http.HttpMethod;[m
 import org.springframework.http.HttpStatus;[m
 import org.springframework.http.ResponseEntity;[m
[36m@@ -15,6 +17,7 @@[m [mimport org.springframework.web.client.RestTemplate;[m
 import java.sql.Timestamp;[m
 [m
 import istic.gla.groupb.nivimoju.entity.Drone;[m
[32m+[m[32mimport istic.gla.groupb.nivimoju.entity.Image;[m
 import istic.gla.groupb.nivimoju.entity.IncidentCode;[m
 import istic.gla.groupb.nivimoju.entity.Intervention;[m
 import istic.gla.groupb.nivimoju.entity.Path;[m
[36m@@ -318,6 +321,16 @@[m [mpublic class SpringService {[m
     }[m
 [m
     /**[m
[32m+[m[32m     * Gets all the image for the intervention and position[m
[32m+[m[32m     * @return An array of drones[m
[32m+[m[32m     */[m
[32m+[m[32m    public ResponseEntity<Image[]> getAllImageForInterventionAndPosition(Long interventionId, LatLng position) {[m
[32m+[m[32m        Log.v(TAG, "getAllImageForInterventionAndPosition start");[m
[32m+[m[32m        final String url = URL + String.format("image/all/%s/%d/%d", interventionId, position.latitude, position.longitude);[m
[32m+[m[32m        return restTemplate.getForEntity(url, Image[].class);[m
[32m+[m[32m    }[m
[32m+[m
[32m+[m[32m    /**[m
      * Change the state of a resource in parameters (waiting, planned, validated...)[m
      * @param params The id of the intervention, the resource label and the new state[m
      * @return The updated intervention[m
[1mdiff --git a/Flerjeco/app/src/main/res/layout/activity_visualisation.xml b/Flerjeco/app/src/main/res/layout/activity_visualisation.xml[m
[1mindex fa11d43..9610c69 100644[m
[1m--- a/Flerjeco/app/src/main/res/layout/activity_visualisation.xml[m
[1m+++ b/Flerjeco/app/src/main/res/layout/activity_visualisation.xml[m
[36m@@ -16,6 +16,7 @@[m [mlimitations under the License.[m
 -->[m
 [m
 <LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"[m
[32m+[m[32m    xmlns:tools="http://schemas.android.com/tools"[m
     android:orientation="horizontal"[m
     android:layout_width="match_parent"[m
     android:layout_height="match_parent">[m
[36m@@ -35,10 +36,18 @@[m [mlimitations under the License.[m
         android:layout_height="match_parent"[m
         android:background="@android:color/darker_gray"/>[m
 [m
[31m-    <fragment android:name="istic.gla.groupeb.flerjeco.agent.droneVisualisation.VisualisationMapFragment"[m
[32m+[m[32m    <refresher android:name="istic.gla.groupeb.flerjeco.agent.droneVisualisation.VisualisationMapFragment"[m
         android:id="@+id/map_fragment"[m
         android:layout_weight="2"[m
         android:layout_width="0dp"[m
         android:layout_height="match_parent" />[m
 [m
[32m+[m
[32m+[m[32m    <refresher android:name="istic.gla.groupeb.flerjeco.agent.droneVisualisation.ImageSliderFragment"[m
[32m+[m[32m        android:id="@+id/image_fragment"[m
[32m+[m[32m        android:layout_weight="2"[m
[32m+[m[32m        android:layout_width="0dp"[m
[32m+[m[32m        android:layout_height="match_parent"[m
[32m+[m[32m        tools:layout="@layout/fragment_image_slider" />[m
[32m+[m
 </LinearLayout>[m
\ No newline at end of file[m
[1mdiff --git a/Flerjeco/app/src/main/res/layout/fragment_tableau.xml b/Flerjeco/app/src/main/res/layout/fragment_tableau.xml[m
[1mindex a5158d8..c378550 100644[m
[1m--- a/Flerjeco/app/src/main/res/layout/fragment_tableau.xml[m
[1m+++ b/Flerjeco/app/src/main/res/layout/fragment_tableau.xml[m
[36m@@ -3,7 +3,7 @@[m
     android:layout_height="match_parent"[m
     tools:context="istic.gla.groupeb.flerjeco.agent.table.TableFragment">[m
 [m
[31m-    <!-- TODO: Update blank fragment layout -->[m
[32m+[m[32m    <!-- TODO: Update blank refresher layout -->[m
     <TableLayout[m
         android:id="@+id/containerTable"[m
         android:layout_width="match_parent"[m
[1mdiff --git a/Flerjeco/app/src/main/res/values-fr/strings.xml b/Flerjeco/app/src/main/res/values-fr/strings.xml[m
[1mindex dd1a24b..acec49e 100644[m
[1m--- a/Flerjeco/app/src/main/res/values-fr/strings.xml[m
[1m+++ b/Flerjeco/app/src/main/res/values-fr/strings.xml[m
[36m@@ -100,7 +100,7 @@[m
     <string name="fail_get_resource_type">Impossible de récupérer la ressource, veuillez réessayer</string>[m
     <string name="fail_post_interventions">Impossible d\'envoyer l\'intervention au serveur, veuillez réessayer</string>[m
     <string name="freeResource">Libérer la ressource</string>[m
[31m-    <string name="hello_blank_fragment">Bonjour fragment vide</string>[m
[32m+[m[32m    <string name="hello_blank_fragment">Bonjour refresher vide</string>[m
     <string name="hello_world">Bonjour le monde !</string>[m
     <string name="intervention">Intervention</string>[m
     <string name="title_activity_visualisation">Visualisation</string>[m
[1mdiff --git a/Flerjeco/app/src/main/res/values/strings.xml b/Flerjeco/app/src/main/res/values/strings.xml[m
[1mindex 4c4cc4b..33bb208 100644[m
[1m--- a/Flerjeco/app/src/main/res/values/strings.xml[m
[1m+++ b/Flerjeco/app/src/main/res/values/strings.xml[m
[36m@@ -118,7 +118,7 @@[m
     <string name="hello_world">Hello world!</string>[m
 [m
 <!-- TODO: Remove or change this placeholder text -->[m
[31m-    <string name="hello_blank_fragment">Hello blank fragment</string>[m
[32m+[m[32m    <string name="hello_blank_fragment">Hello blank refresher</string>[m
 [m
     <string name="table">Table</string>[m
     <string name="visualisation">Visualisation</string>[m
