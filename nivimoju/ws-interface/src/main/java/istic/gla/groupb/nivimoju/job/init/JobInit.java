package istic.gla.groupb.nivimoju.job.init;

import istic.gla.groupb.nivimoju.job.DronePersistJob;
import istic.gla.groupb.nivimoju.job.DronePositionRefresherJob;
import org.apache.log4j.Logger;
import org.quartz.*;
import org.quartz.impl.StdSchedulerFactory;

import javax.servlet.ServletContextEvent;
import javax.servlet.ServletContextListener;

/**
 * Initialise le job qui rafraichit les positions de drones en lisant la simulation
 */
public class JobInit implements ServletContextListener {
    Logger logger = Logger.getLogger(JobInit.class);

    @Override
    public void contextInitialized(ServletContextEvent servletContextEvent) {
        logger.info("initializing context");
        JobDetail jobDronePosition = JobBuilder.newJob(DronePositionRefresherJob.class)
                .withIdentity("DronePositionRefresherJob", "group1").build();
        JobDetail jobDronePersist = JobBuilder.newJob(DronePersistJob.class)
                .withIdentity("DronePersistJob", "group1").build();
        JobDetail jobInterventionPersist = JobBuilder.newJob(DronePersistJob.class)
                .withIdentity("InterventionPersistJob", "group1").build();

        Trigger triggerDronePosition = TriggerBuilder
                .newTrigger()
                .withIdentity("DronePositionRefresherTrigger", "group1")
                .withSchedule(
                        SimpleScheduleBuilder.simpleSchedule()
                                .withIntervalInMilliseconds(200)
                                .repeatForever())
                .build();

        Trigger triggerDronePersist = TriggerBuilder
                .newTrigger()
                .withIdentity("DronePersistTrigger", "group1")
                .withSchedule(
                        SimpleScheduleBuilder.simpleSchedule()
                                .withIntervalInSeconds(10)
                                .repeatForever())
                .build();

        Trigger triggerInterventionPersist = TriggerBuilder
                .newTrigger()
                .withIdentity("InterventionPersistTrigger", "group1")
                .withSchedule(
                        SimpleScheduleBuilder.simpleSchedule()
                                .withIntervalInSeconds(10)
                                .repeatForever())
                .build();

        try{
            Scheduler scheduler = new StdSchedulerFactory().getScheduler();
            scheduler.start();
            scheduler.scheduleJob(jobDronePosition, triggerDronePosition);
            scheduler.scheduleJob(jobDronePersist, triggerDronePersist);
            scheduler.scheduleJob(jobInterventionPersist, triggerInterventionPersist);
        } catch (SchedulerException e){
            logger.error(e);
        }

    }

    @Override
    public void contextDestroyed(ServletContextEvent servletContextEvent) {

    }
}
