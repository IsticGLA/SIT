package istic.gla.groupb.nivimoju.init;

import istic.gla.goupb.nivimoju.drone.engine.DronePersistJob;
import istic.gla.goupb.nivimoju.drone.engine.DronePositionRefresherJob;
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
        JobDetail jobPosition = JobBuilder.newJob(DronePositionRefresherJob.class)
                .withIdentity("DronePositionRefresherJob", "group1").build();
        JobDetail jobPersist = JobBuilder.newJob(DronePersistJob.class)
                .withIdentity("DronePersistJob", "group1").build();

        Trigger triggerPosition = TriggerBuilder
                .newTrigger()
                .withIdentity("DronePositionRefresherTrigger", "group1")
                .withSchedule(
                        SimpleScheduleBuilder.simpleSchedule()
                                .withIntervalInMilliseconds(200)
                                .repeatForever())
                .build();

        Trigger triggerPersist = TriggerBuilder
                .newTrigger()
                .withIdentity("DronePersistTrigger", "group1")
                .withSchedule(
                        SimpleScheduleBuilder.simpleSchedule()
                                .withIntervalInSeconds(10)
                                .repeatForever())
                .build();

        try{
            Scheduler scheduler = new StdSchedulerFactory().getScheduler();
            scheduler.start();
            scheduler.scheduleJob(jobPosition, triggerPosition);
            scheduler.scheduleJob(jobPersist, triggerPersist);
        } catch (SchedulerException e){
            logger.error(e);
        }

    }

    @Override
    public void contextDestroyed(ServletContextEvent servletContextEvent) {

    }
}
