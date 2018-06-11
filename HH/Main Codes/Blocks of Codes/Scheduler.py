from apscheduler.schedulers.blocking import BlockingScheduler
import CV


sched = BlockingScheduler()


# Scheduled to run every weekday from 9AM till program shuts itself
@sched.scheduled_job('cron', day_of_week='mon-fri', hour='9', minute='00')
def scheduled_job():
    CV.job()


sched.start()
