#define ROS

#include <TaskScheduler.h>

#define PERIODO 200
 
Scheduler runner;

void taskUSCallback1();
void taskUSCallback2();
void taskUSCallback3();
void taskUSCallback4();
void taskTOQUECallback();

Task taskUS1(PERIODO, TASK_FOREVER, &taskUSCallback1);
Task taskUS2(PERIODO, TASK_FOREVER, &taskUSCallback2);
Task taskUS3(PERIODO, TASK_FOREVER, &taskUSCallback3);
Task taskUS4(PERIODO, TASK_FOREVER, &taskUSCallback4);
Task taskTOQUE(PERIODO, TASK_FOREVER, &taskTOQUECallback);

void start_TASKS(){

	runner.addTask(taskUS1);
	runner.addTask(taskUS2);
	runner.addTask(taskUS3);
	runner.addTask(taskUS4);
	runner.addTask(taskTOQUE);

	taskUS1.enable();
	taskUS2.enable();
	taskUS3.enable();
	taskUS4.enable();
	taskTOQUE.enable();

}
