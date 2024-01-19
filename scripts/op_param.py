#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy

class OpStatus:

    def __init__(self) -> None:

        # starting up
        self._pause_flag            = False # flag True -> The command output will be 0m/s
        self._start_flag            = True  # if the vehicle is in the start-up sequence, driving from the buffer areas to the track
        self._has_ready             = False # if the vehicle has arrived the task assigning line for the first time
        self._is_yellow_left        = True  # possible use for lane changing, use to help if one side of the lane lines are missing
    
        # Intersection
        self._is_in_intersection    = False # if the robot is in the intersection
        self._request_inter_timer   = False # if ture, the robot will ask for perssmissions in the intersection
        self._has_released          = False # if ture, the robot has notified the manager that it has left, the intersection resources are released

        self._is_missing_line       = False # help increase search area, usaully after paused by ACC before an intersection

        # task list
        self._task_index            = 0 # this is the counter for how many task has this robot performed
        self._node_pointer          = 2 # point to the next node index
        self._task_list = []
        self._next_action           = -1 # incoming action, 0 thur, 1 left, 2 right
        self._request_task_timer    = False # if true, the robot will try get the task in the ros timer
        # set default value
    
    def loadNextAction(self):
        self._node_pointer += 1
        if self._node_pointer == len(self._task_list):
            # There is no more node to go
            self._next_action = -1 # no more next action
        else:
            self._setNode(node_type='last',node_value=self.this_node)
            self._setNode(node_type='this',node_value=self.next_node)
            self._setNode(node_type='next',node_value=self._task_list[self._node_pointer])
            self.setNextAction()

    def enterIntersection(self):
        self._is_in_intersection = True
    
    def loadTaskList(self,task_list:list):
        if len(task_list) < 3:
            rospy.loginfo('Invalid task list, no task loaded')
        else:
            self._setNode(node_type='last',node_value=task_list[0])
            self._setNode(node_type='this',node_value=task_list[1])
            self._setNode(node_type='next',node_value=task_list[2])
            self._task_list = task_list
    
    def setNextAction(self) -> None:
        self._next_action = local_mapper(self.last_node,self.this_node,self.next_node)

    def _setNode(self,node_type:str,node_value:int):
        
        if node_type == 'last':
            self.last_node = node_value
        elif node_type == 'this':
            self.this_node = node_value
        elif node_type == 'next':
            self.next_node = node_value
        else:
            rospy.loginfo("Invalid Node Type, No Value Set")