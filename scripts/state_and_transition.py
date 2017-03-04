#!/usr/bin/env python
#
# Copyright 2017 Robot Garden, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# States and Transitions 

from auto_number import AutoNumber


class STATE(AutoNumber):
    Start = ()
    Following_waypoint = ()
    Avoiding_obstacle = ()
    Driving_toward_cone = ()
    Driving_away_from_cone = ()
    Success = ()
    Failure = ()
    End = ()


class TRANSITION(AutoNumber):
    obstacle_seen = ()
    near_cone = ()
    obstacle_cleared = ()
    touched_cone = ()
    passed_cone = ()
    segment_timeout = ()
    touched_last_cone = ()
    passed_last_cone = ()
    course_timeout = ()



