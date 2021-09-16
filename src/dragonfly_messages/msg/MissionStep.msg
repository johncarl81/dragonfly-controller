int8 TYPE_START=0
int8 TYPE_TAKEOFF=1
int8 TYPE_SLEEP=2
int8 TYPE_LAND=3
int8 TYPE_GOTO_WAYPOINT=4
int8 TYPE_SEMAPHORE=5
int8 TYPE_RTL=6
int8 TYPE_DDSA=7
int8 TYPE_LAWNMOWER=8
int8 TYPE_NAVIGATION=9
int8 TYPE_FLOCK=10
int8 TYPE_GRADIENT=11
int8 TYPE_CURTAIN=12

int8 msg_type

Takeoff takeoff_step
Sleep sleep_step
GotoWaypoint goto_step
Semaphore semaphore_step
DDSAMission ddsa_step
LawnmowerMission lawnmower_step
NavigationMission navigation_step
FlockMission flock_step
CO2GradientFollowing gradient_step
Curtain curtain_step