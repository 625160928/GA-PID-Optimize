import math
dx=10
dy= 0
waypoint_theta=math.pi/2
# waypoint_theta=0


dx=-1.1840728076431581
dy=1.4280087447644334
waypoint_theta=219.48846169956605/180*math.pi#-1.8550484650487755


srx = (dx) * math.cos(waypoint_theta) + (dy) * math.sin(waypoint_theta)
sry = (dy) * math.cos(waypoint_theta) - (dx) * math.sin(waypoint_theta)

nrx = dx* math.cos(waypoint_theta)- dy* math.sin(waypoint_theta)
nry = dx* math.sin(waypoint_theta) + dy* math.cos(waypoint_theta)

print(sry,'========',srx,sry)
# print(nrx,nry)