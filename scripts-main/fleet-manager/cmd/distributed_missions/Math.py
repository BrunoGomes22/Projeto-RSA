import geopy.distance
import math
from time import sleep

def chaseCalc(originLat,originLon,targetLat,targetLon,targetAlt,radius):
    # Check if distance is inferior to 50 meters
    coord1 = (originLat, originLon)
    coord2 = (targetLat,targetLon)

    print("distance =",calcDistance(coord1, coord2))
    if calcDistance(coord1, coord2) < 50:
        return None, None, None
    
    # Find the angle between the target coordinate and my currrent location
    radians = math.atan2(originLat-targetLat, originLon-targetLon)

    latCorrection = radius * math.cos(radians)
    lonCorrection = radius * math.sin(radians) 
    
    targetLatNew = targetLat + lonCorrection
    targetLonNew = targetLon + latCorrection


    return targetLatNew, targetLonNew, targetAlt

def requiresRelay(GS_coords,target_coords,target_dist):
    takeoff_time = 10
    distance = calcDistance(GS_coords,target_coords)
    if distance > target_dist:
        return True
    else:
        return False

# Calculates the distance between 2 coordinates
def calcDistance(coord1, coord2):
    return geopy.distance.distance(coord1, coord2).km * 1000

def relayCalc(GS_coords,target_info,target_dist,tolerance):   
    # Find the angle between the target coordinate and the Ground Station
    radians = math.atan2(GS_coords[0]-target_info[0],GS_coords[1]-target_info[1])

    min_distance = target_dist - tolerance
    max_distance = target_dist + tolerance

    # Base radius used (~ 1000 meters)
    radius = 0.01

    ''' This will look for a coordinate within the target_dist
     (taking into account tolerance)
    Since we can't convert directly latitude/longitude to meters
    we have to calculate several times until we find a proper value
    '''
    
    while True:
        ''' We make the calculations thinking of a 2D coordinate axes
        where X grows to the right and Y grows upwards, so, in this case:
        X => Longitude
        Y => Latitude'''        

        lonCorrection = radius * math.cos(radians)
        latCorrection = radius * math.sin(radians)      

        targetLatNew = target_info[0] + latCorrection 
        targetLonNew = target_info[1] + lonCorrection 

        # Calculate distance between these 2 coordinates
        distance = calcDistance((target_info[0],target_info[1]),(targetLatNew,targetLonNew))
        if distance >= min_distance and distance <= max_distance:
            break

        # Decrease the radius 
        radius = radius * 0.975

        # Failsafe in case the calculations fail to find a proper coordinate
        # (should never happen if the tolerance is not too small)
        if distance < min_distance:
            return None, None, None, None
        
    return targetLatNew, targetLonNew, target_info[2], target_info[3] * 1.5
