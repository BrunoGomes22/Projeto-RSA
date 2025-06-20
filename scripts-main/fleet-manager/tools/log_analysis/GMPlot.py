# Based on https://pypi.org/project/gmplot/ 
# and https://www.geeksforgeeks.org/python-plotting-google-map-using-gmplot-package/
import gmplot

# Create the map plotter:
#apikey = 'AIzaSyB3vxTmpJ97RUO4RaUo4oc8w0vnhl_snNk' # (your API key here)

apikey = ''
coorList = {}

# Initiate map with key
def initMap(key):        
    global apikey      
    global coorList

    apikey = key
    coorList = {}

# Add new coordinate
def addCoordinate(droneID,lat,lon):
    global coorList

    '''
    Dictionary Structure
    { 
        droneID : {{latList = []}, {lonList = []}},
        droneID : {{latList = []}, {lonList = []}}
    }
    '''

    # Check if the droneID is a new one
    if droneID not in coorList:
        coorList[droneID] = {'latList' : [], 'lonList' : []}

    # Retrieve the value for this drone
    tmpList = coorList.get(droneID)
    
    # Retrieve the current lists for this drone    
    tmpLatList = tmpList.get('latList')
    tmpLonList = tmpList.get('lonList')

    # Add the new coordinates
    tmpLatList.append(lat)
    tmpLonList.append(lon)

    # Update the lists

    tmpList['latList'] = tmpLatList
    tmpList['lonList'] = tmpLonList
    coorList[droneID] = tmpList   

def buildMap():
    global coorList
    global apikey

    gmap = None

    for droneID in coorList.keys():
        print("droneID = ",droneID)
        # Retrieve the value for this drone
        tmpList = coorList.get(droneID)
        
        # Retrieve the current lists for this drone    
        tmpLatList = tmpList.get('latList')
        tmpLonList = tmpList.get('lonList')  

        # Initialize map
        if gmap == None:
            gmap = gmplot.GoogleMapPlotter(tmpLatList[0], tmpLonList[0], 19, apikey=apikey) 

        # Draw Start and End markers and plot
        gmap.marker(tmpLatList[0], tmpLonList[0], color='#FF0000', title="Starting point for {}".format(droneID), label="Start {}".format(droneID))
        gmap.marker(tmpLatList[len(tmpLatList) - 1], tmpLonList[len(tmpLonList) - 1], color='#FF0000', title="Ending point for {}".format(droneID), label="End {}".format((droneID)))

        gmap.plot(tmpLatList, tmpLonList, color='cornflowerblue', edge_width=3) 

    # Draw the map:
    print("Drawing map")
    gmap.draw('map.html')