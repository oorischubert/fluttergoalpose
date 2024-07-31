import requests
import json

class FlutterComms():
    def __init__(self,robot_id):
        self.flutterLink = "https://firestore.googleapis.com/v1/projects/astrom/databases/(default)/documents/astrom/{robot_id}"
    
    def getGoalPose(self):
        response = requests.get(self.flutterLink)
        return json.loads(response.text)
    
    def sendCurrentPose(self, pose):
        response = requests.post(self.flutterLink, json=pose)
        return response.text
    
    def sendGoalPose(self, pose):
        response = requests.post(self.flutterLink, json=pose)
        return response.text